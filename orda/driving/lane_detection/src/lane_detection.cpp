// ─────────────────────────────────────────────────────────────────────────────
// lane_detection.cpp
//
// 역할: PIDNet 세그멘테이션 클래스맵 기반 BEV 중앙선 추적 노드
//   1) PIDNet-S가 발행한 클래스맵에서 중앙선(class 1)을 뽑는다.
//   2) 사다리꼴 ROI를 BEV(Bird's Eye View)로 투시변환한다.
//   3) 수평/수직 노이즈를 억제한 뒤 슬라이딩 윈도우로 중앙선을 추적한다.
//   4) 최소자승 직선 피팅 결과를 오프셋(/lane_offset)과 피팅 파라미터
//      (/lane_fit)로 발행한다.
//   5) 같은 클래스맵에서 바깥 실선(left_solid/right_solid)까지의 여유를 재
//      /lane_guardrail 로 발행한다. 중앙선만 보는 Pure Pursuit 은 조향 상한
//      (실측 38.9)에 묶여 있어 차선을 벗어나기 직전에 더 꺾을 수단이 없다.
//
// 입력이 색상 임계값에서 CNN 마스크로 바뀌었을 뿐, (2)~(4)의 BEV·노이즈
// 억제·앵커 추적·강건 피팅은 그대로다. 그래서 JSON의 yellow_* / canny_* /
// gaussian_blur_kernel_size 항목은 더 이상 주행에 영향을 주지 않는다
// (스키마 호환을 위해 남겨두었을 뿐이며, hls_hsv_tuner 전용이다).
//
// 구독: /pidnet_class_map (sensor_msgs/Image, MONO8 라벨 0~5)
//       /mode_info     (std_msgs/Int32MultiArray, [mode, lane])
// 발행: /lane_offset       (std_msgs/Int16, 픽셀 단위 오프셋)
//       /lane_fit           (std_msgs/Float32MultiArray, [m, b])
//       /lane_path_preview  (Float32MultiArray,
//                            [먼 offset, 곡률, 신뢰도, y비율, source offset])
//       /lane_change_state  (std_msgs/Int32MultiArray, [변경중, 성공여부])
//       /lane_guardrail     (std_msgs/Float32MultiArray, [좌 여유, 우 여유] BEV px)
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // Jazzy
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>    // Humble
#else
#error "cv_bridge header not found"
#endif
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>

#include <cstdio>
#include <numeric>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <functional>
#include <algorithm>
#include <chrono>
#include <optional>
#include "lane_change_state_tracker.hpp"
#include "lane_detection/class_map.hpp"
#include "lane_detection/lane_guardrail.hpp"
#include "lane_detection/lane_measurement_publication_policy.hpp"
#include "lane_detection/lane_pipeline_diagnostics.hpp"
#include "lane_detection/path_preview.hpp"
#include "lane_detection/reacquire_histogram.hpp"
#include "parameter_loader.hpp"

using namespace std;
using namespace cv;

// ─────────────────────────────────────────────────────────────────────────────
// 직선 모델: x = m*y + b
//
// 일반적인 y=ax+b 대신 x=f(y) 형태를 사용하는 이유:
//   카메라 영상에서 차선은 세로 방향에 가까우므로 x=f(y)가 기울기 발산 없이
//   안정적으로 피팅된다.
// ─────────────────────────────────────────────────────────────────────────────
struct LineFit { float m; float b; };

// 통합 디버그 모니터 창 이름.
static constexpr const char * MONITOR_WINDOW = "Lane Drive Monitor";


class LaneDetector : public rclcpp::Node
{
public:
  // ─────────────────────────────────────────────────────────────────────
  // 생성자
  //
  // Config 파라미터를 멤버 변수로 초기화하고,
  // ROS 토픽 구독/발행 및 BEV 호모그래피 행렬을 설정한다.
  // ─────────────────────────────────────────────────────────────────────
  LaneDetector(const Config & config)
  : Node("lane_detector_node"), config_(config),
    lane_mode_(config_.lane_mode),
    frame_width_(config_.frame_width),
    frame_height_(config_.frame_height),
    roi_top_width_(static_cast<int>(frame_width_ * config_.roi_top_width_coefficient)),
    roi_bottom_width_(static_cast<int>(frame_width_ * config_.roi_bottom_width_coefficient)),
    roi_top_y_(static_cast<int>(frame_height_ * config_.roi_top_y_coefficient)),
    roi_bottom_y_(static_cast<int>(frame_height_ * config_.roi_bottom_y_coefficient)),
    center_reference_lane_one_(config_.center_reference_lane_one),
    center_reference_lane_two_(config_.center_reference_lane_two),
    center_reference_center_(config_.center_reference_center),
    lane_change_tracker_(config_.lane_change_tol_straight,
      config_.lane_change_tol_curve,
      config_.lane_change_tol_change,
      config_.lane_change_streak_need,
      config_.lane_change_m_split)
  {
    // QoS 프로파일
    // - qos_sensor: 영상/센서처럼 최신성 우선, 유실 허용 (Best Effort)
    // - qos_fast:   수치 토픽용, Best Effort + Volatile (latch 없음)
    // SensorDataQoS()의 기본 깊이는 5다. 마스크가 인지 처리보다 빨리 들어오면
    // 그만큼 큐에 쌓여 계속 과거 프레임으로 조향하게 되므로 깊이를 1로 두어
    // 밀린 프레임은 버리고 항상 최신 마스크만 본다.
    auto qos_sensor = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // PIDNet-S 클래스맵 구독: /pidnet_class_map (MONO8, 라벨 0~5)
    //
    // 예전에는 pidnet 이 중앙선만 골라 만든 /lane_segmentation_mask 를 받았다.
    // 가드레일 항이 같은 프레임의 바깥 실선도 필요한데, 마스크와 클래스맵을 따로
    // 구독하면 pidnet 이 마스크를 먼저 발행하는 탓에 마스크 콜백 시점에 같은
    // stamp 의 클래스맵이 아직 안 와 있을 수 있다. 클래스맵 하나만 받고 중앙선
    // 마스크를 여기서 만들면 그 어긋남 자체가 없어진다.
    const std::string mask_topic = this->declare_parameter<std::string>(
      "mask_topic", "/pidnet_class_map");
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      mask_topic, qos_sensor,
      std::bind(&LaneDetector::imageCallback, this, std::placeholders::_1)
    );

    // 중앙선으로 쓸 클래스. pidnet 의 lane_classes 와 같은 의미이며, 런치에서
    // 두 곳에 같은 값을 넘겨 의미를 맞춘다 (1=center_lane).
    const auto center_classes = this->declare_parameter<std::vector<int64_t>>(
      "center_classes", {1});
    center_classes_.clear();
    for (const auto value : center_classes) {
      center_classes_.push_back(static_cast<int>(value));
    }
    // 가드레일용 바깥 실선 클래스 (2=left_solid, 3=right_solid).
    left_rail_class_ = static_cast<int>(
      this->declare_parameter<int64_t>("left_rail_class", 2));
    right_rail_class_ = static_cast<int>(
      this->declare_parameter<int64_t>("right_rail_class", 3));

    rail_config_.min_row_pixels = static_cast<int>(
      this->declare_parameter<int64_t>("guardrail_min_row_pixels", 3));
    rail_config_.min_valid_row_ratio = static_cast<float>(
      this->declare_parameter<double>("guardrail_min_valid_row_ratio", 0.5));
    rail_config_.band_top_ratio = static_cast<float>(
      this->declare_parameter<double>("guardrail_band_top_ratio", 0.60));
    rail_config_.band_bottom_ratio = static_cast<float>(
      this->declare_parameter<double>("guardrail_band_bottom_ratio", 0.95));

    // 먼 차선 경로 미리보기. 기존 /lane_offset은 손대지 않고, 슬라이딩 윈도우
    // 중심점을 정규화 2차식으로 별도 피팅해 /lane_path_preview로 발행한다.
    // main_node가 기능 플래그를 끄거나 이 신뢰도가 낮으면 기존 제어로 폴백한다.
    path_preview_target_y_ratio_ = static_cast<float>(std::clamp(
      this->declare_parameter<double>("path_preview_target_y_ratio", 0.7),
      0.0, 1.0));
    path_preview_min_points_ = static_cast<std::size_t>(std::max<int64_t>(
      3, this->declare_parameter<int64_t>("path_preview_min_windows", 7)));
    path_preview_min_span_ratio_ = static_cast<float>(std::clamp(
      this->declare_parameter<double>("path_preview_min_span_ratio", 0.45),
      0.05, 1.0));
    path_preview_max_rmse_px_ = static_cast<float>(std::max(
      1.0, this->declare_parameter<double>("path_preview_max_rmse_px", 25.0)));

    // 디버그 게이지 눈금. 제어값이 아니라 표시용이므로 control.py 의
    // GUARDRAIL_PARAMS 와 같은 값으로 맞춰 두기만 하면 된다.
    guardrail_display_margin_px_ =
      this->declare_parameter<double>("guardrail_display_margin_px", 190.0);
    guardrail_display_min_trust_px_ =
      this->declare_parameter<double>("guardrail_display_min_trust_px", 15.0);

    RCLCPP_INFO(
      get_logger(),
      "PIDNet 클래스맵 사용: %s (중앙선 클래스 %zu개, 레일 %d/%d)",
      mask_topic.c_str(), center_classes_.size(),
      left_rail_class_, right_rail_class_);

    // 디버그 창은 JSON 값을 기본으로 하되 ROS 파라미터로 덮어쓸 수 있다.
    // JSON을 고치면 실차 주행에도 그대로 남아 성능을 깎으므로, 볼 때만
    // 런치에서 켜고 끄는 쪽이 안전하다.
    debug_view_ = this->declare_parameter<bool>("debug_view", config_.debug_view);
    debug_lane_view_ =
      this->declare_parameter<bool>("debug_lane_view", config_.debug_lane_view);
    enable_reacquire_full_bev_fallback_ = this->declare_parameter<bool>(
      "enable_reacquire_full_bev_fallback", false);
    RCLCPP_INFO(
      get_logger(), "디버그 창: debug_view=%s debug_lane_view=%s",
      debug_view_ ? "true" : "false", debug_lane_view_ ? "true" : "false");

    // 원본 카메라 영상 구독(모니터 창의 CAMERA 패널 전용).
    // 주행 판단에는 쓰지 않는다. 디버그를 끈 상태에서 프레임을 복사하는
    // 비용을 물지 않도록 debug_view_ 일 때만 구독을 만든다.
    const std::string camera_topic = this->declare_parameter<std::string>(
      "camera_topic", "/resized_image");
    if (debug_view_) {
      camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic, qos_sensor,
        std::bind(&LaneDetector::cameraCallback, this, std::placeholders::_1)
      );
      RCLCPP_INFO(
        get_logger(), "모니터 창 CAMERA 패널 입력: %s", camera_topic.c_str());
    }

    // 모드/차선 정보 구독: /mode_info [mode, lane]
    //   mode: 3=차선주행, 5=차선변경
    //   lane: 0=1차선,    1=2차선
    mode_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/mode_info", qos_fast,
      std::bind(&LaneDetector::modeCallback, this, std::placeholders::_1)
    );

    // 모터 명령 구독: /xycar_motor [조향각, 속도]
    // "Vehicle Dynamics" 디버그 창 시각화에만 사용한다.
    motor_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/xycar_motor", qos_fast,
      std::bind(&LaneDetector::motorCallback, this, std::placeholders::_1)
    );

    // 오프셋 발행: /lane_offset (픽셀 단위 Int16)
    offset_pub_ = this->create_publisher<std_msgs::msg::Int16>("/lane_offset", qos_fast);

    // 현재 프레임 차선 피팅 유효성 발행
    validity_pub_ = this->create_publisher<std_msgs::msg::Bool>("/lane_valid", qos_fast);

    // 차선 회귀 파라미터 발행: /lane_fit ([m, b], 프레임 좌표계)
    fit_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/lane_fit", qos_fast);

    // [먼 목표 offset px, 정규화 signed curvature, confidence, 목표 BEV y 비율,
    //  같은 프레임의 /lane_offset 값]
    // confidence=0은 이번 프레임의 미리보기를 쓰지 말라는 명시적 무효화다.
    path_preview_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/lane_path_preview", qos_fast);

    // 바깥 실선 여유 발행: /lane_guardrail ([좌 여유, 우 여유], BEV px)
    // 음수는 미관측이다. /lane_offset 과 같은 콜백에서 나가므로 항상 같은 프레임이다.
    guardrail_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/lane_guardrail", qos_fast);

    // 차선 변경 상태 발행: /lane_change_state ([변경중여부, 성공여부])
    lane_change_state_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
      "/lane_change_state", qos_fast);

    // 실측 현재 차선 발행: /lane_position (-1=미확정, 0=중앙, 1=왼쪽, 2=오른쪽)
    // /lane_change_state 가 "변경이 끝났다"는 이벤트라면, 이쪽은 매 프레임
    // "지금 어느 차선인가"를 알려주는 연속값이다. 둘은 성격이 달라 공존한다.
    lane_position_pub_ = this->create_publisher<std_msgs::msg::Int16>(
      "/lane_position", qos_fast);

    // 처리 단계 계측은 명시적으로 켠 진단 replay에서만 생성한다. false 경로는
    // publisher를 만들지 않고 image callback에도 null 포인터를 전달하므로,
    // countNonZero·문자열 생성·진단 발행 비용이 production에 들어가지 않는다.
    diagnostics_enabled_ =
      this->declare_parameter<bool>("publish_pipeline_diagnostics", false);
    const std::string diagnostics_topic = this->declare_parameter<std::string>(
      "pipeline_diagnostics_topic", "/lane_detection/pipeline_diagnostics");
    if (diagnostics_enabled_) {
      diagnostics_pub_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        diagnostics_topic, qos_fast);
    }

    // BEV 호모그래피 행렬 계산 (ROI 사다리꼴 → 직사각형)
    buildHomography();

    // ref 비율 스무딩 전환 초기화
    smooth_enabled_ = config_.change_ref_smoothly;
    if (config_.lane_ref_transition_duration_sec > 0.0) {
      ref_transition_duration_sec_ = config_.lane_ref_transition_duration_sec;
    }

    if (smooth_enabled_) {
      ref_ratio_current_ = getTargetRefForMode(lane_mode_);
      ref_ratio_start_ = ref_ratio_current_;
      ref_ratio_target_ = ref_ratio_current_;
      ref_start_time_ = this->now();
      ref_transition_active_ = false;
    }
  }


  // ─────────────────────────────────────────────────────────────────────
  // (2) BEV 호모그래피 행렬 계산
  //
  // ROI 사다리꼴(원본 좌표) → 직사각형(BEV 좌표) 투시변환 행렬 H_ 와
  // 역변환 행렬 H_inv_ 를 계산하여 멤버에 저장한다.
  // ─────────────────────────────────────────────────────────────────────
  void buildHomography()
  {
    int cx = frame_width_ / 2;

    // 원본 사다리꼴 꼭짓점 순서: 좌상, 우상, 우하, 좌하
    Point2f src[4] = {
      Point2f(cx - roi_top_width_ / 2, static_cast<float>(roi_top_y_)),
      Point2f(cx + roi_top_width_ / 2, static_cast<float>(roi_top_y_)),
      Point2f(cx + roi_bottom_width_ / 2, static_cast<float>(roi_bottom_y_)),
      Point2f(cx - roi_bottom_width_ / 2, static_cast<float>(roi_bottom_y_))
    };

    // BEV 결과 크기 결정
    int bev_w = roi_bottom_width_;
    if (bev_w <= 0) {bev_w = std::max(roi_top_width_, frame_width_);}
    int bev_h = std::max(1, roi_bottom_y_ - roi_top_y_);
    bev_size_ = Size(bev_w, bev_h);

    // BEV 직사각형 꼭짓점 순서: 좌상, 우상, 우하, 좌하
    Point2f dst[4] = {
      Point2f(0.f, 0.f),
      Point2f(bev_w - 1.f, 0.f),
      Point2f(bev_w - 1.f, bev_h - 1.f),
      Point2f(0.f, bev_h - 1.f)
    };

    H_ = getPerspectiveTransform(src, dst);
    H_inv_ = H_.inv();      // 역변환: BEV → 원본 프레임
  }

  // ─────────────────────────────────────────────────────────────────────
  // (3-a) 수평 노이즈 행 억제
  //
  // BEV 이진 영상에서 가로 방향으로 길게 이어진 흰색 블록을 노이즈로
  // 판단하여 해당 행 범위를 0으로 지운다.
  //
  // band_h 줄을 세로로 OR 합산하여 대각 방향 노이즈도 검출하고,
  // corridor 내 픽셀 비율 조건을 함께 확인해 오탐을 줄인다.
  //
  // 입력 파라미터:
  //   bev_in                  : BEV 이진 영상 (CV_8UC1)
  //   bev_out                 : 억제 결과 영상
  //   horizontal_noise_width  : 노이즈로 볼 최소 가로 연속 길이 (px)
  //   corridor_ratio_thresh   : 밴드 내 흰 픽셀 / 전체 흰 픽셀 비율 임계값
  //   y_start / y_end         : 검사 세로 범위
  //   band_h                  : 한 번에 OR로 합칠 줄 수
  //   extra_pad_rows          : 삭제 행 위/아래 추가 여유
  //   x_min / x_max           : corridor 가로 범위
  //   vis                     : 디버그 캔버스 (nullptr이면 비활성)
  // ─────────────────────────────────────────────────────────────────────
  bool suppressHorizontalNoiseRows(
    const cv::Mat & bev_in,
    cv::Mat & bev_out,
    int horizontal_noise_width,
    float corridor_ratio_thresh,
    int y_start,
    int y_end,
    int band_h,
    int extra_pad_rows,
    int x_min,
    int x_max,
    cv::Mat * vis = nullptr)
  {
    CV_Assert(bev_in.type() == CV_8UC1);
    const int H = bev_in.rows, W = bev_in.cols;

    bev_out = bev_in.clone();

    // 파라미터 경계 보정
    y_start = std::max(0, y_start);
    y_end = (y_end <= 0 || y_end > H) ? H : y_end;
    band_h = std::max(1, band_h);
    extra_pad_rows = std::max(0, extra_pad_rows);
    x_min = std::max(0, x_min);
    x_max = std::min(W - 1, x_max);
    if (y_start >= y_end || x_min > x_max) {return false;}

    const int corridor_w = x_max - x_min + 1;
    const int corridor_h = y_end - y_start;

    // corridor 전체 흰 픽셀 수 (비율 조건의 분모)
    int corridor_total_whites = 0;
    if (corridor_w > 0 && corridor_h > 0) {
      cv::Mat corridor_roi = bev_in(cv::Rect(x_min, y_start, corridor_w, corridor_h));
      corridor_total_whites = cv::countNonZero(corridor_roi);
    }

    bool suppressed_any = false;

    for (int y = y_start; y <= y_end - band_h; ++y) {
      int best = 0, run = 0;

      // band_h 줄을 세로 OR한 가상 1행에서 연속 흰 픽셀 최대 길이 측정
      for (int x = x_min; x < x_max; ++x) {
        bool on = false;
        for (int dy = 0; dy < band_h; ++dy) {
          if (bev_in.at<uchar>(y + dy, x) > 0) {on = true; break;}
        }
        if (on) {run++; best = std::max(best, run);} else {run = 0;}
      }

      // 비율 조건: 밴드 내 흰 픽셀 수 / corridor 전체 흰 픽셀 수 >= 임계값
      bool ratio_noise = false;
      if (corridor_total_whites > 0) {
        cv::Mat band_roi = bev_in(cv::Rect(x_min, y, corridor_w, band_h));
        int band_whites = cv::countNonZero(band_roi);
        float ratio = static_cast<float>(band_whites) /
          static_cast<float>(corridor_total_whites);
        ratio_noise = (ratio >= corridor_ratio_thresh);
      }

      // 두 조건 모두 충족하면 해당 행 범위를 0으로 삭제
      if (best >= horizontal_noise_width && ratio_noise) {
        const int y0 = std::max(y_start, y - extra_pad_rows);
        const int y1 = std::min(H, y + band_h + extra_pad_rows);

        for (int yy = y0; yy < y1; ++yy) {
          uchar * row = bev_out.ptr<uchar>(yy);
          std::memset(row + x_min, 0, (x_max - x_min + 1));
        }

        if (vis && !vis->empty()) {
          cv::rectangle(
            *vis,
            cv::Rect(x_min, y0, x_max - x_min + 1, y1 - y0),
            cv::Scalar(0, 0, 255), cv::FILLED);
        }

        suppressed_any = true;

        // 패딩 끝 이후로 스캔 위치 이동 (중복 검출 방지)
        y = std::min(y1, y_end - band_h + 1) - 1;
      }
    }

    return suppressed_any;
  }

  // ─────────────────────────────────────────────────────────────────────
  // (3-b) 수직 열 밴드 노이즈 억제
  //
  // BEV 이진 영상에서 band_w 폭의 열 밴드 단위로 세로 픽셀 합을 측정하여
  // 절대 기준(min_pixels) AND 피크 비율(peak_ratio) 조건을 모두 충족하면
  // 해당 열 범위와 양쪽 half_w 여유를 0으로 지운다.
  //
  // 입력 파라미터:
  //   bev_in      : BEV 이진 영상 (CV_8UC1)
  //   bev_out     : 억제 결과 영상
  //   x_min/x_max : 검사할 가로 범위
  //   y_start/y_end : 검사할 세로 범위
  //   band_w      : 열 밴드 폭 (px)
  //   min_pixels  : 밴드 내 최소 픽셀 수 (절대 기준)
  //   peak_ratio  : 최대 피크 대비 비율 기준 (0~1)
  //   half_w      : 밴드 삭제 시 좌우 추가 여유 (px)
  //   vis         : 디버그 캔버스 (nullptr이면 비활성)
  // ─────────────────────────────────────────────────────────────────────
  bool suppressColumnBands(
    const cv::Mat & bev_in,
    cv::Mat & bev_out,
    int x_min, int x_max,
    int y_start, int y_end,
    int band_w,
    int min_pixels,
    float peak_ratio,
    int half_w,
    cv::Mat * vis = nullptr)
  {
    CV_Assert(bev_in.type() == CV_8UC1);
    const int H = bev_in.rows, W = bev_in.cols;

    x_min = std::max(0, x_min);
    x_max = std::min(W - 1, x_max);
    y_start = std::max(0, y_start);
    y_end = (y_end <= 0 || y_end > H) ? H : y_end;
    band_w = std::max(1, band_w);
    half_w = std::max(0, half_w);

    bev_out = bev_in.clone();
    if (x_min > x_max || y_start >= y_end) {return false;}

    // ROI 영역의 열(세로) 픽셀 합 계산
    cv::Mat roi = bev_in(cv::Rect(x_min, y_start, x_max - x_min + 1, y_end - y_start));
    cv::Mat nz = (roi > 0);
    cv::Mat colSum32S;
    cv::reduce(nz, colSum32S, 0, cv::REDUCE_SUM, CV_32S);

    // 전체 열 중 최대 픽셀 수
    int maxVal = 0;
    for (int i = 0; i < colSum32S.cols; ++i) {
      maxVal = std::max(maxVal, colSum32S.at<int>(0, i) / 255);
    }

    bool any = false;

    // band_w 폭 단위로 검사
    for (int i = 0; i < colSum32S.cols; i += band_w) {
      int x_band_start = x_min + i;
      int x_band_end = std::min(x_max, x_band_start + band_w - 1);

      int bandPix = 0;
      for (int j = i; j < i + band_w && j < colSum32S.cols; ++j) {
        bandPix += colSum32S.at<int>(0, j) / 255;
      }

      bool cond_abs = (bandPix >= min_pixels);
      bool cond_rel = (peak_ratio > 0.f) ? (bandPix >= maxVal * peak_ratio) : true;

      if (cond_abs && cond_rel) {
        // 삭제 구간: 밴드 + 좌우 half_w 여유
        int xl = std::max(0, x_band_start - half_w);
        int xr = std::min(W - 1, x_band_end + half_w);

        for (int yy = y_start; yy < y_end; ++yy) {
          uchar * row = bev_out.ptr<uchar>(yy);
          std::memset(row + xl, 0, (xr - xl + 1));
        }

        if (vis && !vis->empty()) {
          cv::rectangle(
            *vis,
            cv::Rect(xl, y_start, xr - xl + 1, y_end - y_start),
            cv::Scalar(0, 0, 255), cv::FILLED);
        }

        any = true;
      }
    }

    return any;
  }

  // ─────────────────────────────────────────────────────────────────────
  // 점 집합에 x = m*y + b 직선을 SVD 최소자승으로 피팅한다.
  // pts가 비어있으면 {0,0}을 반환한다(호출부에서 크기를 먼저 확인할 것).
  // ─────────────────────────────────────────────────────────────────────
  static LineFit fitLineSVD(const vector<Point> & pts)
  {
    Mat X(pts.size(), 2, CV_32F), Y(pts.size(), 1, CV_32F);
    for (size_t i = 0; i < pts.size(); ++i) {
      X.at<float>(i, 0) = static_cast<float>(pts[i].y);
      X.at<float>(i, 1) = 1.f;
      Y.at<float>(i, 0) = static_cast<float>(pts[i].x);
    }
    Mat coeff;
    solve(X, Y, coeff, DECOMP_SVD);
    return LineFit{coeff.at<float>(0, 0), coeff.at<float>(1, 0)};
  }

  // ─────────────────────────────────────────────────────────────────────
  // 반복적 잔차 클리핑(iterative outlier rejection)으로 직선을 강건하게
  // 피팅한다. 슬라이딩 윈도우 20개 중 1~2개가 노이즈(반사광 등)를 주워도,
  // 그 노이즈가 순수 최소제곱 피팅 전체를 흔드는 것을 막기 위함.
  //
  // 절차: 1) 전체 점으로 1차 피팅 → 2) 그 직선에서 reject_px 보다 먼 점을
  // 버리고 재피팅 → 3) iterations 만큼 반복. 남은 점이 최소 임계치보다
  // 적어지면(=이상치 제거가 과했으면) 그 직전 결과를 그대로 유지한다.
  // ─────────────────────────────────────────────────────────────────────
  static LineFit fitLineRobust(
    const vector<Point> & pts, float reject_px, int iterations,
    std::size_t * retained_points = nullptr)
  {
    if (retained_points) {*retained_points = pts.size();}
    LineFit fit = fitLineSVD(pts);
    if (reject_px <= 0.f || iterations <= 0) {return fit;}

    vector<Point> kept = pts;
    for (int iter = 0; iter < iterations; ++iter) {
      vector<Point> inliers;
      inliers.reserve(kept.size());
      for (const auto & p : kept) {
        float pred = fit.m * static_cast<float>(p.y) + fit.b;
        if (std::abs(static_cast<float>(p.x) - pred) <= reject_px) {
          inliers.push_back(p);
        }
      }
      // 너무 많이 걸러졌으면(이상치 제거가 아니라 실제 신호를 지운 것으로
      // 보고) 직전 반복의 피팅을 그대로 쓴다.
      if (inliers.size() < 10 || inliers.size() == kept.size()) {
        kept = std::move(inliers);
        if (retained_points) {*retained_points = kept.size();}
        break;
      }
      kept = std::move(inliers);
      if (retained_points) {*retained_points = kept.size();}
      fit = fitLineSVD(kept);
    }
    return fit;
  }

  // ─────────────────────────────────────────────────────────────────────
  // (4) 슬라이딩 윈도우 직선 피팅
  //
  // BEV 이진 영상에서 중앙선을 슬라이딩 윈도우로 추적하고
  // 최소자승법(SVD)으로 x = m*y + b 직선을 피팅하여 반환한다.
  //
  // 알고리즘:
  //   1) corridor + 가중 히스토그램으로 시작점(base_x) 결정
  //      - ref_x_ 근처에 가우시안 가중치를 부여하여 기준선 부근에서 우선 탐색
  //   2) 아래 → 위로 num_windows 개 윈도우를 올리며 포인트 수집
  //      - 충분한 픽셀(minpix) 발견 시 다음 윈도우 중심을 평균 x로 재조정
  //   3) 수집된 포인트로 SVD 기반 최소자승 직선 피팅 (x = m*y + b)
  //
  // 입력:
  //   bev_binary : BEV 이진 영상 (CV_8UC1)
  //   ok         : 피팅 성공 여부 (출력)
  //   dbg_out    : 디버그 캔버스 포인터 (nullptr이면 비활성)
  // 출력:
  //   LineFit {m, b}: x = m*y + b 형태의 직선 파라미터
  // ─────────────────────────────────────────────────────────────────────
  LineFit fitLaneFromBEV(
    const Mat & bev_binary, bool & ok,
    lane_detection::LanePipelineDiagnostics * diagnostics = nullptr,
    cv::Mat * dbg_out = nullptr,
    lane_detection::PathPreview * path_preview_out = nullptr)
  {
    ok = false;
    if (path_preview_out) {*path_preview_out = lane_detection::PathPreview{};}

    const int h = bev_binary.rows, w = bev_binary.cols;
    if (diagnostics) {
      diagnostics->consecutive_fail_count_before = consecutive_fail_count_;
      diagnostics->fit_reacquire_after_frames = config_.fit_reacquire_after_frames;
    }

    // ── 1) corridor 범위 계산 ───────────────────────────────────────
    // 현재 차선 모드의 ref_ratio로 고정 기준 x(ref_x_)를 계산해둔다.
    // (다른 함수의 노이즈 억제 corridor·디버그 표시에서 계속 쓰인다.)
    float ref_ratio = std::clamp(getActiveRefRatio(), 0.0f, 1.0f);
    ref_x_ = static_cast<int>(std::round(ref_ratio * w));

    // 코리도어/가중치 중심은 ref_x_(고정)가 아니라 "직전 프레임에 실제로
    // 찾은 위치"를 앵커로 쓴다. 예전엔 매 프레임 ref_x_ 주변에서 새로
    // 찾았는데, 그러면 신호가 잠깐 부족해 fallback으로 버티다가 다시
    // 신호가 잡히는 순간 직전 위치와 무관하게 ref_x_ 근처 아무 노란
    // 노이즈로 확 튀는 문제가 있었다(실측: 프레임당 평균 70px, 최대
    // 590px 이동). 트래킹(직전 위치 기준)으로 바꾸면 탐색이 "방금 본
    // 자리" 주변에 머물러 있어서 튈 여지가 줄어든다.
    //
    // 단, 실패(ok=false)가 fit_reacquire_after_frames 프레임 넘게
    // 계속되면 그 앵커 자체를 못 믿는다는 뜻이므로 ref_x_로 되돌아가
    // 다시 찾는다. 이게 없으면 앵커가 한 번 잘못된 자리에 멈췄을 때
    // 좁아진 코리도어 안에서 영원히 못 빠져나오는 문제가 있었다
    // (실측: ok=false 최장 818프레임 연속).
    int anchor_x = ref_x_;
    const bool anchor_trustworthy =
      has_prev_center_fit_ &&
      consecutive_fail_count_ < config_.fit_reacquire_after_frames;
    const bool reacquire_active = has_prev_center_fit_ && !anchor_trustworthy;
    if (anchor_trustworthy) {
      anchor_x = static_cast<int>(prev_center_fit_.m * (h - 1) + prev_center_fit_.b);
      anchor_x = std::clamp(anchor_x, 0, w - 1);
    }
    if (diagnostics) {
      diagnostics->ref_x = ref_x_;
      diagnostics->anchor_x = anchor_x;
      diagnostics->anchor_from_previous = anchor_trustworthy;
      diagnostics->reacquire_active = reacquire_active;
    }

    int x_min = std::max(0, anchor_x - static_cast<int>(config_.corridor_width / 2));
    int x_max = std::min(w - 1, anchor_x + static_cast<int>(config_.corridor_width / 2));
    int histW = x_max - x_min + 1;
    if (histW <= 2) {x_min = 0; x_max = w - 1; histW = w;}
    if (diagnostics) {
      diagnostics->corridor_x_min = x_min;
      diagnostics->corridor_x_max = x_max;
      diagnostics->corridor_pixels = static_cast<std::size_t>(
        cv::countNonZero(bev_binary(cv::Rect(x_min, 0, histW, h))));
    }

    // ── 2) 가중 히스토그램으로 시작점(base_x) 결정 ─────────────────
    // 하단 70% 영역에서 corridor 범위의 열별 픽셀 합 계산 후,
    // anchor_x 근처에 가우시안 가중치를 곱하여 최댓값 열을 시작점으로 선택
    int y_start_hist = std::min(std::max(static_cast<int>(h * 0.3), 0), h - 1);
    // sigma_ratio가 작을수록 anchor 근처만 강하게 밀어줌(보수적 시작점)
    // w_min을 0.3~0.7 사이로 조절하면 멀리 있어도 완전히 무시되지 않음
    const float sigma_ratio = (config_.ref_hist_sigma_ratio > 0.f) ?
      config_.ref_hist_sigma_ratio : 0.35f;
    const float w_min = (config_.ref_hist_min_weight > 0.f &&
      config_.ref_hist_min_weight < 1.f) ?
      config_.ref_hist_min_weight : 0.50f;
    const auto histogram_selection = lane_detection::selectReacquireHistogram(
      bev_binary, x_min, x_max, y_start_hist, anchor_x,
      sigma_ratio, w_min,
      reacquire_active && enable_reacquire_full_bev_fallback_);
    const int base_x = histogram_selection.base_x;
    const int tracking_x_min = histogram_selection.tracking_x_min;
    const int tracking_x_max = histogram_selection.tracking_x_max;
    if (diagnostics) {
      diagnostics->reacquire_fallback_attempted = histogram_selection.fallback_attempted;
      diagnostics->reacquire_fallback_candidate = histogram_selection.fallback_candidate;
    }
    if (diagnostics) {diagnostics->base_x = base_x;}

    // ── 3) 디버그 캔버스 준비 ───────────────────────────────────────
    cv::Mat dbg;
    if (dbg_out) {
      cv::cvtColor(bev_binary, dbg, cv::COLOR_GRAY2BGR);
      // ref_x_: 초록 수직선(차선 모드 고정 기준점)
      cv::line(dbg, {ref_x_, 0}, {ref_x_, h - 1}, {0, 255, 0}, 1);
      // anchor_x: 청록색 수직선(이번 프레임 탐색 중심 = 직전 위치 추적)
      cv::line(dbg, {anchor_x, 0}, {anchor_x, h - 1}, {255, 255, 0}, 1);
      // corridor 좌우 경계: 파란 점선
      for (int y = 0; y < h; y += 6) {
        dbg.at<cv::Vec3b>(y, std::clamp(x_min, 0, w - 1)) = {255, 0, 0};
        dbg.at<cv::Vec3b>(y, std::clamp(x_max, 0, w - 1)) = {255, 0, 0};
      }
      // base_x: 보라색 마커
      cv::line(dbg, {base_x, h - 1}, {base_x, h - 21}, {255, 0, 255}, 2);
    }

    // ── 4) 슬라이딩 윈도우로 포인트 수집 ──────────────────────────
    // 아래에서 위로 윈도우를 올리며 non-zero 픽셀을 수집한다.
    // minpix 이상 발견 시 다음 윈도우 중심을 평균 x로 재조정(recenter)한다.
    const int num_windows = std::max(2, config_.sliding_window_num_windows);
    const int margin = std::max(5, config_.sliding_window_margin);
    const size_t minpix = std::max<size_t>(5, config_.sliding_window_minpix);
    const int win_h = h / num_windows;

    vector<Point> pts;
    // 곡선 피팅에는 선 굵기에 따라 개수가 달라지는 모든 흰 픽셀이 아니라
    // 각 슬라이딩 윈도우의 중심 한 점만 쓴다. 그래야 가까운 굵은 마스크가 먼
    // 경로를 압도하지 않고, BEV 전 구간이 비슷한 가중치로 들어간다.
    vector<Point2f> window_centers;
    window_centers.reserve(static_cast<std::size_t>(num_windows));
    int x_current = base_x;

    for (int i = 0; i < num_windows; ++i) {
      int y_low = std::max(0, h - (i + 1) * win_h);
      int y_high = std::min(h, h - i * win_h);
      int xl = std::max(0, x_current - margin);
      int xr = std::min(w, x_current + margin);

      vector<int> xs;        // 이번 윈도우에서 발견된 픽셀들의 x 좌표
      for (int yy = y_low; yy < y_high; ++yy) {
        const uchar * row = bev_binary.ptr<uchar>(yy);
        for (int xx = xl; xx < xr; ++xx) {
          if (row[xx] > 0) {pts.emplace_back(xx, yy); xs.push_back(xx);}
        }
      }

      bool recentered = false;
      float measured_x = static_cast<float>(x_current);
      if (xs.size() >= minpix) {
        // 픽셀이 충분하면 다음 윈도우 중심을 평균 x로 재조정
        int sum = std::accumulate(xs.begin(), xs.end(), 0);
        measured_x = static_cast<float>(sum) / static_cast<float>(xs.size());
        x_current = static_cast<int>(std::round(measured_x));
        recentered = true;
      }
      x_current = std::min(std::max(x_current, tracking_x_min), tracking_x_max);
      if (recentered) {
        // 다음 윈도우의 탐색 중심은 corridor 안에 제한하되, 곡선 피팅에는
        // 제한 전 실제 픽셀 평균을 넣는다. 경계에 붙인 추적 상태를 측정값으로
        // 쓰면 잘못된 평탄 곡선이 낮은 RMSE로 승인될 수 있다.
        window_centers.emplace_back(
          measured_x,
          0.5F * static_cast<float>(y_low + y_high - 1));
      }

      if (dbg_out) {
        cv::Scalar boxColor = recentered ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 165, 255);
        cv::rectangle(dbg, {xl, y_low}, {xr, y_high}, boxColor, 2);
        cv::drawMarker(
          dbg, {x_current, (y_low + y_high) / 2},
          {255, 255, 255}, cv::MARKER_CROSS, 10, 1);
      }
    }
    if (diagnostics) {diagnostics->sliding_points = pts.size();}

    // ── 5) 포인트 부족 시 이전 프레임 값으로 fallback ──────────────
    if (pts.size() < 10) {
      ok = false;
      ++consecutive_fail_count_;
      if (diagnostics) {
        diagnostics->fit_valid = false;
        diagnostics->consecutive_fail_count_after = consecutive_fail_count_;
      }
      if (dbg_out && has_prev_center_fit_) {
        float m = prev_center_fit_.m, b = prev_center_fit_.b;
        cv::line(
          dbg,
          {std::clamp((int)(m * 0 + b), 0, w - 1), 0},
          {std::clamp((int)(m * (h - 1) + b), 0, w - 1), h - 1},
          {200, 200, 200}, 1);
      }
      if (dbg_out) {*dbg_out = std::move(dbg);}
      return has_prev_center_fit_ ? prev_center_fit_ : LineFit{0.f, 0.f};
    }

    // ── 6) 강건 직선 피팅: x = m*y + b (SVD + 반복 이상치 제거) ────
    // 1차 SVD 피팅 후, 그 직선에서 fit_outlier_reject_px 보다 먼 점을
    // 버리고 재피팅하기를 fit_outlier_iterations 회 반복한다. 노이즈를
    // 주운 윈도우 1~2개가 전체 피팅을 흔드는 것을 막는다.
    std::size_t * retained_points =
      diagnostics ? &diagnostics->robust_retained_points : nullptr;
    if (diagnostics) {diagnostics->robust_input_points = pts.size();}
    LineFit fit = fitLineRobust(
      pts, config_.fit_outlier_reject_px,
      config_.fit_outlier_iterations, retained_points);
    if (path_preview_out) {
      *path_preview_out = lane_detection::estimatePathPreview(
        window_centers,
        w,
        h,
        static_cast<float>(ref_x_),
        path_preview_target_y_ratio_,
        path_preview_min_points_,
        path_preview_min_span_ratio_,
        path_preview_max_rmse_px_);
    }
    ok = true;
    consecutive_fail_count_ = 0;
    if (diagnostics) {
      diagnostics->fit_valid = true;
      diagnostics->reacquire_fallback_success =
        histogram_selection.fallback_attempted && histogram_selection.fallback_candidate;
      diagnostics->consecutive_fail_count_after = consecutive_fail_count_;
    }

    // ── 7) 최종 선 디버그 표시 ─────────────────────────────────────
    if (dbg_out) {
      float m = fit.m, b = fit.b;
      cv::line(
        dbg,
        {std::clamp((int)(m * 0 + b), 0, w - 1), 0},
        {std::clamp((int)(m * (h - 1) + b), 0, w - 1), h - 1},
        {0, 255, 0}, 2);
      *dbg_out = std::move(dbg);
    }
    return fit;
  }

  // ─────────────────────────────────────────────────────────────────────
  // (5) 기준선 대비 오프셋 계산
  //
  // BEV 근거리 두 지점(y=30%, y=80%)에서 중앙선 x를 샘플링하여 평균을 내고,
  // 현재 모드의 기준선(ref_ratio × bev_width)과의 차이를 반환한다.
  //
  // 반환값:
  //   + : 중앙선이 기준선보다 오른쪽 (목표 경로가 오른쪽 → 우조향 필요)
  //   - : 중앙선이 기준선보다 왼쪽
  // ─────────────────────────────────────────────────────────────────────
  float calcOffsetFromCenterLine(const LineFit & lf, int bev_width) const
  {
    // 근거리 우선 샘플링: BEV 높이의 30% 및 80% 지점
    float y1 = bev_size_.height * 0.3f;
    float y2 = bev_size_.height * 0.8f;

    float x1 = lf.m * y1 + lf.b;
    float x2 = lf.m * y2 + lf.b;
    float x_mean = 0.5f * (x1 + x2);

    float ref_ratio = std::clamp(getActiveRefRatio(), 0.0f, 1.0f);
    float x_ref = ref_ratio * static_cast<float>(bev_width);

    return x_mean - x_ref;
  }

  // ─────────────────────────────────────────────────────────────────────
  // 가드레일: 바깥 실선까지의 여유 측정
  //
  // 중앙선과 **같은 사다리꼴 ROI, 같은 호모그래피**로 warp 하므로 결과가
  // /lane_offset 과 같은 BEV px 단위로 나온다. 차량 중심은 사다리꼴이 cx
  // 대칭이라 BEV 폭의 절반이다.
  //
  // 중앙선용 수평/수직 노이즈 억제는 일부러 적용하지 않는다. 그 억제기들은
  // "얇은 선 하나"를 전제로 튜닝돼 있어 굵은 실선을 통째로 지워 버린다.
  // 스페클은 lane_guardrail.hpp 의 행 중앙값이 걸러낸다.
  // ─────────────────────────────────────────────────────────────────────
  lane_detection::RailMargins measureRailMargins(const Mat & class_map)
  {
    if (debug_view_) {
      // 시각화용 사본은 사다리꼴로 자르기 전에 떠 둔다. 모델이 ROI 밖에서
      // 무엇을 봤는지도 보여야 "왜 여유가 저렇게 나왔나"를 판단할 수 있다.
      left_rail_display_ =
        lane_detection::extractClassMask(class_map, left_rail_class_);
      right_rail_display_ =
        lane_detection::extractClassMask(class_map, right_rail_class_);
    }
    if (bev_size_.width <= 1 || bev_size_.height <= 1 || H_.empty()) {
      last_rail_margins_ = {};
      return last_rail_margins_;
    }
    const Mat trapezoid = trapezoidMask(class_map.size());
    const int ego_x = bev_size_.width / 2;

    auto measure = [&](int rail_class, Mat * bev_out) {
        Mat rail = lane_detection::extractClassMask(class_map, rail_class);
        bitwise_and(rail, trapezoid, rail);
        Mat bev_rail;
        warpPerspective(
          rail, bev_rail, H_, bev_size_,
          INTER_NEAREST, BORDER_CONSTANT, Scalar(0));
        if (bev_out) {*bev_out = bev_rail;}
        return lane_detection::railMargins(bev_rail, ego_x, rail_config_);
      };

    const bool want_bev = debug_view_ && debug_lane_view_;
    last_rail_margins_ = lane_detection::mergeRailMargins(
      measure(left_rail_class_, want_bev ? &left_rail_bev_ : nullptr),
      measure(right_rail_class_, want_bev ? &right_rail_bev_ : nullptr));
    return last_rail_margins_;
  }

  // ─────────────────────────────────────────────────────────────────────
  // 세 차선 클래스를 색으로 구분한 원본 해상도 마스크 (사다리꼴 자르기 전).
  // pidnet 색 규약: center_lane=초록, left_solid=파랑, right_solid=빨강.
  // ─────────────────────────────────────────────────────────────────────
  cv::Mat buildClassMaskView(const cv::Mat & center_mask) const
  {
    cv::Mat canvas(frame_height_, frame_width_, CV_8UC3, cv::Scalar(0, 0, 0));
    auto paint = [&](const cv::Mat & mask, const cv::Scalar & color) {
        if (!mask.empty() && mask.size() == canvas.size()) {
          canvas.setTo(color, mask);
        }
      };
    paint(left_rail_display_, {255, 80, 0});
    paint(right_rail_display_, {0, 80, 255});
    paint(center_mask, {0, 255, 0});

    // 기준선은 CAMERA 패널과 동일하게 그린다. 라벨 ROI와 BEV 윗변은 둘 다
    // y=216에서 시작하며, 주황 사다리꼴은 그중 실제 투시 변환에 사용하는
    // 가로 범위까지 함께 보여 준다.
    drawReferenceLines(canvas);
    return canvas;
  }

  // ─────────────────────────────────────────────────────────────────────
  // 가드레일 BEV 디버그 창: 레일 마스크 위에 실제 측정값을 겹쳐 그린다.
  //
  // 숫자만 봐서는 "레일이 저기 있는 게 맞나"를 알 수 없다. 샘플링한 근거리
  // 행 구간, 차량 중심선, 반발이 시작되는 임계 여유, 그리고 그 프레임에서
  // 실제로 고른 좌/우 레일 위치를 한 장에 겹쳐 놓는다.
  // ─────────────────────────────────────────────────────────────────────
  cv::Mat buildGuardrailBevView() const
  {
    if (left_rail_bev_.empty() || right_rail_bev_.empty()) {
      return placeholderPanel(bev_size_.width, bev_size_.height, "no rail BEV");
    }
    cv::Mat canvas(bev_size_, CV_8UC3, cv::Scalar(0, 0, 0));
    // pidnet 색 규약을 따른다: left_solid=파랑, right_solid=빨강.
    canvas.setTo(cv::Scalar(255, 80, 0), left_rail_bev_);
    canvas.setTo(cv::Scalar(0, 80, 255), right_rail_bev_);

    const int ego_x = bev_size_.width / 2;
    const int y_from = static_cast<int>(bev_size_.height * rail_config_.band_top_ratio);
    const int y_to = std::min(
      bev_size_.height - 1,
      static_cast<int>(bev_size_.height * rail_config_.band_bottom_ratio));

    // 샘플링 밴드 밖은 어둡게 죽인다 — 여기 픽셀은 측정에 안 들어간다.
    if (y_from > 0) {canvas(cv::Rect(0, 0, canvas.cols, y_from)) *= 0.35;}
    if (y_to + 1 < canvas.rows) {
      canvas(cv::Rect(0, y_to + 1, canvas.cols, canvas.rows - y_to - 1)) *= 0.35;
    }

    // 차량 중심선(흰색)과 반발 시작 임계선(노란 점선 대용: 얇은 실선).
    cv::line(canvas, {ego_x, 0}, {ego_x, canvas.rows - 1}, {255, 255, 255}, 1);
    const int threshold = static_cast<int>(std::round(guardrail_display_margin_px_));
    if (threshold > 0) {
      for (int y = 0; y < canvas.rows; y += 6) {
        const int y2 = std::min(canvas.rows - 1, y + 3);
        cv::line(canvas, {ego_x - threshold, y}, {ego_x - threshold, y2}, {0, 220, 220}, 1);
        cv::line(canvas, {ego_x + threshold, y}, {ego_x + threshold, y2}, {0, 220, 220}, 1);
      }
    }

    // 이번 프레임에 실제로 고른 레일 위치(굵은 세로선).
    const int mid_y = (y_from + y_to) / 2;
    if (last_rail_margins_.leftKnown()) {
      const int x = ego_x - static_cast<int>(std::round(last_rail_margins_.left));
      cv::line(canvas, {x, y_from}, {x, y_to}, {255, 255, 0}, 2);
      cv::line(canvas, {x, mid_y}, {ego_x, mid_y}, {255, 255, 0}, 1);
    }
    if (last_rail_margins_.rightKnown()) {
      const int x = ego_x + static_cast<int>(std::round(last_rail_margins_.right));
      cv::line(canvas, {x, y_from}, {x, y_to}, {0, 255, 255}, 2);
      cv::line(canvas, {ego_x, mid_y}, {x, mid_y}, {0, 255, 255}, 1);
    }
    return canvas;
  }

  // ─────────────────────────────────────────────────────────────────────
  // ROI 사다리꼴 마스크 생성
  // roi_*_ 좌표로 정의된 사다리꼴 내부를 흰색(255)으로 채운 마스크를 반환한다.
  // ─────────────────────────────────────────────────────────────────────
  Mat trapezoidMask(Size sz) const
  {
    Mat mask(sz, CV_8UC1, Scalar(0));
    int cx = frame_width_ / 2;
    Point pts[1][4] = {{
      Point(cx - roi_top_width_ / 2, roi_top_y_),           // 좌상
      Point(cx + roi_top_width_ / 2, roi_top_y_),           // 우상
      Point(cx + roi_bottom_width_ / 2, roi_bottom_y_),     // 우하
      Point(cx - roi_bottom_width_ / 2, roi_bottom_y_)      // 좌하
    }};
    const Point * ppt[1] = {pts[0]};
    int npt[] = {4};
    fillPoly(mask, ppt, npt, 1, Scalar(255));
    return mask;
  }

  // ─────────────────────────────────────────────────────────────────────
  // ROI 사다리꼴 디버그 시각화
  // 원본 프레임 위에 ROI 사다리꼴 외곽선(빨강) + 반투명 오버레이를 그린다.
  // ─────────────────────────────────────────────────────────────────────
  void drawROIPolygon(Mat & frame) const
  {
    int cx = frame_width_ / 2;
    vector<Point> pts = {
      Point(cx - roi_top_width_ / 2, roi_top_y_),
      Point(cx + roi_top_width_ / 2, roi_top_y_),
      Point(cx + roi_bottom_width_ / 2, roi_bottom_y_),
      Point(cx - roi_bottom_width_ / 2, roi_bottom_y_)
    };
    polylines(frame, pts, true, Scalar(0, 0, 255), 2);

    // 반투명 빨간색 채우기 (20% 알파)
    Mat overlay = frame.clone();
    fillPoly(overlay, vector<vector<Point>>{pts}, Scalar(0, 0, 255));
    addWeighted(overlay, 0.2, frame, 0.8, 0, frame);
  }

  // ─────────────────────────────────────────────────────────────────────
  // 오프셋 슬라이더 시각화
  // 화면 상단에 오프셋 크기를 막대+점으로 표시하고 모드/값 텍스트를 출력한다.
  // ─────────────────────────────────────────────────────────────────────
  Mat drawOffsetSlider(
    const Mat & bgr, float offset_px, LaneMode mode,
    const lane_detection::PathPreview & path_preview) const
  {
    int sw = frame_width_, sh = 50;
    Mat slider(sh, sw, CV_8UC3, Scalar(50, 50, 50));
    int cx = sw / 2;
    line(slider, Point(cx, 0), Point(cx, sh - 1), Scalar(150, 150, 150), 1);

    // 오프셋 크기에 비례한 점 위치 표시
    int dot_x = cx + static_cast<int>(std::round(offset_px));
    dot_x = std::clamp(dot_x, 0, sw - 1);
    circle(slider, Point(dot_x, sh / 2), 6, Scalar(0, 0, 255), FILLED);
    if (path_preview.valid) {
      int preview_x = cx + static_cast<int>(std::round(path_preview.target_offset_px));
      preview_x = std::clamp(preview_x, 0, sw - 1);
      drawMarker(
        slider, Point(preview_x, sh / 2), Scalar(255, 0, 255),
        MARKER_DIAMOND, 13, 2);
    }

    string mode_str = (mode == LaneMode::CENTER) ? "Mode: Center" :
      (mode == LaneMode::LANE_ONE) ? "Mode: 1-Lane" : "Mode: 2-Lane";
    string offset_str = "Offset(px): " + std::to_string(static_cast<int>(std::round(offset_px)));
    putText(slider, mode_str, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(220, 220, 220), 1);
    putText(slider, offset_str, Point(10, 42), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(220, 220, 220), 1);
    if (path_preview.valid) {
      char preview_text[96];
      std::snprintf(
        preview_text, sizeof(preview_text), "Preview %+.0f  conf %.2f",
        path_preview.target_offset_px, path_preview.confidence);
      putText(
        slider, preview_text, Point(std::max(250, sw - 300), 42),
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 120, 255), 1);
    }

    // 슬라이더를 원본 영상 위에 세로로 이어붙임
    Mat out;
    vconcat(slider, bgr, out);
    return out;
  }

  // ─────────────────────────────────────────────────────────────────────
  // 토픽 발행 + 디버그 시각화
  //
  // 1) /lane_offset  (Int16)                발행
  // 2) /lane_fit     (Float32MultiArray [m, b], 프레임 좌표계) 발행
  // 3) debug_view_ ON 시 슬라이딩 윈도우 BEV + 오프셋 슬라이더 창 표시
  // ─────────────────────────────────────────────────────────────────────
  lane_detection::LaneMeasurementPublicationPolicy publishAndDebug(
    const cv::Mat & frame_in,
    float offset,
    const LineFit & fit_bev,
    bool fit_valid,
    bool show_dbg,
    const cv::Mat & lane_mask_for_display,
    const lane_detection::PathPreview & path_preview)
  {
    cv::Mat frame = frame_in.clone();

    // BEV 직선을 원본 프레임 좌표계의 두 점으로 역투영
    cv::Point2f P0, P1;
    bool mapped = bevLineToFrame(fit_bev, P0, P1);

    // 역투영된 두 점으로 프레임 좌표계 x = m*y + b 복원
    LineFit fit_frame{0.f, 0.f};
    if (mapped) {
      float dy = P1.y - P0.y;
      if (std::abs(dy) < 1e-6f) {
        dy = (dy >= 0 ? 1e-6f : -1e-6f);
      }
      fit_frame.m = (P1.x - P0.x) / dy;
      fit_frame.b = P0.x - fit_frame.m * P0.y;
    }

    const auto publication_policy =
      lane_detection::measurementPublicationPolicy(fit_valid, mapped);

    // 기존 offset/fit과 독립된 선택 입력. invalid 프레임도 confidence=0으로
    // 내보내 main_node가 직전 곡선을 계속 붙들지 않고 즉시 기존 제어로
    // 폴백하게 한다.
    std_msgs::msg::Float32MultiArray preview_msg;
    preview_msg.data = {
      path_preview.valid ? path_preview.target_offset_px : 0.0F,
      path_preview.valid ? path_preview.signed_curvature : 0.0F,
      path_preview.valid ? path_preview.confidence : 0.0F,
      path_preview_target_y_ratio_,
      static_cast<float>(std::round(offset)),
    };
    path_preview_pub_->publish(preview_msg);

    // /lane_offset 발행
    if (publication_policy.publish_offset) {
      std_msgs::msg::Int16 offset_msg;
      offset_msg.data = static_cast<int16_t>(std::round(offset));
      offset_pub_->publish(offset_msg);
    }

    // /lane_fit 발행 (프레임 좌표계 [m, b])
    if (publication_policy.publish_fit) {
      std_msgs::msg::Float32MultiArray fit_msg;
      fit_msg.data = {fit_frame.m, fit_frame.b};
      fit_pub_->publish(fit_msg);
    }

    // 슬라이딩 윈도우 BEV 디버그 창 표시 기능은 아래 통합 모니터에 포함된다.
    // 통합 모니터 창: CAMERA / VEHICLE 2개 패널을 한 창에 그린다.
    // 조향·속도는 /xycar_motor 콜백이 캐시해 둔 최신 값을 쓰므로, 여기서
    // 프레임당 한 번만 그려도 값이 최신이다.
    if (debug_view_ && show_dbg) {
      cv::imshow(MONITOR_WINDOW, buildMonitorView(lane_mask_for_display));
    }

    // 오프셋 슬라이더 창 표시 (debug_lane_view 설정 시)
    // waitKey는 debug_lane_view와 무관하게 호출해야 한다. HighGUI 이벤트 펌프
    // 역할을 하므로, 이걸 건너뛰면 남아있는 창이 갱신되지 않는다.
    if (debug_view_ && show_dbg && debug_lane_view_) {
      cv::Mat vis = drawOffsetSlider(frame, offset, lane_mode_, path_preview);
      cv::imshow("Lane View + Offset", vis);
      cv::waitKey(1);
    } else {
      cv::waitKey(1);
    }
    return publication_policy;
  }

  // ─────────────────────────────────────────────────────────────────────
  // BEV 직선 → 원본 프레임 좌표 역투영
  //
  // BEV 세로 양 끝(y=0, y=bev_h-1)에서 x를 계산한 뒤
  // H_inv_(BEV→프레임 역호모그래피)로 변환한다.
  // ─────────────────────────────────────────────────────────────────────
  bool bevLineToFrame(const LineFit & lf, cv::Point2f & p_frame0, cv::Point2f & p_frame1)
  {
    if (bev_size_.width <= 1 || bev_size_.height <= 1 || H_inv_.empty()) {return false;}

    float y0 = 0.0f;
    float y1 = static_cast<float>(bev_size_.height - 1);
    float x0 = lf.m * y0 + lf.b;
    float x1 = lf.m * y1 + lf.b;

    auto clampf = [](float v, float lo, float hi) {return std::max(lo, std::min(hi, v));};
    x0 = clampf(x0, 0.0f, static_cast<float>(bev_size_.width - 1));
    x1 = clampf(x1, 0.0f, static_cast<float>(bev_size_.width - 1));

    std::vector<cv::Point2f> src = {{x0, y0}, {x1, y1}};
    std::vector<cv::Point2f> dst;
    cv::perspectiveTransform(src, dst, H_inv_);

    if (dst.size() != 2 ||
      !std::isfinite(dst[0].x) || !std::isfinite(dst[0].y) ||
      !std::isfinite(dst[1].x) || !std::isfinite(dst[1].y)) {return false;}
    p_frame0 = dst[0];
    p_frame1 = dst[1];
    return true;
  }

  // ─────────────────────────────────────────────────────────────────────
  // 회전된 사각형(바퀴 등)을 그리기 위한 헬퍼: center 기준 angle_deg만큼
  // 회전한 (w x h) 크기의 사각형 꼭짓점 4개를 계산해 채운다.
  // ─────────────────────────────────────────────────────────────────────
  static void fillRotatedRect(
    cv::Mat & canvas, cv::Point2f center, float w, float h,
    float angle_deg, cv::Scalar color)
  {
    cv::RotatedRect rr(center, cv::Size2f(w, h), angle_deg);
    cv::Point2f pts[4];
    rr.points(pts);
    std::vector<cv::Point> poly = {pts[0], pts[1], pts[2], pts[3]};
    cv::fillConvexPoly(canvas, poly, color, cv::LINE_AA);
  }

  // ─────────────────────────────────────────────────────────────────────
  // 모니터 창 패널 공통: 위쪽에 제목 띠를 붙인다.
  // ─────────────────────────────────────────────────────────────────────
  static cv::Mat withPanelTitle(const cv::Mat & panel, const std::string & title)
  {
    const int bar_h = 26;
    cv::Mat out(panel.rows + bar_h, panel.cols, CV_8UC3, cv::Scalar(24, 24, 24));
    panel.copyTo(out(cv::Rect(0, bar_h, panel.cols, panel.rows)));

    // 제목은 패널 폭 안에서 끝나야 한다. putText 는 잘라주지 않으므로 긴 제목이
    // 옆 패널 제목 위로 넘어가 겹쳐 읽힌다(실제로 그랬다). 폰트를 줄여 보고,
    // 그래도 안 들어가면 뒤를 잘라낸다.
    const int margin = 10;
    const int usable = std::max(1, panel.cols - 2 * margin);
    double scale = 0.55;
    int baseline = 0;
    std::string text = title;
    while (scale > 0.34 &&
      cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline).width > usable)
    {
      scale -= 0.05;
    }
    while (text.size() > 4 &&
      cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, 1, &baseline).width > usable)
    {
      text.erase(text.size() - 4, 1);
      text.replace(text.size() - 3, 3, "...");
    }
    cv::putText(
      out, text, cv::Point(margin, 18), cv::FONT_HERSHEY_SIMPLEX, scale,
      cv::Scalar(235, 235, 235), 1, cv::LINE_AA);
    return out;
  }

  // 패널이 없을 때(카메라 미수신 등) 자리를 채우는 회색 판.
  static cv::Mat placeholderPanel(int width, int height, const std::string & text)
  {
    cv::Mat panel(height, width, CV_8UC3, cv::Scalar(45, 45, 45));
    cv::putText(
      panel, text, cv::Point(14, height / 2), cv::FONT_HERSHEY_SIMPLEX, 0.6,
      cv::Scalar(160, 160, 160), 1, cv::LINE_AA);
    return panel;
  }

  // ─────────────────────────────────────────────────────────────────────
  // 통합 모니터 창: 3개 패널을 한 창에 합친다.
  //
  //   [ CAMERA (라벨 ROI 오버레이) ] [ MASK (마스크 단독) ] [ VEHICLE (조향/속도) ]
  //
  // CAMERA 패널: 라벨 ROI(하단 40%, y>=label_roi_top_) 위쪽은 어둡게 죽이고
  // 그 아래에는 중앙선 세그멘테이션 마스크를 반투명 초록으로 겹쳐 그린다.
  // 세그멘테이션이 실제 화면 위에서 잘 맞는지 한눈에 확인하기 위함이다.
  //
  // 창을 여러 개 띄우면 각각 imshow+waitKey 비용이 붙고 배치도 흐트러진다.
  // 하나로 합치면 imshow 가 프레임당 한 번만 돈다.
  // ─────────────────────────────────────────────────────────────────────
  cv::Mat buildMonitorView(const cv::Mat & lane_mask_for_display) const
  {
    const int panel_h = 360;

    cv::Mat camera_panel;
    if (latest_camera_.empty()) {
      camera_panel = placeholderPanel(640, panel_h, "no camera frame");
    } else {
      cv::Mat vis = latest_camera_.clone();
      const int roi_top = std::clamp(label_roi_top_, 0, vis.rows);

      // 라벨 ROI 밖(위쪽): 어둡게 죽여서 판단 대상이 아님을 표시한다.
      if (roi_top > 0) {
        cv::Mat above = vis(cv::Rect(0, 0, vis.cols, roi_top));
        above *= CAMERA_ABOVE_ROI_DIM_FACTOR;
      }

      // 라벨 ROI 안(아래쪽): 세그멘테이션 마스크들을 반투명으로 겹친다.
      // 색은 pidnet 규약(infer_pidnet.py CLASS_COLORS)을 그대로 따른다.
      //   center_lane = 초록(조향 목표), left_solid = 파랑, right_solid = 빨강
      // 중앙선과 레일을 구분해서 보여야 "가드레일이 무엇을 보고 밀었나"를
      // 화면에서 바로 확인할 수 있다.
      if (roi_top < vis.rows) {
        const cv::Rect below_rect(0, roi_top, vis.cols, vis.rows - roi_top);
        auto overlay = [&](const cv::Mat & mask, const cv::Scalar & color,
          double alpha) {
            if (mask.empty() || mask.size() != vis.size()) {return;}
            cv::Mat below = vis(below_rect);
            cv::Mat tint(below.size(), below.type(), color);
            cv::Mat blended;
            cv::addWeighted(below, 1.0 - alpha, tint, alpha, 0.0, blended);
            blended.copyTo(below, mask(below_rect));
          };
        // 레일을 먼저 깔고 중앙선을 위에 올린다. 겹치는 픽셀에서는 조향 목표인
        // 중앙선이 보이는 편이 낫다.
        overlay(left_rail_display_, cv::Scalar(255, 80, 0), RAIL_OVERLAY_ALPHA);
        overlay(right_rail_display_, cv::Scalar(0, 80, 255), RAIL_OVERLAY_ALPHA);
        overlay(lane_mask_for_display, cv::Scalar(0, 255, 0), CENTER_LANE_OVERLAY_ALPHA);
      }

      // 라벨 ROI(빨강)와 BEV 사다리꼴(주황)의 윗변은 y=216으로 맞췄다.
      // 실제 오프셋과 레일 여유는 그 아래의 주황 사다리꼴 안쪽만 쓰므로,
      // 두 선을 함께 그려 세로·가로 측정 범위를 모두 확인할 수 있게 한다.
      // Mask-PIDNet-Lanes 창도 같은 함수를 써서 기준선을 맞춘다.
      drawReferenceLines(vis);
      drawGuardrailReadout(vis);

      cv::resize(
        vis, camera_panel,
        cv::Size(std::max(1, vis.cols * panel_h / std::max(1, vis.rows)), panel_h));
    }

    // 마스크 단독 패널. CAMERA 패널은 카메라 영상 위에 반투명(alpha 0.45)으로
    // 겹치므로 바닥 무늬·반사와 섞여 마스크 경계가 어디까지인지 흐려 보인다.
    // 같은 마스크를 검은 배경에 불투명하게 한 번 더 그려 나란히 두면, 모델이
    // 실제로 집은 화소만 그대로 읽을 수 있다. Mask-PIDNet-Lanes 보조 창과 같은
    // 함수를 쓰므로 색 규약과 기준선이 자동으로 일치한다.
    cv::Mat mask_panel;
    {
      const cv::Mat mask_view = buildClassMaskView(lane_mask_for_display);
      cv::resize(
        mask_view, mask_panel,
        cv::Size(
          std::max(1, mask_view.cols * panel_h / std::max(1, mask_view.rows)),
          panel_h));
    }

    // 조향/속도 패널. 정사각 캔버스를 패널 높이에 맞춘다.
    cv::Mat vehicle_panel;
    cv::resize(drawVehicleDynamicsView(), vehicle_panel, cv::Size(panel_h, panel_h));

    cv::Mat monitor;
    // 색 범례는 제목이 아니라 패널 안에 그린다(drawGuardrailReadout).
    // 제목에 넣으면 폭을 넘겨 옆 패널 제목과 겹쳤다.
    //
    // CAMERA 와 MASK 를 붙여 둔다. 오버레이와 마스크 단독을 눈으로 비교하는
    // 것이 이 창의 목적이라, 사이에 VEHICLE 이 끼면 대조가 어렵다.
    const std::vector<cv::Mat> panels{
      withPanelTitle(camera_panel, "CAMERA  segmentation + guardrail"),
      withPanelTitle(mask_panel, "MASK  segmentation only"),
      withPanelTitle(vehicle_panel, "VEHICLE  steer / speed"),
    };
    cv::hconcat(panels, monitor);
    return monitor;
  }

  // ─────────────────────────────────────────────────────────────────────
  // BEV 사다리꼴을 카메라 영상 위에 그려, BEV 창이 실제로 어느 범위를 보고
  // 있는지 원본 화면과 대응시킨다.
  //
  // 라벨 ROI 선(빨강)과 사다리꼴 윗변은 둘 다 height*0.6이다. 학습된
  // 하단 40% 전체를 BEV가 쓰도록 세로 시야를 맞췄으며, 오프셋과 레일 여유는
  // 주황 사다리꼴 안쪽에서만 나온다. 아래쪽 변은 화면 밖(bottom_width 계수 2
  // → 폭 1280)이라 옆변이 아래로 벌어져 보인다.
  //
  // 가드레일이 실제 측정에 쓰는 근거리 밴드(band_top~band_bottom)는 BEV 세로
  // 비율이므로, 사다리꼴 높이를 그 비율로 나눠 원본 위에 다시 표시한다.
  // ─────────────────────────────────────────────────────────────────────
  void drawReferenceLines(cv::Mat & vis) const
  {
    if (vis.empty() || frame_width_ <= 0 || frame_height_ <= 0) {return;}
    const double sy = static_cast<double>(vis.rows) / frame_height_;
    const int roi_top = std::clamp(
      static_cast<int>(std::lround(label_roi_top_ * sy)), 0, vis.rows - 1);
    cv::line(vis, {0, roi_top}, {vis.cols - 1, roi_top}, {0, 0, 255}, 1, cv::LINE_AA);
    cv::putText(
      vis, "label ROI", {vis.cols - 84, std::max(10, roi_top - 5)},
      cv::FONT_HERSHEY_SIMPLEX, 0.36, {0, 0, 255}, 1, cv::LINE_AA);
    drawBevFootprint(vis);
    drawCenterReference(vis);
  }

  // ─────────────────────────────────────────────────────────────────────
  // 조향 기준선: "중앙선이 화면 어디에 보이도록 맞추려는가" 를 그린다.
  //
  // /lane_offset 은 BEV 좌표에서 (중앙선 위치 - 기준선) 이고, 기준선은 BEV 폭
  // 비율(center_reference_*)로만 주어진다. 그 비율이 카메라 화면 어디인지 볼
  // 방법이 없어서, 차가 한쪽으로 치우쳐 가도 기준선이 틀린 건지 인지가 틀린
  // 건지 구분할 수 없었다. 초록 마스크(검출된 중앙선)와 이 선이 겹치면 차가
  // 제자리에 있는 것이고, 벌어진 만큼이 곧 offset 이다.
  //
  // 투영변환은 직선을 직선으로 보내므로 BEV 세로선의 양 끝 두 점이면 충분하다.
  // ─────────────────────────────────────────────────────────────────────
  void drawCenterReference(cv::Mat & vis) const
  {
    if (vis.empty() || frame_width_ <= 0 || frame_height_ <= 0) {return;}
    if (bev_size_.width <= 1 || bev_size_.height <= 1 || H_inv_.empty()) {return;}

    const float ratio = std::clamp(getActiveRefRatio(), 0.0f, 1.0f);
    const float bev_x = ratio * static_cast<float>(bev_size_.width);
    std::vector<cv::Point2f> src = {
      {bev_x, 0.0f}, {bev_x, static_cast<float>(bev_size_.height - 1)}};
    std::vector<cv::Point2f> dst;
    cv::perspectiveTransform(src, dst, H_inv_);
    if (dst.size() != 2 ||
      !std::isfinite(dst[0].x) || !std::isfinite(dst[0].y) ||
      !std::isfinite(dst[1].x) || !std::isfinite(dst[1].y)) {return;}

    const double sx = static_cast<double>(vis.cols) / frame_width_;
    const double sy = static_cast<double>(vis.rows) / frame_height_;
    auto scaled = [&](const cv::Point2f & p) {
        return cv::Point(
          static_cast<int>(std::lround(p.x * sx)),
          static_cast<int>(std::lround(p.y * sy)));
      };
    const cv::Scalar color{255, 255, 0};   // cyan — 초록 마스크와 구분된다
    cv::line(vis, scaled(dst[0]), scaled(dst[1]), color, 2, cv::LINE_AA);

    const char * mode_text = (lane_mode_ == LaneMode::CENTER) ? "CENTER" :
      (lane_mode_ == LaneMode::LANE_ONE) ? "1-LANE" : "2-LANE";
    char label[96];
    std::snprintf(
      label, sizeof(label), "aim %s %.3f  x=%.0f", mode_text, ratio, dst[1].x);
    const cv::Point anchor = scaled(dst[1]);
    cv::putText(
      vis, label,
      {std::clamp(anchor.x - 60, 2, std::max(2, vis.cols - 190)),
        std::max(12, vis.rows - 8)},
      cv::FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv::LINE_AA);
  }

  void drawBevFootprint(cv::Mat & vis) const
  {
    if (vis.empty() || frame_width_ <= 0 || frame_height_ <= 0) {return;}
    const double sx = static_cast<double>(vis.cols) / frame_width_;
    const double sy = static_cast<double>(vis.rows) / frame_height_;
    const int cx = frame_width_ / 2;

    auto point = [&](int x, int y) {
        return cv::Point(
          static_cast<int>(std::lround(x * sx)), static_cast<int>(std::lround(y * sy)));
      };

    const std::vector<cv::Point> pts = {
      point(cx - roi_top_width_ / 2, roi_top_y_),
      point(cx + roi_top_width_ / 2, roi_top_y_),
      point(cx + roi_bottom_width_ / 2, roi_bottom_y_),
      point(cx - roi_bottom_width_ / 2, roi_bottom_y_),
    };
    cv::polylines(vis, pts, true, {0, 165, 255}, 1, cv::LINE_AA);
    cv::putText(
      vis, "BEV", {pts[0].x + 4, pts[0].y - 5},
      cv::FONT_HERSHEY_SIMPLEX, 0.36, {0, 165, 255}, 1, cv::LINE_AA);

    // 가드레일 샘플 밴드. BEV 세로 비율을 원본 y 로 되돌린다.
    const int span = roi_bottom_y_ - roi_top_y_;
    auto band_y = [&](float ratio) {
        return roi_top_y_ + static_cast<int>(std::lround(span * ratio));
      };
    for (const float ratio : {rail_config_.band_top_ratio, rail_config_.band_bottom_ratio}) {
      const int y = std::clamp(band_y(ratio), 0, frame_height_ - 1);
      // 그 높이에서의 사다리꼴 폭만큼만 긋는다.
      const double t = span > 0 ? static_cast<double>(y - roi_top_y_) / span : 0.0;
      const int half = static_cast<int>(std::lround(
          (roi_top_width_ + (roi_bottom_width_ - roi_top_width_) * t) / 2.0));
      cv::line(vis, point(cx - half, y), point(cx + half, y), {0, 255, 255}, 1, cv::LINE_AA);
    }
    cv::putText(
      vis, "guardrail band",
      {point(cx - roi_top_width_ / 2, band_y(rail_config_.band_top_ratio)).x + 4,
        point(0, band_y(rail_config_.band_top_ratio)).y - 4},
      cv::FONT_HERSHEY_SIMPLEX, 0.36, {0, 255, 255}, 1, cv::LINE_AA);
  }

  // ─────────────────────────────────────────────────────────────────────
  // 가드레일 계기: 좌/우 여유와 반발이 얼마나 걸렸는지를 카메라 패널에 겹친다.
  //
  // ★ 여기 게이지는 보기용이다. 실제 조향에 더해지는 값은 main 의
  //   control.py GUARDRAIL_PARAMS 가 계산한다. 이 창의 임계값은 그것과 같게
  //   맞춰 두려고 guardrail_display_margin_px 파라미터로 받는다 — 값이 어긋나면
  //   게이지만 틀리고 주행은 영향을 받지 않는다.
  // ─────────────────────────────────────────────────────────────────────
  void drawGuardrailReadout(cv::Mat & vis) const
  {
    const float threshold = guardrail_display_margin_px_;
    auto engagement = [&](float margin) {
        if (threshold <= 0.f || margin < guardrail_display_min_trust_px_) {return 0.f;}
        if (margin >= threshold) {return 0.f;}
        return (threshold - margin) / threshold;      // control.py 와 같은 선형 램프
      };

    struct Row
    {
      const char * label;
      bool known;
      float margin;
      cv::Scalar color;
    };
    const Row rows[2] = {
      {"L", last_rail_margins_.leftKnown(), last_rail_margins_.left, {255, 80, 0}},
      {"R", last_rail_margins_.rightKnown(), last_rail_margins_.right, {0, 80, 255}},
    };

    const int x0 = 8, bar_w = 150, bar_h = 9;

    // 글자가 밝은 노면 위에 그대로 얹히면 안 읽힌다. 어두운 판을 반투명으로 깐다.
    const cv::Rect plate(4, 4, std::min(vis.cols - 8, x0 + 74 + bar_w + 8), 78);
    if (plate.width > 0 && plate.height > 0 &&
      plate.br().x <= vis.cols && plate.br().y <= vis.rows)
    {
      cv::Mat roi = vis(plate);
      cv::Mat dark(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
      cv::addWeighted(roi, 0.45, dark, 0.55, 0.0, roi);
    }

    // 색 범례: 오버레이 색이 무엇을 뜻하는지 패널 안에서 바로 읽히게 한다.
    int lx = x0;
    const struct {const char * name; cv::Scalar color;} legend[3] = {
      {"center", {0, 255, 0}}, {"left", {255, 80, 0}}, {"right", {0, 80, 255}},
    };
    for (const auto & item : legend) {
      cv::rectangle(vis, {lx, 8}, {lx + 9, 15}, item.color, cv::FILLED);
      cv::putText(
        vis, item.name, {lx + 13, 15}, cv::FONT_HERSHEY_SIMPLEX, 0.36,
        {225, 225, 225}, 1, cv::LINE_AA);
      lx += 13 + 9 +
        cv::getTextSize(item.name, cv::FONT_HERSHEY_SIMPLEX, 0.36, 1, nullptr).width;
    }

    int y = 24;
    for (const Row & row : rows) {
      char text[64];
      if (row.known) {
        std::snprintf(text, sizeof(text), "%s %4.0fpx", row.label, row.margin);
      } else {
        std::snprintf(text, sizeof(text), "%s   --", row.label);
      }
      cv::putText(
        vis, text, {x0, y + bar_h}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
        {235, 235, 235}, 1, cv::LINE_AA);

      const int bx = x0 + 74;
      cv::rectangle(vis, {bx, y}, {bx + bar_w, y + bar_h}, {90, 90, 90}, 1);
      const float ratio = row.known ? engagement(row.margin) : 0.f;
      if (ratio > 0.f) {
        const int filled = static_cast<int>(std::round(bar_w * ratio));
        cv::rectangle(
          vis, {bx + 1, y + 1}, {bx + std::max(1, filled), y + bar_h - 1},
          row.color, cv::FILLED);
      }
      y += bar_h + 8;
    }

    // 두 레일 중 실제로 반발을 만드는 쪽(=가까운 쪽)과 그 세기.
    float best = 0.f;
    const char * side = "none";
    for (const Row & row : rows) {
      if (!row.known) {continue;}
      const float ratio = engagement(row.margin);
      if (ratio > best) {best = ratio; side = row.label;}
    }
    char summary[80];
    std::snprintf(
      summary, sizeof(summary), "GUARDRAIL %s %3.0f%%  (thr %.0fpx)",
      side, best * 100.f, threshold);
    cv::putText(
      vis, summary, {x0, y + bar_h}, cv::FONT_HERSHEY_SIMPLEX, 0.42,
      best > 0.f ? cv::Scalar(0, 235, 235) : cv::Scalar(150, 150, 150),
      1, cv::LINE_AA);
  }

  // ─────────────────────────────────────────────────────────────────────
  // "Vehicle Dynamics" 디버그 창: 차량 중심(스키매틱) 기준 조향/속도 시각화
  //
  // 위에서 내려다본 차량 아이콘의 앞바퀴가 조향 명령만큼 실제로 꺾이는 모습으로
  // 표시하고(뒷바퀴는 고정), 속도는 옆의 숫자+막대 게이지로 표시한다.
  // 정성적 근사 표시이며 물리적으로 보정된 값이 아니다.
  // ─────────────────────────────────────────────────────────────────────
  cv::Mat drawVehicleDynamicsView() const
  {
    const int W = 300, H = 300;
    cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(30, 30, 30));

    const cv::Point center(110, H / 2);
    const int body_w = 70, body_h = 130;
    const int wheel_w = 14, wheel_h = 32;
    const int half_w = body_w / 2, half_h = body_h / 2;

    // ── 1) 차체 (위에서 내려다본 모습, 전방 = 위쪽) ─────────────────
    cv::rectangle(
      canvas,
      cv::Rect(center.x - half_w, center.y - half_h, body_w, body_h),
      cv::Scalar(0, 165, 255), cv::FILLED, cv::LINE_AA);
    cv::rectangle(
      canvas,
      cv::Rect(center.x - half_w, center.y - half_h, body_w, body_h),
      cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    // 전방 표시 삼각형
    std::vector<cv::Point> nose = {
      {center.x, center.y - half_h - 12},
      {center.x - 10, center.y - half_h + 2},
      {center.x + 10, center.y - half_h + 2},
    };
    cv::fillConvexPoly(canvas, nose, cv::Scalar(0, 255, 255), cv::LINE_AA);

    // ── 2) 뒷바퀴 (고정, 차체와 평행) ───────────────────────────────
    fillRotatedRect(
      canvas, {float(center.x - half_w), float(center.y + half_h - wheel_h / 2.f)},
      wheel_w, wheel_h, 0.f, cv::Scalar(40, 40, 40));
    fillRotatedRect(
      canvas, {float(center.x + half_w), float(center.y + half_h - wheel_h / 2.f)},
      wheel_w, wheel_h, 0.f, cv::Scalar(40, 40, 40));

    // ── 3) 앞바퀴 (조향각만큼 실제로 회전) ───────────────────────────
    // steer_cmd를 그대로 도(degree)처럼 취급해 ±45도 범위로 클램프 후 표시 (실측 변환식 없음)
    const float steer_deg_display = std::clamp(last_steer_cmd_, -45.f, 45.f);
    fillRotatedRect(
      canvas, {float(center.x - half_w), float(center.y - half_h + wheel_h / 2.f)},
      wheel_w, wheel_h, steer_deg_display, cv::Scalar(0, 255, 255));
    fillRotatedRect(
      canvas, {float(center.x + half_w), float(center.y - half_h + wheel_h / 2.f)},
      wheel_w, wheel_h, steer_deg_display, cv::Scalar(0, 255, 255));

    {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "Steer: %.1f", last_steer_cmd_);
      putText(canvas, buf, Point(10, H - 34), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    }

    // ── 4) 속도 게이지 (막대 + 수치) ─────────────────────────────────
    const int gauge_x = 210, gauge_y = 40, gauge_w = 24, gauge_h = 200;
    cv::rectangle(
      canvas, cv::Rect(gauge_x, gauge_y, gauge_w, gauge_h),
      cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    float ratio = std::clamp(last_speed_cmd_ / std::max(1.f, speed_gauge_max_), 0.f, 1.f);
    int fill_h = static_cast<int>(std::round(gauge_h * ratio));
    cv::rectangle(
      canvas, cv::Rect(gauge_x, gauge_y + gauge_h - fill_h, gauge_w, fill_h),
      cv::Scalar(0, 255, 0), cv::FILLED);
    {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "Speed: %.1f", last_speed_cmd_);
      putText(
        canvas, buf, Point(gauge_x - 15, gauge_y - 10),
        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
    }

    putText(
      canvas, "Vehicle-centered schematic (approx)", Point(10, H - 10),
      FONT_HERSHEY_SIMPLEX, 0.4, Scalar(150, 150, 150), 1);

    return canvas;
  }

  // ─────────────────────────────────────────────────────────────────────
  // 차선 변경 성공 감지 및 /lane_change_state 발행
  //
  // 차선 변경 모드(mode==5) 중에 2단계 상태 머신으로 성공 여부를 판단한다:
  //   WAIT_SPIKE  : 오프셋 >= TOL_CHANGE_ → WAIT_SETTLE로 전환
  //   WAIT_SETTLE : 연속 STREAK_NEED_ 프레임 동안 오프셋 <= tol_settle → 성공
  //
  // 발행 형식: [변경중여부(0/1), 성공여부(0/1)]
  // ─────────────────────────────────────────────────────────────────────
  void updateLaneChangeState(bool valid, const LineFit & center_fit, float offset)
  {
    const auto feedback = lane_change_tracker_.update(
      valid,
      center_fit.m,
      offset);

    std_msgs::msg::Int32MultiArray st;
    st.data = {feedback.changing, feedback.success};
    lane_change_state_pub_->publish(st);
  }

  // 차선 모드에 대응하는 목표 ref 비율(0~1) 반환
  float getTargetRefForMode(LaneMode m) const
  {
    float r;
    if (m == LaneMode::CENTER) {
      // 중앙 주행은 독립 설정값을 쓴다(기본 0.5 = 화면 정중앙).
      // 예전에는 좌/우 기준의 평균이었는데, 두 값이 0.5 대칭이 아니면
      // (0.63/0.35 → 0.49) 중앙 주행이 조용히 한쪽으로 밀렸다. 그러면서
      // 좌/우 차선 기준을 손댈 때마다 중앙까지 같이 움직여 원인을 찾기 어려웠다.
      r = center_reference_center_;
    } else {
      r = (m == LaneMode::LANE_ONE) ? center_reference_lane_one_ :
        center_reference_lane_two_;
    }
    return std::clamp(r, 0.0f, 1.0f);
  }

  // 스무딩 ON이면 보간 중인 ref 비율, OFF면 목표 비율 즉시 반환
  float getActiveRefRatio() const
  {
    return smooth_enabled_ ? ref_ratio_current_ : getTargetRefForMode(lane_mode_);
  }

  // ref 선형 보간 전환 시작: 현재값 → 새 모드 목표값을 duration 동안 선형 보간
  void startRefTransition(LaneMode new_mode)
  {
    if (!smooth_enabled_) {return;}
    ref_ratio_start_ = ref_ratio_current_;
    ref_ratio_target_ = getTargetRefForMode(new_mode);
    ref_start_time_ = this->now();
    ref_transition_active_ = true;
  }

  // 매 프레임 호출하여 ref_ratio_current_를 선형 보간 갱신
  void updateRefRatio()
  {
    if (!smooth_enabled_ || !ref_transition_active_) {return;}
    const double t = (this->now() - ref_start_time_).seconds();
    if (t >= ref_transition_duration_sec_) {
      ref_ratio_current_ = ref_ratio_target_;
      ref_transition_active_ = false;
      return;
    }
    const double dur = std::max(1e-6, ref_transition_duration_sec_);
    const float a = static_cast<float>(t / dur);         // 0~1 보간 계수
    ref_ratio_current_ = ref_ratio_start_ + (ref_ratio_target_ - ref_ratio_start_) * a;
  }

private:
  // ── ROS 통신 객체 ───────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  // 모니터 창 CAMERA 패널 전용. debug_view_ 일 때만 생성된다.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr offset_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fit_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr path_preview_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr lane_change_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr lane_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr guardrail_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  // ── 클래스맵 해석 / 가드레일 설정 ───────────────────────────────────
  std::vector<int> center_classes_{1};
  int left_rail_class_ = 2;
  int right_rail_class_ = 3;
  lane_detection::RailMarginConfig rail_config_{};
  // 이번 프레임 측정값. 발행과 시각화가 같이 쓴다.
  lane_detection::RailMargins last_rail_margins_{};
  // 시각화 전용 사본 (debug_view_ 일 때만 채운다).
  cv::Mat left_rail_display_;
  cv::Mat right_rail_display_;
  cv::Mat left_rail_bev_;
  cv::Mat right_rail_bev_;
  // 게이지 눈금용. 제어에는 쓰이지 않는다 — control.py 의 GUARDRAIL_PARAMS 가
  // 진짜 값이고, 여기 값이 어긋나도 주행은 바뀌지 않는다.
  double guardrail_display_margin_px_ = 190.0;
  double guardrail_display_min_trust_px_ = 15.0;

  // ── 곡선 경로 미리보기 설정 ─────────────────────────────────────────
  float path_preview_target_y_ratio_ = 0.7F;
  std::size_t path_preview_min_points_ = 7U;
  float path_preview_min_span_ratio_ = 0.45F;
  float path_preview_max_rmse_px_ = 25.0F;

  // ── 실측 현재 차선 판정 상태 (인지 기반, 제어 목표와 무관) ──────────
  int detected_lane_ = -1;     // 확정된 실측 차선 (-1=미확정)
  int pending_lane_ = -1;      // 디바운스 중인 후보 차선
  int pending_streak_ = 0;     // 후보 차선 연속 프레임 수
  static constexpr int LANE_DETECT_STREAK_NEED_ = 5;               // 확정에 필요한 연속 프레임 수
  static constexpr float LANE_CLASSIFY_DEADZONE_RATIO_ = 0.05f;    // 애매한 경계 임계값

  // ── 조향/속도 명령 시각화 (정성적 근사, 실측 휠베이스/조향각 보정값 아님) ──
  float last_steer_cmd_ = 0.f;     // /xycar_motor에서 수신한 마지막 조향 명령
  float last_speed_cmd_ = 0.f;     // /xycar_motor에서 수신한 마지막 속도 명령
  float speed_gauge_max_ = 31.f;   // 속도 게이지 만땅 기준값 (control.py LANE_DRIVE max_speed와 동일)

  // ── 설정 파라미터 ───────────────────────────────────────────────────
  Config config_;
  LaneMode lane_mode_;
  int frame_width_, frame_height_;
  int roi_top_width_, roi_bottom_width_;
  int roi_top_y_, roi_bottom_y_;
  float center_reference_lane_one_;
  float center_reference_lane_two_;
  float center_reference_center_;

  // ── BEV 투시변환 행렬 ───────────────────────────────────────────────
  cv::Mat H_;           // 사다리꼴 → BEV 정변환
  cv::Mat H_inv_;       // BEV → 원본 프레임 역변환
  cv::Size bev_size_;

  // ── 이전 프레임 fallback 정보 ───────────────────────────────────────
  LineFit prev_center_fit_{0.f, 0.f};
  bool has_prev_center_fit_ = false;
  float prev_offset_ = 0.f;
  int ref_x_ = 0;
  int consecutive_fail_count_ = 0;        // 연속 ok=false 횟수 (재획득 판단용)

  // ── 디버그 제어 ─────────────────────────────────────────────────────
  int frame_count_ = 0;
  int debug_stride_ = 1;      // 디버그 출력 주기 (1이면 매 프레임)
  bool debug_view_ = config_.debug_view;
  // true면 통합 모니터 창에 더해 보조 차선 창들을 추가로 띄운다.
  // debug_view_ 가 false면 이 값과 무관하게 모든 창이 꺼진다.
  bool debug_lane_view_ = config_.debug_lane_view;
  bool diagnostics_enabled_ = false;
  bool enable_reacquire_full_bev_fallback_ = false;
  // 통합 모니터 창의 CAMERA 패널에 쓸 최신 원본 프레임.
  // 단일 스레드 spin 이라 콜백 간 경쟁이 없어 별도 락이 필요 없다.
  cv::Mat latest_camera_;
  // 라벨링 ROI 상단 y (하단 40%). segmentation_tools/core.py의
  // label_roi_top()과 같은 공식이다: height - round(height*0.4)
  // (640x360 기준 216). 모니터 CAMERA 패널에서 이 위는 어둡게, 이 아래는
  // 세그멘테이션 마스크를 겹쳐 그린다.
  int label_roi_top_ = frame_height_ - static_cast<int>(std::lround(frame_height_ * 0.4));
  // CAMERA 패널에서 라벨 ROI 밖(위쪽)을 죽이는 밝기 배율(0~1).
  static constexpr double CAMERA_ABOVE_ROI_DIM_FACTOR = 0.55;
  // CAMERA 패널에서 중앙선 세그멘테이션 마스크를 겹쳐 그릴 때의 초록색 알파.
  static constexpr double CENTER_LANE_OVERLAY_ALPHA = 0.45;
  // 바깥 실선(left_solid/right_solid) 오버레이 알파. 중앙선보다 옅게 깔아
  // 조향 목표인 중앙선이 화면에서 먼저 읽히게 한다.
  static constexpr double RAIL_OVERLAY_ALPHA = 0.35;

  // ── 모드/차선 상태 ──────────────────────────────────────────────────
  int current_mode_ = 0;
  int current_lane_ = 0;
  lane_detection::LaneChangeStateTracker lane_change_tracker_;

  // ── ref 비율 스무딩 전환 변수 ───────────────────────────────────────
  bool smooth_enabled_ = config_.change_ref_smoothly;
  float ref_ratio_current_ = 0.f;                     // 현재 보간 중인 ref 비율
  float ref_ratio_start_ = 0.f;                       // 전환 시작 시점 값
  float ref_ratio_target_ = 0.f;                      // 전환 목표 값
  rclcpp::Time ref_start_time_;
  bool ref_transition_active_ = false;
  double ref_transition_duration_sec_ = 0.8;          // 전환 소요 시간 (초)


  // ─────────────────────────────────────────────────────────────────────
  // 이미지 콜백: 전체 처리 파이프라인
  //
  // (0) ROS 메시지 → OpenCV Mat
  // (1) 노란 차선 전처리 → 엣지 이미지
  // (2) BEV 투시변환
  // (3) 수평/수직 노이즈 억제
  // (4) 슬라이딩 윈도우 직선 피팅
  // (5) 오프셋 계산 및 히스토리 갱신
  // (6) 토픽 발행 + 디버그 출력
  // ─────────────────────────────────────────────────────────────────────
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const auto diagnostics_started = diagnostics_enabled_ ?
      std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    std::optional<lane_detection::LanePipelineDiagnostics> diagnostics_data;
    if (diagnostics_enabled_) {diagnostics_data.emplace();}
    auto * diagnostics = diagnostics_data ? &diagnostics_data.value() : nullptr;

    if (smooth_enabled_) {updateRefRatio();}

    // (0) PIDNet-S mono8 클래스맵 → 이진 OpenCV 마스크
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge 클래스맵 오류: %s", e.what());
      return;
    }
    Mat class_map = cv_ptr->image;
    if (class_map.empty()) {
      if (diagnostics) {
        diagnostics->processing_time_us = static_cast<std::uint64_t>(
          std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - diagnostics_started).count());
        diagnostics_pub_->publish(
          lane_detection::makeLanePipelineDiagnostic(msg->header, *diagnostics));
      }
      return;
    }

    // 모델 출력 해상도가 JSON의 frame_width/height와 다르면 맞춘다.
    // 라벨 영상이므로 보간은 반드시 최근접(INTER_NEAREST)이어야 한다.
    if (class_map.size() != Size(frame_width_, frame_height_)) {
      resize(class_map, class_map, Size(frame_width_, frame_height_), 0, 0, INTER_NEAREST);
    }

    // 클래스맵에서 중앙선만 뽑는다. 예전처럼 threshold(0, THRESH_BINARY)를 쓰면
    // road/shortcut 까지 전부 차선이 되어 버린다. extractClassMask 는 pidnet 의
    // np.isin(labels, lane_classes)*255 와 같은 결과를 낸다.
    Mat lane_mask = lane_detection::extractClassMask(class_map, center_classes_);
    if (diagnostics) {
      diagnostics->input_mask_pixels = static_cast<std::size_t>(cv::countNonZero(lane_mask));
    }

    // 가드레일용 바깥 실선. 중앙선과 같은 프레임에서 뽑으므로 시간 정렬 문제가 없다.
    const lane_detection::RailMargins rail_margins = measureRailMargins(class_map);

    // 모니터 CAMERA 패널 오버레이용: 주행 트라페조이드 ROI로 잘리기 전의
    // 원본 세그멘테이션 결과. 라벨 ROI(하단 40%) 전체에서 모델이 실제로
    // 무엇을 예측했는지 그대로 보여주기 위해 트라페조이드 AND 이전에 떠둔다.
    Mat lane_mask_for_display = lane_mask.clone();

    bitwise_and(lane_mask, trapezoidMask(lane_mask.size()), lane_mask);
    if (diagnostics) {
      diagnostics->roi_mask_pixels = static_cast<std::size_t>(cv::countNonZero(lane_mask));
    }

    // 세그멘테이션 경계의 자잘한 구멍/점을 정리한다. 색상 파이프라인의
    // 모폴로지와 목적은 같지만, 마스크가 이미 꽉 차 있어 커널이 더 작다.
    Mat close_kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    Mat open_kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    morphologyEx(lane_mask, lane_mask, MORPH_CLOSE, close_kernel);
    morphologyEx(lane_mask, lane_mask, MORPH_OPEN, open_kernel);
    if (diagnostics) {
      diagnostics->roi_after_morphology_pixels =
        static_cast<std::size_t>(cv::countNonZero(lane_mask));
    }

    // 디버그 오버레이(publishAndDebug의 "Lane View + Offset" 등)는 BGR 캔버스를
    // 요구한다. 원본 카메라 영상은 더 이상 구독하지 않으므로 마스크를 3채널로
    // 올려 캔버스로 쓴다.
    Mat frame;
    cvtColor(lane_mask, frame, COLOR_GRAY2BGR);

    // ROI 사다리꼴 디버그 시각화 (debug_lane_view 설정 시)
    if (debug_view_ && debug_lane_view_) {
      Mat frame_roi_vis = frame.clone();
      drawROIPolygon(frame_roi_vis);
      imshow("ROI Polygon", frame_roi_vis);
    }

    // (1) 전처리 완료된 중앙선 마스크 (보조 창)
    if (debug_view_ && debug_lane_view_) {
      imshow("Mask-PIDNet-Center-Lane", lane_mask);
      // 세 클래스를 한 장에 색으로 구분해 둔다. 중앙선만 보던 예전 창으로는
      // 레일이 안 잡히는 건지 잘못 잡히는 건지 구분할 수 없었다.
      //
      // 반드시 lane_mask 가 아니라 lane_mask_for_display(사다리꼴 자르기 전)를
      // 넘긴다. lane_mask 는 이미 잘리고 모폴로지까지 끝난 상태라, 레일(자르기
      // 전)과 섞으면 중앙선만 짧게 나와 CAMERA 패널과 달라 보인다. 실제로 그
      // 차이 때문에 "카메라엔 중앙선이 둘인데 마스크 창엔 하나"로 보였다.
      imshow("Mask-PIDNet-Lanes", buildClassMaskView(lane_mask_for_display));
    }

    // (2) BEV 투시변환: 사다리꼴 → 직사각형
    Mat bev_yellow;
    warpPerspective(
      lane_mask, bev_yellow, H_, bev_size_,
      INTER_NEAREST, BORDER_CONSTANT, Scalar(0));
    if (diagnostics) {
      diagnostics->bev_pixels = static_cast<std::size_t>(cv::countNonZero(bev_yellow));
    }
    if (debug_view_ && debug_lane_view_) {
      imshow("BEV-PIDNet-Center-Lane", bev_yellow);
      // 가드레일이 실제로 무엇을 재고 있는지: 레일 마스크 + 샘플 밴드 +
      // 차량 중심선 + 임계 여유 + 이번 프레임에 고른 좌/우 레일 위치.
      imshow("BEV-Guardrail-Rails", buildGuardrailBevView());
    }

    // (3) 노이즈 억제 준비: corridor 범위 및 디버그 캔버스 준비
    const int y_start_chk = (int)std::round(bev_size_.height * 0.0f);
    const int y_end_chk = (int)std::round(bev_size_.height * 1.0f);
    int x_min = ref_x_ - static_cast<int>(config_.corridor_width / 2);
    int x_max = ref_x_ + static_cast<int>(config_.corridor_width / 2);

    cv::Mat bev_color;
    cv::cvtColor(bev_yellow, bev_color, cv::COLOR_GRAY2BGR);

    // (3-a) 수평 노이즈 행 억제
    cv::Mat bev_clean;
    bool suppressed = suppressHorizontalNoiseRows(
      bev_yellow, bev_clean,
      config_.horizontal_noise_width,
      config_.horizontal_band_corridor_ratio_thresh,
      y_start_chk, y_end_chk,
      config_.horizontal_noise_band_h,
      config_.horizontal_noise_extra_pad,
      x_min, x_max,
      &bev_color
    );
    if (suppressed) {bev_yellow = bev_clean;}
    if (diagnostics) {
      diagnostics->horizontal_suppressed = suppressed;
      diagnostics->after_horizontal_pixels =
        static_cast<std::size_t>(cv::countNonZero(bev_yellow));
    }

    // (3-b) 수직 열 밴드 노이즈 억제
    // vertical_noise_peak_use_corridor: true면 corridor만, false면 전체 폭 검사
    int cx_min = config_.vertical_noise_peak_use_corridor ? x_min : 0;
    int cx_max = config_.vertical_noise_peak_use_corridor ? x_max : (bev_size_.width - 1);
    cv::Mat bev_clean2;
    bool suppressed2 = suppressColumnBands(
      bev_yellow, bev_clean2,
      cx_min, cx_max,
      y_start_chk, y_end_chk,
      config_.vertical_noise_band_w,
      config_.vertical_noise_min_pixels,
      config_.vertical_noise_peak_ratio,
      config_.vertical_noise_extra_pad_half_width,
      &bev_color
    );
    if (suppressed2) {bev_yellow = bev_clean2;}
    if (diagnostics) {
      diagnostics->column_suppressed = suppressed2;
      diagnostics->after_column_pixels =
        static_cast<std::size_t>(cv::countNonZero(bev_yellow));
    }

    if (debug_view_ && debug_lane_view_) {
      cv::imshow("BEV-Mask (suppressed rows/columns in red)", bev_color);
    }

    // (4) 슬라이딩 윈도우 직선 피팅
    frame_count_++;
    bool show_dbg = debug_view_ && (frame_count_ % debug_stride_ == 0);

    bool valid = false;
    lane_detection::PathPreview path_preview;
    LineFit center_fit = fitLaneFromBEV(
      bev_yellow, valid, diagnostics, nullptr, &path_preview);

    std_msgs::msg::Bool validity_msg;
    validity_msg.data = valid;
    validity_pub_->publish(validity_msg);

    // (5) 오프셋 계산 및 히스토리 갱신
    float offset = 0.f;
    if (valid) {
      offset = calcOffsetFromCenterLine(center_fit, bev_size_.width);
      prev_offset_ = offset;
      prev_center_fit_ = center_fit;
      has_prev_center_fit_ = true;
    } else {
      // 피팅 실패: 이전 오프셋 재사용, 데이터 없으면 0
      offset = has_prev_center_fit_ ? prev_offset_ : 0.f;
    }

    // 가드레일 여유 발행. 중앙선 피팅이 실패해도 레일은 별개로 볼 수 있으므로
    // valid 와 무관하게 매 프레임 낸다 — 실제로 중앙선을 놓치는 순간이야말로
    // 바깥 실선이 유일하게 남은 근거인 경우가 많다.
    std_msgs::msg::Float32MultiArray guardrail_msg;
    guardrail_msg.data = {rail_margins.left, rail_margins.right};
    guardrail_pub_->publish(guardrail_msg);

    // (6) 토픽 발행 + 디버그 출력
    const auto publication_policy = publishAndDebug(
      frame, offset, center_fit, valid, show_dbg, lane_mask_for_display,
      path_preview);

    if (diagnostics) {
      diagnostics->fit_valid = valid;
      diagnostics->frame_mapping_ok = publication_policy.publish_fit;
      diagnostics->processing_time_us = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - diagnostics_started).count());
      diagnostics_pub_->publish(
        lane_detection::makeLanePipelineDiagnostic(msg->header, *diagnostics));
    }

    // A failed fit may retain prior geometry for internal tracking/debug,
    // but invalid evidence must reset lane-change completion progress.
    updateLaneChangeState(valid, center_fit, offset);

    // 실측 현재 차선 판정 및 /lane_position 발행 (제어 목표와 무관한 인지값)
    if (publication_policy.publish_lane_position) {
      updateAndPublishDetectedLane(center_fit, true);
    }
  }

  // ─────────────────────────────────────────────────────────────────────
  // 노란 중앙선의 BEV 가로 위치 비율로 현재 차선을 분류한다.
  //
  // 두 기준값(1차선/2차선 ref)의 정중앙 부근은 노이즈만으로도 쉽게 뒤집히는
  // 애매한 영역이라, 그 구간에서는 -1(판정 보류)을 반환해 디바운스 카운터에
  // 아예 반영되지 않게 한다. 애매한 프레임이 우연히 5번 연속 한쪽에 쏠려도
  // 잘못 확정되지 않도록 하기 위함이다.
  //
  // 반환값: 0=Lane1, 1=Lane2, -1=판정 보류
  // ─────────────────────────────────────────────────────────────────────
  int classifyLaneFromRatio(float x_ratio) const
  {
    // 기본 주행이 중앙이 되면서 중앙도 하나의 상태가 되었다. 세 기준값
    // (1차선 / 중앙 / 2차선) 중 가장 가까운 것을 고른다.
    const float mid = 0.5f * (center_reference_lane_one_ + center_reference_lane_two_);
    // 반환 규약(README): 0=중앙, 1=왼쪽(1차선), 2=오른쪽(2차선)
    const float d[3] = {
      std::fabs(x_ratio - mid),                               // 0 = 중앙
      std::fabs(x_ratio - center_reference_lane_one_),        // 1 = 왼쪽
      std::fabs(x_ratio - center_reference_lane_two_),        // 2 = 오른쪽
    };

    int best = 0, second = 1;
    for (int i = 1; i < 3; ++i) {if (d[i] < d[best]) {best = i;}}
    for (int i = 0; i < 3; ++i) {
      if (i != best && (second == best || d[i] < d[second])) {second = i;}}

    // 1·2등이 비슷하면 경계라 판정을 보류한다. 애매한 프레임이 우연히
    // 연속으로 한쪽에 쏠려 잘못 확정되는 것을 막는다.
    if (std::fabs(d[second] - d[best]) < LANE_CLASSIFY_DEADZONE_RATIO_) {return -1;}
    return best;
  }

  // 디바운스(5프레임 연속) 후 detected_lane_ 갱신 및 /lane_position 발행
  void updateAndPublishDetectedLane(const LineFit & fit_bev, bool fit_ok)
  {
    if (fit_ok && bev_size_.width > 0 && bev_size_.height > 0) {
      const float x_near =
        fit_bev.m * static_cast<float>(bev_size_.height - 1) + fit_bev.b;
      const float ratio = x_near / static_cast<float>(bev_size_.width);
      const int cls = classifyLaneFromRatio(ratio);

      if (cls == -1) {
        // 애매한 프레임: 디바운스 상태를 건드리지 않고 이전 상태 유지
      } else if (cls == pending_lane_) {
        ++pending_streak_;
      } else {
        pending_lane_ = cls;
        pending_streak_ = 1;
      }
      if (pending_streak_ >= LANE_DETECT_STREAK_NEED_) {
        detected_lane_ = pending_lane_;
      }
    }

    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(detected_lane_);
    lane_position_pub_->publish(msg);
  }

  // ─────────────────────────────────────────────────────────────────────
  // 모드/차선 변경 콜백
  //
  // /mode_info [mode, lane] 수신 시 모드와 차선을 갱신하고,
  // 차선 모드가 바뀌면 ref 비율 전환 애니메이션을 시작한다.
  //
  // lane_mode_ 갱신 규칙:
  //   - 변경 모드(5) 진입 시 즉시 갱신
  //   - 비(非)3 → 3 전환 시에도 갱신
  // ─────────────────────────────────────────────────────────────────────
  void modeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 2) {return;}

    const int new_mode = msg->data[0];
    const int new_lane = msg->data[1];
    const int prev_mode = current_mode_;
    const bool valid_lane_target = new_lane >= 0 && new_lane <= 2;

    lane_change_tracker_.handleCommand(new_mode, new_lane);

    current_mode_ = new_mode;
    current_lane_ = new_lane;

    // lane_mode_ 갱신: 모드 5이거나 비3 → 3 전환 시에만 업데이트
    LaneMode old_lane_mode = lane_mode_;
    if (valid_lane_target &&
      (new_mode == 5 || (prev_mode != 3 && new_mode == 3)))
    {
      lane_mode_ = (new_lane == 1) ? LaneMode::LANE_ONE :
        (new_lane == 2) ? LaneMode::LANE_TWO :
        LaneMode::CENTER;
    }

    // 차선 모드가 실제로 변경되었으면 ref 전환 시작
    if (smooth_enabled_ && lane_mode_ != old_lane_mode) {
      startRefTransition(lane_mode_);
    }

  }

  // ─────────────────────────────────────────────────────────────────────
  // 원본 카메라 콜백: 모니터 창 CAMERA 패널에만 쓴다.
  //
  // 차선 판단은 /pidnet_class_map 으로만 한다. 이 프레임은 어떤 인지
  // 경로에도 들어가지 않는다.
  // ─────────────────────────────────────────────────────────────────────
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      latest_camera_ =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_WARN(get_logger(), "cv_bridge 카메라 패널 오류: %s", e.what());
    }
  }

  // ─────────────────────────────────────────────────────────────────────
  // 모터 명령 콜백: /xycar_motor [조향각, 속도] 수신
  //
  // 모니터 창 VEHICLE 패널 시각화에만 사용한다 (제어에 개입하지 않음).
  // ─────────────────────────────────────────────────────────────────────
  void motorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) {return;}
    last_steer_cmd_ = msg->data[0];
    if (msg->data.size() >= 2) {last_speed_cmd_ = msg->data[1];}

    // 값만 캐시하고 그리지는 않는다. /xycar_motor 는 제어 주기(50 Hz)로
    // 들어오는데 그때마다 imshow + waitKey 를 부르면 같은 스레드에서 도는
    // 차선 인지가 밀린다(실측 bag: 카메라 18.8 Hz 입력에 /lane_offset 5.9 Hz
    // 출력). 통합 모니터 창은 카메라 프레임당 한 번만 그리고, 그때 이
    // 캐시된 최신 값을 읽는다.
  }
};


// ─────────────────────────────────────────────────────────────────────────────
// 파라미터 JSON 경로 결정
//
// 1) `--config <경로>` 명령행 인수가 있으면 그 경로를 사용
// 2) 없으면 설치된 package share의 JSON을 사용한다.
// ─────────────────────────────────────────────────────────────────────────────
static std::string resolve_config_path(int argc, char ** argv)
{
  std::vector<std::string> config_path_candidates;
  try {
    config_path_candidates.push_back(
      ament_index_cpp::get_package_share_directory("lane_detection") +
      "/lane_detection_parameter.json");
  } catch (const std::exception &) {
    // An explicit --config remains available in source-only execution.
  }

  auto readable = [](const std::string & p) {
      if (p.empty()) {return false;}
      std::ifstream f(p);
      return f.is_open();
    };

  for (int i = 1; i + 1 < argc; ++i) {
    if (std::string(argv[i]) == "--config") {
      const std::string p = argv[i + 1];
      if (readable(p)) {return p;}
      throw std::runtime_error("--config로 지정된 파일을 열 수 없습니다: " + p);
    }
  }

  for (const auto & candidate : config_path_candidates) {
    if (readable(candidate)) {return candidate;}}

  std::string msg = "lane_detection_parameter.json을 찾지 못했습니다. 확인한 경로:";
  for (const auto & candidate : config_path_candidates) {
    msg += "\n  - " + candidate;
  }
  msg += "\n--config <경로> 로 직접 지정할 수 있습니다.";
  throw std::runtime_error(msg);
}


int main(int argc, char ** argv)
{
  std::cout << "OpenCV 버전: " << CV_VERSION << std::endl;
  rclcpp::init(argc, argv);

  Config config;
  try {
    const std::string config_path = resolve_config_path(argc, argv);
    std::cout << "[lane_detection] 파라미터 파일: " << config_path << std::endl;
    config = load_config(config_path);
  } catch (const std::exception & e) {
    std::cerr << "[lane_detection] 설정 로드 실패: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  // 로드된 설정값 확인용 로그 (config 수정이 실제로 반영됐는지 디버깅용)
  std::cout << "===== [lane_detection] 로드된 Config 값 =====" << std::endl;
  std::cout << "[Config] debug_view = " << config.debug_view << std::endl;
  std::cout << "[Config] debug_lane_view = " << config.debug_lane_view << std::endl;
  std::cout << "[Config] roi_top_width_coefficient = " << config.roi_top_width_coefficient <<
    std::endl;
  std::cout << "[Config] roi_bottom_width_coefficient = " << config.roi_bottom_width_coefficient <<
    std::endl;
  std::cout << "[Config] sliding_window_num_windows = " << config.sliding_window_num_windows <<
    std::endl;
  std::cout << "[Config] sliding_window_margin = " << config.sliding_window_margin << std::endl;
  std::cout << "[Config] sliding_window_minpix = " << config.sliding_window_minpix << std::endl;
  std::cout << "[Config] corridor_width = " << config.corridor_width << std::endl;
  std::cout << "[Config] fit_outlier_reject_px = " << config.fit_outlier_reject_px << std::endl;
  std::cout << "[Config] fit_outlier_iterations = " << config.fit_outlier_iterations << std::endl;
  std::cout << "[Config] fit_reacquire_after_frames = " << config.fit_reacquire_after_frames <<
    std::endl;
  std::cout << "==============================================" << std::endl;

  auto node = std::make_shared<LaneDetector>(config);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
