// ─────────────────────────────────────────────────────────────────────────────
// lane_detection.cpp
//
// 역할: BEV 기반 노란 차선 검출 노드
//   1) 카메라 영상에서 노란색 픽셀을 추출하고 Canny 엣지를 계산한다.
//   2) 사다리꼴 ROI를 BEV(Bird's Eye View)로 투시변환한다.
//   3) 수평/수직 노이즈를 억제한 뒤 슬라이딩 윈도우로 중앙선을 추적한다.
//   4) 최소자승 직선 피팅 결과를 오프셋(/lane_offset)과 피팅 파라미터
//      (/lane_fit)로 발행한다.
//
// 구독: /resized_image (sensor_msgs/Image, BGR8)
//       /mode_info     (std_msgs/Int32MultiArray, [mode, lane])
// 발행: /lane_offset       (std_msgs/Int16, 픽셀 단위 오프셋)
//       /lane_fit           (std_msgs/Float32MultiArray, [m, b])
//       /lane_change_state  (std_msgs/Int32MultiArray, [변경중, 성공여부])
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>

#include <cstdio>
#include <numeric>
#include <string>
#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <chrono>
#include "lane_change_state_tracker.hpp"
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


class LaneDetector : public rclcpp::Node {
public:
    // ─────────────────────────────────────────────────────────────────────
    // 생성자
    //
    // Config 파라미터를 멤버 변수로 초기화하고,
    // ROS 토픽 구독/발행 및 BEV 호모그래피 행렬을 설정한다.
    // ─────────────────────────────────────────────────────────────────────
    LaneDetector(const Config& config)
    : Node("lane_detector_node"), config_(config),
      lane_mode_(config_.lane_mode),
      frame_width_(config_.frame_width),
      frame_height_(config_.frame_height),
      roi_top_width_   (static_cast<int>(frame_width_  * config_.roi_top_width_coefficient)),
      roi_bottom_width_(static_cast<int>(frame_width_  * config_.roi_bottom_width_coefficient)),
      roi_top_y_       (static_cast<int>(frame_height_ * config_.roi_top_y_coefficient)),
      roi_bottom_y_    (static_cast<int>(frame_height_ * config_.roi_bottom_y_coefficient)),
      center_reference_lane_one_(config_.center_reference_lane_one),
      center_reference_lane_two_(config_.center_reference_lane_two),
      lane_change_tracker_(config_.lane_change_tol_straight,
                           config_.lane_change_tol_curve,
                           config_.lane_change_tol_change,
                           config_.lane_change_streak_need,
                           config_.lane_change_m_split)
    {
        // QoS 프로파일
        // - qos_sensor: 영상/센서처럼 최신성 우선, 유실 허용 (Best Effort)
        // - qos_fast:   수치 토픽용, Best Effort + Volatile (latch 없음)
        auto qos_sensor = rclcpp::SensorDataQoS().best_effort();
        auto qos_fast   = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        // 카메라 영상 구독: /resized_image (BGR8)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", qos_sensor,
            std::bind(&LaneDetector::imageCallback, this, std::placeholders::_1)
        );

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

        // 차선 변경 상태 발행: /lane_change_state ([변경중여부, 성공여부])
        lane_change_state_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/lane_change_state", qos_fast);

        // BEV 호모그래피 행렬 계산 (ROI 사다리꼴 → 직사각형)
        buildHomography();

        // ref 비율 스무딩 전환 초기화
        smooth_enabled_ = config_.change_ref_smoothly;
        if (config_.lane_ref_transition_duration_sec > 0.0)
            ref_transition_duration_sec_ = config_.lane_ref_transition_duration_sec;

        if (smooth_enabled_) {
            ref_ratio_current_     = getTargetRefForMode(lane_mode_);
            ref_ratio_start_       = ref_ratio_current_;
            ref_ratio_target_      = ref_ratio_current_;
            ref_start_time_        = this->now();
            ref_transition_active_ = false;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // (1) 노란 차선 전처리
    //
    // 입력 프레임에서 노란색 픽셀을 HLS ∩ HSV ∩ YCrCb 조건으로 추출하고,
    // 모폴로지 연산(닫힘 → 열림) 후 Canny 엣지를 검출하여 반환한다.
    // ROI 사다리꼴 마스크를 먼저 적용해 관심 영역 외부를 제외한다.
    // ─────────────────────────────────────────────────────────────────────
    Mat preprocessYellow(const Mat& frame) {
        // ROI 마스크 적용
        Mat roi_mask = trapezoidMask(frame.size());
        Mat roi_frame;
        frame.copyTo(roi_frame, roi_mask);

        // 색공간 변환: HLS, HSV, YCrCb
        Mat hls, hsv, ycrcb;
        cvtColor(roi_frame, hls,   COLOR_BGR2HLS);
        cvtColor(roi_frame, hsv,   COLOR_BGR2HSV);
        cvtColor(roi_frame, ycrcb, COLOR_BGR2YCrCb);  // 채널 순서: [Y, Cr, Cb]

        // 각 색공간에서 노란색 범위 마스크 생성
        Mat y_hls, y_hsv, y_ycc, y_mask;
        inRange(hls,
                Scalar(config_.yellow_hls_min_h, config_.yellow_hls_min_l, config_.yellow_hls_min_s),
                Scalar(config_.yellow_hls_max_h, config_.yellow_hls_max_l, config_.yellow_hls_max_s),
                y_hls);

        inRange(hsv,
                Scalar(config_.yellow_hsv_min_h, config_.yellow_hsv_min_s, config_.yellow_hsv_min_v),
                Scalar(config_.yellow_hsv_max_h, config_.yellow_hsv_max_s, config_.yellow_hsv_max_v),
                y_hsv);

        // YCrCb: OpenCV 채널 순서가 [Y, Cr, Cb]임에 주의
        inRange(ycrcb,
                Scalar(config_.yellow_ycrcb_min_y,  config_.yellow_ycrcb_min_cr, config_.yellow_ycrcb_min_cb),
                Scalar(config_.yellow_ycrcb_max_y,  config_.yellow_ycrcb_max_cr, config_.yellow_ycrcb_max_cb),
                y_ycc);

        // YCrCb 채널 합산 디버그 시각화 (debug_view 설정 시)
        if (config_.debug_view) {
            std::vector<cv::Mat> ch;
            split(ycrcb, ch);
            imshow("YCrCb channels", (ch[0] + ch[1] + ch[2]) / 3);
        }

        // 세 색공간 마스크의 교집합 계산
        bitwise_and(y_hls, y_hsv, y_mask);
        bitwise_and(y_mask, y_ycc, y_mask);

        // 매우 어두운 픽셀(Y ≤ 5) 억제: 검정 영역이 재유입되는 현상 방지
        {
            std::vector<cv::Mat> ch;
            split(ycrcb, ch);
            cv::Mat y_gate = ch[0] > 5;
            y_gate.convertTo(y_gate, CV_8U, 255);
            bitwise_and(y_mask, y_gate, y_mask);
        }

        // 모폴로지: 닫힘(구멍 메우기) → 열림(잔여 노이즈 제거)
        Mat k_close = getStructuringElement(MORPH_RECT,
                        Size(config_.kernel_yellow_closing_size, config_.kernel_yellow_closing_size));
        Mat k_open  = getStructuringElement(MORPH_RECT,
                        Size(config_.kernel_yellow_opening_size, config_.kernel_yellow_opening_size));
        morphologyEx(y_mask, y_mask, MORPH_CLOSE, k_close);
        morphologyEx(y_mask, y_mask, MORPH_OPEN,  k_open);

        // 가우시안 블러 + Canny 엣지 검출 (노란색 마스크 범위 내에서만)
        Mat gray, blurred, masked_gray, edges;
        cvtColor(roi_frame, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, blurred,
                     Size(config_.gaussian_blur_kernel_size, config_.gaussian_blur_kernel_size), 0);

        // 마스크를 3×3 팽창하여 엣지 검출 범위를 조금 확장
        Mat y_mask_dil;
        Mat k3 = getStructuringElement(MORPH_RECT, Size(3, 3));
        dilate(y_mask, y_mask_dil, k3);

        blurred.copyTo(masked_gray, y_mask_dil);
        Canny(masked_gray, edges,
              config_.canny_yellow_low_threshold, config_.canny_yellow_high_threshold);

        // 최종 마스크로 엣지 이미지를 다시 제한
        bitwise_and(edges, edges, edges, y_mask);
        return edges;
    }


    // ─────────────────────────────────────────────────────────────────────
    // (2) BEV 호모그래피 행렬 계산
    //
    // ROI 사다리꼴(원본 좌표) → 직사각형(BEV 좌표) 투시변환 행렬 H_ 와
    // 역변환 행렬 H_inv_ 를 계산하여 멤버에 저장한다.
    // ─────────────────────────────────────────────────────────────────────
    void buildHomography() {
        int cx = frame_width_ / 2;

        // 원본 사다리꼴 꼭짓점 순서: 좌상, 우상, 우하, 좌하
        Point2f src[4] = {
            Point2f(cx - roi_top_width_/2,    static_cast<float>(roi_top_y_)),
            Point2f(cx + roi_top_width_/2,    static_cast<float>(roi_top_y_)),
            Point2f(cx + roi_bottom_width_/2, static_cast<float>(roi_bottom_y_)),
            Point2f(cx - roi_bottom_width_/2, static_cast<float>(roi_bottom_y_))
        };

        // BEV 결과 크기 결정
        int bev_w = roi_bottom_width_;
        if (bev_w <= 0) bev_w = std::max(roi_top_width_, frame_width_);
        int bev_h = std::max(1, roi_bottom_y_ - roi_top_y_);
        bev_size_ = Size(bev_w, bev_h);

        // BEV 직사각형 꼭짓점 순서: 좌상, 우상, 우하, 좌하
        Point2f dst[4] = {
            Point2f(0.f,         0.f),
            Point2f(bev_w - 1.f, 0.f),
            Point2f(bev_w - 1.f, bev_h - 1.f),
            Point2f(0.f,         bev_h - 1.f)
        };

        H_     = getPerspectiveTransform(src, dst);
        H_inv_ = H_.inv();  // 역변환: BEV → 원본 프레임
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
    bool suppressHorizontalNoiseRows(const cv::Mat& bev_in,
                                     cv::Mat& bev_out,
                                     int horizontal_noise_width,
                                     float corridor_ratio_thresh,
                                     int y_start,
                                     int y_end,
                                     int band_h,
                                     int extra_pad_rows,
                                     int x_min,
                                     int x_max,
                                     cv::Mat* vis = nullptr)
    {
        CV_Assert(bev_in.type() == CV_8UC1);
        const int H = bev_in.rows, W = bev_in.cols;

        bev_out = bev_in.clone();

        // 파라미터 경계 보정
        y_start        = std::max(0, y_start);
        y_end          = (y_end <= 0 || y_end > H) ? H : y_end;
        band_h         = std::max(1, band_h);
        extra_pad_rows = std::max(0, extra_pad_rows);
        x_min          = std::max(0, x_min);
        x_max          = std::min(W - 1, x_max);
        if (y_start >= y_end || x_min > x_max) return false;

        const int corridor_w = x_max - x_min + 1;
        const int corridor_h = y_end  - y_start;

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
                    if (bev_in.at<uchar>(y + dy, x) > 0) { on = true; break; }
                }
                if (on) { run++; best = std::max(best, run); }
                else    { run = 0; }
            }

            // 비율 조건: 밴드 내 흰 픽셀 수 / corridor 전체 흰 픽셀 수 >= 임계값
            bool ratio_noise = false;
            if (corridor_total_whites > 0) {
                cv::Mat band_roi    = bev_in(cv::Rect(x_min, y, corridor_w, band_h));
                int     band_whites = cv::countNonZero(band_roi);
                float   ratio       = static_cast<float>(band_whites)
                                    / static_cast<float>(corridor_total_whites);
                ratio_noise = (ratio >= corridor_ratio_thresh);
            }

            // 두 조건 모두 충족하면 해당 행 범위를 0으로 삭제
            if (best >= horizontal_noise_width && ratio_noise) {
                const int y0 = std::max(y_start, y - extra_pad_rows);
                const int y1 = std::min(H, y + band_h + extra_pad_rows);

                for (int yy = y0; yy < y1; ++yy) {
                    uchar* row = bev_out.ptr<uchar>(yy);
                    std::memset(row + x_min, 0, (x_max - x_min + 1));
                }

                if (vis && !vis->empty()) {
                    cv::rectangle(*vis,
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
    bool suppressColumnBands(const cv::Mat& bev_in,
                             cv::Mat& bev_out,
                             int x_min, int x_max,
                             int y_start, int y_end,
                             int band_w,
                             int min_pixels,
                             float peak_ratio,
                             int half_w,
                             cv::Mat* vis = nullptr)
    {
        CV_Assert(bev_in.type() == CV_8UC1);
        const int H = bev_in.rows, W = bev_in.cols;

        x_min   = std::max(0, x_min);
        x_max   = std::min(W - 1, x_max);
        y_start = std::max(0, y_start);
        y_end   = (y_end <= 0 || y_end > H) ? H : y_end;
        band_w  = std::max(1, band_w);
        half_w  = std::max(0, half_w);

        bev_out = bev_in.clone();
        if (x_min > x_max || y_start >= y_end) return false;

        // ROI 영역의 열(세로) 픽셀 합 계산
        cv::Mat roi = bev_in(cv::Rect(x_min, y_start, x_max - x_min + 1, y_end - y_start));
        cv::Mat nz  = (roi > 0);
        cv::Mat colSum32S;
        cv::reduce(nz, colSum32S, 0, cv::REDUCE_SUM, CV_32S);

        // 전체 열 중 최대 픽셀 수
        int maxVal = 0;
        for (int i = 0; i < colSum32S.cols; ++i)
            maxVal = std::max(maxVal, colSum32S.at<int>(0, i) / 255);

        bool any = false;

        // band_w 폭 단위로 검사
        for (int i = 0; i < colSum32S.cols; i += band_w) {
            int x_band_start = x_min + i;
            int x_band_end   = std::min(x_max, x_band_start + band_w - 1);

            int bandPix = 0;
            for (int j = i; j < i + band_w && j < colSum32S.cols; ++j)
                bandPix += colSum32S.at<int>(0, j) / 255;

            bool cond_abs = (bandPix >= min_pixels);
            bool cond_rel = (peak_ratio > 0.f) ? (bandPix >= maxVal * peak_ratio) : true;

            if (cond_abs && cond_rel) {
                // 삭제 구간: 밴드 + 좌우 half_w 여유
                int xl = std::max(0,    x_band_start - half_w);
                int xr = std::min(W-1, x_band_end   + half_w);

                for (int yy = y_start; yy < y_end; ++yy) {
                    uchar* row = bev_out.ptr<uchar>(yy);
                    std::memset(row + xl, 0, (xr - xl + 1));
                }

                if (vis && !vis->empty()) {
                    cv::rectangle(*vis,
                                  cv::Rect(xl, y_start, xr - xl + 1, y_end - y_start),
                                  cv::Scalar(0, 0, 255), cv::FILLED);
                }

                any = true;
            }
        }

        return any;
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
    LineFit fitLaneFromBEV(const Mat& bev_binary, bool& ok, cv::Mat* dbg_out = nullptr) {
        ok = false;

        const int h = bev_binary.rows, w = bev_binary.cols;

        // ── 1) corridor 범위 계산 ───────────────────────────────────────
        // 현재 차선 모드의 ref_ratio로 기준 x(ref_x_)를 결정하고
        // corridor_width만큼 좌우로 탐색 범위를 제한한다.
        float ref_ratio = std::clamp(getActiveRefRatio(), 0.0f, 1.0f);
        ref_x_ = static_cast<int>(std::round(ref_ratio * w));

        int x_min = std::max(0,   ref_x_ - static_cast<int>(config_.corridor_width / 2));
        int x_max = std::min(w-1, ref_x_ + static_cast<int>(config_.corridor_width / 2));
        int histW = x_max - x_min + 1;
        if (histW <= 2) { x_min = 0; x_max = w - 1; histW = w; }

        // ── 2) 가중 히스토그램으로 시작점(base_x) 결정 ─────────────────
        // 하단 70% 영역에서 corridor 범위의 열별 픽셀 합 계산 후,
        // ref_x_ 근처에 가우시안 가중치를 곱하여 최댓값 열을 시작점으로 선택
        int y_start_hist = std::min(std::max(static_cast<int>(h * 0.3), 0), h - 1);
        cv::Mat hist_roi = bev_binary(cv::Rect(x_min, y_start_hist, histW, h - y_start_hist));
        cv::Mat nz       = (hist_roi > 0);
        cv::Mat colSum;
        cv::reduce(nz, colSum, 0, cv::REDUCE_SUM, CV_32S);

        std::vector<int> hist(histW);
        for (int i = 0; i < histW; ++i) hist[i] = colSum.at<int>(0, i) / 255;

        // sigma_ratio가 작을수록 ref 근처만 강하게 밀어줌(보수적 시작점)
        // w_min을 0.3~0.7 사이로 조절하면 멀리 있어도 완전히 무시되지 않음
        const float sigma_ratio  = (config_.ref_hist_sigma_ratio > 0.f) ?
                                    config_.ref_hist_sigma_ratio : 0.35f;
        const float w_min        = (config_.ref_hist_min_weight > 0.f &&
                                    config_.ref_hist_min_weight < 1.f) ?
                                    config_.ref_hist_min_weight : 0.50f;
        const float sigma_pixels = std::max(1.f, sigma_ratio * static_cast<float>(histW));
        const float two_sigma2   = 2.f * sigma_pixels * sigma_pixels;

        std::vector<float> weighted_hist(histW);
        for (int i = 0; i < histW; ++i) {
            int   x = x_min + i;
            float d = static_cast<float>(std::abs(x - ref_x_));
            float w = w_min + (1.f - w_min) * std::exp(-(d*d) / two_sigma2);
            weighted_hist[i] = static_cast<float>(hist[i]) * w;
        }

        int base_x   = ref_x_;
        auto it_w    = std::max_element(weighted_hist.begin(), weighted_hist.end());
        const float bestValW = (it_w != weighted_hist.end()) ? *it_w : 0.f;

        if (bestValW > 0.f) {
            int base_x_rel = static_cast<int>(std::distance(weighted_hist.begin(), it_w));
            base_x = x_min + base_x_rel;
        } else {
            // 신호 없음: 이전 프레임 피팅 하단값 또는 ref_x_ 사용
            if (has_prev_center_fit_) {
                int x_prev_bottom = static_cast<int>(prev_center_fit_.m * (h - 1) + prev_center_fit_.b);
                base_x = std::clamp(x_prev_bottom, x_min, x_max);
            } else {
                base_x = std::clamp(ref_x_, x_min, x_max);
            }
        }

        // ── 3) 디버그 캔버스 준비 ───────────────────────────────────────
        cv::Mat dbg;
        if (dbg_out) {
            cv::cvtColor(bev_binary, dbg, cv::COLOR_GRAY2BGR);
            // ref_x_: 초록 수직선
            cv::line(dbg, {ref_x_, 0}, {ref_x_, h-1}, {0, 255, 0}, 1);
            // corridor 좌우 경계: 파란 점선
            for (int y = 0; y < h; y += 6) {
                dbg.at<cv::Vec3b>(y, std::clamp(x_min, 0, w-1)) = {255, 0, 0};
                dbg.at<cv::Vec3b>(y, std::clamp(x_max, 0, w-1)) = {255, 0, 0};
            }
            // base_x: 보라색 마커
            cv::line(dbg, {base_x, h-1}, {base_x, h-21}, {255, 0, 255}, 2);
        }

        // ── 4) 슬라이딩 윈도우로 포인트 수집 ──────────────────────────
        // 아래에서 위로 윈도우를 올리며 non-zero 픽셀을 수집한다.
        // minpix 이상 발견 시 다음 윈도우 중심을 평균 x로 재조정(recenter)한다.
        const int    num_windows = std::max(2, config_.sliding_window_num_windows);
        const int    margin      = std::max(5, config_.sliding_window_margin);
        const size_t minpix      = std::max<size_t>(5, config_.sliding_window_minpix);
        const int    win_h       = h / num_windows;

        vector<Point> pts;
        int x_current = base_x;

        for (int i = 0; i < num_windows; ++i) {
            int y_low  = std::max(0, h - (i+1)*win_h);
            int y_high = std::min(h, h -  i   *win_h);
            int xl     = std::max(0, x_current - margin);
            int xr     = std::min(w, x_current + margin);

            vector<int> xs;  // 이번 윈도우에서 발견된 픽셀들의 x 좌표
            for (int yy = y_low; yy < y_high; ++yy) {
                const uchar* row = bev_binary.ptr<uchar>(yy);
                for (int xx = xl; xx < xr; ++xx) {
                    if (row[xx] > 0) { pts.emplace_back(xx, yy); xs.push_back(xx); }
                }
            }

            bool recentered = false;
            if (xs.size() >= minpix) {
                // 픽셀이 충분하면 다음 윈도우 중심을 평균 x로 재조정
                int sum = std::accumulate(xs.begin(), xs.end(), 0);
                x_current  = sum / static_cast<int>(xs.size());
                recentered = true;
            }
            x_current = std::min(std::max(x_current, x_min), x_max);

            if (dbg_out) {
                cv::Scalar boxColor = recentered ? cv::Scalar(0,255,255) : cv::Scalar(0,165,255);
                cv::rectangle(dbg, {xl, y_low}, {xr, y_high}, boxColor, 2);
                cv::drawMarker(dbg, {x_current, (y_low+y_high)/2},
                               {255,255,255}, cv::MARKER_CROSS, 10, 1);
            }
        }

        // ── 5) 포인트 부족 시 이전 프레임 값으로 fallback ──────────────
        if (pts.size() < 10) {
            ok = false;
            if (dbg_out && has_prev_center_fit_) {
                float m = prev_center_fit_.m, b = prev_center_fit_.b;
                cv::line(dbg,
                         {std::clamp((int)(m*0    + b), 0, w-1), 0},
                         {std::clamp((int)(m*(h-1)+ b), 0, w-1), h-1},
                         {200, 200, 200}, 1);
            }
            if (dbg_out) *dbg_out = std::move(dbg);
            return has_prev_center_fit_ ? prev_center_fit_ : LineFit{0.f, 0.f};
        }

        // ── 6) 최소자승 직선 피팅: x = m*y + b (SVD 방식) ─────────────
        // 설계행렬 X(Nx2): 각 행 = [y_i, 1]
        // 타겟벡터 Y(Nx1): 각 행 = x_i
        // 해 θ = [m, b] 를 SVD로 수치 안정적으로 계산
        Mat X(pts.size(), 2, CV_32F), Y(pts.size(), 1, CV_32F);
        for (size_t i = 0; i < pts.size(); ++i) {
            float yf = static_cast<float>(pts[i].y);
            X.at<float>(i, 0) = yf;
            X.at<float>(i, 1) = 1.f;
            Y.at<float>(i, 0) = static_cast<float>(pts[i].x);
        }
        Mat coeff;
        solve(X, Y, coeff, DECOMP_SVD);

        LineFit fit{ coeff.at<float>(0,0), coeff.at<float>(1,0) };
        ok = true;

        // ── 7) 최종 선 디버그 표시 ─────────────────────────────────────
        if (dbg_out) {
            float m = fit.m, b = fit.b;
            cv::line(dbg,
                     {std::clamp((int)(m*0    + b), 0, w-1), 0},
                     {std::clamp((int)(m*(h-1)+ b), 0, w-1), h-1},
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
    //   + : 중앙선이 기준선보다 오른쪽 (차량이 우측 편향 → 좌조향 필요)
    //   - : 중앙선이 기준선보다 왼쪽
    // ─────────────────────────────────────────────────────────────────────
    float calcOffsetFromCenterLine(const LineFit& lf, int bev_width) const {
        // 근거리 우선 샘플링: BEV 높이의 30% 및 80% 지점
        float y1 = bev_size_.height * 0.3f;
        float y2 = bev_size_.height * 0.8f;

        float x1     = lf.m * y1 + lf.b;
        float x2     = lf.m * y2 + lf.b;
        float x_mean = 0.5f * (x1 + x2);

        float ref_ratio = std::clamp(getActiveRefRatio(), 0.0f, 1.0f);
        float x_ref     = ref_ratio * static_cast<float>(bev_width);

        return (x_mean - x_ref);
    }

    // ─────────────────────────────────────────────────────────────────────
    // ROI 사다리꼴 마스크 생성
    // roi_*_ 좌표로 정의된 사다리꼴 내부를 흰색(255)으로 채운 마스크를 반환한다.
    // ─────────────────────────────────────────────────────────────────────
    Mat trapezoidMask(Size sz) const {
        Mat mask(sz, CV_8UC1, Scalar(0));
        int cx = frame_width_ / 2;
        Point pts[1][4] = {{
            Point(cx - roi_top_width_/2,    roi_top_y_),    // 좌상
            Point(cx + roi_top_width_/2,    roi_top_y_),    // 우상
            Point(cx + roi_bottom_width_/2, roi_bottom_y_), // 우하
            Point(cx - roi_bottom_width_/2, roi_bottom_y_)  // 좌하
        }};
        const Point* ppt[1] = { pts[0] };
        int npt[] = {4};
        fillPoly(mask, ppt, npt, 1, Scalar(255));
        return mask;
    }

    // ─────────────────────────────────────────────────────────────────────
    // ROI 사다리꼴 디버그 시각화
    // 원본 프레임 위에 ROI 사다리꼴 외곽선(빨강) + 반투명 오버레이를 그린다.
    // ─────────────────────────────────────────────────────────────────────
    void drawROIPolygon(Mat& frame) const {
        int cx = frame_width_ / 2;
        vector<Point> pts = {
            Point(cx - roi_top_width_/2,    roi_top_y_),
            Point(cx + roi_top_width_/2,    roi_top_y_),
            Point(cx + roi_bottom_width_/2, roi_bottom_y_),
            Point(cx - roi_bottom_width_/2, roi_bottom_y_)
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
    Mat drawOffsetSlider(const Mat& bgr, float offset_px, LaneMode mode) const {
        int sw = frame_width_, sh = 50;
        Mat slider(sh, sw, CV_8UC3, Scalar(50, 50, 50));
        int cx = sw / 2;
        line(slider, Point(cx, 0), Point(cx, sh-1), Scalar(150, 150, 150), 1);

        // 오프셋 크기에 비례한 점 위치 표시
        int dot_x = cx + static_cast<int>(std::round(offset_px));
        dot_x = std::clamp(dot_x, 0, sw - 1);
        circle(slider, Point(dot_x, sh/2), 6, Scalar(0, 0, 255), FILLED);

        string mode_str   = (mode == LaneMode::LANE_ONE) ? "Mode: 1-Lane" : "Mode: 2-Lane";
        string offset_str = "Offset(px): " + std::to_string(static_cast<int>(std::round(offset_px)));
        putText(slider, mode_str,   Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(220,220,220), 1);
        putText(slider, offset_str, Point(10, 42), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(220,220,220), 1);

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
    void publishAndDebug(const cv::Mat& frame_in,
                         float offset,
                         const LineFit& fit_bev,
                         bool show_dbg,
                         const cv::Mat* dbg_from_fit = nullptr)
    {
        cv::Mat frame = frame_in.clone();

        // BEV 직선을 원본 프레임 좌표계의 두 점으로 역투영
        cv::Point2f P0, P1;
        bool mapped = bevLineToFrame(fit_bev, P0, P1);

        // 역투영된 두 점으로 프레임 좌표계 x = m*y + b 복원
        LineFit fit_frame{0.f, 0.f};
        if (mapped) {
            float dy = P1.y - P0.y;
            if (std::abs(dy) < 1e-6f)
                dy = (dy >= 0 ? 1e-6f : -1e-6f);
            fit_frame.m = (P1.x - P0.x) / dy;
            fit_frame.b = P0.x - fit_frame.m * P0.y;
        }

        // /lane_offset 발행
        std_msgs::msg::Int16 offset_msg;
        offset_msg.data = static_cast<int16_t>(std::round(offset));
        offset_pub_->publish(offset_msg);

        // /lane_fit 발행 (프레임 좌표계 [m, b])
        std_msgs::msg::Float32MultiArray fit_msg;
        fit_msg.data = { fit_frame.m, fit_frame.b };
        fit_pub_->publish(fit_msg);

        // 슬라이딩 윈도우 BEV 디버그 창 표시
        if (debug_view_ && show_dbg && dbg_from_fit && !dbg_from_fit->empty())
            cv::imshow("SlidingWindows", *dbg_from_fit);

        // 오프셋 슬라이더 창 표시
        if (debug_view_ && show_dbg) {
            cv::Mat vis = drawOffsetSlider(frame, offset, lane_mode_);
            cv::imshow("Lane View + Offset", vis);
            cv::waitKey(1);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // BEV 직선 → 원본 프레임 좌표 역투영
    //
    // BEV 세로 양 끝(y=0, y=bev_h-1)에서 x를 계산한 뒤
    // H_inv_(BEV→프레임 역호모그래피)로 변환한다.
    // ─────────────────────────────────────────────────────────────────────
    bool bevLineToFrame(const LineFit& lf, cv::Point2f& p_frame0, cv::Point2f& p_frame1) {
        if (bev_size_.width <= 1 || bev_size_.height <= 1 || H_inv_.empty()) return false;

        float y0 = 0.0f;
        float y1 = static_cast<float>(bev_size_.height - 1);
        float x0 = lf.m * y0 + lf.b;
        float x1 = lf.m * y1 + lf.b;

        auto clampf = [](float v, float lo, float hi){ return std::max(lo, std::min(hi, v)); };
        x0 = clampf(x0, 0.0f, static_cast<float>(bev_size_.width - 1));
        x1 = clampf(x1, 0.0f, static_cast<float>(bev_size_.width - 1));

        std::vector<cv::Point2f> src = { {x0, y0}, {x1, y1} };
        std::vector<cv::Point2f> dst;
        cv::perspectiveTransform(src, dst, H_inv_);

        if (dst.size() != 2) return false;
        p_frame0 = dst[0];
        p_frame1 = dst[1];
        return true;
    }

    // ─────────────────────────────────────────────────────────────────────
    // 회전된 사각형(바퀴 등)을 그리기 위한 헬퍼: center 기준 angle_deg만큼
    // 회전한 (w x h) 크기의 사각형 꼭짓점 4개를 계산해 채운다.
    // ─────────────────────────────────────────────────────────────────────
    static void fillRotatedRect(cv::Mat& canvas, cv::Point2f center, float w, float h,
                                 float angle_deg, cv::Scalar color) {
        cv::RotatedRect rr(center, cv::Size2f(w, h), angle_deg);
        cv::Point2f pts[4];
        rr.points(pts);
        std::vector<cv::Point> poly = { pts[0], pts[1], pts[2], pts[3] };
        cv::fillConvexPoly(canvas, poly, color, cv::LINE_AA);
    }

    // ─────────────────────────────────────────────────────────────────────
    // "Vehicle Dynamics" 디버그 창: 차량 중심(스키매틱) 기준 조향/속도 시각화
    //
    // 위에서 내려다본 차량 아이콘의 앞바퀴가 조향 명령만큼 실제로 꺾이는 모습으로
    // 표시하고(뒷바퀴는 고정), 속도는 옆의 숫자+막대 게이지로 표시한다.
    // 정성적 근사 표시이며 물리적으로 보정된 값이 아니다.
    // ─────────────────────────────────────────────────────────────────────
    cv::Mat drawVehicleDynamicsView() const {
        const int W = 300, H = 300;
        cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(30, 30, 30));

        const cv::Point center(110, H / 2);
        const int body_w = 70, body_h = 130;
        const int wheel_w = 14, wheel_h = 32;
        const int half_w = body_w / 2, half_h = body_h / 2;

        // ── 1) 차체 (위에서 내려다본 모습, 전방 = 위쪽) ─────────────────
        cv::rectangle(canvas,
                      cv::Rect(center.x - half_w, center.y - half_h, body_w, body_h),
                      cv::Scalar(0, 165, 255), cv::FILLED, cv::LINE_AA);
        cv::rectangle(canvas,
                      cv::Rect(center.x - half_w, center.y - half_h, body_w, body_h),
                      cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        // 전방 표시 삼각형
        std::vector<cv::Point> nose = {
            {center.x,          center.y - half_h - 12},
            {center.x - 10,     center.y - half_h + 2},
            {center.x + 10,     center.y - half_h + 2},
        };
        cv::fillConvexPoly(canvas, nose, cv::Scalar(0, 255, 255), cv::LINE_AA);

        // ── 2) 뒷바퀴 (고정, 차체와 평행) ───────────────────────────────
        fillRotatedRect(canvas, {float(center.x - half_w), float(center.y + half_h - wheel_h/2.f)},
                         wheel_w, wheel_h, 0.f, cv::Scalar(40, 40, 40));
        fillRotatedRect(canvas, {float(center.x + half_w), float(center.y + half_h - wheel_h/2.f)},
                         wheel_w, wheel_h, 0.f, cv::Scalar(40, 40, 40));

        // ── 3) 앞바퀴 (조향각만큼 실제로 회전) ───────────────────────────
        // steer_cmd를 그대로 도(degree)처럼 취급해 ±45도 범위로 클램프 후 표시 (실측 변환식 없음)
        const float steer_deg_display = std::clamp(last_steer_cmd_, -45.f, 45.f);
        fillRotatedRect(canvas, {float(center.x - half_w), float(center.y - half_h + wheel_h/2.f)},
                         wheel_w, wheel_h, steer_deg_display, cv::Scalar(0, 255, 255));
        fillRotatedRect(canvas, {float(center.x + half_w), float(center.y - half_h + wheel_h/2.f)},
                         wheel_w, wheel_h, steer_deg_display, cv::Scalar(0, 255, 255));

        {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Steer: %.1f", last_steer_cmd_);
            putText(canvas, buf, Point(10, H - 34), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
        }

        // ── 4) 속도 게이지 (막대 + 수치) ─────────────────────────────────
        const int gauge_x = 210, gauge_y = 40, gauge_w = 24, gauge_h = 200;
        cv::rectangle(canvas, cv::Rect(gauge_x, gauge_y, gauge_w, gauge_h),
                      cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        float ratio = std::clamp(last_speed_cmd_ / std::max(1.f, speed_gauge_max_), 0.f, 1.f);
        int   fill_h = static_cast<int>(std::round(gauge_h * ratio));
        cv::rectangle(canvas, cv::Rect(gauge_x, gauge_y + gauge_h - fill_h, gauge_w, fill_h),
                      cv::Scalar(0, 255, 0), cv::FILLED);
        {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "Speed: %.1f", last_speed_cmd_);
            putText(canvas, buf, Point(gauge_x - 15, gauge_y - 10),
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);
        }

        putText(canvas, "Vehicle-centered schematic (approx)", Point(10, H - 10),
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
    void updateLaneChangeState(bool valid, const LineFit& center_fit, float offset) {
        const auto feedback = lane_change_tracker_.update(
            valid,
            center_fit.m,
            offset);

        std_msgs::msg::Int32MultiArray st;
        st.data = {feedback.changing, feedback.success};
        lane_change_state_pub_->publish(st);
    }

    // 차선 모드에 대응하는 목표 ref 비율(0~1) 반환
    float getTargetRefForMode(LaneMode m) const {
        float r = (m == LaneMode::LANE_ONE) ? center_reference_lane_one_
                                            : center_reference_lane_two_;
        return std::clamp(r, 0.0f, 1.0f);
    }

    // 스무딩 ON이면 보간 중인 ref 비율, OFF면 목표 비율 즉시 반환
    float getActiveRefRatio() const {
        return smooth_enabled_ ? ref_ratio_current_ : getTargetRefForMode(lane_mode_);
    }

    // ref 선형 보간 전환 시작: 현재값 → 새 모드 목표값을 duration 동안 선형 보간
    void startRefTransition(LaneMode new_mode) {
        if (!smooth_enabled_) return;
        ref_ratio_start_       = ref_ratio_current_;
        ref_ratio_target_      = getTargetRefForMode(new_mode);
        ref_start_time_        = this->now();
        ref_transition_active_ = true;
    }

    // 매 프레임 호출하여 ref_ratio_current_를 선형 보간 갱신
    void updateRefRatio() {
        if (!smooth_enabled_ || !ref_transition_active_) return;
        const double t = (this->now() - ref_start_time_).seconds();
        if (t >= ref_transition_duration_sec_) {
            ref_ratio_current_     = ref_ratio_target_;
            ref_transition_active_ = false;
            return;
        }
        const double dur = std::max(1e-6, ref_transition_duration_sec_);
        const float  a   = static_cast<float>(t / dur);  // 0~1 보간 계수
        ref_ratio_current_ = ref_ratio_start_ + (ref_ratio_target_ - ref_ratio_start_) * a;
    }


private:
    // ── ROS 통신 객체 ───────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        image_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr              offset_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               validity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  fit_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr    lane_change_state_pub_;

    // ── 조향/속도 명령 시각화 (정성적 근사, 실측 휠베이스/조향각 보정값 아님) ──
    float last_steer_cmd_  = 0.f;  // /xycar_motor에서 수신한 마지막 조향 명령
    float last_speed_cmd_  = 0.f;  // /xycar_motor에서 수신한 마지막 속도 명령
    float speed_gauge_max_ = 31.f; // 속도 게이지 만땅 기준값 (control.py LANE_DRIVE max_speed와 동일)

    // ── 설정 파라미터 ───────────────────────────────────────────────────
    Config   config_;
    LaneMode lane_mode_;
    int frame_width_, frame_height_;
    int roi_top_width_, roi_bottom_width_;
    int roi_top_y_,     roi_bottom_y_;
    float center_reference_lane_one_;
    float center_reference_lane_two_;

    // ── BEV 투시변환 행렬 ───────────────────────────────────────────────
    cv::Mat  H_;        // 사다리꼴 → BEV 정변환
    cv::Mat  H_inv_;    // BEV → 원본 프레임 역변환
    cv::Size bev_size_;

    // ── 이전 프레임 fallback 정보 ───────────────────────────────────────
    LineFit prev_center_fit_{0.f, 0.f};
    bool    has_prev_center_fit_ = false;
    float   prev_offset_         = 0.f;
    int     ref_x_               = 0;

    // ── 디버그 제어 ─────────────────────────────────────────────────────
    int  frame_count_  = 0;
    int  debug_stride_ = 1;   // 디버그 출력 주기 (1이면 매 프레임)
    bool debug_view_   = config_.debug_view;

    // ── 모드/차선 상태 ──────────────────────────────────────────────────
    int  current_mode_ = 0;
    int  current_lane_ = 0;
    lane_detection::LaneChangeStateTracker lane_change_tracker_;

    // ── ref 비율 스무딩 전환 변수 ───────────────────────────────────────
    bool         smooth_enabled_              = config_.change_ref_smoothly;
    float        ref_ratio_current_           = 0.f;  // 현재 보간 중인 ref 비율
    float        ref_ratio_start_             = 0.f;  // 전환 시작 시점 값
    float        ref_ratio_target_            = 0.f;  // 전환 목표 값
    rclcpp::Time ref_start_time_;
    bool         ref_transition_active_       = false;
    double       ref_transition_duration_sec_ = 0.8;  // 전환 소요 시간 (초)


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
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (smooth_enabled_) updateRefRatio();

        // (0) ROS 이미지 → OpenCV BGR Mat
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge 오류: %s", e.what());
            return;
        }
        Mat frame = cv_ptr->image;
        if (frame.empty()) return;

        // ROI 사다리꼴 디버그 시각화
        if (debug_view_) {
            Mat frame_roi_vis = frame.clone();
            drawROIPolygon(frame_roi_vis);
            imshow("ROI Polygon", frame_roi_vis);
        }

        // (1) 노란 차선 전처리: ROI 내 노란 엣지 픽셀 추출
        Mat yellow_edges = preprocessYellow(frame);
        if (debug_view_) imshow("Mask-Yellow", yellow_edges);

        // (2) BEV 투시변환: 사다리꼴 → 직사각형
        Mat bev_yellow;
        warpPerspective(yellow_edges, bev_yellow, H_, bev_size_,
                        INTER_LINEAR, BORDER_CONSTANT, Scalar(0));
        if (debug_view_) imshow("BEV-Yellow", bev_yellow);

        // (3) 노이즈 억제 준비: corridor 범위 및 디버그 캔버스 준비
        const int y_start_chk = (int)std::round(bev_size_.height * 0.0f);
        const int y_end_chk   = (int)std::round(bev_size_.height * 1.0f);
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
        if (suppressed) bev_yellow = bev_clean;

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
        if (suppressed2) bev_yellow = bev_clean2;

        if (debug_view_)
            cv::imshow("BEV-Yellow (suppressed rows/columns in red)", bev_color);

        // (4) 슬라이딩 윈도우 직선 피팅
        frame_count_++;
        bool show_dbg = debug_view_ && (frame_count_ % debug_stride_ == 0);

        bool valid = false;
        cv::Mat dbg;
        LineFit center_fit = fitLaneFromBEV(bev_yellow, valid, show_dbg ? &dbg : nullptr);

        std_msgs::msg::Bool validity_msg;
        validity_msg.data = valid;
        validity_pub_->publish(validity_msg);

        // (5) 오프셋 계산 및 히스토리 갱신
        float offset = 0.f;
        if (valid) {
            offset               = calcOffsetFromCenterLine(center_fit, bev_size_.width);
            prev_offset_         = offset;
            prev_center_fit_     = center_fit;
            has_prev_center_fit_ = true;
        } else {
            // 피팅 실패: 이전 오프셋 재사용, 데이터 없으면 0
            offset = has_prev_center_fit_ ? prev_offset_ : 0.f;
        }

        // (6) 토픽 발행 + 디버그 출력
        publishAndDebug(frame, offset, center_fit, show_dbg,
                        dbg.empty() ? nullptr : &dbg);

        // Use this frame's explicit fitting validity. A failed fit may reuse
        // the prior offset for lane control, but it must reset completion
        // progress rather than treating stale geometry as fresh evidence.
        updateLaneChangeState(valid, center_fit, offset);
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
    void modeCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;

        const int new_mode  = msg->data[0];
        const int new_lane  = msg->data[1];
        const int prev_mode = current_mode_;
        const bool valid_lane_target = new_lane == 0 || new_lane == 1;

        lane_change_tracker_.handleCommand(new_mode, new_lane);

        current_mode_ = new_mode;
        current_lane_ = new_lane;

        // lane_mode_ 갱신: 모드 5이거나 비3 → 3 전환 시에만 업데이트
        LaneMode old_lane_mode = lane_mode_;
        if (valid_lane_target &&
            (new_mode == 5 || (prev_mode != 3 && new_mode == 3))) {
            lane_mode_ = (new_lane == 0) ? LaneMode::LANE_ONE : LaneMode::LANE_TWO;
        }

        // 차선 모드가 실제로 변경되었으면 ref 전환 시작
        if (smooth_enabled_ && lane_mode_ != old_lane_mode)
            startRefTransition(lane_mode_);

    }

    // ─────────────────────────────────────────────────────────────────────
    // 모터 명령 콜백: /xycar_motor [조향각, 속도] 수신
    //
    // "Vehicle Dynamics" 디버그 창 시각화에만 사용한다 (제어에 개입하지 않음).
    // ─────────────────────────────────────────────────────────────────────
    void motorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.empty()) return;
        last_steer_cmd_ = msg->data[0];
        if (msg->data.size() >= 2) last_speed_cmd_ = msg->data[1];

        // /xycar_motor가 도착할 때마다 즉시 갱신: main.py의 "Status" 창과 같은
        // 주기(제어 루프 주기)로 표시되도록, 카메라 프레임 콜백을 기다리지 않는다.
        if (debug_view_) {
            cv::imshow("Vehicle Dynamics", drawVehicleDynamicsView());
            cv::waitKey(1);
        }
    }
};


int main(int argc, char** argv) {
    std::cout << "OpenCV 버전: " << CV_VERSION << std::endl;
    rclcpp::init(argc, argv);

    Config config = load_config(
        "/home/xytron/xycar_ws/src/orda/driving/lane_detection/lane_detection_parameter.json"
    );

    // 로드된 설정값 확인용 로그 (config 수정이 실제로 반영됐는지 디버깅용)
    std::cout << "===== [lane_detection] 로드된 Config 값 =====" << std::endl;
    std::cout << "[Config] debug_view = " << config.debug_view << std::endl;
    std::cout << "[Config] roi_top_width_coefficient = " << config.roi_top_width_coefficient << std::endl;
    std::cout << "[Config] roi_bottom_width_coefficient = " << config.roi_bottom_width_coefficient << std::endl;
    std::cout << "[Config] sliding_window_num_windows = " << config.sliding_window_num_windows << std::endl;
    std::cout << "[Config] sliding_window_margin = " << config.sliding_window_margin << std::endl;
    std::cout << "[Config] sliding_window_minpix = " << config.sliding_window_minpix << std::endl;
    std::cout << "[Config] corridor_width = " << config.corridor_width << std::endl;
    std::cout << "==============================================" << std::endl;

    auto node = std::make_shared<LaneDetector>(config);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
