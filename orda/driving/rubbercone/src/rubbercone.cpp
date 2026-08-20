// ─────────────────────────────────────────────────────────────────────────────
// rubbercone.cpp
//
// LiDAR 기반 라바콘 통로 추종 노드
//
// scan에서 연속 반사점을 하나의 콘 후보로 묶고, 좌/우 경계를 각각 선형으로
// 추정한다. 따라서 콘 사이 간격이나 출발 방향이 바뀌어도 "가까운 콘에서
// 일정 간격만큼" 확장하는 방식에 의존하지 않는다.
//
// 탐색 영역은 통로를 만드는 가장 가까운 콘들만 남기도록 제한한다:
//   scan_max_range       - 최대 거리 (기본 1.10m)
//   scan_max_angle       - 전방 기준 좌우 최대 각도 (기본 ±85°)
//   max_lateral_distance - 좌우 최대 편차 (기본 0.70m, 벽/옆 코스 제거)
//   max_cone_centers     - 사용할 콘 대표점 개수 (기본 6, 가까운 것부터)
//   boundary_points      - 한쪽 경계 직선 피팅에 쓸 콘 개수 (기본 3)
//
// 발행: rubbercone_info (std_msgs/Int32MultiArray)
//   [0] offset     : 조향 오프셋 (양수=오른쪽 편향)
//   [1] end_flag   : 0=주행 중, 1=라바콘 구간 종료
//   [2] confidence : 경로 추정 신뢰도 (0~100)
// 구독: /rubbercone_reset (std_msgs/Empty) - 다음 라바콘 세션 준비
//
// 디버그 창 (enable_gui:=true, 기본 true):
//   "Rubbercone Debug" - 탑뷰로 스캔 포인트 / 콘 후보 / 좌우 경계 직선 /
//                        목표점 / 오프셋을 시각화한다.
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

using std::placeholders::_1;

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr char kRubberconeResetTopic[] = "/rubbercone_reset";

template<typename T>
T clampValue(T value, T lower, T upper)
{
    return std::max(lower, std::min(value, upper));
}

float pointDistance(const cv::Point2f& a, const cv::Point2f& b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

float pointRange(const cv::Point2f& point)
{
    return std::hypot(point.x, point.y);
}

struct BoundaryModel {
    bool valid{false};
    int count{0};
    float slope{0.0f};
    float intercept{0.0f};
    float min_x{0.0f};
    float max_x{0.0f};
    float mean_residual{0.0f};

    float at(float x) const
    {
        return slope * x + intercept;
    }
};

struct PathEstimate {
    bool valid{false};
    bool bilateral{false};
    cv::Point2f target{};
    float confidence{0.0f};
};

// 스캔 콜백에서 계산한 중간 결과를 디버그 창에 그대로 옮겨 그리기 위한 스냅샷.
struct DebugSnapshot {
    std::vector<cv::Point2f> raw_points;    // 필터를 통과한 원본 반사점
    std::vector<cv::Point2f> cone_centers;  // 클러스터링으로 뽑은 콘 대표점
    std::vector<cv::Point2f> left_points;   // 좌 경계로 분류된 콘
    std::vector<cv::Point2f> right_points;  // 우 경계로 분류된 콘
    BoundaryModel left;
    BoundaryModel right;
    PathEstimate path;
    cv::Point2f filtered_target{};
    bool has_filtered_target{false};
    float half_width{0.0f};
    int32_t offset{0};
    int32_t end_flag{0};
    int32_t confidence{0};
    bool armed{false};
};

}  // namespace


class LidarViewer : public rclcpp::Node {
public:
    LidarViewer()
    : Node("rubbercone"),
      offset_gain_(230.0f),
      scan_min_range_(0.18f),
      scan_max_range_(1.10f),
      scan_max_angle_(85.0f * kPi / 180.0f),
      max_lateral_distance_(0.70f),
      front_ignore_angle_(13.0f * kPi / 180.0f),
      front_ignore_range_(0.32f),
      cone_cluster_break_(0.08f),
      cone_cluster_max_width_(0.22f),
      cone_merge_distance_(0.10f),
      side_deadband_(0.03f),
      min_forward_x_(0.04f),
      max_cone_centers_(6),
      max_boundary_points_(3),
      max_boundary_slope_(1.20f),
      fit_residual_limit_(0.18f),
      target_lookahead_(0.70f),
      min_target_lookahead_(0.35f),
      nominal_half_width_(0.30f),
      adaptive_half_width_(0.30f),
      min_corridor_width_(0.35f),
      max_corridor_width_(1.40f),
      offset_limit_(40.0f),
      max_offset_step_(20.0f),
      valid_frame_count_(0),
      missing_frame_count_(0),
      cone_section_armed_(false),
      end_latched_(false),
      filtered_target_y_(0.0f),
      has_filtered_target_y_(false),
      filtered_offset_(0.0f),
      has_filtered_offset_(false),
      offset_filter_alpha_(0.80f),
      target_filter_alpha_(0.85f),
      one_side_target_filter_alpha_(0.75f),
      max_target_y_step_bilateral_(0.20f),
      max_target_y_step_one_side_(0.20f),
      end_missing_frames_(3),
      rubber_offset_value_(0),
      rubber_end_value_(0),
      rubber_confidence_value_(0),
      enable_gui_(false),
      px_per_m_(200.0f)
    {
        // 제어 입력은 오래된 메시지보다 최신 메시지가 중요하므로 큐를 한 프레임으로 제한한다.
        auto scan_qos = rclcpp::SensorDataQoS();
        scan_qos.keep_last(1);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", scan_qos,
            std::bind(&LidarViewer::scanCallback, this, _1));

        auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
        info_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("rubbercone_info", qos_fast);

        offset_filter_alpha_ = clampValue(
            static_cast<float>(declare_parameter<double>("offset_filter_alpha", 0.80)),
            0.05f, 1.0f);
        end_missing_frames_ = std::max(
            1, static_cast<int>(declare_parameter<int>("end_missing_frames", 3)));

        // 230에서는 가로 오차 0.17m만 넘어도 오프셋이 ±40으로 포화돼, 깊은
        // 코너가 전부 같은 값으로 뭉개졌다(실주행 bag으로 확인). 150이면
        // 포화점이 0.27m로 밀려 통로 폭 안에서 비례 제어가 유지된다.
        offset_gain_ = clampValue(
            static_cast<float>(declare_parameter<double>("offset_gain", 150.0)),
            100.0f, 350.0f);
        offset_limit_ = clampValue(
            static_cast<float>(declare_parameter<double>("offset_limit", 45.0)),
            20.0f, 45.0f);

        // 코스별로 달라질 수 있는 기하 파라미터는 런타임 인수로도 조정할 수 있다.
        // 아래 네 값이 "LiDAR가 어디까지 보는지"를 결정한다. 좁힐수록 옆 코스나
        // 벽을 콘으로 오인할 여지가 줄지만, 커브에서 다음 콘을 늦게 잡는다.
        scan_max_range_ = clampValue(
            static_cast<float>(declare_parameter<double>("scan_max_range", 1.10)),
            0.50f, 3.00f);
        scan_max_angle_ = clampValue(
            static_cast<float>(declare_parameter<double>("scan_max_angle", 85.0)),
            30.0f, 90.0f) * kPi / 180.0f;
        max_lateral_distance_ = clampValue(
            static_cast<float>(declare_parameter<double>("max_lateral_distance", 0.70)),
            0.25f, 2.00f);
        max_cone_centers_ = std::max(
            2, static_cast<int>(declare_parameter<int>("max_cone_centers", 6)));
        max_boundary_points_ = clampValue(
            static_cast<int>(declare_parameter<int>("boundary_points", 3)), 2, 6);

        target_lookahead_ = clampValue(
            static_cast<float>(declare_parameter<double>("target_lookahead", 0.70)),
            min_target_lookahead_, scan_max_range_);
        nominal_half_width_ = clampValue(
            static_cast<float>(declare_parameter<double>("nominal_half_width", 0.30)),
            0.15f, 0.70f);
        resetSessionState();

        // Reset is an edge-triggered command. The default mutually-exclusive
        // callback group and single-threaded spin serialize it with scanCallback.
        auto reset_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliable().durability_volatile();
        reset_sub_ = create_subscription<std_msgs::msg::Empty>(
            kRubberconeResetTopic, reset_qos,
            std::bind(&LidarViewer::resetCallback, this, _1));

        // ── 디버그 창 ────────────────────────────────────────────────────────
        enable_gui_ = declare_parameter<bool>("enable_gui", true);
        px_per_m_ = clampValue(
            static_cast<float>(declare_parameter<double>("debug_px_per_m", 200.0)),
            60.0f, 400.0f);

        if (enable_gui_) {
            try {
                cv::namedWindow(kDebugWindow, cv::WINDOW_AUTOSIZE);
                // 스캔 콜백과 분리된 20Hz 타이머로 갱신해, LiDAR 주기가 흔들려도
                // 창이 멈춘 것처럼 보이지 않게 한다.
                gui_timer_ = create_wall_timer(
                    std::chrono::milliseconds(50),
                    std::bind(&LidarViewer::drawDebug, this));
            } catch (const cv::Exception& e) {
                // SSH 등 디스플레이가 없는 환경에서도 주행은 계속되어야 한다.
                enable_gui_ = false;
                RCLCPP_WARN(get_logger(),
                            "디버그 창을 열 수 없어 GUI를 끕니다: %s", e.what());
            }
        }

        RCLCPP_INFO(get_logger(), "rubbercone_node 시작 (enable_gui=%s)",
                    enable_gui_ ? "true" : "false");
    }

    ~LidarViewer() override
    {
        if (enable_gui_) {
            cv::destroyWindow(kDebugWindow);
        }
    }

private:
    void resetSessionState()
    {
        valid_frame_count_ = 0;
        missing_frame_count_ = 0;
        cone_section_armed_ = false;
        end_latched_ = false;

        adaptive_half_width_ = nominal_half_width_;
        filtered_target_y_ = 0.0f;
        has_filtered_target_y_ = false;
        filtered_offset_ = 0.0f;
        has_filtered_offset_ = false;

        rubber_offset_value_ = 0;
        rubber_end_value_ = 0;
        rubber_confidence_value_ = 0;

        // The GUI is diagnostic state, but clearing it here prevents an old
        // latched end/debounce display from being mistaken for the new session.
        std::lock_guard<std::mutex> lock(debug_mutex_);
        debug_ = DebugSnapshot{};
        debug_.half_width = adaptive_half_width_;
    }

    void resetCallback(const std_msgs::msg::Empty::SharedPtr /*msg*/)
    {
        resetSessionState();
        RCLCPP_INFO(get_logger(), "Rubber-cone session reset");
    }

    void publishInfo()
    {
        std_msgs::msg::Int32MultiArray msg;
        msg.data.resize(3);
        msg.data[0] = rubber_offset_value_;
        msg.data[1] = rubber_end_value_;
        msg.data[2] = rubber_confidence_value_;
        info_pub_->publish(msg);
    }

    // 연속한 LaserScan 반사점 중 실제 콘 하나에 해당하는 대표점을 추출한다.
    // raw_points가 주어지면 필터를 통과한 원본 반사점도 함께 돌려준다(디버그 창용).
    std::vector<cv::Point2f> extractConeCenters(const sensor_msgs::msg::LaserScan& scan,
                                                std::vector<cv::Point2f>* raw_points = nullptr) const
    {
        std::vector<cv::Point2f> centers;
        std::vector<cv::Point2f> cluster;

        const auto flush_cluster = [&]() {
            if (cluster.empty()) {
                return;
            }

            // 큰 연속 반사면(벽, 차체 등)은 콘으로 사용하지 않는다.
            if (cluster.size() > 1 &&
                pointDistance(cluster.front(), cluster.back()) > cone_cluster_max_width_)
            {
                cluster.clear();
                return;
            }

            const auto closest = std::min_element(
                cluster.begin(), cluster.end(),
                [](const cv::Point2f& a, const cv::Point2f& b) {
                    return pointRange(a) < pointRange(b);
                });
            centers.push_back(*closest);
            cluster.clear();
        };

        float angle = scan.angle_min;
        for (const float range : scan.ranges) {
            const bool in_search_area =
                std::isfinite(range) &&
                range >= scan_min_range_ && range <= scan_max_range_ &&
                std::abs(angle) <= scan_max_angle_;

            // 가까운 전방 반사는 차체/센서 마운트일 가능성이 높다. 멀리 있는
            // 전방 콘은 보존해 반대 방향 출발 시 첫 목표점을 놓치지 않게 한다.
            const bool body_reflection =
                std::abs(angle) < front_ignore_angle_ && range < front_ignore_range_;

            const cv::Point2f point{
                range * std::cos(angle),
                range * std::sin(angle)};

            // 옆으로 너무 멀리 떨어진 반사는 통로 바깥(벽, 옆 코스)이다.
            const bool too_far_sideways = std::abs(point.y) > max_lateral_distance_;

            if (!in_search_area || body_reflection || too_far_sideways) {
                flush_cluster();
            } else {
                if (!cluster.empty() &&
                    pointDistance(point, cluster.back()) > cone_cluster_break_)
                {
                    flush_cluster();
                }
                cluster.push_back(point);
                if (raw_points != nullptr) {
                    raw_points->push_back(point);
                }
            }
            angle += scan.angle_increment;
        }
        flush_cluster();

        // 한 콘이 불연속 반사로 두 번 나뉜 경우에는 가까운 대표점 하나만 남긴다.
        std::sort(centers.begin(), centers.end(),
                  [](const cv::Point2f& a, const cv::Point2f& b) {
                      return pointRange(a) < pointRange(b);
                  });

        std::vector<cv::Point2f> merged;
        for (const auto& center : centers) {
            const bool duplicate = std::any_of(
                merged.begin(), merged.end(),
                [&](const cv::Point2f& existing) {
                    return pointDistance(center, existing) < cone_merge_distance_;
                });
            if (!duplicate) {
                merged.push_back(center);
            }
        }

        // centers가 거리순으로 정렬돼 있으므로, 앞에서 자르면 가장 가까운 콘만 남는다.
        // 통로를 만드는 것은 바로 옆의 콘 몇 개뿐이고, 멀리 있는 콘은 경계 직선을
        // 흔들기만 한다.
        if (static_cast<int>(merged.size()) > max_cone_centers_) {
            merged.resize(static_cast<std::size_t>(max_cone_centers_));
        }
        return merged;
    }

    void fitWeightedLine(const std::vector<cv::Point2f>& points,
                         float& slope, float& intercept) const
    {
        float sum_w = 0.0f;
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        float sum_xx = 0.0f;
        float sum_xy = 0.0f;

        for (const auto& point : points) {
            // 가까운 콘일수록 실제 차량 경로에 미치는 영향이 크다.
            const float weight = 1.0f / (0.15f + pointRange(point));
            sum_w += weight;
            sum_x += weight * point.x;
            sum_y += weight * point.y;
            sum_xx += weight * point.x * point.x;
            sum_xy += weight * point.x * point.y;
        }

        const float denominator = sum_w * sum_xx - sum_x * sum_x;
        if (std::abs(denominator) < 1e-5f) {
            slope = 0.0f;
            intercept = sum_w > 0.0f ? sum_y / sum_w : 0.0f;
            return;
        }

        slope = (sum_w * sum_xy - sum_x * sum_y) / denominator;
        slope = clampValue(slope, -max_boundary_slope_, max_boundary_slope_);
        intercept = (sum_y - slope * sum_x) / sum_w;
    }

    BoundaryModel fitBoundary(std::vector<cv::Point2f> points) const
    {
        BoundaryModel model;
        if (points.empty()) {
            return model;
        }

        std::sort(points.begin(), points.end(),
                  [](const cv::Point2f& a, const cv::Point2f& b) {
                      return a.x < b.x;
                  });
        if (static_cast<int>(points.size()) > max_boundary_points_) {
            points.resize(static_cast<std::size_t>(max_boundary_points_));
        }

        float slope = 0.0f;
        float intercept = 0.0f;
        fitWeightedLine(points, slope, intercept);

        // 첫 추정에서 크게 벗어난 점은 인접 코스/잡음일 가능성이 높으므로 한 번 제거한다.
        std::vector<cv::Point2f> inliers;
        for (const auto& point : points) {
            if (std::abs(point.y - (slope * point.x + intercept)) <= fit_residual_limit_) {
                inliers.push_back(point);
            }
        }
        if (inliers.size() >= 2 && inliers.size() < points.size()) {
            points = inliers;
            fitWeightedLine(points, slope, intercept);
        }

        float residual_sum = 0.0f;
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        for (const auto& point : points) {
            residual_sum += std::abs(point.y - (slope * point.x + intercept));
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
        }

        model.valid = true;
        model.count = static_cast<int>(points.size());
        model.slope = slope;
        model.intercept = intercept;
        model.min_x = min_x;
        model.max_x = max_x;
        model.mean_residual = residual_sum / static_cast<float>(points.size());
        return model;
    }

    float chooseTargetX(const BoundaryModel* left, const BoundaryModel* right) const
    {
        float available_x = target_lookahead_;
        if (left != nullptr && right != nullptr) {
            available_x = std::min(left->max_x, right->max_x);
        } else if (left != nullptr) {
            available_x = left->max_x;
        } else if (right != nullptr) {
            available_x = right->max_x;
        }

        return clampValue(std::min(target_lookahead_, available_x),
                          min_target_lookahead_, target_lookahead_);
    }

    float boundaryQuality(const BoundaryModel& boundary) const
    {
        const float count_score = clampValue(
            static_cast<float>(boundary.count) / 2.0f, 0.0f, 1.0f);
        const float residual_score = 1.0f - clampValue(
            boundary.mean_residual / fit_residual_limit_, 0.0f, 1.0f);
        const float coverage_score = clampValue(
            boundary.max_x / target_lookahead_, 0.0f, 1.0f);
        return 0.50f * count_score + 0.25f * residual_score + 0.25f * coverage_score;
    }

    PathEstimate estimatePath(const std::vector<cv::Point2f>& centers)
    {
        std::vector<cv::Point2f> left_points;
        std::vector<cv::Point2f> right_points;
        left_points.reserve(centers.size());
        right_points.reserve(centers.size());

        for (const auto& center : centers) {
            if (center.x < min_forward_x_) {
                continue;
            }
            if (center.y > side_deadband_) {
                left_points.push_back(center);
            } else if (center.y < -side_deadband_) {
                right_points.push_back(center);
            }
        }

        const BoundaryModel left = fitBoundary(left_points);
        const BoundaryModel right = fitBoundary(right_points);
        PathEstimate path;

        if (enable_gui_) {
            std::lock_guard<std::mutex> lock(debug_mutex_);
            debug_.left_points = left_points;
            debug_.right_points = right_points;
            debug_.left = left;
            debug_.right = right;
        }

        // 한쪽의 가까운 콘 한 개와 반대 경계의 선도 통로 중심을 잡는 데 유용하다.
        // 다만 실제 모터 명령은 아래의 목표점/조향 변화율 제한을 거치므로 단발
        // 반사점으로 최대 조향까지 즉시 튀지 않는다.
        if (left.valid && right.valid &&
            left.count >= kMinBilateralPoints &&
            right.count >= kMinBilateralPoints) {
            const float target_x = chooseTargetX(&left, &right);
            const float left_y = left.at(target_x);
            const float right_y = right.at(target_x);
            const float corridor_width = left_y - right_y;

            if (corridor_width >= min_corridor_width_ &&
                corridor_width <= max_corridor_width_)
            {
                // 실제 폭을 천천히 학습해 한쪽 경계만 보이는 구간에도 활용한다.
                const float measured_half_width = 0.5f * corridor_width;
                adaptive_half_width_ += 0.15f * (measured_half_width - adaptive_half_width_);
                adaptive_half_width_ = clampValue(
                    adaptive_half_width_, 0.15f, max_corridor_width_ * 0.5f);

                const float quality = 0.5f * (boundaryQuality(left) + boundaryQuality(right));
                path.valid = true;
                path.bilateral = true;
                path.target = cv::Point2f{target_x, 0.5f * (left_y + right_y)};
                path.confidence = clampValue(0.65f + 0.35f * quality, 0.0f, 1.0f);
                return path;
            }
        }

        // 한쪽 경계가 가려지거나 시작 위치가 치우친 경우에는 최근에 학습한 통로 폭으로
        // 반대편 경계를 추정한다. 이 상태는 유효하지만 신뢰도를 낮춰 속도를 제한한다.
        const bool use_left = left.valid &&
            (!right.valid || boundaryQuality(left) >= boundaryQuality(right));
        const BoundaryModel* boundary = use_left ? &left : (right.valid ? &right : nullptr);
        if (boundary == nullptr) {
            return path;
        }

        const float target_x = chooseTargetX(use_left ? boundary : nullptr,
                                              use_left ? nullptr : boundary);
        const float boundary_y = boundary->at(target_x);
        path.valid = true;
        path.bilateral = false;
        path.target = cv::Point2f{
            target_x,
            use_left ? boundary_y - adaptive_half_width_
                     : boundary_y + adaptive_half_width_};
        path.confidence = clampValue(
            0.32f + 0.46f * boundaryQuality(*boundary), 0.0f, 0.78f);
        return path;
    }

    float smoothTargetY(const PathEstimate& path)
    {
        // 포화된 offset을 중앙값 처리하면 실제 목표점의 크기 정보가 사라지고
        // S자 구간에서 두 프레임 늦게 반대 방향으로 따라갈 수 있다. 따라서
        // 거리 단위의 목표점에서 먼저 변화량을 제한한 뒤 조향 오프셋으로 변환한다.
        if (!has_filtered_target_y_) {
            filtered_target_y_ = path.target.y;
            has_filtered_target_y_ = true;
            return filtered_target_y_;
        }

        const float max_step = path.bilateral
            ? max_target_y_step_bilateral_
            : max_target_y_step_one_side_;
        const float alpha = path.bilateral
            ? target_filter_alpha_
            : one_side_target_filter_alpha_;
        const float bounded_delta = clampValue(
            path.target.y - filtered_target_y_, -max_step, max_step);
        filtered_target_y_ += alpha * bounded_delta;
        return filtered_target_y_;
    }

    void updateDetectionState(const PathEstimate& path)
    {
        if (path.valid) {
            missing_frame_count_ = 0;

            // 한 개의 잡음 콘으로 종료 검출이 무장되지 않도록, 신뢰도 있는 프레임만 센다.
            if (!cone_section_armed_) {
                if (path.confidence >= 0.55f) {
                    ++valid_frame_count_;
                } else {
                    valid_frame_count_ = 0;
                }
                if (valid_frame_count_ >= kArmValidFrames) {
                    cone_section_armed_ = true;
                    RCLCPP_INFO(get_logger(), "Rubber-cone exit detection armed");
                }
            }

            const float target_y = smoothTargetY(path);
            const float raw_offset = clampValue(
                -target_y * offset_gain_, -offset_limit_, offset_limit_);
            if (!has_filtered_offset_) {
                filtered_offset_ = raw_offset;
                has_filtered_offset_ = true;
            } else {
                // 한쪽 경계만 보이는 프레임은 목표점 단계와 오프셋 EMA를 모두
                // 더 천천히 반영해 경계 상태가 바뀌는 순간의 급조향을 막는다.
                const float alpha = path.bilateral
                    ? offset_filter_alpha_
                    : std::min(one_side_target_filter_alpha_, offset_filter_alpha_);
                const float bounded_delta = clampValue(
                    raw_offset - filtered_offset_, -max_offset_step_, max_offset_step_);
                filtered_offset_ += alpha * bounded_delta;
            }

            rubber_offset_value_ = static_cast<int32_t>(std::round(filtered_offset_));
            rubber_end_value_ = 0;
            rubber_confidence_value_ = static_cast<int32_t>(std::round(path.confidence * 100.0f));
            return;
        }

        // 짧은 스캔 누락에서는 마지막 조향값을 유지하되, 메인 노드가 감속할 수 있도록
        // 신뢰도만 빠르게 내린다.
        rubber_confidence_value_ = std::max(0, rubber_confidence_value_ - 25);
        if (!cone_section_armed_) {
            valid_frame_count_ = 0;
            rubber_end_value_ = 0;
            return;
        }

        ++missing_frame_count_;
        if (missing_frame_count_ >= end_missing_frames_) {
            end_latched_ = true;
            rubber_end_value_ = 1;
            rubber_confidence_value_ = 0;
            RCLCPP_INFO(get_logger(), "Rubber-cone end latched after %d missing frames",
                        missing_frame_count_);
        } else {
            rubber_end_value_ = 0;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // 디버그 창 (탑뷰: 위쪽=차량 전방 +x, 왼쪽=차량 좌측 +y)
    //   회색 점    : 필터를 통과한 원본 LiDAR 반사점
    //   흰 원      : 클러스터링으로 뽑은 콘 대표점
    //   초록       : 좌 경계 콘과 피팅 직선
    //   주황       : 우 경계 콘과 피팅 직선
    //   빨강 원    : 이번 스캔의 목표점 (점선 = 한쪽 경계만 추정)
    //   노랑 화살표: 필터를 거친 최종 조향 목표
    // ─────────────────────────────────────────────────────────────────────────
    void drawDebug()
    {
        DebugSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(debug_mutex_);
            snapshot = debug_;
        }

        const int W = 640;
        const int H = 640;
        const int base_y = H - 60;
        cv::Mat canvas(H, W, CV_8UC3, cv::Scalar(25, 25, 25));

        const auto toPx = [&](const cv::Point2f& p) -> cv::Point {
            return cv::Point(
                static_cast<int>(std::lround(W / 2.0f - p.y * px_per_m_)),
                static_cast<int>(std::lround(base_y - p.x * px_per_m_)));
        };
        const cv::Point origin = toPx({0.0f, 0.0f});

        // 거리 격자 (0.2m 간격) + 탐색 최대 거리
        for (float r = 0.2f; r <= scan_max_range_ + 1e-3f; r += 0.2f) {
            cv::circle(canvas, origin, static_cast<int>(r * px_per_m_),
                       cv::Scalar(48, 48, 48), 1);
        }
        cv::circle(canvas, origin, static_cast<int>(scan_max_range_ * px_per_m_),
                   cv::Scalar(80, 80, 120), 1);

        // 탐색 영역 경계: ±scan_max_angle 쐐기와 좌우 lateral 한계
        const cv::Scalar area_color(70, 70, 110);
        for (const float sign : {1.0f, -1.0f}) {
            const float a = sign * scan_max_angle_;
            cv::line(canvas, origin,
                     toPx({scan_max_range_ * std::cos(a), scan_max_range_ * std::sin(a)}),
                     area_color, 1);
            const int lateral_px = toPx({0.0f, sign * max_lateral_distance_}).x;
            cv::line(canvas, {lateral_px, 0}, {lateral_px, H}, area_color, 1);
        }

        // 차량 전방 기준선, 목표점 lookahead 선, 좌/우 분류 데드밴드
        cv::line(canvas, origin, toPx({scan_max_range_, 0.0f}), cv::Scalar(60, 60, 60), 1);
        const int lookahead_py = toPx({target_lookahead_, 0.0f}).y;
        cv::line(canvas, {0, lookahead_py}, {W, lookahead_py}, cv::Scalar(70, 70, 40), 1);
        cv::drawMarker(canvas, origin, cv::Scalar(200, 200, 200),
                       cv::MARKER_TRIANGLE_UP, 16, 2);

        // 원본 반사점
        for (const auto& p : snapshot.raw_points) {
            cv::circle(canvas, toPx(p), 2, cv::Scalar(95, 95, 95), cv::FILLED);
        }
        // 클러스터 대표점 (좌/우로 분류되지 않은 것도 보이도록 먼저 그린다)
        for (const auto& p : snapshot.cone_centers) {
            cv::circle(canvas, toPx(p), 6, cv::Scalar(180, 180, 180), 1);
        }

        drawBoundary(canvas, toPx, snapshot.left, snapshot.left_points,
                     cv::Scalar(0, 220, 0));
        drawBoundary(canvas, toPx, snapshot.right, snapshot.right_points,
                     cv::Scalar(255, 140, 0));

        // 목표점: 양쪽 경계로 잡았으면 실선 원, 한쪽만이면 얇은 원으로 구분한다.
        if (snapshot.path.valid) {
            const cv::Point target_px = toPx(snapshot.path.target);
            cv::circle(canvas, target_px, 8, cv::Scalar(0, 0, 255),
                       snapshot.path.bilateral ? 2 : 1);
        }
        if (snapshot.has_filtered_target) {
            const cv::Point filtered_px = toPx(snapshot.filtered_target);
            cv::arrowedLine(canvas, origin, filtered_px, cv::Scalar(0, 255, 255), 2, 8, 0, 0.2);
            cv::circle(canvas, filtered_px, 4, cv::Scalar(0, 255, 255), cv::FILLED);
        }

        // ── 텍스트 오버레이 ─────────────────────────────────────────────────
        const auto put = [&](const std::string& text, int row, const cv::Scalar& color) {
            cv::putText(canvas, text, {10, 24 + row * 22},
                        cv::FONT_HERSHEY_SIMPLEX, 0.52, color, 1);
        };
        const auto fmt = [](float value, int digits = 2) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(digits) << value;
            return oss.str();
        };

        std::string state = "NO PATH";
        cv::Scalar state_color{0, 0, 255};
        if (snapshot.end_flag) {
            state = "END LATCHED";
            state_color = cv::Scalar{0, 0, 255};
        } else if (snapshot.path.valid) {
            state = snapshot.path.bilateral ? "BOTH SIDES" : "ONE SIDE";
            state_color = snapshot.path.bilateral ? cv::Scalar{0, 255, 0}
                                                  : cv::Scalar{0, 200, 255};
        }

        put(state + (snapshot.armed ? "  [armed]" : "  [arming]"), 0, state_color);
        put("cones=" + std::to_string(snapshot.cone_centers.size()) +
            "  L=" + std::to_string(snapshot.left_points.size()) +
            "  R=" + std::to_string(snapshot.right_points.size()), 1,
            {255, 255, 255});
        put("offset=" + std::to_string(snapshot.offset) +
            "  conf=" + std::to_string(snapshot.confidence) +
            "  end=" + std::to_string(snapshot.end_flag), 2,
            snapshot.end_flag ? cv::Scalar{0, 0, 255} : cv::Scalar{0, 255, 0});
        put(snapshot.path.valid
                ? "target=(" + fmt(snapshot.path.target.x) + ", " +
                      fmt(snapshot.path.target.y) + ")  filt_y=" +
                      fmt(snapshot.filtered_target.y)
                : "target 없음",
            3, {180, 180, 255});
        put("half_width=" + fmt(snapshot.half_width) +
            "  fit L/R residual=" +
            (snapshot.left.valid ? fmt(snapshot.left.mean_residual) : "-") + "/" +
            (snapshot.right.valid ? fmt(snapshot.right.mean_residual) : "-"), 4,
            {160, 160, 160});
        put("area: r<=" + fmt(scan_max_range_) +
            "m  ang<=" + fmt(scan_max_angle_ * 180.0f / kPi, 0) +
            "deg  side<=" + fmt(max_lateral_distance_) +
            "m  cones<=" + std::to_string(max_cone_centers_), 5,
            {120, 120, 180});

        cv::putText(canvas, "LEFT", {12, base_y + 24},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, {120, 120, 120}, 1);
        cv::putText(canvas, "RIGHT", {W - 70, base_y + 24},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, {120, 120, 120}, 1);

        cv::imshow(kDebugWindow, canvas);
        cv::waitKey(1);
    }

    // 경계에 속한 콘과 피팅된 직선을 함께 그린다.
    void drawBoundary(cv::Mat& canvas,
                      const std::function<cv::Point(const cv::Point2f&)>& toPx,
                      const BoundaryModel& boundary,
                      const std::vector<cv::Point2f>& points,
                      const cv::Scalar& color) const
    {
        for (const auto& p : points) {
            cv::circle(canvas, toPx(p), 6, color, cv::FILLED);
        }
        if (!boundary.valid) {
            return;
        }
        // 피팅에 쓰인 구간부터 목표점 전방 거리까지 직선을 연장해 보여준다.
        const float x0 = std::max(0.0f, boundary.min_x - 0.10f);
        const float x1 = std::max(boundary.max_x, target_lookahead_);
        cv::line(canvas,
                 toPx({x0, boundary.at(x0)}),
                 toPx({x1, boundary.at(x1)}),
                 color, 1, cv::LINE_AA);
    }

    static constexpr int kArmValidFrames = 5;
    static constexpr int kMinBilateralPoints = 1;
    static constexpr const char* kDebugWindow = "Rubbercone Debug";

    float offset_gain_;
    float scan_min_range_;
    float scan_max_range_;
    float scan_max_angle_;
    float max_lateral_distance_;
    float front_ignore_angle_;
    float front_ignore_range_;
    float cone_cluster_break_;
    float cone_cluster_max_width_;
    float cone_merge_distance_;
    float side_deadband_;
    float min_forward_x_;
    int max_cone_centers_;
    int max_boundary_points_;
    float max_boundary_slope_;
    float fit_residual_limit_;
    float target_lookahead_;
    float min_target_lookahead_;
    float nominal_half_width_;
    float adaptive_half_width_;
    float min_corridor_width_;
    float max_corridor_width_;
    float offset_limit_;
    float max_offset_step_;

    int valid_frame_count_;
    int missing_frame_count_;
    bool cone_section_armed_;
    bool end_latched_;
    float filtered_target_y_;
    bool has_filtered_target_y_;
    float filtered_offset_;
    bool has_filtered_offset_;
    float offset_filter_alpha_;
    float target_filter_alpha_;
    float one_side_target_filter_alpha_;
    float max_target_y_step_bilateral_;
    float max_target_y_step_one_side_;
    int end_missing_frames_;
    int32_t rubber_offset_value_;
    int32_t rubber_end_value_;
    int32_t rubber_confidence_value_;

    bool enable_gui_;
    float px_per_m_;
    mutable std::mutex debug_mutex_;
    DebugSnapshot debug_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr info_pub_;
    rclcpp::TimerBase::SharedPtr gui_timer_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (end_latched_) {
            rubber_end_value_ = 1;
            rubber_confidence_value_ = 0;
            publishInfo();
            // 종료 래치 이후에는 경로 추정을 멈추지만, 무엇이 보이는지는 계속 표시한다.
            if (enable_gui_) {
                std::vector<cv::Point2f> raw_points;
                const auto centers = extractConeCenters(*msg, &raw_points);
                std::lock_guard<std::mutex> lock(debug_mutex_);
                debug_ = DebugSnapshot{};
                debug_.raw_points = std::move(raw_points);
                debug_.cone_centers = centers;
                debug_.end_flag = 1;
                debug_.armed = cone_section_armed_;
            }
            return;
        }

        std::vector<cv::Point2f> raw_points;
        const auto centers = extractConeCenters(*msg, enable_gui_ ? &raw_points : nullptr);
        const auto path = estimatePath(centers);
        updateDetectionState(path);
        publishInfo();

        if (enable_gui_) {
            std::lock_guard<std::mutex> lock(debug_mutex_);
            debug_.raw_points = std::move(raw_points);
            debug_.cone_centers = centers;
            debug_.path = path;
            debug_.filtered_target = cv::Point2f{path.target.x, filtered_target_y_};
            debug_.has_filtered_target = has_filtered_target_y_ && path.valid;
            debug_.half_width = adaptive_half_width_;
            debug_.offset = rubber_offset_value_;
            debug_.end_flag = rubber_end_value_;
            debug_.confidence = rubber_confidence_value_;
            debug_.armed = cone_section_armed_;
        }
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<LidarViewer>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rubbercone_main"), "예외: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
