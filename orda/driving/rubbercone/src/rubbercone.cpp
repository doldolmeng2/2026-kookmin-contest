// ─────────────────────────────────────────────────────────────────────────────
// rubbercone.cpp
//
// LiDAR 기반 라바콘 통로 추종 노드
//
// scan에서 연속 반사점을 하나의 콘 후보로 묶고, 좌/우 경계를 각각 선형으로
// 추정한다. 따라서 콘 사이 간격이나 출발 방향이 바뀌어도 "가까운 콘에서
// 일정 간격만큼" 확장하는 방식에 의존하지 않는다.
//
// 발행: rubbercone_info (std_msgs/Int32MultiArray)
//   [0] offset     : 조향 오프셋 (양수=오른쪽 편향)
//   [1] end_flag   : 0=주행 중, 1=라바콘 구간 종료
//   [2] confidence : 경로 추정 신뢰도 (0~100)
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

using std::placeholders::_1;

namespace {

constexpr float kPi = 3.14159265358979323846f;

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

}  // namespace


class LidarViewer : public rclcpp::Node {
public:
    LidarViewer()
    : Node("rubbercone"),
      offset_gain_(230.0f),
      scan_min_range_(0.18f),
      scan_max_range_(1.30f),
      front_ignore_angle_(13.0f * kPi / 180.0f),
      front_ignore_range_(0.32f),
      cone_cluster_break_(0.08f),
      cone_cluster_max_width_(0.22f),
      cone_merge_distance_(0.10f),
      side_deadband_(0.03f),
      min_forward_x_(0.04f),
      max_boundary_points_(4),
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
      rubber_confidence_value_(0)
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

        offset_gain_ = clampValue(
            static_cast<float>(declare_parameter<double>("offset_gain", 230.0)),
            100.0f, 350.0f);
        offset_limit_ = clampValue(
            static_cast<float>(declare_parameter<double>("offset_limit", 40.0)),
            20.0f, 45.0f);

        // 코스별로 달라질 수 있는 기하 파라미터는 런타임 인수로도 조정할 수 있다.
        scan_max_range_ = clampValue(
            static_cast<float>(declare_parameter<double>("scan_max_range", 1.30)),
            0.50f, 3.00f);
        target_lookahead_ = clampValue(
            static_cast<float>(declare_parameter<double>("target_lookahead", 0.70)),
            min_target_lookahead_, scan_max_range_);
        nominal_half_width_ = clampValue(
            static_cast<float>(declare_parameter<double>("nominal_half_width", 0.30)),
            0.15f, 0.70f);
        adaptive_half_width_ = nominal_half_width_;
    }

private:
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
    std::vector<cv::Point2f> extractConeCenters(const sensor_msgs::msg::LaserScan& scan) const
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
                std::abs(angle) <= (90.0f * kPi / 180.0f);

            // 가까운 전방 반사는 차체/센서 마운트일 가능성이 높다. 멀리 있는
            // 전방 콘은 보존해 반대 방향 출발 시 첫 목표점을 놓치지 않게 한다.
            const bool body_reflection =
                std::abs(angle) < front_ignore_angle_ && range < front_ignore_range_;

            if (!in_search_area || body_reflection) {
                flush_cluster();
            } else {
                const cv::Point2f point{
                    range * std::cos(angle),
                    range * std::sin(angle)};

                if (!cluster.empty() &&
                    pointDistance(point, cluster.back()) > cone_cluster_break_)
                {
                    flush_cluster();
                }
                cluster.push_back(point);
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

    static constexpr int kArmValidFrames = 5;
    static constexpr int kMinBilateralPoints = 1;

    float offset_gain_;
    float scan_min_range_;
    float scan_max_range_;
    float front_ignore_angle_;
    float front_ignore_range_;
    float cone_cluster_break_;
    float cone_cluster_max_width_;
    float cone_merge_distance_;
    float side_deadband_;
    float min_forward_x_;
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

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr info_pub_;

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (end_latched_) {
            rubber_end_value_ = 1;
            rubber_confidence_value_ = 0;
            publishInfo();
            return;
        }

        const auto centers = extractConeCenters(*msg);
        const auto path = estimatePath(centers);
        updateDetectionState(path);
        publishInfo();
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
