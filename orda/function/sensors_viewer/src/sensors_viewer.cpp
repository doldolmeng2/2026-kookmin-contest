// ─────────────────────────────────────────────────────────────────────────────
// sensors_viewer.cpp
//
// 역할: 카메라와 LiDAR 데이터를 실시간으로 시각화하는 디버그 뷰어 노드
//
// 구독:
//   /resized_image (sensor_msgs/Image, BGR8)  - 640×360 카메라 영상
//   /scan          (sensor_msgs/LaserScan)    - LiDAR 거리 데이터
//
// 표시 창:
//   "Camera View"      : 카메라 영상
//   "LiDAR View (max 2m)" : 극좌표 → 직교좌표 LiDAR 포인트 맵 (최대 2m)
// ─────────────────────────────────────────────────────────────────────────────

#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>


class SensorsViewerNode : public rclcpp::Node {
public:
    SensorsViewerNode() : Node("sensors_viewer") {
        // 카메라 영상 구독 (QoS 기본값)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", 10,
            std::bind(&SensorsViewerNode::imageCallback, this, std::placeholders::_1));

        // LiDAR 스캔 구독 (SensorDataQoS: Best Effort, 최신성 우선)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&SensorsViewerNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
                    "센서 뷰어 노드 시작. /resized_image, /scan 구독 중");
    }

private:
    // 카메라 영상 콜백: BGR 그대로 imshow
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            cv::imshow("Camera View", frame);
            cv::waitKey(1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // LiDAR 스캔 콜백
    //
    // 700×700 픽셀 캔버스에 극좌표 → 직교좌표 변환 후 포인트를 점으로 그린다.
    // 중심점(차량 위치)을 빨간 점으로, 0.5m 간격 원형 가이드를 회색으로 표시한다.
    // ─────────────────────────────────────────────────────────────────────
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        const int   width     = 700;
        const int   height    = 700;
        const float max_range = 2.0f;  // 표시 최대 거리 (m)
        // 픽셀 스케일: 가장자리 50px 여유를 두고 max_range가 절반 너비에 들어오도록
        const float scale     = (width / 2 - 50) / max_range;

        cv::Mat lidar_img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::Point center(width / 2, height / 2);

        // LiDAR 포인트 그리기 (극좌표 → 직교좌표 변환)
        float angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            float r = msg->ranges[i];
            if (std::isfinite(r) && r <= max_range) {
                int x = static_cast<int>(center.x + r * scale * std::cos(angle));
                int y = static_cast<int>(center.y - r * scale * std::sin(angle));
                cv::circle(lidar_img, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), -1);
            }
            angle += msg->angle_increment;
        }

        // 차량 위치(중심점) 빨간 점으로 강조
        cv::circle(lidar_img, center, 6, cv::Scalar(0, 0, 255), -1);

        // 0.5m 간격 원형 가이드 라인 (회색)
        for (int i = 1; i <= 4; i++) {
            int radius = static_cast<int>(i * 0.5f * scale);
            cv::circle(lidar_img, center, radius, cv::Scalar(50, 50, 50), 1);
        }

        cv::imshow("LiDAR View (max 2m)", lidar_img);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr     image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorsViewerNode>());
    rclcpp::shutdown();
    return 0;
}
