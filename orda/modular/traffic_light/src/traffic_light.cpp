// ─────────────────────────────────────────────────────────────────────────────
// traffic_light.cpp
//
// 역할: 영상에서 초록색 신호등을 검출하여 주행 출발 신호를 발행한다.
//
// 알고리즘:
//   1) 프레임 오른쪽 상단 ROI (x: 6/11~끝, y: 0~1/3) 추출
//   2) ROI를 HSV로 변환하고 초록색 범위(Hue 50~150) 마스크 생성
//   3) 초록 픽셀 수 / 전체 픽셀 수 >= threshold_ratio_ 이면 초록불로 판정
//
// 구독: /resized_image (sensor_msgs/Image, BGR8)
// 발행: /traffic_detection (std_msgs/Bool)
//   true  = 초록불 감지
//   false = 미감지
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class TrafficLightDetector : public rclcpp::Node {
public:
    TrafficLightDetector() : Node("traffic_light_detector") {
        // QoS: 수치 토픽용 Best Effort + Volatile
        auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        // 결과 발행: /traffic_detection (Bool)
        pub_ = this->create_publisher<std_msgs::msg::Bool>("/traffic_detection", qos_fast);

        // 영상 구독: /resized_image (BGR8)
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", qos_fast,
            std::bind(&TrafficLightDetector::image_callback, this, std::placeholders::_1));

        // 초록 픽셀 비율 임계값: 전체 ROI 픽셀의 2% 이상이면 초록불로 판정
        threshold_ratio_ = 0.02;
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // ROS 이미지 → OpenCV BGR Mat 변환
        cv::Mat bgr;
        try {
            bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
            return;
        }

        // ROI 영역: 오른쪽 5/11 너비, 상단 1/3 높이
        // 신호등이 프레임 오른쪽 상단에 위치한다는 경험적 가정에 기반
        int h     = bgr.rows;
        int w     = bgr.cols;
        int roi_x = static_cast<int>(w * 6.0 / 11.0);  // 오른쪽 5/11 시작 x
        int roi_y = 0;
        int roi_w = w - roi_x;
        int roi_h = h / 3;
        cv::Rect roi(roi_x, roi_y, roi_w, roi_h);
        cv::Mat region = bgr(roi);

        // BGR → HSV 변환
        cv::Mat hsv;
        cv::cvtColor(region, hsv, cv::COLOR_BGR2HSV);

        // 초록색 HSV 범위: Hue 50~150, Sat 100~255, Val 100~255
        cv::Scalar lower_green(50,  100, 100);
        cv::Scalar upper_green(150, 255, 255);
        cv::Mat mask;
        cv::inRange(hsv, lower_green, upper_green, mask);

        // 초록 픽셀 비율로 신호등 감지 판정
        int  green_pixels = cv::countNonZero(mask);
        int  total_pixels = mask.rows * mask.cols;
        bool detected     = (green_pixels > total_pixels * threshold_ratio_);

        // 결과 발행
        std_msgs::msg::Bool out;
        out.data = detected;
        pub_->publish(out);

        RCLCPP_DEBUG(this->get_logger(),
                     "초록 픽셀: %d (%.2f%%) → %s",
                     green_pixels,
                     100.0 * green_pixels / total_pixels,
                     detected ? "감지" : "미감지");
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr            pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr     sub_;
    double threshold_ratio_;  // 초록불 판정 최소 픽셀 비율
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrafficLightDetector>());
    rclcpp::shutdown();
    return 0;
}
