// ─────────────────────────────────────────────────────────────────────────────
// resize.cpp
//
// 역할: 카메라 원본 영상을 640×360으로 리사이즈하여 재발행하는 전처리 노드
//
// 구독: image_raw   (sensor_msgs/Image, BGR8) - 카메라 원본 영상
// 발행: resized_image (sensor_msgs/Image, BGR8) - 리사이즈 결과
//
// 용도: 각 처리 노드(lane_detection, object_detection, traffic_light)가
//       고해상도 원본 대신 640×360 영상을 구독함으로써 처리 부하를 줄인다.
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class ResizeNode : public rclcpp::Node {
public:
    ResizeNode() : Node("resize_node") {
        // QoS: Best Effort + Volatile (이미지 토픽 표준 설정)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.best_effort();
        qos.durability_volatile();

        // 리사이즈 결과 발행: resized_image
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("resized_image", qos);

        // 원본 영상 구독: image_raw
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos,
            std::bind(&ResizeNode::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Resize 노드 시작 (QoS: BestEffort, 출력: 640×360)");
    }

private:
    void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat input_image = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // 640×360으로 리사이즈 (종횡비 유지: 원본이 16:9 가정)
            cv::Mat resized;
            cv::resize(input_image, resized, cv::Size(640, 360));

            // 원본 헤더(타임스탬프, frame_id) 유지하여 발행
            auto output_msg = cv_bridge::CvImage(msg->header, "bgr8", resized).toImageMsg();
            pub_->publish(*output_msg);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "리사이즈 오류: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ResizeNode>());
    rclcpp::shutdown();
    return 0;
}
