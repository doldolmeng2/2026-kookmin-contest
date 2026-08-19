// ─────────────────────────────────────────────────────────────────────────────
// hls_hsv_tuner.cpp
//
// 역할: HLS / HSV / YCrCb 색공간 임계값 튜닝 도구
//   /image_raw 토픽에서 영상을 수신하고, OpenCV 트랙바를 통해
//   HLS 및 HSV 임계값을 실시간으로 조절하며 결과 마스크를 확인한다.
//
//   lane_detection_parameter.json의 yellow_hls_*/yellow_hsv_* 값을
//   현장에서 조정할 때 사용한다.
//
// 구독: /image_raw (sensor_msgs/Image, BGR8)
// 표시: HLS Mask, HSV Mask, Combined Mask (트랙바 창)
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

using namespace std;
using namespace cv;

// ── HLS 임계값 (트랙바로 조절) ───────────────────────────────────────────────
int h_hls_low  = 0,   l_hls_low  = 0,   s_hls_low  = 0;
int h_hls_high = 179, l_hls_high = 255, s_hls_high = 255;

// ── HSV 임계값 (트랙바로 조절) ───────────────────────────────────────────────
int h_hsv_low  = 0,   s_hsv_low  = 0,   v_hsv_low  = 0;
int h_hsv_high = 179, s_hsv_high = 255, v_hsv_high = 255;

// ── YCrCb 임계값 (OpenCV 채널 순서: Y, Cr, Cb) ─────────────────────────────
int y_ycc_low  = 0,   cr_ycc_low  = 0,   cb_ycc_low  = 0;
int y_ycc_high = 255, cr_ycc_high = 255, cb_ycc_high = 255;

// 트랙바 콜백: 값 변경을 즉시 반영하기 위해 선언만 함 (본문 불필요)
void on_trackbar(int, void*) {}


class HSLHSVTunerNode : public rclcpp::Node {
public:
    HSLHSVTunerNode() : Node("hls_hsv_tuner_node") {
        // /image_raw 구독 (QoS 기본값 사용)
        // 카메라/리사이즈 영상은 보통 BEST_EFFORT로 발행되므로
        // SensorDataQoS를 사용해야 QoS 불일치로 콜백이 끊기지 않는다.
        auto image_qos = rclcpp::SensorDataQoS().best_effort();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw",
            image_qos,
            std::bind(
                &HSLHSVTunerNode::imageCallback,
                this,
                std::placeholders::_1
            )
        );

        // HLS 트랙바 창 생성
        namedWindow("HLS Mask", WINDOW_NORMAL);
        resizeWindow("HLS Mask", 640, 360);
        createTrackbar("H low",  "HLS Mask", &h_hls_low,  179, on_trackbar);
        createTrackbar("H high", "HLS Mask", &h_hls_high, 179, on_trackbar);
        createTrackbar("L low",  "HLS Mask", &l_hls_low,  255, on_trackbar);
        createTrackbar("L high", "HLS Mask", &l_hls_high, 255, on_trackbar);
        createTrackbar("S low",  "HLS Mask", &s_hls_low,  255, on_trackbar);
        createTrackbar("S high", "HLS Mask", &s_hls_high, 255, on_trackbar);

        // HSV 트랙바 창 생성
        namedWindow("HSV Mask", WINDOW_NORMAL);
        resizeWindow("HSV Mask", 640, 360);
        createTrackbar("H low",  "HSV Mask", &h_hsv_low,  179, on_trackbar);
        createTrackbar("H high", "HSV Mask", &h_hsv_high, 179, on_trackbar);
        createTrackbar("S low",  "HSV Mask", &s_hsv_low,  255, on_trackbar);
        createTrackbar("S high", "HSV Mask", &s_hsv_high, 255, on_trackbar);
        createTrackbar("V low",  "HSV Mask", &v_hsv_low,  255, on_trackbar);
        createTrackbar("V high", "HSV Mask", &v_hsv_high, 255, on_trackbar);

        // YCrCb 트랙바 창 생성
        namedWindow("YCrCb Mask", WINDOW_NORMAL);
        resizeWindow("YCrCb Mask", 640, 360);
        createTrackbar("Y low",   "YCrCb Mask", &y_ycc_low,   255, on_trackbar);
        createTrackbar("Y high",  "YCrCb Mask", &y_ycc_high,  255, on_trackbar);
        createTrackbar("Cr low",  "YCrCb Mask", &cr_ycc_low,  255, on_trackbar);
        createTrackbar("Cr high", "YCrCb Mask", &cr_ycc_high, 255, on_trackbar);
        createTrackbar("Cb low",  "YCrCb Mask", &cb_ycc_low,  255, on_trackbar);
        createTrackbar("Cb high", "YCrCb Mask", &cb_ycc_high, 255, on_trackbar);

        // 세 색공간 중 2개 이상이 동의한 결과 창
        namedWindow("Combined 2of3", WINDOW_NORMAL);
        resizeWindow("Combined 2of3", 640, 360);

        // OpenCV 창 이벤트를 영상 콜백과 분리해서 처리한다.
        // 영상이 아직 오지 않아도 창이 생성되고 응답하도록 한다.
        gui_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            []() {
                cv::waitKey(1);
            }
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for images on /image_raw (remap allowed)"
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr gui_timer_;
    bool first_frame_received_ = false;

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // ROS 이미지 → OpenCV BGR Mat 변환
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
            return;
        }

        Mat frame = cv_ptr->image;
        if (frame.empty()) {
            return;
        }

        if (!first_frame_received_) {
            first_frame_received_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "First image received: %dx%d",
                frame.cols,
                frame.rows
            );
        }

        resize(frame, frame, Size(640, 360));

        // HLS 마스크 생성 (OpenCV HLS 채널 순서: H, L, S)
        Mat hls, mask_hls;
        cvtColor(frame, hls, COLOR_BGR2HLS);
        inRange(hls,
                Scalar(h_hls_low, l_hls_low, s_hls_low),
                Scalar(h_hls_high, l_hls_high, s_hls_high),
                mask_hls);

        // HSV 마스크 생성 (OpenCV HSV 채널 순서: H, S, V)
        Mat hsv, mask_hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv,
                Scalar(h_hsv_low, s_hsv_low, v_hsv_low),
                Scalar(h_hsv_high, s_hsv_high, v_hsv_high),
                mask_hsv);

        // YCrCb 마스크 생성 (채널 순서: Y, Cr, Cb)
        Mat ycrcb, mask_ycc;
        cvtColor(frame, ycrcb, COLOR_BGR2YCrCb);
        inRange(ycrcb,
                Scalar(y_ycc_low, cr_ycc_low, cb_ycc_low),
                Scalar(y_ycc_high, cr_ycc_high, cb_ycc_high),
                mask_ycc);

        // 2-of-3:
        // (HLS ∩ HSV) ∪ (HSV ∩ YCrCb) ∪ (HLS ∩ YCrCb)
        Mat hls_hsv, hsv_ycc, hls_ycc, combined;
        bitwise_and(mask_hls, mask_hsv, hls_hsv);
        bitwise_and(mask_hsv, mask_ycc, hsv_ycc);
        bitwise_and(mask_hls, mask_ycc, hls_ycc);

        bitwise_or(hls_hsv, hsv_ycc, combined);
        bitwise_or(combined, hls_ycc, combined);

        // 검정 영역 억제
        vector<Mat> ycc_channels;
        split(ycrcb, ycc_channels);
        Mat y_gate = ycc_channels[0] > 5;
        y_gate.convertTo(y_gate, CV_8U, 255);
        bitwise_and(combined, y_gate, combined);

        imshow("HLS Mask",      mask_hls);
        imshow("HSV Mask",      mask_hsv);
        imshow("YCrCb Mask",    mask_ycc);
        imshow("Combined 2of3", combined);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HSLHSVTunerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
