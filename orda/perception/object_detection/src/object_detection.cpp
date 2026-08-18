// ─────────────────────────────────────────────────────────────────────────────
// object_detection.cpp
//
// 역할: LiDAR 클러스터링 + YOLO 바운딩 박스를 결합한 장애물 감지 노드
//
// 구독:
//   /scan              (sensor_msgs/LaserScan)           - LiDAR 거리 데이터 (디버그 표시용)
//   /resized_image     (sensor_msgs/Image, BGR8)          - 신호등 크롭 분류 원본 + 디버그 영상
//   /object_yolo       (std_msgs/Float32MultiArray)       - ONNX Runtime 차량 검출(가장 가까운 1개)
//   /traffic_boxes     (std_msgs/Float32MultiArray)       - ONNX Runtime 신호등 "위치" 후보 박스(전부)
//   /lane_fit          (std_msgs/Float32MultiArray [m,b]) - 차선 회귀 결과
//
// 신호등 인식은 traffic_node 도, Python 쪽 분류기도 거치지 않고 이 노드가
// 직접 한다: /traffic_boxes 로 들어오는 박스마다 /resized_image 에서 직접
// 잘라(ROI 크롭) light_cls.onnx(cv::dnn)로 분류하고, 그 결과를 모아 우선순위
// (좌회전 > 직진 > 정지) 판정 + 디바운스(debounce_frames 연속)를 계산한다.
//
// /traffic_boxes 의 class_id/confidence 는 object_yolo_node.py 검출기가 매긴
// 값이라 색 판정에는 안 쓴다 — "신호등처럼 생긴 위치" 후보로만 쓰고 실제 색은
// 위 크롭 분류가 다시 정한다. 검출기의 신호등 클래스 판정은 거리에 따라
// 뒤집혀 신뢰할 수 없었다(홀드아웃 클래스 정확도 86.9% -> 크롭 분류 98.3%).
// 분류 확신도가 낮은 박스는 증거로 안 쓴다(light_min_confidence_, 기본 0.90) —
// 자세한 배경은 README 참고.
//
// 발행:
//   /object_info (std_msgs/Int32MultiArray, 3개 필드) — FSM 이 구독하는
//   유일한 최종 출력. 신호등/고정차량/방해차량 정보를 이 하나로 통합해서
//   낸다 (별도 /traffic_detection 토픽은 발행하지 않는다).
//   [신호등 정보, 고정차량 위치, 방해차량 위치]
//     신호등 정보:   0=인식x  1=빨간불/주황불(정지)  2=직진(초록)  3=좌회전
//     고정차량 위치: 0=인식x  1=1차선  2=2차선   (object_type=FIXED 일 때만)
//     방해차량 위치: 0=인식x  1=1차선  2=2차선   (object_type=MOVING 일 때만)
//   차량 위치 둘은 /object_yolo 로 받은 최신 박스(lane_label)를 object_type 에
//   따라 갈라 넣는다. 각 값 모두 box_max_age_s_ 보다 오래되면 0(인식x)으로 리셋된다.
//
//   /object_info_raw (std_msgs/Float32MultiArray, 12개 필드) [내부 상세]
//   [exists, min_dist, angle, span, cluster_size,
//    box_size, box_cx, box_cy, dx, car_lane, object_type, confidence]
//     car_lane   : 0=중앙, 1=왼쪽, 2=오른쪽
//     object_type: -1=미확정, 0=고정장애물, 1=방해차량
//   /object_info 계산에 실제로 쓰인 원시값을 그대로 낸다 — box_*, car_lane,
//   object_type, confidence 는 box_max_age_s_ 스테일 리셋을 적용하지 않은
//   값이다(/object_info 의 fixed_lane/moving_lane 은 리셋된다). 디버깅·로깅
//   용이고 FSM 계약과는 무관하다.
//
// 디버그 창 (enable_gui=true 시):
//   "OBJECT DEBUG" : exists / distance / cluster_size (LiDAR 클러스터링, 표시 전용)
//   "CAMERA VIEW"  : 입력 영상 + 차량/신호등 박스 오버레이
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <string>
#include <mutex>
#include <limits>
#include <cmath>
#include <vector>
#include <iomanip>
#include <sstream>

using std::placeholders::_1;


class ObjectDetectionNode : public rclcpp::Node {
public:
    ObjectDetectionNode() : Node("object_node") {
        // ── ROS 파라미터 선언 ────────────────────────────────────────────
        // LiDAR 전방 시야각 (±front_fov_deg 범위만 검사)
        front_fov_deg_      = this->declare_parameter<double>("front_fov_deg",       10.0);
        // 유효 거리 범위 (m)
        range_min_m_        = this->declare_parameter<double>("range_min_m",          0.05);
        range_max_m_        = this->declare_parameter<double>("range_max_m",          2.0);
        // DBSCAN 클러스터링 epsilon (m): 연속 포인트 간 최대 거리 차
        cluster_epsilon_m_  = this->declare_parameter<double>("cluster_epsilon_m",    0.20);
        // 클러스터로 인정할 최소 포인트 수.
        // 5는 너무 빡빡했다. 1.5~2 m 앞의 차는 ±10도 창에 4~7점밖에 안 맺혀서
        // 반사가 한두 개만 빠져도 클러스터가 통째로 탈락했다. bag 실측(전방에
        // 차가 있는 구간)에서 검출률 32.4% → 89.7%, 최장 끊김 74스캔 → 18스캔.
        // 앞차가 없는 주행 bag의 오검출은 8~11% → 11~20%로 소폭만 늘었다.
        min_cluster_points_ = this->declare_parameter<int>("min_cluster_points",      3);
        // 장애물 존재로 판단할 최대 거리 (m)
        detect_threshold_m_ = this->declare_parameter<double>("detect_threshold_m",   6.0);
        // OpenCV imshow 디버그 창 활성화 여부.
        //
        // 기본값을 false로 둔다. "OBJECT DEBUG"/"CAMERA VIEW" 두 창을 30 Hz로
        // 갱신하면 같은 프로세스의 YOLO 추론과 CPU를 다투고, 그만큼 /object_info
        // 와 /lane_offset 이 늦어진다. 실측(bag): 카메라는 18.8 Hz 로 들어오는데
        // 인지 결과는 5.9 Hz 로 나왔다. 필요할 때만 enable_gui:=true 로 켠다.
        enable_gui_         = this->declare_parameter<bool>("enable_gui",             false);
        // 박스 중심이 중앙선에서 ±lane_split_margin_px_ 이상 벗어나면 차선 레이블 부여.
        //
        // 10 px 은 /lane_fit 의 흔들림보다 작아서 사실상 데드밴드가 없었다.
        // rosbag2_2026_08_13-09_30_09 실측: 같은 방해차량을 보는 동안 box_cx 는
        // 155.5 -> 159.0 으로 거의 안 움직였는데 x_line 이 약 68 px 튀어
        // dx 가 -41 -> +31 로 부호까지 뒤집혔다(car_lane 1 -> 2). 지금은 마진을
        // 다시 10px로 낮추는 대신, 아래 lane_debounce_frames_ 로 순간 튐을 걸러
        // 낸다 — raw 판정이 lane_debounce_frames_ 프레임 연속 같아야 확정한다.
        lane_split_margin_px_ = this->declare_parameter<double>("lane_split_margin_px", 10.0);
        // 차선 판정 디바운스: raw lane_label 이 이 프레임 수만큼 연속 같아야
        // last_lane_label_ 에 반영한다. margin 을 좁힌 대신, 노이즈로 한두
        // 프레임만 튀는 값이 그대로 나가지 않도록 막는다.
        lane_debounce_n_ = this->declare_parameter<int>("lane_debounce_frames", 3);
        if (lane_debounce_n_ < 1) lane_debounce_n_ = 1;
        // /lane_fit 이 이보다 오래되면 차선 판정에 쓰지 않는다. 낡은 회귀선으로
        // 계산한 x_line 은 박스가 그대로여도 dx 부호를 바꿔 놓는다.
        lane_fit_max_age_s_ = this->declare_parameter<double>("lane_fit_max_age_s",    0.5);
        // YOLO 결과가 이보다 오래되면 박스 필드를 0으로 내보낸다.
        //
        // /object_info 는 50 Hz 타이머로 나가는데 YOLO 는 그보다 훨씬 느리다.
        // 갱신되지 않은 박스를 계속 재발행하면 소비자는 "방금 본 장애물"과
        // "300 ms 전에 본 장애물"을 구분할 수 없고, 카메라가 아예 죽어도
        // 마지막 박스가 영원히 살아 있는 것처럼 보인다. 0을 내보내면 기존
        // 계약 그대로 "이 프레임에는 카메라 증거 없음"으로 읽힌다.
        box_max_age_s_      = this->declare_parameter<double>("box_max_age_s",         0.5);
        // lane_fit이 프레임 좌표계면 true (BEV 좌표계면 false)
        lane_fit_is_frame_  = this->declare_parameter<bool>("lane_fit_is_frame",     true);
        // YOLO 모델(.onnx) 경로. 빈 문자열이면 share 디렉터리를 탐색한다.
        model_path_         = this->declare_parameter<std::string>("model_path",     "");
        // J4012의 OpenCV 4.5.4 DNN은 현재 Ultralytics ONNX forward에서
        // shape assertion으로 실패한다. 기본 경로는 검증된 Python ONNX Runtime
        // 노드(/object_yolo)를 사용하고, 옛 C++ DNN은 명시적으로 끈다.
        use_external_yolo_  = this->declare_parameter<bool>("use_external_yolo",    true);
        // 신호등 코드 디바운스: 같은 raw 코드가 이 프레임 수만큼 연속돼야 확정한다.
        // (예전 traffic_node 의 debounce_frames 기본값과 동일)
        debounce_n_ = this->declare_parameter<int>("debounce_frames", 3);
        if (debounce_n_ < 1) debounce_n_ = 1;
        // ── 신호등 크롭 분류기 파라미터 ─────────────────────────────────
        // 빈 문자열이면 share 디렉터리(model/light_cls.onnx)를 탐색한다.
        // best.onnx 가 실제로 로드되는 경로(Python object_yolo_node.py 의
        // share 디렉터리 기본값 + model_path 오버라이드)와 같은 패턴이다.
        // 아래 resolveModelPath() 의 하드코딩된 개발PC/실차 경로 후보는 예전
        // xycar_ws 워크스페이스를 가리키는 죽은 코드(use_external_yolo_ 기본
        // true 라 실제로는 안 쓰인다)라 새로 참조하지 않았다.
        light_classifier_path_ = this->declare_parameter<std::string>("light_classifier_path", "");
        light_crop_margin_     = this->declare_parameter<double>("light_crop_margin", 0.15);
        light_input_size_      = this->declare_parameter<int>("light_input_size", 64);
        // 이 값 아래면 "판단 보류"(증거로 안 씀)로 처리한다. traffic2~5 bag
        // 검증에서 오답과 정답이 확신도로 거의 완전히 갈렸다(정답 25%분위
        // 0.998, 오답 최대 0.889). 자세한 배경은 README 참고.
        light_min_confidence_  = this->declare_parameter<double>("light_min_confidence", 0.90);


        // QoS 프로파일
        // - qos_fast: 수치 토픽용 Best Effort + Volatile
        auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        // LiDAR 스캔 구독
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&ObjectDetectionNode::onScan, this, _1));

        // 카메라 영상 구독
        sub_img_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/resized_image", qos_fast,
            std::bind(&ObjectDetectionNode::onImage, this, _1));

        // 차선 회귀 파라미터 구독
        sub_lane_fit_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/lane_fit", qos_fast,
            std::bind(&ObjectDetectionNode::onLaneFit, this, _1));

        sub_yolo_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/object_yolo", qos_fast,
            std::bind(&ObjectDetectionNode::onYoloDetection, this, _1));

        // 신호등 박스 수신. 여기서 상태(0~3)를 직접 계산해 /object_info 의
        // 첫 필드로만 낸다 (traffic_node 를 거치지도, 별도 토픽으로 내지도 않는다).
        sub_traffic_boxes_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/traffic_boxes", qos_fast,
            std::bind(&ObjectDetectionNode::onTrafficBoxes, this, _1));

        // 장애물 정보 발행 [신호등, 고정차량 위치, 방해차량 위치]
        pub_obj_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/object_info", qos_fast);
        // 내부 상세 12필드 — /object_info 계산에 쓰인 원시값을 그대로 낸다
        // (스테일 리셋 미적용). 디버깅/로깅용, FSM 은 이 토픽을 구독하지 않는다.
        pub_obj_raw_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/object_info_raw", qos_fast);

        // ── YOLO 모델 초기화 ────────────────────────────────────────────
        // 모델 경로는 개발 PC / 실차 두 곳을 순서대로 확인한다 (resolveModelPath).
        // 예전에는 실차 경로 하나만 박혀 있어서, 개발 PC에서 재생할 때 모델
        if (use_external_yolo_) {
            yolo_ok_ = false;
            RCLCPP_INFO(this->get_logger(),
                        "외부 ONNX Runtime YOLO 입력(/object_yolo)을 사용합니다.");
        } else {
            const std::string resolved = resolveModelPath();
            if (resolved.empty()) {
                RCLCPP_ERROR(this->get_logger(),
                             "YOLO 모델(.onnx)을 찾지 못했습니다. "
                             "model_path 파라미터로 경로를 직접 지정하세요.");
                yolo_ok_ = false;
            } else {
                try {
                    net_ = cv::dnn::readNet(resolved);
                    net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                    net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                    yolo_ok_ = true;
                    RCLCPP_INFO(this->get_logger(), "YOLO 로드 완료: %s", resolved.c_str());
                } catch (const cv::Exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "YOLO 로드 실패: %s", e.what());
                    yolo_ok_ = false;
                }
            }
        }

        // ── 신호등 크롭 분류기 초기화 ────────────────────────────────────
        // 이전에는 object_yolo_node.py 가 onnxruntime 으로 이 모델을 불러와
        // 크롭·분류까지 했다. YOLOv8 검출기와 달리 이건 텐서 1개짜리 단순
        // 분류 헤드라 cv::dnn 이 문제없이 돌린다 — 실차와 같은 OpenCV 4.5.4로
        // 직접 확인했다(출력 합 1.0, softmax 내장). 위 YOLO 검출기가 cv::dnn
        // 을 피해 Python 으로 간 이유(shape assertion)는 검출 후처리 특유의
        // 문제라 여기엔 해당하지 않는다.
        {
            std::string light_path = light_classifier_path_;
            if (light_path.empty()) {
                try {
                    light_path = ament_index_cpp::get_package_share_directory("object_detection")
                               + "/model/light_cls.onnx";
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "object_detection share 디렉터리를 찾지 못했습니다: %s", e.what());
                }
            }
            std::ifstream lf(light_path, std::ios::binary);
            if (!light_path.empty() && lf.good()) {
                try {
                    light_net_ = cv::dnn::readNet(light_path);
                    light_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                    light_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                    light_ok_ = true;
                    RCLCPP_INFO(this->get_logger(),
                                "신호등 크롭 분류기 로드 완료: %s (min_conf=%.2f)",
                                light_path.c_str(), light_min_confidence_);
                } catch (const cv::Exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "신호등 분류기 로드 실패: %s", e.what());
                    light_ok_ = false;
                }
            } else {
                // 모델이 없어도 차량 경로는 그대로 돈다 — 파이프라인 절반이
                // 빠졌다고 전체가 죽는 것보다 낫다. 이 경우 신호등 상태는
                // 항상 0(인식x)으로 나간다.
                RCLCPP_WARN(this->get_logger(),
                            "신호등 분류기 모델이 없어 신호등 인식을 하지 않습니다: %s",
                            light_path.c_str());
                light_ok_ = false;
            }
        }

        // 재학습 모델(빨간 고정 방해차량) 기준. 검증용 bag에서 이 값이 검출
        // 가능한 프레임의 99%를 잡고, 차가 없는 프레임의 오검출은 0이었다.
        // 이전 0.83은 옛 모델 최고 conf가 0.82라 사실상 전부 버리고 있었다.
        conf_threshold_ = 0.50f;  // 신뢰도 임계값
        nms_threshold_  = 0.40f;  // NMS IoU 임계값
        min_w_pix_      = 12;     // 유효 박스 최소 너비 (px)
        min_h_pix_      = 12;     // 유효 박스 최소 높이 (px)

        // ── 디버그 창 및 타이머 설정 ────────────────────────────────────
        if (enable_gui_) {
            cv::namedWindow("OBJECT DEBUG", cv::WINDOW_AUTOSIZE);
            cv::namedWindow("CAMERA VIEW",  cv::WINDOW_AUTOSIZE);
            // 디버그 창 갱신 타이머 (~30Hz)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(33),
                std::bind(&ObjectDetectionNode::onTimer, this));
        }

        // 장애물 정보 발행 타이머 (50Hz)
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&ObjectDetectionNode::onPublishTick, this));
    }

    ~ObjectDetectionNode() override {
        if (enable_gui_) {
            cv::destroyWindow("OBJECT DEBUG");
            cv::destroyWindow("CAMERA VIEW");
        }
    }

private:
    // ─────────────────────────────────────────────────────────────────────
    // YOLO 모델 경로 결정
    //
    // 1) model_path 파라미터가 지정되면 그대로 사용
    // 2) 없으면 아래 후보 경로를 순서대로 확인한다
    // 읽을 수 있는 파일이 없으면 빈 문자열을 반환한다.
    //
    // ★ 새 PC에서 쓰려면 MODEL_PATH_CANDIDATES에 그 PC의 경로를 추가해야 한다.
    //   임시로는 model_path 파라미터로 넘겨도 된다:
    //   ros2 run object_detection object_node --ros-args -p model_path:=<경로>
    // ─────────────────────────────────────────────────────────────────────
    std::string resolveModelPath() {
        // 소스 트리를 직접 가리키므로 모델을 교체하면 재빌드 없이 반영된다.
        static const std::vector<std::string> MODEL_PATH_CANDIDATES = {
            // 개발 PC
            "/home/dxer0/xycar_ws/src/orda/perception/object_detection/best.onnx",
            // 실차 (Xycar)
            "/home/xytron/xycar_ws/src/orda/perception/object_detection/best.onnx",
        };

        auto readable = [](const std::string& p) {
            if (p.empty()) return false;
            std::ifstream f(p, std::ios::binary);
            return f.good();
        };

        if (!model_path_.empty()) {
            if (readable(model_path_)) return model_path_;
            RCLCPP_ERROR(this->get_logger(),
                         "model_path로 지정된 파일을 열 수 없습니다: %s",
                         model_path_.c_str());
            return "";
        }

        for (const auto& candidate : MODEL_PATH_CANDIDATES)
            if (readable(candidate)) return candidate;

        for (const auto& candidate : MODEL_PATH_CANDIDATES)
            RCLCPP_ERROR(this->get_logger(), "  후보 경로 없음: %s", candidate.c_str());
        return "";
    }

    // ─────────────────────────────────────────────────────────────────────
    // 차선 회귀 파라미터 수신 콜백
    // /lane_fit [m, b] 를 수신하여 뮤텍스로 보호된 멤버에 저장한다.
    // ─────────────────────────────────────────────────────────────────────
    void onLaneFit(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;
        std::lock_guard<std::mutex> lk(mtx_lane_);
        fit_m_     = msg->data[0];
        fit_b_     = msg->data[1];
        fit_valid_ = std::isfinite(fit_m_) && std::isfinite(fit_b_);
        fit_stamp_ = now();
    }

    // /object_yolo 계약:
    // [detected, object_type, confidence, box_area, cx, cy, x, y, w, h]
    void onYoloDetection(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (!use_external_yolo_) return;
        if (msg->data.size() < 10) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "/object_yolo 필드가 10개보다 적습니다.");
            return;
        }

        const bool detected = msg->data[0] >= 0.5f;
        const int object_type = static_cast<int>(std::lround(msg->data[1]));
        const float confidence = msg->data[2];
        const float box_area = msg->data[3];
        const float box_cx = msg->data[4];
        const float box_cy = msg->data[5];
        if (!std::isfinite(confidence) || !std::isfinite(box_area) ||
            !std::isfinite(box_cx) || !std::isfinite(box_cy) ||
            (detected && object_type != 0 && object_type != 1)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "유효하지 않은 /object_yolo 메시지를 무시합니다.");
            return;
        }

        float box_dx = 0.0f;
        int lane_label = 0;
        float m = 0.0f, b = 0.0f;
        bool lane_ok = false;
        rclcpp::Time fit_stamp{0, 0, RCL_ROS_TIME};
        {
            std::lock_guard<std::mutex> lk(mtx_lane_);
            lane_ok = fit_valid_;
            m = fit_m_;
            b = fit_b_;
            fit_stamp = fit_stamp_;
        }
        if (lane_ok && fit_stamp.nanoseconds() > 0) {
            const double age_s = (now() - fit_stamp).seconds();
            lane_ok = age_s >= 0.0 && age_s <= lane_fit_max_age_s_;
        }
        if (detected && lane_ok && lane_fit_is_frame_) {
            const float x_line = m * box_cy + b;
            box_dx = box_cx - x_line;
            if (box_dx <= -static_cast<float>(lane_split_margin_px_)) lane_label = 1;
            else if (box_dx >= static_cast<float>(lane_split_margin_px_)) lane_label = 2;
        }

        // 차선 판정 디바운스: margin(10px)이 좁아진 만큼 /lane_fit 흔들림으로
        // raw lane_label 이 한두 프레임 튈 수 있다. 같은 값이
        // lane_debounce_n_ 프레임 연속돼야 lane_stable_ 에 반영한다.
        if (lane_label == lane_cand_) lane_cnt_++;
        else { lane_cand_ = lane_label; lane_cnt_ = 1; }
        if (lane_cnt_ >= lane_debounce_n_) lane_stable_ = lane_cand_;

        // 화면에는 여기서 직접 그리지 않는다. onYoloDetection/onTrafficBoxes
        // 콜백은 카메라 프레임과 비동기로, 서로 다른 빈도로 들어와서 last_img_에
        // 바로 그리면 지우는 시점이 없어 이전 박스가 남아 "박스 2개"로 겹쳐
        // 보이는 문제가 있었다. 대신 최신 상태만 저장해두고, onTimer()가
        // 매 렌더 틱마다 원본 프레임에서 다시 그린다 (누적 없음).
        const float box_x = detected ? msg->data[6] : 0.0f;
        const float box_y = detected ? msg->data[7] : 0.0f;
        const float box_w = detected ? msg->data[8] : 0.0f;
        const float box_h = detected ? msg->data[9] : 0.0f;
        const bool  geom_ok = std::isfinite(box_x) && std::isfinite(box_y) &&
                              std::isfinite(box_w) && std::isfinite(box_h);

        std::lock_guard<std::mutex> lk(mtx_box_);
        last_box_area_pix_ = detected ? box_area : 0.0f;
        last_box_cx_ = detected ? box_cx : 0.0f;
        last_box_cy_ = detected ? box_cy : 0.0f;
        last_box_dx_ = detected ? box_dx : 0.0f;
        last_box_x_  = (detected && geom_ok) ? box_x : 0.0f;
        last_box_y_  = (detected && geom_ok) ? box_y : 0.0f;
        last_box_w_  = (detected && geom_ok) ? box_w : 0.0f;
        last_box_h_  = (detected && geom_ok) ? box_h : 0.0f;
        last_lane_label_ = lane_stable_;
        last_object_type_ = detected ? object_type : -1;
        last_object_confidence_ = detected ? confidence : 0.0f;
        last_box_stamp_ = now();
    }

    // ─────────────────────────────────────────────────────────────────────
    // 신호등 박스 하나를 잘라(ROI 크롭) 분류기에 넣는다.
    //
    // 예전 object_detection/light_classifier.py 의 crop_with_margin() +
    // classify_crop() 을 그대로 옮긴 것. 전처리(정사각 stretch, INTER_CUBIC,
    // BGR->RGB, /255, CHW)가 학습 크롭 생성과 반드시 같아야 하므로 순서를
    // 그대로 지켰다.
    //
    // NAMES 순서(0=green 1=left_green 2=orange 3=red)는 학습 시 폴더명 정렬
    // 순서 = onnx 출력 인덱스다. 재학습해서 순서가 바뀌면 아래 스위치문(호출
    // 쪽)도 같이 고쳐야 한다.
    // ─────────────────────────────────────────────────────────────────────
    bool classifyLight(const cv::Rect& box, int& out_index, float& out_score) {
        if (!light_ok_) return false;

        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lk(mtx_raw_img_);
            if (last_raw_img_.empty()) return false;
            frame = last_raw_img_;  // 얕은 복사: 아래에서 읽기만 한다
        }

        const double grow_x = box.width  * light_crop_margin_ * 0.5;
        const double grow_y = box.height * light_crop_margin_ * 0.5;
        int x1 = static_cast<int>(std::lround(box.x - grow_x));
        int y1 = static_cast<int>(std::lround(box.y - grow_y));
        int x2 = static_cast<int>(std::lround(box.x + box.width  + grow_x));
        int y2 = static_cast<int>(std::lround(box.y + box.height + grow_y));
        x1 = std::max(0, std::min(x1, frame.cols - 1));
        y1 = std::max(0, std::min(y1, frame.rows - 1));
        x2 = std::max(x1 + 1, std::min(x2, frame.cols));
        y2 = std::max(y1 + 1, std::min(y2, frame.rows));
        cv::Mat crop = frame(cv::Rect(x1, y1, x2 - x1, y2 - y1));

        cv::Mat resized;
        cv::resize(crop, resized, cv::Size(light_input_size_, light_input_size_),
                   0, 0, cv::INTER_CUBIC);
        cv::Mat blob = cv::dnn::blobFromImage(
            resized, 1.0 / 255.0, cv::Size(light_input_size_, light_input_size_),
            cv::Scalar(0, 0, 0), /*swapRB=*/true, /*crop=*/false);

        cv::Mat out;
        try {
            light_net_.setInput(blob);
            out = light_net_.forward();
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "신호등 분류 추론 실패: %s", e.what());
            return false;
        }

        double max_val;
        cv::Point max_loc;
        cv::minMaxLoc(out.reshape(1, 1), nullptr, &max_val, nullptr, &max_loc);
        out_index = max_loc.x;
        out_score = static_cast<float>(max_val);
        return out_index >= 0 && out_index < 4;
    }

    // ─────────────────────────────────────────────────────────────────────
    // 신호등 박스 수신 콜백. 박스마다 크롭 분류(classifyLight)를 돌리고,
    // 그 결과로 표시용 저장(enable_gui 일 때만) + 상태 계산을 여기서 직접
    // 한다 — traffic_node 도, Python 쪽 분류기도 거치지 않는다.
    //
    // 메시지 형식: 6개씩 반복 [class_id, confidence, x, y, w, h]. x/y/w/h만
    // 쓴다 — class_id/confidence 는 object_yolo_node.py 검출기가 매긴 값이라
    // 신뢰할 수 없다(홀드아웃 클래스 정확도 86.9%, 위치만 100%). 그래서
    // "신호등처럼 생긴 위치" 후보로만 쓰고, 실제 색 판정은 아래 크롭 분류로
    // 다시 한다.
    //
    // 분류 확신도가 light_min_confidence_ 아래면 그 박스는 증거로 안 쓴다
    // (raw=0 쪽으로 흘러간다). traffic2~5 bag 검증에서 이 게이트 하나로
    // 화면 상단에 잘려 붙은 과폭 박스발 오답을 전부 걸렀다 — 자세한 배경은
    // README 참고.
    //
    // 상태 판정 우선순위: 좌회전(3) > 직진(2) > 정지(1) > 없음(0).
    // 좌회전 화살표는 초록 원과 함께 켜지는 경우가 많아, left 를 먼저 봐야
    // 좌회전 상황이 단순 직진으로 뭉개지지 않는다 (traffic_node 원래 로직 이식).
    // 같은 raw 코드가 debounce_n_ 프레임 연속돼야 stable 로 확정한다.
    // ─────────────────────────────────────────────────────────────────────
    void onTrafficBoxes(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::vector<TrafficBoxResult> results;
        bool tl_green = false, tl_left = false, tl_red = false, tl_orange = false;

        for (size_t i = 0; i + 6 <= msg->data.size(); i += 6) {
            const float x = msg->data[i + 2];
            const float y = msg->data[i + 3];
            const float w = msg->data[i + 4];
            const float h = msg->data[i + 5];
            if (!std::isfinite(x) || !std::isfinite(y) ||
                !std::isfinite(w) || !std::isfinite(h)) continue;
            if (w <= 0.f || h <= 0.f) continue;

            cv::Rect box(static_cast<int>(std::lround(x)), static_cast<int>(std::lround(y)),
                         static_cast<int>(std::lround(w)), static_cast<int>(std::lround(h)));

            int index; float score;
            if (!classifyLight(box, index, score)) continue;
            if (score < static_cast<float>(light_min_confidence_)) continue;

            results.push_back({box, index, score});
            switch (index) {
                case 0: tl_green  = true; break;  // green_light
                case 1: tl_left   = true; break;  // left_green_light
                case 2: tl_orange = true; break;  // orange_light
                case 3: tl_red    = true; break;  // red_light
            }
        }

        if (enable_gui_) {
            std::lock_guard<std::mutex> lk(mtx_traffic_boxes_);
            last_traffic_results_ = std::move(results);
            last_traffic_boxes_stamp_ = now();
        }

        int traffic_raw = 0;
        if      (tl_left)             traffic_raw = 3;
        else if (tl_green)            traffic_raw = 2;
        else if (tl_red || tl_orange) traffic_raw = 1;

        if (traffic_raw == tl_cand_) tl_cnt_++;
        else { tl_cand_ = traffic_raw; tl_cnt_ = 1; }
        if (tl_cnt_ >= debounce_n_) tl_stable_ = tl_cand_;

        // 별도 토픽으로는 내지 않는다 — /object_info 의 첫 필드로만 나간다
        // (onPublishTick 이 last_traffic_state_ 를 읽어서 발행).
        if (tl_stable_ != tl_last_) {
            RCLCPP_INFO(this->get_logger(),
                        "신호등 상태 = %d (raw=%d)", tl_stable_, traffic_raw);
            tl_last_ = tl_stable_;
        }

        {
            std::lock_guard<std::mutex> lk(mtx_traffic_state_);
            last_traffic_state_ = tl_stable_;
            last_traffic_state_stamp_ = now();
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // LiDAR 스캔 콜백
    //
    // 전방 ±front_fov_deg_ 범위의 포인트를 필터링하고
    // 연속 인덱스 기반으로 클러스터링하여 가장 가까운 클러스터를 저장한다.
    // ─────────────────────────────────────────────────────────────────────
    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        const int N = static_cast<int>(msg->ranges.size());
        if (N == 0 || msg->angle_increment <= 0.0) {
            publishEmpty();
            return;
        }

        const double fov_rad = front_fov_deg_ * M_PI / 180.0;
        const double ang_lo  = -fov_rad;
        const double ang_hi  = +fov_rad;

        // 각도 → 인덱스 변환 헬퍼
        auto angleToIndex = [&](double angle) -> int {
            int idx = static_cast<int>(std::round(
                (angle - msg->angle_min) / msg->angle_increment));
            if (idx < 0 || idx >= N) return -1;
            return idx;
        };

        int i_lo = angleToIndex(ang_lo);
        int i_hi = angleToIndex(ang_hi);
        if (i_lo == -1 && i_hi == -1) { publishEmpty(); return; }
        if (i_lo == -1) i_lo = 0;
        if (i_hi == -1) i_hi = N - 1;
        if (i_lo > i_hi) std::swap(i_lo, i_hi);

        // 유효 포인트 필터링
        struct Pnt { int idx; float r; double ang; };
        std::vector<Pnt> valid;
        valid.reserve(i_hi - i_lo + 1);

        for (int i = i_lo; i <= i_hi; ++i) {
            float r = msg->ranges[i];
            if (!std::isfinite(r)) continue;
            if (r < msg->range_min || r > msg->range_max) continue;
            if (r < range_min_m_   || r > range_max_m_)   continue;
            double ang = msg->angle_min + i * msg->angle_increment;
            valid.push_back({i, r, ang});
        }
        if (valid.empty()) { publishEmpty(); return; }

        // 연속 인덱스 + 거리 차이 기반 클러스터링
        struct Cluster { int start_idx, end_idx; float min_r; double min_r_ang; int count; };
        std::vector<Cluster> clusters;
        clusters.reserve(32);

        int    start       = 0;
        float  cur_min_r   = valid[0].r;
        double cur_min_ang = valid[0].ang;
        int    count       = 1;

        for (size_t k = 1; k < valid.size(); ++k) {
            const float dr          = std::fabs(valid[k].r - valid[k-1].r);
            const bool  contiguous  = (valid[k].idx == valid[k-1].idx + 1);
            if (contiguous && dr <= cluster_epsilon_m_) {
                ++count;
                if (valid[k].r < cur_min_r) {
                    cur_min_r   = valid[k].r;
                    cur_min_ang = valid[k].ang;
                }
            } else {
                clusters.push_back({ valid[start].idx, valid[k-1].idx,
                                     cur_min_r, cur_min_ang, count });
                start       = static_cast<int>(k);
                cur_min_r   = valid[k].r;
                cur_min_ang = valid[k].ang;
                count       = 1;
            }
        }
        clusters.push_back({ valid[start].idx, valid.back().idx,
                              cur_min_r, cur_min_ang, count });

        // 최소 포인트 수를 충족하는 클러스터 중 가장 가까운 것 선택
        bool    found = false;
        Cluster best{};
        best.min_r = std::numeric_limits<float>::infinity();

        for (const auto& c : clusters) {
            if (c.count < min_cluster_points_) continue;
            if (c.min_r < best.min_r) { best = c; found = true; }
        }

        if (!found) { publishEmpty(); return; }

        // 클러스터 각도 폭 계산
        const double ang_start = msg->angle_min + best.start_idx * msg->angle_increment;
        const double ang_end   = msg->angle_min + best.end_idx   * msg->angle_increment;
        const double span      = std::fabs(ang_end - ang_start);

        // LiDAR 감지 상태 저장 (onPublishTick에서 읽음)
        {
            std::lock_guard<std::mutex> lk(mtx_state_);
            lidar_valid_  = true;
            st_min_r_     = best.min_r;
            st_min_r_ang_ = static_cast<float>(best.min_r_ang);
            st_span_      = static_cast<float>(span);
            st_count_     = best.count;
        }

        // 디버그 패널 업데이트
        if (enable_gui_) {
            std::lock_guard<std::mutex> lk(mtx_);
            dbg_dist_     = best.min_r;
            dbg_csize_    = static_cast<float>(best.count);
            last_rx_ok_   = true;
            last_rx_time_ = now();
        }
    }

    // LiDAR 상태 초기화
    void resetLidarState() {
        std::lock_guard<std::mutex> lk(mtx_state_);
        lidar_valid_  = false;
        st_min_r_     = std::numeric_limits<float>::infinity();
        st_min_r_ang_ = 0.0f;
        st_span_      = 0.0f;
        st_count_     = 0;
    }

    // 유효 포인트 없을 때 상태 초기화 및 디버그 갱신
    void publishEmpty() {
        resetLidarState();
        if (enable_gui_) {
            std::lock_guard<std::mutex> lk(mtx_);
            dbg_exists_   = 0.0f;
            dbg_dist_     = std::numeric_limits<float>::infinity();
            dbg_csize_    = 0.0f;
            last_rx_ok_   = false;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // 발행 타이머 콜백 (50Hz)
    //
    // LiDAR 상태와 YOLO 박스 상태를 스냅샷으로 읽고
    // /object_info를 발행한다. 스레드 안전성을 뮤텍스로 보장한다.
    // ─────────────────────────────────────────────────────────────────────
    void onPublishTick() {
        // LiDAR 클러스터링 결과는 /object_info(3필드)엔 안 실린다. OBJECT
        // DEBUG 패널과 /object_info_raw(12필드) 표시에 쓴다.
        bool  lidar_ok;
        float minr, ang, span;
        int   cnt;
        {
            std::lock_guard<std::mutex> lk(mtx_state_);
            lidar_ok = lidar_valid_;
            minr     = st_min_r_;
            ang      = st_min_r_ang_;
            span     = st_span_;
            cnt      = st_count_;
        }

        float box_area, box_cx, box_cy, box_dx, confidence;
        int   raw_lane_label, raw_object_type;
        rclcpp::Time box_stamp{0, 0, RCL_ROS_TIME};
        {
            std::lock_guard<std::mutex> lk(mtx_box_);
            box_area        = last_box_area_pix_;
            box_cx          = last_box_cx_;
            box_cy          = last_box_cy_;
            box_dx          = last_box_dx_;
            raw_lane_label  = last_lane_label_;
            raw_object_type = last_object_type_;
            confidence      = last_object_confidence_;
            box_stamp       = last_box_stamp_;
        }
        const double box_age_s =
            box_stamp.nanoseconds() > 0
                ? (now() - box_stamp).seconds()
                : std::numeric_limits<double>::infinity();
        const bool box_fresh = box_age_s >= 0.0 && box_age_s <= box_max_age_s_;
        int lane_label  = raw_lane_label;
        int object_type = raw_object_type;
        if (!box_fresh || box_area <= 0.0f) {
            lane_label  = 0;
            object_type = -1;
        }
        // /object_info_raw 에도 같은 스테일 리셋을 적용한다 — box_* 계열
        // 필드는 "이 프레임 기준 실제로 유효한 값"만 낸다.
        const bool box_stale = !box_fresh || box_area <= 0.0f;
        const float out_box_area   = box_stale ? 0.0f : box_area;
        const float out_box_cx     = box_stale ? 0.0f : box_cx;
        const float out_box_cy     = box_stale ? 0.0f : box_cy;
        const float out_box_dx     = box_stale ? 0.0f : box_dx;
        const float out_confidence = box_stale ? 0.0f : confidence;

        int32_t traffic_state;
        rclcpp::Time traffic_stamp{0, 0, RCL_ROS_TIME};
        {
            std::lock_guard<std::mutex> lk(mtx_traffic_state_);
            traffic_state = last_traffic_state_;
            traffic_stamp = last_traffic_state_stamp_;
        }
        const double traffic_age_s =
            traffic_stamp.nanoseconds() > 0
                ? (now() - traffic_stamp).seconds()
                : std::numeric_limits<double>::infinity();
        const bool traffic_staled = !(traffic_age_s >= 0.0 && traffic_age_s <= box_max_age_s_);
        if (traffic_staled) {
            traffic_state = 0;
        }

        // lane_label 이 0(중앙/미확정)이면 "그 차선을 못 정했다"는 뜻이라
        // object_type 이 맞아도 인식x(0)로 내보낸다 — 애매한 값을 1/2차선
        // 어느 한쪽으로 억지로 밀어넣지 않는다.
        const int32_t fixed_lane  = (object_type == 0 && lane_label != 0) ? lane_label : 0;
        const int32_t moving_lane = (object_type == 1 && lane_label != 0) ? lane_label : 0;

        // /object_info: [신호등 정보, 고정차량 위치, 방해차량 위치]
        std_msgs::msg::Int32MultiArray out;
        out.data = { traffic_state, fixed_lane, moving_lane };
        pub_obj_->publish(out);

        const float exists = (lidar_ok && (minr <= detect_threshold_m_)) ? 1.0f : 0.0f;

        // /object_info_raw: [exists, min_dist, angle, span, cluster_size,
        // box_size, box_cx, box_cy, dx, car_lane, object_type, confidence]
        // box_size/box_cx/box_cy/dx/car_lane/object_type/confidence 는
        // /object_info 의 fixed_lane/moving_lane 과 같은 스테일 리셋을
        // 적용한다(box_max_age_s_ 지나거나 박스가 없으면 0/-1). LiDAR 쪽
        // (exists/min_dist/angle/span/cluster_size)은 원래부터 별도 스테일
        // 타이머 없이 "이번 스캔" 값을 그대로 낸다 — onScan()이 이미 감쇠를
        // 담당한다.
        std_msgs::msg::Float32MultiArray out_raw;
        out_raw.data = {
            exists, minr, ang, span, static_cast<float>(cnt),
            out_box_area, out_box_cx, out_box_cy, out_box_dx,
            static_cast<float>(lane_label), static_cast<float>(object_type),
            out_confidence,
        };
        pub_obj_raw_->publish(out_raw);

        // 디버그 패널 갱신 (LiDAR exists 는 여기서만 쓰인다)
        if (enable_gui_) {
            std::lock_guard<std::mutex> lk(mtx_);
            dbg_exists_   = exists;
            dbg_dist_     = minr;
            dbg_csize_    = static_cast<float>(cnt);
            last_rx_ok_   = lidar_ok;
            last_rx_time_ = now();
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // 카메라 영상 콜백
    //
    // YOLO 추론으로 바운딩 박스를 검출하고, 차선 회귀 결과(/lane_fit)와
    // 비교하여 박스가 어느 차선에 있는지(lane_label) 판단한다.
    //
    // lane_label:
    //   0 = 중앙선 근처
    //   1 = 왼쪽 (1차선)
    //   2 = 오른쪽 (2차선)
    // ─────────────────────────────────────────────────────────────────────
    void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        // light_ok_ 가 추가된 뒤로는 GUI/내부 YOLO 가 둘 다 꺼져 있어도(=실차
        // 기본 운용) 신호등 분류를 위해 프레임을 받아야 한다.
        if (!enable_gui_ && !yolo_ok_ && !light_ok_) return;

        cv::Mat img;
        try {
            img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
            return;
        }
        if (img.empty()) return;

        // 신호등 크롭 분류용 원본 사본. last_img_ 는 GUI 가 박스/선을 직접
        // 그려 넣어 크롭 소스로 못 쓰므로 따로 둔다.
        if (light_ok_) {
            std::lock_guard<std::mutex> lk(mtx_raw_img_);
            last_raw_img_ = img.clone();
        }

        if (enable_gui_) {
            std::lock_guard<std::mutex> lk(mtx_img_);
            last_img_ = img.clone();
        }

        float box_area  = 0.0f;  // YOLO 박스 없으면 0
        float box_cx    = 0.f, box_cy = 0.f;
        float box_dx    = 0.f;   // box_cx - x_line(box_cy)
        int   lane_label = 0;    // 0=중앙, 1=왼쪽(1차선), 2=오른쪽(2차선)

        cv::Rect best_box;

        if (yolo_ok_) {
            const int W = img.cols, H = img.rows;

            // 레터박스로 640×640 변환 (비율 유지 + 회색 114 패딩).
            // 그냥 640×640으로 늘리면 640×360 원본의 세로가 1.78배 왜곡되는데,
            // YOLOv8은 레터박스로 학습되므로 전처리가 어긋나 검출률이 떨어진다.
            // 검증용 bag 기준 conf>=0.5 검출률 82.3% → 98.1% (stop_2, 260프레임).
            const float scale = std::min(640.f / W, 640.f / H);
            const int   nw = static_cast<int>(std::round(W * scale));
            const int   nh = static_cast<int>(std::round(H * scale));
            const float pad_x = (640 - nw) * 0.5f;
            const float pad_y = (640 - nh) * 0.5f;

            cv::Mat fitted;
            cv::resize(img, fitted, cv::Size(nw, nh));
            cv::Mat resized(640, 640, img.type(), cv::Scalar(114, 114, 114));
            fitted.copyTo(resized(cv::Rect(static_cast<int>(pad_x),
                                           static_cast<int>(pad_y), nw, nh)));
            cv::Mat blob;
            cv::dnn::blobFromImage(resized, blob, 1.0/255.0,
                                   cv::Size(), cv::Scalar(), true, false);
            net_.setInput(blob);

            // 순방향 추론 후 (8400,5) 형태로 변환: [cx, cy, w, h, conf]
            cv::Mat out;
            try {
                out = net_.forward();
                out = out.reshape(1, {5, 8400});
                out = out.t();
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "YOLO 추론 오류: %s", e.what());
                out.release();
            }

            if (!out.empty()) {
                std::vector<cv::Rect> boxes;
                std::vector<float>   confs;
                boxes.reserve(64); confs.reserve(64);

                for (int i = 0; i < out.rows; ++i) {
                    float* d    = out.ptr<float>(i);
                    float  cx_  = d[0], cy_ = d[1], w_ = d[2], h_ = d[3], conf = d[4];
                    if (conf < conf_threshold_) continue;

                    // 640→원본 스케일 복원
                    float x  = ((cx_ - w_/2.f) - pad_x) / scale;
                    float y  = ((cy_ - h_/2.f) - pad_y) / scale;
                    float ww = w_ / scale;
                    float hh = h_ / scale;

                    int left = static_cast<int>(std::round(x));
                    int top  = static_cast<int>(std::round(y));
                    int wi   = static_cast<int>(std::round(ww));
                    int hi   = static_cast<int>(std::round(hh));
                    if (wi < min_w_pix_ || hi < min_h_pix_) continue;

                    // 프레임 경계 클리핑
                    left = std::max(0, std::min(left, W-1));
                    top  = std::max(0, std::min(top,  H-1));
                    wi   = std::max(1, std::min(wi, W - left));
                    hi   = std::max(1, std::min(hi, H - top));

                    boxes.emplace_back(left, top, wi, hi);
                    confs.push_back(conf);
                }

                if (!boxes.empty()) {
                    std::vector<int> keep;
                    cv::dnn::NMSBoxes(boxes, confs, conf_threshold_, nms_threshold_, keep);

                    if (!keep.empty()) {
                        // 가장 '가까운'(=박스 면적이 가장 큰) 장애물을 선택한다.
                        //
                        // 예전에는 신뢰도가 가장 높은 박스를 골랐는데, 차선마다
                        // 방해차량이 있어 두 대가 동시에 잡히면 신뢰도가 엎치락뒤치락
                        // 하면서 프레임마다 다른 차가 선택되고, 그 결과 lane_label
                        // (=car_lane)이 L1/L2 사이를 계속 뒤집혔다. 게다가 정작
                        // 충돌 위험이 있는 건 '가장 가까운' 차인데 멀리 있는 차의
                        // 차선을 판정해 버리는 문제도 있었다.
                        // box_area 는 이미 접근도 지표로 쓰이므로(box_size > 1900
                        // 트리거) 면적 기준 선택이 나머지 로직과도 일관된다.
                        auto area_of = [&](int i) {
                            return static_cast<long>(boxes[i].width) *
                                   static_cast<long>(boxes[i].height);
                        };
                        int best = keep[0];
                        for (int idx : keep)
                            if (area_of(idx) > area_of(best)) best = idx;

                        best_box = boxes[best];
                        box_area = static_cast<float>(best_box.width) *
                                   static_cast<float>(best_box.height);

                        box_cx = static_cast<float>(best_box.x + best_box.width  * 0.5f);
                        box_cy = static_cast<float>(best_box.y + best_box.height * 0.5f);

                        // 박스 중심과 차선 중앙선 x 비교 → lane_label 결정
                        float m = 0.f, b = 0.f;
                        bool lane_ok = false;
                        rclcpp::Time fit_stamp{0, 0, RCL_ROS_TIME};
                        {
                            std::lock_guard<std::mutex> lk(mtx_lane_);
                            lane_ok = fit_valid_;
                            m = fit_m_; b = fit_b_;
                            fit_stamp = fit_stamp_;
                        }
                        // 낡은 /lane_fit 은 쓰지 않는다. 차선 판정을 포기하면
                        // lane_label 은 0(미확정)으로 남고, 소비자(runtime_adapter)
                        // 는 이를 "반대 증거"가 아니라 "증거 없음"으로 다룬다.
                        if (lane_ok && fit_stamp.nanoseconds() > 0) {
                            const double fit_age_s = (now() - fit_stamp).seconds();
                            if (fit_age_s < 0.0 || fit_age_s > lane_fit_max_age_s_) {
                                lane_ok = false;
                                RCLCPP_WARN_THROTTLE(
                                    this->get_logger(), *this->get_clock(), 5000,
                                    "/lane_fit 이 %.2f초 지연되어 차선 판정을 건너뜁니다.",
                                    fit_age_s);
                            }
                        }
                        if (lane_ok && lane_fit_is_frame_) {
                            float x_line = m * box_cy + b;
                            box_dx = box_cx - x_line;
                            if      (box_dx <= -static_cast<float>(lane_split_margin_px_)) lane_label = 1;
                            else if (box_dx >=  static_cast<float>(lane_split_margin_px_)) lane_label = 2;
                            else                                                             lane_label = 0;
                        }

                        // 디버그 영상에 박스 및 중앙선 오버레이
                        if (enable_gui_) {
                            std::lock_guard<std::mutex> lk(mtx_img_);
                            if (!last_img_.empty()) {
                                cv::rectangle(last_img_, best_box, cv::Scalar(0, 255, 0), 2);
                                cv::circle(last_img_,
                                           cv::Point((int)std::round(box_cx), (int)std::round(box_cy)),
                                           4, {0,255,0}, cv::FILLED);

                                if (lane_ok) {
                                    // 차선 중앙선 그리기
                                    int x_top = (int)std::round(m * 0.0f  + b);
                                    int x_bot = (int)std::round(m * (H-1.0f) + b);
                                    x_top = std::max(0, std::min(W-1, x_top));
                                    x_bot = std::max(0, std::min(W-1, x_bot));
                                    cv::line(last_img_, {x_top,0}, {x_bot,H-1},
                                             {0,200,255}, 2);

                                    // 박스 중심 → 중앙선까지 델타 시각화
                                    int x_line_cy = (int)std::round(m * box_cy + b);
                                    cv::line(last_img_,
                                             {x_line_cy,    (int)std::round(box_cy)},
                                             {(int)std::round(box_cx), (int)std::round(box_cy)},
                                             {255,255,255}, 2);

                                    std::string tag = (lane_label==1 ? "L1" :
                                                       lane_label==2 ? "L2" : "C");
                                    cv::putText(last_img_,
                                                "dx=" + std::to_string((int)std::round(box_dx))
                                                      + " " + tag,
                                                {best_box.x, std::max(0, best_box.y-6)},
                                                cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                                {255,255,255}, 2);
                                }
                            }
                        }
                    }
                }
            }
        }

        // YOLO 결과 공유 변수 갱신 (onPublishTick에서 읽음).
        // 박스를 못 찾은 프레임도 갱신 시각을 남긴다. "방금 봤는데 아무것도
        // 없었다"와 "한참 전 결과가 남아 있다"는 다르다.
        {
            std::lock_guard<std::mutex> lk(mtx_box_);
            last_box_area_pix_ = box_area;
            last_box_cx_       = box_cx;
            last_box_cy_       = box_cy;
            last_box_dx_       = box_dx;
            last_lane_label_   = lane_label;
            last_object_type_  = box_area > 0.0f ? 0 : -1;
            last_object_confidence_ = 0.0f;
            last_box_stamp_    = now();
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // 디버그 창 갱신 타이머 콜백 (~30Hz)
    //
    // "OBJECT DEBUG": 장애물 존재 여부 / 거리 / 클러스터 크기 텍스트 패널
    // "CAMERA VIEW" : 최신 카메라 영상 (YOLO 오버레이 포함)
    // ─────────────────────────────────────────────────────────────────────
    void onTimer() {
        // OBJECT DEBUG 패널
        {
            cv::Mat canvas(240, 480, CV_8UC3, cv::Scalar(30, 30, 30));
            float exists, dist, csz;
            bool ok;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                exists = dbg_exists_; dist = dbg_dist_; csz = dbg_csize_;
                ok = last_rx_ok_;
            }

            // 소수점 형식 문자열 변환 헬퍼
            auto fmt = [](float v, int p=2) {
                std::ostringstream o;
                o.setf(std::ios::fixed);
                o << std::setprecision(p) << v;
                return o.str();
            };

            std::string l1 = "exists: "       + std::string((exists >= 0.5f) ? "1" : "0");
            std::string l2 = "distance[m]: "  + (std::isfinite(dist) ? fmt(dist, 2) : "inf");
            std::string l3 = "cluster_size: " + fmt(csz, 0);

            cv::putText(canvas, l1, {20,  80}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
                        (exists >= 0.5f ? cv::Scalar(60,220,60) : cv::Scalar(40,40,200)), 2);
            cv::putText(canvas, l2, {20, 130}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(230,230,230), 2);
            cv::putText(canvas, l3, {20, 180}, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(230,230,230), 2);
            cv::imshow("OBJECT DEBUG", canvas);
        }

        // CAMERA VIEW 패널
        //
        // 원본 프레임을 매번 새로 복제해서, 그 위에 최신 상태만 다시 그린다.
        // (콜백에서 직접 last_img_ 에 그리면 카메라 프레임 갱신 사이에 여러
        // 검출 콜백이 겹쳐 들어올 때 이전 박스가 안 지워지고 남아서 박스가
        // 2개로 보이는 문제가 있었다.)
        {
            cv::Mat disp;
            {
                std::lock_guard<std::mutex> lk(mtx_img_);
                if (last_img_.empty()) { cv::waitKey(1); return; }
                disp = last_img_.clone();
            }
            const int W = disp.cols, H = disp.rows;

            // 방해차량/고정장애물 박스 (최신 1개, 오래되면 안 그림)
            {
                float bx, by, bw, bh, cx, cy, dx;
                int   lane_label;
                rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
                {
                    std::lock_guard<std::mutex> lk(mtx_box_);
                    bx = last_box_x_; by = last_box_y_; bw = last_box_w_; bh = last_box_h_;
                    cx = last_box_cx_; cy = last_box_cy_; dx = last_box_dx_;
                    lane_label = last_lane_label_;
                    stamp = last_box_stamp_;
                }
                const double age_s = stamp.nanoseconds() > 0
                                          ? (now() - stamp).seconds()
                                          : std::numeric_limits<double>::infinity();
                if (bw > 0.0f && bh > 0.0f && age_s >= 0.0 && age_s <= box_max_age_s_) {
                    cv::Rect box(static_cast<int>(std::round(bx)), static_cast<int>(std::round(by)),
                                 static_cast<int>(std::round(bw)), static_cast<int>(std::round(bh)));
                    cv::rectangle(disp, box, cv::Scalar(0, 255, 0), 2);
                    cv::circle(disp, cv::Point((int)std::round(cx), (int)std::round(cy)),
                               4, {0,255,0}, cv::FILLED);

                    bool lane_ok; float m, b;
                    rclcpp::Time fit_stamp{0, 0, RCL_ROS_TIME};
                    {
                        std::lock_guard<std::mutex> lk(mtx_lane_);
                        lane_ok = fit_valid_; m = fit_m_; b = fit_b_; fit_stamp = fit_stamp_;
                    }
                    if (lane_ok && fit_stamp.nanoseconds() > 0) {
                        const double fit_age_s = (now() - fit_stamp).seconds();
                        lane_ok = fit_age_s >= 0.0 && fit_age_s <= lane_fit_max_age_s_;
                    }
                    if (lane_ok && lane_fit_is_frame_) {
                        int x_top = (int)std::round(m * 0.0f  + b);
                        int x_bot = (int)std::round(m * (H-1.0f) + b);
                        x_top = std::max(0, std::min(W-1, x_top));
                        x_bot = std::max(0, std::min(W-1, x_bot));
                        cv::line(disp, {x_top,0}, {x_bot,H-1}, {0,200,255}, 2);

                        int x_line_cy = (int)std::round(m * cy + b);
                        cv::line(disp, {x_line_cy, (int)std::round(cy)},
                                 {(int)std::round(cx), (int)std::round(cy)}, {255,255,255}, 2);

                        std::string tag = (lane_label==1 ? "L1" : lane_label==2 ? "L2" : "C");
                        cv::putText(disp,
                                    "dx=" + std::to_string((int)std::round(dx)) + " " + tag,
                                    {box.x, std::max(0, box.y-6)},
                                    cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,255}, 2);
                    }
                }
            }

            // 신호등 박스 (여러 개일 수 있음, 오래되면 안 그림)
            // 색·라벨은 크롭 분류 결과(index) 기준 — 검출기의 raw class_id
            // 가 아니다. 신뢰도(score)도 분류기 확신도다.
            {
                static const std::vector<std::string> kLightNames = {
                    "green", "left_green", "orange", "red"};
                static const std::vector<cv::Scalar> kLightColors = {
                    {0,220,0}, {255,255,0}, {0,165,255}, {0,0,255}};

                std::vector<TrafficBoxResult> results;
                rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
                {
                    std::lock_guard<std::mutex> lk(mtx_traffic_boxes_);
                    results = last_traffic_results_;
                    stamp = last_traffic_boxes_stamp_;
                }
                const double age_s = stamp.nanoseconds() > 0
                                          ? (now() - stamp).seconds()
                                          : std::numeric_limits<double>::infinity();
                if (age_s >= 0.0 && age_s <= box_max_age_s_) {
                    for (const auto& r : results) {
                        if (r.index < 0 || r.index > 3) continue;
                        const cv::Scalar& col = kLightColors[r.index];
                        cv::rectangle(disp, r.box, col, 2);
                        cv::putText(disp,
                                    kLightNames[r.index] + " " + std::to_string(r.score).substr(0,4),
                                    {r.box.x, std::max(0, r.box.y-6)},
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv::LINE_AA);
                    }
                }
            }

            cv::imshow("CAMERA VIEW", disp);
        }

        cv::waitKey(1);
    }

    // 현재 ROS 시간 반환 헬퍼
    rclcpp::Time now() { return this->get_clock()->now(); }

    // ── ROS 파라미터 ────────────────────────────────────────────────────
    double front_fov_deg_, range_min_m_, range_max_m_;
    double cluster_epsilon_m_, detect_threshold_m_;
    int    min_cluster_points_;
    bool   enable_gui_;
    double lane_split_margin_px_;
    int    lane_debounce_n_ = 3;   // 차선 판정 디바운스 프레임 수
    double lane_fit_max_age_s_;
    double box_max_age_s_;
    bool   lane_fit_is_frame_;
    bool   use_external_yolo_;
    std::string model_path_;
    int    debounce_n_ = 3;   // 신호등 디바운스 프레임 수
    std::string light_classifier_path_;
    double light_crop_margin_    = 0.15;
    int    light_input_size_     = 64;
    double light_min_confidence_ = 0.90;

    // ── ROS 통신 객체 ───────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       sub_scan_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           sub_img_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  sub_lane_fit_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  sub_yolo_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  sub_traffic_boxes_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr       pub_obj_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr     pub_obj_raw_;
    rclcpp::TimerBase::SharedPtr timer_;      // 디버그 창 갱신 타이머
    rclcpp::TimerBase::SharedPtr pub_timer_;  // 발행 타이머

    // ── 디버그 상태 (뮤텍스: mtx_) ─────────────────────────────────────
    std::mutex   mtx_;
    float        dbg_exists_    = 0.0f;
    float        dbg_dist_      = std::numeric_limits<float>::infinity();
    float        dbg_csize_     = 0.0f;
    bool         last_rx_ok_    = false;
    rclcpp::Time last_rx_time_{0, 0, RCL_ROS_TIME};

    // ── 카메라 영상 공유 변수 (뮤텍스: mtx_img_) ───────────────────────
    std::mutex mtx_img_;
    cv::Mat    last_img_;

    // ── 신호등 분류용 원본 프레임 (뮤텍스: mtx_raw_img_) ────────────────
    // last_img_ 와 별개다 — GUI 오버레이용은 박스/선이 그려져 크롭 소스로
    // 못 쓴다.
    std::mutex mtx_raw_img_;
    cv::Mat    last_raw_img_;

    // ── YOLO 모델 ───────────────────────────────────────────────────────
    cv::dnn::Net net_;
    bool  yolo_ok_         = false;
    float conf_threshold_  = 0.f;
    float nms_threshold_   = 0.f;
    int   min_w_pix_       = 0;
    int   min_h_pix_       = 0;

    // ── 신호등 크롭 분류기 ───────────────────────────────────────────────
    cv::dnn::Net light_net_;
    bool  light_ok_ = false;

    // classifyLight() 결과 하나(박스 + 분류 인덱스 + 확신도). GUI 오버레이용.
    struct TrafficBoxResult { cv::Rect box; int index; float score; };

    // ── YOLO 박스 공유 변수 (뮤텍스: mtx_box_) ─────────────────────────
    std::mutex mtx_box_;
    float last_box_area_pix_ = 0.0f;
    float last_box_cx_       = 0.f, last_box_cy_ = 0.f;
    float last_box_dx_       = 0.f;
    float last_box_x_ = 0.f, last_box_y_ = 0.f, last_box_w_ = 0.f, last_box_h_ = 0.f;
    int   last_lane_label_   = 0;
    int   last_object_type_  = -1;
    float last_object_confidence_ = 0.0f;
    rclcpp::Time last_box_stamp_{0, 0, RCL_ROS_TIME};  // 마지막 YOLO 갱신 시각

    // ── 신호등 박스 공유 변수 (뮤텍스: mtx_traffic_boxes_, 표시 전용) ──
    std::mutex                       mtx_traffic_boxes_;
    std::vector<TrafficBoxResult>    last_traffic_results_;
    rclcpp::Time                     last_traffic_boxes_stamp_{0, 0, RCL_ROS_TIME};

    // ── 신호등 상태 공유 변수 (뮤텍스: mtx_traffic_state_) ─────────────
    // onPublishTick() 이 읽는 쪽. 쓰는 쪽은 onTrafficBoxes() 하나뿐이라
    // 그쪽 디바운스 상태(tl_*)는 뮤텍스 없이 콜백 스레드 안에서만 접근한다.
    std::mutex   mtx_traffic_state_;
    int32_t      last_traffic_state_ = 0;
    rclcpp::Time last_traffic_state_stamp_{0, 0, RCL_ROS_TIME};

    // ── 신호등 디바운스 상태 (onTrafficBoxes 콜백에서만 접근, 뮤텍스 불필요) ──
    int tl_cand_   = 0;   // 현재 후보 코드
    int tl_cnt_    = 0;   // 후보 연속 프레임 수
    int tl_stable_ = 0;   // 확정된 코드 (발행값)
    int tl_last_   = -1;  // 마지막으로 로그를 남긴 코드

    // ── 차선 판정 디바운스 상태 (onYoloDetection 콜백에서만 접근, 뮤텍스 불필요) ──
    int lane_cand_   = 0;   // 현재 후보 lane_label
    int lane_cnt_    = 0;   // 후보 연속 프레임 수
    int lane_stable_ = 0;   // 확정된 lane_label (last_lane_label_ 에 반영되는 값)

    // ── 차선 회귀 공유 변수 (뮤텍스: mtx_lane_) ────────────────────────
    std::mutex mtx_lane_;
    float fit_m_     = 0.f, fit_b_ = 0.f;
    bool  fit_valid_ = false;
    rclcpp::Time fit_stamp_{0, 0, RCL_ROS_TIME};  // 마지막 /lane_fit 수신 시각

    // ── LiDAR 상태 공유 변수 (뮤텍스: mtx_state_) ──────────────────────
    std::mutex mtx_state_;
    bool  lidar_valid_  = false;
    float st_min_r_     = std::numeric_limits<float>::infinity();
    float st_min_r_ang_ = 0.0f;
    float st_span_      = 0.0f;
    int   st_count_     = 0;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
