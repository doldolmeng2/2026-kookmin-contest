// ─────────────────────────────────────────────────────────────────────────────
// object_detection.cpp
//
// 역할: LiDAR 클러스터링 + YOLO 바운딩 박스를 결합한 장애물 감지 노드
//
// 구독:
//   /scan          (sensor_msgs/LaserScan)           - LiDAR 거리 데이터
//   /resized_image (sensor_msgs/Image, BGR8)          - 카메라 영상
//   /lane_fit      (std_msgs/Float32MultiArray [m,b]) - 차선 회귀 결과
//
// 발행:
//   /object_info (std_msgs/Float32MultiArray, 11개 필드)
//   [exists, min_dist, angle, span, cluster_size,
//    box_size, box_cx, box_cy, dx(cx-x_line), lane_label, is_moving]
//   is_moving: 0=고정장애물, 1=방해차량
//
// 디버그 창 (enable_gui=true 시):
//   "OBJECT DEBUG" : exists / distance / cluster_size
//   "CAMERA VIEW"  : 입력 영상 + 중앙선/박스 중심/델타 오버레이
// ─────────────────────────────────────────────────────────────────────────────

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <cv_bridge/cv_bridge.h>
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
        // dx 가 -41 -> +31 로 부호까지 뒤집혔다(car_lane 1 -> 2). 마진을 실측
        // 노이즈 수준으로 올리면 그런 프레임은 "미확정(0)"이 되어 반대 방향으로
        // 오인되지 않는다. 진짜 좌/우 판정의 |dx| 는 같은 구간에서 77~84 px 였다.
        lane_split_margin_px_ = this->declare_parameter<double>("lane_split_margin_px", 45.0);
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

        // 장애물 정보 발행
        pub_obj_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/object_info", qos_fast);

        // ── YOLO 모델 초기화 ────────────────────────────────────────────
        // 모델 경로는 개발 PC / 실차 두 곳을 순서대로 확인한다 (resolveModelPath).
        // 예전에는 실차 경로 하나만 박혀 있어서, 개발 PC에서 재생할 때 모델
        // 로드가 조용히 실패하고 CAMERA VIEW에 박스가 전혀 안 그려졌다.
        // 에러가 터미널이 아니라 노드 로그에만 남아 원인 파악이 어려웠다.
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
        bool  lidar_ok;
        float minr, ang, spn;
        int   cnt;
        float box_area;
        float cx, cy, dx;
        int   lane_label;

        {
            std::lock_guard<std::mutex> lk(mtx_state_);
            lidar_ok = lidar_valid_;
            minr     = st_min_r_;
            ang      = st_min_r_ang_;
            spn      = st_span_;
            cnt      = st_count_;
        }
        rclcpp::Time box_stamp{0, 0, RCL_ROS_TIME};
        {
            std::lock_guard<std::mutex> lk(mtx_box_);
            box_area   = last_box_area_pix_;
            cx         = last_box_cx_;
            cy         = last_box_cy_;
            dx         = last_box_dx_;
            lane_label = last_lane_label_;
            box_stamp  = last_box_stamp_;
        }

        // 오래된 YOLO 결과는 재발행하지 않는다 (box_max_age_s_ 주석 참고).
        const double box_age_s =
            box_stamp.nanoseconds() > 0
                ? (now() - box_stamp).seconds()
                : std::numeric_limits<double>::infinity();
        if (!(box_age_s >= 0.0 && box_age_s <= box_max_age_s_)) {
            box_area   = 0.0f;
            cx = cy = dx = 0.0f;
            lane_label = 0;
        }

        const float exists = (lidar_ok && (minr <= detect_threshold_m_)) ? 1.0f : 0.0f;

        // /object_info 필드:
        // [exists, min_dist, angle, span, cluster_size,
        //  box_size, box_cx, box_cy, dx, lane_label, is_moving]
        //
        // is_moving 은 지금 학습된 YOLO 모델이 고정장애물만 구분하므로 항상
        // 0(고정장애물)이다. 이동체를 구분하는 모델이 들어오면 여기서 그
        // 분류 결과를 넣는다.
        constexpr float kIsMovingFixedObstacle = 0.0f;
        std_msgs::msg::Float32MultiArray out;
        out.data = { exists, minr, ang, spn, static_cast<float>(cnt),
                     box_area, cx, cy, dx, static_cast<float>(lane_label),
                     kIsMovingFixedObstacle };
        pub_obj_->publish(out);

        // 디버그 패널 갱신
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
        if (!enable_gui_ && !yolo_ok_) return;

        cv::Mat img;
        try {
            img = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
            return;
        }
        if (img.empty()) return;

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
        {
            std::lock_guard<std::mutex> lk(mtx_img_);
            if (!last_img_.empty())
                cv::imshow("CAMERA VIEW", last_img_);
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
    double lane_fit_max_age_s_;
    double box_max_age_s_;
    bool   lane_fit_is_frame_;
    std::string model_path_;

    // ── ROS 통신 객체 ───────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr       sub_scan_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr           sub_img_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  sub_lane_fit_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr     pub_obj_;
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

    // ── YOLO 모델 ───────────────────────────────────────────────────────
    cv::dnn::Net net_;
    bool  yolo_ok_         = false;
    float conf_threshold_  = 0.f;
    float nms_threshold_   = 0.f;
    int   min_w_pix_       = 0;
    int   min_h_pix_       = 0;

    // ── YOLO 박스 공유 변수 (뮤텍스: mtx_box_) ─────────────────────────
    std::mutex mtx_box_;
    float last_box_area_pix_ = 0.0f;
    float last_box_cx_       = 0.f, last_box_cy_ = 0.f;
    float last_box_dx_       = 0.f;
    int   last_lane_label_   = 0;
    rclcpp::Time last_box_stamp_{0, 0, RCL_ROS_TIME};  // 마지막 YOLO 갱신 시각

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
