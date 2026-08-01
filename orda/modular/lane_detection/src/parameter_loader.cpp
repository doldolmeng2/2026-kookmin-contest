#include <fstream>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include "parameter_loader.hpp"

using json = nlohmann::json;

// 문자열 → LaneMode 열거형 변환
// "LANE_ONE" → LaneMode::LANE_ONE
// "LANE_TWO" → LaneMode::LANE_TWO
// 그 외 문자열 → std::runtime_error 예외
LaneMode lane_mode_from_string(const std::string& mode_str) {
    if (mode_str == "LANE_ONE") return LaneMode::LANE_ONE;
    if (mode_str == "LANE_TWO") return LaneMode::LANE_TWO;
    throw std::runtime_error("잘못된 lane_mode 값: " + mode_str);
}

// JSON 파일을 파싱하여 Config 구조체를 완성하고 반환한다.
Config load_config(const std::string& path) {
    std::ifstream file(path);
    json j;
    file >> j;

    Config config;

    // ── 노란색 차선 HLS 임계값 ─────────────────────────────────────────
    config.yellow_hls_min_h = j["yellow_hls_min_h"];
    config.yellow_hls_max_h = j["yellow_hls_max_h"];
    config.yellow_hls_min_l = j["yellow_hls_min_l"];
    config.yellow_hls_max_l = j["yellow_hls_max_l"];
    config.yellow_hls_min_s = j["yellow_hls_min_s"];
    config.yellow_hls_max_s = j["yellow_hls_max_s"];

    // ── 노란색 차선 HSV 임계값 ─────────────────────────────────────────
    config.yellow_hsv_min_h = j["yellow_hsv_min_h"];
    config.yellow_hsv_max_h = j["yellow_hsv_max_h"];
    config.yellow_hsv_min_s = j["yellow_hsv_min_s"];
    config.yellow_hsv_max_s = j["yellow_hsv_max_s"];
    config.yellow_hsv_min_v = j["yellow_hsv_min_v"];
    config.yellow_hsv_max_v = j["yellow_hsv_max_v"];

    // ── 노란색 차선 YCrCb 임계값 ──────────────────────────────────────
    config.yellow_ycrcb_min_y  = j["yellow_ycrcb_min_y"];
    config.yellow_ycrcb_max_y  = j["yellow_ycrcb_max_y"];
    config.yellow_ycrcb_min_cr = j["yellow_ycrcb_min_cr"];
    config.yellow_ycrcb_max_cr = j["yellow_ycrcb_max_cr"];
    config.yellow_ycrcb_min_cb = j["yellow_ycrcb_min_cb"];
    config.yellow_ycrcb_max_cb = j["yellow_ycrcb_max_cb"];

    // ── 기본 프레임 및 차선 설정 ──────────────────────────────────────
    config.lane_mode    = lane_mode_from_string(j["lane_mode"]);
    config.frame_width  = j["frame_width"];
    config.frame_height = j["frame_height"];

    // ── ROI 사다리꼴 좌표 계수 ────────────────────────────────────────
    config.roi_top_width_coefficient    = j["roi_top_width_coefficient"];
    config.roi_bottom_width_coefficient = j["roi_bottom_width_coefficient"];
    config.roi_top_y_coefficient        = j["roi_top_y_coefficient"];
    config.roi_bottom_y_coefficient     = j["roi_bottom_y_coefficient"];

    // ── 히스토그램 가중치 파라미터 ────────────────────────────────────
    config.ref_hist_sigma_ratio = j["ref_hist_sigma_ratio"];
    config.ref_hist_min_weight  = j["ref_hist_min_weight"];

    // ── 슬라이딩 윈도우 파라미터 ──────────────────────────────────────
    config.sliding_window_num_windows = j["sliding_window_num_windows"];
    config.sliding_window_margin      = j["sliding_window_margin"];
    config.sliding_window_minpix      = j["sliding_window_minpix"];
    config.corridor_width             = j["corridor_width"];

    // ── 전처리 파라미터 ───────────────────────────────────────────────
    config.gaussian_blur_kernel_size   = j["gaussian_blur_kernel_size"];
    config.canny_yellow_high_threshold = j["canny_yellow_high_threshold"];
    config.canny_yellow_low_threshold  = j["canny_yellow_low_threshold"];
    config.kernel_yellow_closing_size  = j["kernel_yellow_closing_size"];
    config.kernel_yellow_opening_size  = j["kernel_yellow_opening_size"];

    // ── 차선별 기준선 비율 ────────────────────────────────────────────
    config.center_reference_lane_one = j["center_reference_lane_one"];
    config.center_reference_lane_two = j["center_reference_lane_two"];

    // ── 수평 노이즈 억제 파라미터 ─────────────────────────────────────
    config.horizontal_noise_width                = j["horizontal_noise_width"];
    config.horizontal_noise_band_h               = j["horizontal_noise_band_h"];
    config.horizontal_noise_extra_pad            = j["horizontal_noise_extra_pad"];
    config.horizontal_band_corridor_ratio_thresh = j["horizontal_band_corridor_ratio_thresh"];

    // ── 수직 열 밴드 노이즈 억제 파라미터 ────────────────────────────
    config.vertical_noise_band_w               = j["vertical_noise_band_w"];
    config.vertical_noise_min_pixels           = j["vertical_noise_min_pixels"];
    config.vertical_noise_peak_ratio           = j["vertical_noise_peak_ratio"];
    config.vertical_noise_extra_pad_half_width = j["vertical_noise_extra_pad_half_width"];
    config.vertical_noise_peak_use_corridor    = j["vertical_noise_peak_use_corridor"];

    // ── 차선 변경 감지 임계값 ─────────────────────────────────────────
    config.lane_change_tol_straight = j["lane_change_tol_straight"];
    config.lane_change_tol_curve    = j["lane_change_tol_curve"];
    config.lane_change_tol_change   = j["lane_change_tol_change"];
    config.lane_change_streak_need  = j["lane_change_streak_need"];
    config.lane_change_m_split      = j["lane_change_m_split"];

    // ── ref 전환 및 디버그 설정 ───────────────────────────────────────
    config.lane_ref_transition_duration_sec = j["lane_ref_transition_duration_sec"];
    config.debug_view          = j["debug_view"];
    config.change_ref_smoothly = j["change_ref_smoothly"];

    return config;
}
