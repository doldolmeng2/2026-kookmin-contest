#pragma once
#include <string>

// 차선 모드 열거형
// LANE_ONE: 1차선 주행 기준, LANE_TWO: 2차선 주행 기준
enum class LaneMode
{
  LANE_ONE,
  LANE_TWO,
  // 기본 주행 모드. 두 차선 기준값의 중간을 기준선으로 쓴다.
  CENTER
};

// JSON 파라미터 파일에서 로드하는 전체 설정 구조체
struct Config
{
  // ── 노란색 차선 HLS 색공간 임계값 ───────────────────────────────────
  // OpenCV HLS 채널 순서: [H(색조), L(밝기), S(채도)]
  int yellow_hls_min_h, yellow_hls_max_h;    // 색조(Hue) 범위
  int yellow_hls_min_l, yellow_hls_max_l;    // 밝기(Lightness) 범위
  int yellow_hls_min_s, yellow_hls_max_s;    // 채도(Saturation) 범위

  // ── 노란색 차선 HSV 색공간 임계값 ───────────────────────────────────
  // OpenCV HSV 채널 순서: [H(색조), S(채도), V(명도)]
  int yellow_hsv_min_h, yellow_hsv_max_h;    // 색조(Hue) 범위
  int yellow_hsv_min_s, yellow_hsv_max_s;    // 채도(Saturation) 범위
  int yellow_hsv_min_v, yellow_hsv_max_v;    // 명도(Value) 범위

  // ── 노란색 차선 YCrCb 색공간 임계값 ─────────────────────────────────
  // OpenCV YCrCb 채널 순서: [Y(휘도), Cr(적색 색차), Cb(청색 색차)]
  int yellow_ycrcb_min_y, yellow_ycrcb_max_y;      // 휘도(Luma) 범위
  int yellow_ycrcb_min_cr, yellow_ycrcb_max_cr;    // 적색 색차(Cr) 범위
  int yellow_ycrcb_min_cb, yellow_ycrcb_max_cb;    // 청색 색차(Cb) 범위

  // ── 기본 프레임 및 차선 초기 설정 ───────────────────────────────────
  LaneMode lane_mode;     // 시작 시 사용할 차선 모드
  int frame_width;        // 입력 프레임 가로 크기 (px)
  int frame_height;       // 입력 프레임 세로 크기 (px)

  // ── ROI 사다리꼴 좌표 계수 (프레임 크기 대비 비율) ──────────────────
  float roi_top_width_coefficient;       // 사다리꼴 상단 폭 / frame_width
  float roi_bottom_width_coefficient;    // 사다리꼴 하단 폭 / frame_width
  float roi_top_y_coefficient;           // 사다리꼴 상단 y  / frame_height
  float roi_bottom_y_coefficient;        // 사다리꼴 하단 y  / frame_height

  // ── 슬라이딩 윈도우 히스토그램 가중치 파라미터 ──────────────────────
  // 기준점(ref_x_) 주변에 가우시안 가중치를 곱해 시작점 선택 편향을 부여한다.
  float ref_hist_sigma_ratio;    // 가우시안 σ = sigma_ratio × corridor_width
  float ref_hist_min_weight;     // ref에서 먼 위치의 최소 가중치 (0~1)

  // ── 슬라이딩 윈도우 탐색 파라미터 ───────────────────────────────────
  int sliding_window_num_windows;       // 세로 방향 윈도우 분할 수
  int sliding_window_margin;            // 윈도우 좌우 탐색 반폭 (px)
  size_t sliding_window_minpix;         // 중심 재조정에 필요한 최소 픽셀 수

  // ── 차선 탐색 코리도어 폭 ────────────────────────────────────────────
  int corridor_width;    // 기준선(ref_x_) 중심 탐색 범위 (px)

  // ── 추적 실패 시 재획득(reacquire) ──────────────────────────────────
  // fitLaneFromBEV 이 이 프레임 수만큼 연속으로 실패(ok=false, pts<10)하면
  // 직전 위치 앵커를 버리고 ref_x_(차선 모드 고정 기준점)로 되돌아가
  // 다시 찾는다. 없으면 한 번 놓친 앵커에 영원히 갇힐 수 있다.
  int fit_reacquire_after_frames;

  // ── 직선 피팅 이상치 제거 (반복적 잔차 클리핑) ──────────────────────
  // 1차 SVD 피팅 후, 그 직선에서 이 값(px)보다 멀리 떨어진 점을 제거하고
  // 재피팅한다. 슬라이딩 윈도우 일부가 노이즈(반사광 등)를 주웠을 때
  // 그 노이즈가 최소제곱 피팅 전체를 흔드는 것을 막기 위함.
  float fit_outlier_reject_px;     // 잔차 임계값 (px)
  int fit_outlier_iterations;      // 재피팅 반복 횟수 (0이면 기존과 동일하게 비활성)

  // ── 전처리: 블러 및 Canny 엣지 파라미터 ─────────────────────────────
  int gaussian_blur_kernel_size;      // 가우시안 블러 커널 크기 (홀수)
  int canny_yellow_high_threshold;    // Canny 상위 임계값
  int canny_yellow_low_threshold;     // Canny 하위 임계값
  int kernel_yellow_closing_size;     // 모폴로지 닫힘 커널 크기 (구멍 메우기)
  int kernel_yellow_opening_size;     // 모폴로지 열림 커널 크기 (노이즈 제거)

  // ── 차선별 기준선 비율 (0~1, BEV 폭 기준) ───────────────────────────
  float center_reference_lane_one;    // 1차선 주행 시 기준선 x 비율
  float center_reference_lane_two;    // 2차선 주행 시 기준선 x 비율

  // ── 수평 노이즈 억제 파라미터 ────────────────────────────────────────
  int horizontal_noise_width;                    // 노이즈로 판단할 최소 가로 길이 (px)
  int horizontal_noise_band_h;                   // 수평 스캔 밴드 높이 (px)
  int horizontal_noise_extra_pad;                // 노이즈 행 삭제 시 위/아래 추가 여유
  float horizontal_band_corridor_ratio_thresh;   // 밴드 내 픽셀 / 전체 픽셀 비율 임계값

  // ── 수직 열 밴드 노이즈 억제 파라미터 ───────────────────────────────
  int vertical_noise_band_w;                   // 열 밴드 폭 (px)
  int vertical_noise_min_pixels;               // 밴드 내 최소 픽셀 수 (절대 기준)
  float vertical_noise_peak_ratio;             // 최대 피크 대비 비율 임계값
  int vertical_noise_extra_pad_half_width;     // 밴드 삭제 시 좌우 추가 여유 (px)
  bool vertical_noise_peak_use_corridor;       // true면 corridor 범위에서만 검사

  // ── 차선 변경 감지 임계값 ────────────────────────────────────────────
  float lane_change_tol_straight;    // 직선 구간 정착 허용 오프셋 (px)
  float lane_change_tol_curve;       // 곡선 구간 정착 허용 오프셋 (px, 직선보다 완만)
  float lane_change_tol_change;      // 차선 변경 스파이크로 볼 최소 오프셋 (px)
  int lane_change_streak_need;       // 정착 판정에 필요한 연속 프레임 수
  float lane_change_m_split;         // |기울기| >= 이 값이면 곡선으로 간주

  // ── ref 전환 및 디버그 설정 ──────────────────────────────────────────
  float lane_ref_transition_duration_sec;    // 차선 전환 시 ref 선형 보간 소요 시간 (초)
  bool debug_view;              // true면 OpenCV imshow 디버그 창 표시
  // false면 보조 차선 디버그 창(YCrCb channels, ROI Polygon, BEV-Yellow 3종,
  // Lane View + Offset)을 숨기고 SlidingWindows / Mask-Yellow만 표시한다.
  // debug_view가 false면 이 값과 무관하게 모든 창이 꺼진다.
  bool debug_lane_view;
  bool change_ref_smoothly;     // true면 차선 전환 시 ref를 선형 보간
};

// JSON 문자열 → LaneMode 열거형 변환 (미지원 문자열이면 예외 발생)
LaneMode lane_mode_from_string(const std::string & mode_str);

// JSON 파일 경로로부터 Config 구조체를 파싱하여 반환
Config load_config(const std::string & path);
