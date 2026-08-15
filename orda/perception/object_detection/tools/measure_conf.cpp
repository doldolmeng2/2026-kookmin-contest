// object_detection.cpp와 동일한 전처리/후처리로 프레임별 YOLO 신뢰도를 측정한다.
// 임계값을 걸지 않고 원시 conf 분포를 본다.
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <vector>

namespace fs = std::filesystem;

struct Det { float conf; cv::Rect box; };

// 전처리 방식. stretch 는 현재 object_detection.cpp 가 하는 것(비율 무시),
// letterbox 는 ultralytics 가 학습 때 쓰는 것(비율 유지 + 회색 패딩).
enum class Pre { Stretch, Letterbox };

static std::vector<Det> infer(cv::dnn::Net& net, const cv::Mat& img,
                              int min_wh, Pre pre) {
    const int W = img.cols, H = img.rows;

    // 640 공간으로 보내는 변환: x640 = x*sx + px
    float sx, sy, px = 0.f, py = 0.f;
    cv::Mat resized;
    if (pre == Pre::Stretch) {
        cv::resize(img, resized, cv::Size(640, 640));
        sx = 640.f / W;
        sy = 640.f / H;
    } else {
        float s = std::min(640.f / W, 640.f / H);
        int nw = int(std::round(W * s)), nh = int(std::round(H * s));
        cv::Mat small_;
        cv::resize(img, small_, cv::Size(nw, nh));
        resized = cv::Mat(640, 640, img.type(), cv::Scalar(114, 114, 114));
        px = (640 - nw) / 2.f;
        py = (640 - nh) / 2.f;
        small_.copyTo(resized(cv::Rect(int(px), int(py), nw, nh)));
        sx = sy = s;
    }

    cv::Mat blob;
    cv::dnn::blobFromImage(resized, blob, 1.0/255.0, cv::Size(), cv::Scalar(), true, false);
    net.setInput(blob);
    cv::Mat out = net.forward();
    out = out.reshape(1, {5, 8400});
    out = out.t();

    std::vector<Det> dets;
    for (int i = 0; i < out.rows; ++i) {
        const float* d = out.ptr<float>(i);
        float cx = d[0], cy = d[1], w = d[2], h = d[3], conf = d[4];
        // 640 공간 → 원본 좌표로 되돌린다 (패딩을 빼고 스케일로 나눈다)
        float x  = ((cx - w/2.f) - px) / sx;
        float y  = ((cy - h/2.f) - py) / sy;
        float ww = w / sx;
        float hh = h / sy;
        if (ww < min_wh || hh < min_wh) continue;
        dets.push_back({conf, cv::Rect(int(std::round(x)), int(std::round(y)),
                                       std::max(1,int(std::round(ww))),
                                       std::max(1,int(std::round(hh))))});
    }
    return dets;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: measure_conf <onnx> <frame_dir> [sheet.jpg] [stretch|letterbox]\n"
                     "  stretch(기본)   = 현재 object_detection.cpp 의 전처리\n"
                     "  letterbox       = ultralytics 학습 시 전처리\n";
        return 1;
    }
    Pre pre = Pre::Stretch;
    if (argc >= 5 && std::string(argv[4]) == "letterbox") pre = Pre::Letterbox;
    std::cout << "전처리: " << (pre == Pre::Stretch ? "stretch" : "letterbox") << "\n";

    cv::dnn::Net net = cv::dnn::readNet(argv[1]);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    std::vector<fs::path> files;
    for (auto& e : fs::directory_iterator(argv[2]))
        if (e.path().extension() == ".png") files.push_back(e.path());
    std::sort(files.begin(), files.end());
    if (files.empty()) { std::cerr << "no frames\n"; return 1; }

    std::vector<float> best;
    std::vector<std::vector<Det>> all;
    for (auto& f : files) {
        cv::Mat img = cv::imread(f.string());
        auto dets = infer(net, img, 12, pre);
        float mx = 0.f;
        for (auto& d : dets) mx = std::max(mx, d.conf);
        best.push_back(mx);
        all.push_back(std::move(dets));
    }

    std::vector<float> sorted = best;
    std::sort(sorted.begin(), sorted.end());
    float sum = 0; for (float v : best) sum += v;
    std::cout << "=== " << fs::path(argv[2]).filename().string()
              << " (" << best.size() << " frames) ===\n";
    std::cout << "  frame-best conf: max=" << sorted.back()
              << "  mean=" << sum/best.size()
              << "  median=" << sorted[sorted.size()/2] << "\n";
    for (float th : {0.83f, 0.70f, 0.50f, 0.30f, 0.15f, 0.05f}) {
        int n = 0; for (float v : best) if (v >= th) ++n;
        printf("  conf >= %.2f: %4d frames (%5.1f%%)\n", th, n, 100.0*n/best.size());
    }

    if (argc >= 4) {  // 대표 프레임 6장 시각화 (임계값 없이 상위 3박스)
        std::vector<cv::Mat> tiles;
        for (int k = 0; k < 6; ++k) {
            size_t i = files.size() * k / 6;
            cv::Mat img = cv::imread(files[i].string());
            auto dets = all[i];
            std::sort(dets.begin(), dets.end(),
                      [](const Det& a, const Det& b){ return a.conf > b.conf; });
            for (size_t j = 0; j < std::min<size_t>(3, dets.size()); ++j) {
                cv::Scalar c = dets[j].conf >= 0.83 ? cv::Scalar(0,255,0)
                             : dets[j].conf >= 0.30 ? cv::Scalar(0,165,255)
                                                    : cv::Scalar(0,0,255);
                cv::rectangle(img, dets[j].box, c, 2);
                cv::putText(img, cv::format("%.2f", dets[j].conf),
                            {dets[j].box.x, std::max(14, dets[j].box.y-5)},
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, c, 2);
            }
            cv::putText(img, cv::format("#%zu", i), {6,22},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,255,255}, 2);
            tiles.push_back(img);
        }
        cv::Mat r1, r2, sheet;
        cv::hconcat(std::vector<cv::Mat>(tiles.begin(), tiles.begin()+3), r1);
        cv::hconcat(std::vector<cv::Mat>(tiles.begin()+3, tiles.end()), r2);
        cv::vconcat(r1, r2, sheet);
        cv::imwrite(argv[3], sheet);
    }
    return 0;
}
