#include <gtest/gtest.h>

#include <vector>

#include <opencv2/core.hpp>

#include "lane_detection/class_map.hpp"

namespace
{

// pidnet(infer_pidnet.py)의 참조 구현:
//   mask = np.isin(labels, lane_classes).astype(np.uint8) * 255
// 클래스맵에서 뽑은 마스크가 이것과 한 픽셀도 다르면 안 된다. 그래야 입력을
// /lane_segmentation_mask 에서 /pidnet_class_map 으로 바꿔도 주행이 안 바뀐다.
cv::Mat referencePidnetMask(const cv::Mat & labels, const std::vector<int> & classes)
{
  cv::Mat mask = cv::Mat::zeros(labels.size(), CV_8UC1);
  for (int y = 0; y < labels.rows; ++y) {
    for (int x = 0; x < labels.cols; ++x) {
      const int label = labels.at<uchar>(y, x);
      const bool hit =
        std::find(classes.begin(), classes.end(), label) != classes.end();
      mask.at<uchar>(y, x) = hit ? 255 : 0;
    }
  }
  return mask;
}

// 6개 클래스(background/center_lane/left_solid/right_solid/road/shortcut)가
// 골고루 섞인 라벨 영상.
cv::Mat syntheticLabels(int rows, int cols)
{
  cv::Mat labels(rows, cols, CV_8UC1);
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      labels.at<uchar>(y, x) = static_cast<uchar>((x * 7 + y * 13) % 6);
    }
  }
  return labels;
}

}  // namespace

TEST(ClassMapExtraction, CenterLaneMatchesPidnetReference)
{
  const cv::Mat labels = syntheticLabels(37, 53);
  const std::vector<int> classes{1};

  const cv::Mat produced = lane_detection::extractClassMask(labels, classes);
  const cv::Mat expected = referencePidnetMask(labels, classes);

  EXPECT_EQ(0, cv::countNonZero(produced != expected));
}

// lane_classes 를 [1,2,3] 으로 띄우던 구성도 그대로 재현돼야 한다.
TEST(ClassMapExtraction, MultipleClassesMatchPidnetReference)
{
  const cv::Mat labels = syntheticLabels(41, 29);
  const std::vector<int> classes{1, 2, 3};

  const cv::Mat produced = lane_detection::extractClassMask(labels, classes);
  const cv::Mat expected = referencePidnetMask(labels, classes);

  EXPECT_EQ(0, cv::countNonZero(produced != expected));
}

TEST(ClassMapExtraction, RailClassesAreSeparable)
{
  const cv::Mat labels = syntheticLabels(24, 24);

  const cv::Mat left = lane_detection::extractClassMask(labels, 2);
  const cv::Mat right = lane_detection::extractClassMask(labels, 3);

  // 두 레일은 겹치지 않는다.
  cv::Mat overlap;
  cv::bitwise_and(left, right, overlap);
  EXPECT_EQ(0, cv::countNonZero(overlap));

  EXPECT_EQ(0, cv::countNonZero(left != referencePidnetMask(labels, {2})));
  EXPECT_EQ(0, cv::countNonZero(right != referencePidnetMask(labels, {3})));
}

// road(4)/shortcut(5) 가 차선으로 새어 들어오면 안 된다. 클래스맵을 그대로
// threshold 하던 실수를 막는 회귀 테스트다.
TEST(ClassMapExtraction, DrivableSurfaceIsNotLane)
{
  cv::Mat labels = cv::Mat::zeros(10, 10, CV_8UC1);
  labels.setTo(4);                       // 전부 road
  labels.at<uchar>(5, 5) = 1;            // 중앙선 한 점

  const cv::Mat mask = lane_detection::extractClassMask(labels, 1);
  EXPECT_EQ(1, cv::countNonZero(mask));
  EXPECT_EQ(255, mask.at<uchar>(5, 5));
}

TEST(ClassMapExtraction, EmptyClassListProducesEmptyMask)
{
  const cv::Mat labels = syntheticLabels(8, 8);
  const cv::Mat mask =
    lane_detection::extractClassMask(labels, std::vector<int>{});
  EXPECT_EQ(0, cv::countNonZero(mask));
}
