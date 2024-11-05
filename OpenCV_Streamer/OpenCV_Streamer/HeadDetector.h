#pragma once
#include <opencv2/opencv.hpp>

struct HeadDetection {
	cv::Point center;
	int radius;
};

class HeadDetector
{
public:
	HeadDetection find_head(cv::InputArray single_person);
private:
};

