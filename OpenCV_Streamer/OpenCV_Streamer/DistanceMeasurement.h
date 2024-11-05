#pragma once
#include <opencv2/opencv.hpp>
#include "inference.h"

struct DISTANCE_DATA {
	float distance;
	//Detection* detect;
};
//https://photo.stackexchange.com/questions/12434/how-do-i-calculate-the-distance-of-an-object-in-a-photo <- 거리계산 알고리즘
//https://github.com/Asadullah-Dal17/Distance_measurement_using_single_camera/blob/main/distance.py#L46 <- example
class DistanceMeasurement
{
public:
	DISTANCE_DATA calc_distance(const Detection& obj);
	DISTANCE_DATA calc_distance_face(const cv::Rect& face);
	inline float get_focal_length(const float& ref_obj_width) { return (ref_obj_width * KNOWN_DISTANCE) / KNOWN_WIDTH; }
	inline float get_distance(const float& focal_length, const float& pixel_width) { return (KNOWN_WIDTH * focal_length) / pixel_width; }
private:
	const float human_height = 1700.0f; // 일반적인 사람의 키 mm단위
	const float f = 1;

	//const float KNOWN_DISTANCE = 200.0f; // centimeter
	const float KNOWN_DISTANCE = 100.0f; // centimeter
	const float KNOWN_WIDTH = 40.0f;
	
};

