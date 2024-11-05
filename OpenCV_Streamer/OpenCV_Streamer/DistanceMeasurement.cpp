#include "DistanceMeasurement.h"

//Distance to object(mm)=f(mm)¡¿real height(mm)¡¿image / height(pixels)object height(pixels)¡¿sensor height(mm)
DISTANCE_DATA DistanceMeasurement::calc_distance(const Detection& obj)
{
	cv::Rect rect = obj.box;
	DISTANCE_DATA res;
	res.distance = (f * human_height * 640) / (rect.height * 1780);
	return res;
}

DISTANCE_DATA DistanceMeasurement::calc_distance_face(const cv::Rect& face)
{
	return DISTANCE_DATA();
}



