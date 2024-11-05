#include "HeadDetector.h"

//�־��� ȭ�鿡�� ������ ã�Ƴ��� �� �� ���� ū���� ��ȯ�ϸ� ���� ������?
HeadDetection HeadDetector::find_head(cv::InputArray single_person)
{
	int largest = 0;
	cv::Mat gray;
	cv::cvtColor(single_person, gray, cv::COLOR_RGB2GRAY);

	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 200, 100, 0, 0);
	
	for (int i = 1; i < circles.size(); i++) {
		if (circles[i][2] > circles[largest][2]) largest = i;
	}
	HeadDetection head;
	cv::Vec3f c = circles[largest];
	cv::Point center = cv::Point(cvRound(c[0]), cvRound(c[1]));

	head.center = center;
	head.radius = c[2];
	
	return head;
}
