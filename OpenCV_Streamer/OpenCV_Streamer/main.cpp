#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/cudaarithm.hpp>

#include "inference.h"
#include "DistanceMeasurement.h"
#include "HeadDetector.h"
#include "WebSocketSender.h"

using namespace std;
float get_ref_pixel_width(Inference* inf, cv::Mat* img);

int main(int argc, char** argv)
{
    WebSocketSender wss;
    //wss.connectwebsocket();
    //wss.connectudp();
    

    /*wss.test();
    return 0;*/
    std::string projectBasePath = ""; // Set your ultralytics base path

    bool runOnGPU = true;
    auto start = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    float fps = -1;
    int total_frames = 0;

    cv::Mat frame;
    cv::VideoCapture capture(0); //camera
    if (!capture.isOpened()) {
        std::cerr << "Error opening video file\n";
        return -1;
    }
    
    try {
        cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());
        int cuda_devices_number = cv::cuda::getCudaEnabledDeviceCount();
        cout << "CUDA Device(s) Number: " << cuda_devices_number << endl;
        cv::cuda::DeviceInfo _deviceInfo;
        bool _isd_evice_compatible = _deviceInfo.isCompatible();
        cout << "CUDA Device(s) Compatible: " << _isd_evice_compatible << endl;
    }
    catch (cv::Exception e){
        cerr << e.msg;
        cout << "\nCUDA Device Required!\n";
        return -1;
    }
    //
    // Pass in either:
    //
    // "yolov8s.onnx" or "yolov5s.onnx"
    //
    // To run Inference with yolov8/yolov5 (ONNX)
    //

    // Note that in this example the classes are hard-coded and 'classes.txt' is a place holder.
    Inference inf(projectBasePath + "./yolo/yolov8s.onnx", cv::Size(640, 640), "classes.txt", runOnGPU);
    //cv::CascadeClassifier face_detector = cv::CascadeClassifier("./face/haarcascade_frontalface_default.xml");
    cv::CascadeClassifier face_detector = cv::CascadeClassifier("./face/haarcascade_mcs_upperbody.xml");
    //Inference inf(projectBasePath + "./yolo/yolov8n-face_no_optim.onnx", cv::Size(640, 640), "classes.txt", runOnGPU);
    DistanceMeasurement dm;

    //focal length를 구하기 위해 첫 프레임만 레퍼런스 이미지로 활용
    capture.read(frame);
    float ref_width = get_ref_pixel_width(&inf, &frame);
    float focal_length = dm.get_focal_length(ref_width);

    //wss.sendframe_via_udp(frame);

    while(true)
    {
        capture.read(frame);
        if (frame.empty()) {
            cout << "End of stream\n";
            break;
        }

        //face detect
        vector<cv::Rect> faces;
        //face_detector.detectMultiScale(frame, faces);
        //face detect end;

        // Inference starts here...
        std::vector<Detection> output = inf.runInference(frame);
        
        frame_count++;
        total_frames++;

        int detections = output.size();
        //std::cout << "Number of detections:" << detections << std::endl;

        for (int i = 0; i < detections; ++i)
        {
            Detection detection = output[i];

            cv::Rect box = detection.box;
            cv::Scalar color = detection.color;

            // Detection box
            cv::rectangle(frame, box, color, 2);

            float distance = dm.get_distance(focal_length, box.width);
            // Detection box text
            std::string classString = detection.className + ' ' + std::to_string(detection.confidence).substr(0, 4) + ' ' + to_string(distance);
            cv::Size textSize = cv::getTextSize(classString, cv::FONT_HERSHEY_DUPLEX, 1, 2, 0);
            cv::Rect textBox(box.x, box.y - 40, textSize.width + 10, textSize.height + 20);

            cv::rectangle(frame, textBox, color, cv::FILLED);
            cv::putText(frame, classString, cv::Point(box.x + 5, box.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
        }
        // Inference ends here...

        //face draw
        for (int i = 0; i < faces.size(); i++) {
            cv::Rect face = faces.at(i);
            DISTANCE_DATA distance = dm.calc_distance_face(face);
            cv::rectangle(frame,face.tl(), face.br(), cv::Scalar(255, 0, 255), 3);
            cv::putText(frame, to_string(distance.distance), cv::Point(face.x + 5, face.y - 10), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 0, 0), 2, 0);
        }

        if (frame_count >= 30)
        {

            auto end = std::chrono::high_resolution_clock::now();
            fps = frame_count * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            frame_count = 0;
            start = std::chrono::high_resolution_clock::now();
        }

        if (fps >= 0)
        {

            std::ostringstream fps_label;
            fps_label << std::fixed << std::setprecision(2);
            fps_label << "FPS: " << fps;
            std::string fps_label_str = fps_label.str();

            cv::putText(frame, fps_label_str.c_str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }

        // This is only for preview purposes
        //float scale = 1;
        //cv::resize(frame, frame, cv::Size(frame.cols * scale, frame.rows * scale));
        wss.sendframe_via_udp(frame);
        cv::imshow("Inference", frame);

        if (cv::waitKey(1) != -1)
        {
            capture.release();
            cout << "finished by user\n";
            break;
        }
    }
    cout << "Total frames: " << total_frames << "\n";
}

float get_ref_pixel_width(Inference* inf, cv::Mat* img) {
    vector<Detection> mono_person;
    mono_person = inf->runInference(*img);
    if (mono_person.size() != 1) {
        cout << (mono_person.size() > 1 ? "Single Person only" : "Person Not Detected") << endl;
        return -1;
    }
    return mono_person[0].box.width;
}