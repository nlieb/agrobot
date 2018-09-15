#include <iostream>
#include <iomanip> 
#include <string>
#include <vector>
#include <queue>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable

#ifdef _WIN32
#define OPENCV
#define GPU
#endif


#include "yolo_v2_class.hpp"    // imported functions from DLL
#include "opencv.hpp"
#include "yolo_detector.hpp"

int main(int argc, char *argv[])
{
    std::string  cfg_file = "yolov3-obj.cfg"; // "cfg/yolov3.cfg";
    std::string  weights_file = "yolov3-obj_1200.weights"; // "yolov3.weights";
    std::string filename;

    if (argc > 1) filename = argv[1];

    cv::Mat mat_img = cv::imread(filename);
    YoloDetector yd = YoloDetector(cfg_file, weights_file);

    try {
        std::vector<bbox_t> result_vec = yd.detect_img(mat_img);
        mat_img = yd.draw_rois(mat_img, result_vec);
        cv::imwrite("./SAVED.jpg", mat_img);
    }
    catch (std::exception &e) { std::cerr << "exception: " << e.what() << "\n"; getchar(); }
    catch (...) { std::cerr << "unknown exception \n"; getchar(); }

    return 0;
}
