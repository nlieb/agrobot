#include <iostream>
#include <iomanip> 
#include <string>
#include <vector>

#include "yolo_v2_class.hpp"    // imported functions from DLL
#include "opencv.hpp"


// Hardcode
class YoloDetector{
    public:
        YoloDetector(std::string cfg_file, std::string weights_file) : detector(cfg_file, weights_file){
            obj_names.push_back("weed");
        }

        std::vector<bbox_t> detect_img(cv::Mat mat_img){ 
            auto start = std::chrono::steady_clock::now();
            std::vector<bbox_t> result_vec = detector.detect(mat_img);
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> spent = end - start;
            std::cout << " Time: " << spent.count() << " sec \n";

            show_console_result(result_vec);
            return result_vec;
        }

        cv::Mat draw_rois(cv::Mat mat_img, std::vector<bbox_t> result_vec) {
            draw_boxes(mat_img, result_vec, obj_names);
            return mat_img;
        }

    private:
        void show_console_result(std::vector<bbox_t> const result_vec) {
            for (auto &i : result_vec) {
                if (obj_names.size() > i.obj_id) std::cout << obj_names[i.obj_id] << " - ";
                std::cout << "obj_id = " << i.obj_id << ",  x = " << i.x << ", y = " << i.y 
                    << ", w = " << i.w << ", h = " << i.h
                    << std::setprecision(3) << ", prob = " << i.prob << std::endl;
            }
        }
		std::vector<std::string> obj_names;
        Detector detector;
};
