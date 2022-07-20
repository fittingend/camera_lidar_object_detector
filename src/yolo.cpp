#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp> //OpenCV header to use VideoCapture class

#include "yolo.h"

using namespace std;

std::vector<std::string> load_class_list() {
    std::vector<std::string> class_list;
    std::ifstream ifs(CLASSES_PATH);
    std::string line;
    while (getline(ifs, line))
    {
        class_list.push_back(line);
    } 
    return class_list;
}

void load_net(cv::dnn::Net &net, bool is_cuda) {
    auto result = cv::dnn::readNetFromDarknet(CFG_PATH, WEIGHTS_PATH);
    if (is_cuda) {
        std::cout << "Attempty to use CUDA\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA_FP16);
    } else {
        std::cout << "Running on CPU\n";
        result.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        result.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    net = result;
}