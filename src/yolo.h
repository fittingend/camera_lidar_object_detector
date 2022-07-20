using namespace std;

extern string WEIGHTS_PATH;
extern string CFG_PATH;
extern string CLASSES_PATH;

const std::vector<cv::Scalar> colors = {cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 0)};

std::vector<std::string> load_class_list();
void load_net(cv::dnn::Net &net, bool is_cuda);

