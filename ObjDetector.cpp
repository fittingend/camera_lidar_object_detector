
#include "rs_driver/api/lidar_driver.h"
#include "lib/lidar_worker.h"
#include "lib/lidar_out.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <thread>
#include <queue>
//#include <term.h>
#include <termios.h>
#include <unistd.h>

#include <fstream> 
#include <opencv2/opencv.hpp>  //OpenCV header to use VideoCapture class

// user include
#include "ConfigParser.h"
#include "yolo.h"

std::vector<pcl::PointXYZI> xyzi_vector(1000+1);


/*
================================
config.ini
================================
*/

CConfigParser UserCfg("./config.ini");

int CAM_PORT     = UserCfg.GetInt("CAM_PORT");
string WEIGHTS_PATH     = UserCfg.GetString("WEIGHTS_PATH");
string CFG_PATH         = UserCfg.GetString("CFG_PATH");
string CLASSES_PATH     = UserCfg.GetString("CLASSES_PATH");

float SCALE_FACTOR     = UserCfg.GetFloat("SCALE_FACTOR");


/*
================================
최적화된 라이다 xyz축 필터값
================================
*/
float minZ_filter = -0.38;
// float minZ_filter = -0.27;
float maxZ_filter = 2;
float minY_filter = -20;
float maxY_filter = 20;

/*
===========================================
demo_online 실행시 키보드 값을 받는 함수 getch()
===========================================
*/
int getch(void)
{
  int ch;
  struct termios buf;
  struct termios save;

  tcgetattr(0, &save);
  buf = save;
  buf.c_lflag &= ~(ICANON | ECHO);
  buf.c_cc[VMIN] = 1;
  buf.c_cc[VTIME] = 0;
  tcsetattr(0, TCSAFLUSH, &buf);
  ch = getchar();
  tcsetattr(0, TCSAFLUSH, &save);
  return ch;
}

/*
=========================================
thread handler():키보드 값을 받아 실시간으로 clustering 에 영향을 미치는 값을 조정한다
_________________________________________
키보드 값 | 의미               
0       | 증가/감소를 (+/-) 바꾼다
1       | leafsize 0.1 단위로 변화 
2       | clusterTolerance 를 0.1 단위로 변화 
3       | minClusterSize 를 1 단위로 변화 
4       | maxClusterSize 를 1 단위로 변화
5       | minZ_filter 를 0.1 단위로 변화
6       | maxZ_filter 를 0.1 단위로 변화
7       | minY_filter 를 0.1 단위로 변화
8       | maxY_filter 를 0.1 단위로 변화
===================================
*/

void keyboard_input_handler()
{
  char input;
  bool flag = 1;
  while (1)
  {
    input = getch();
    switch (input)
    {
      case '0':
        flag ^= 1;
        std::cout << "flag : " << flag << std::endl;
        break;
      case '1':
        if (flag)
          leafsize += 0.1;
        else
          leafsize -= 0.1;
        std::cout << "leafsize : " << leafsize << std::endl;
        break;
      case '2':
        if (flag)
          clusterTolerance += 0.1;
        else
          clusterTolerance -= 0.1;
        std::cout << "clusterTolerance : " << clusterTolerance << std::endl;
        break;
      case '3':
        if (flag)
          minClusterSize += 1;
        else
          minClusterSize -= 1;
        std::cout << "minClusterSize : " << minClusterSize << std::endl;
        break;
      case '4':
        if (flag)
          maxClusterSize += 1;
        else
          maxClusterSize -= 1;
        std::cout << "maxClusterSize : " << maxClusterSize << std::endl;
        break;
      case '5':
        if (flag)
          minZ_filter += 0.1;
        else
          minZ_filter -= 0.1;
        std::cout << "minZ_filter : " << minZ_filter << std::endl;
        break;
      case '6':
        if (flag)
          maxZ_filter += 0.1;
        else
          maxZ_filter -= 0.1;
        std::cout << "maxZ_filter : " << maxZ_filter << std::endl;
        break;
      case '7':
        if (flag)
          minY_filter += 0.1;
        else
          minY_filter -= 0.1;
        std::cout << "minY_filter : " << minY_filter << std::endl;
        break;
      case '8':
        if (flag)
          maxY_filter += 0.1;
        else
          maxY_filter -= 0.1;
        std::cout << "maxY_filter : " << maxY_filter << std::endl;
        break;
    }
  }
}

void loadCalibrationData(cv::Mat& P_rect_00, cv::Mat& RT)
{
  RT.at<double>(0, 0) = -4.410411534819819179e-02;
  RT.at<double>(0, 1) = -9.989318195782288523e-01;
  RT.at<double>(0, 2) = -1.378574783904967793e-02;
  RT.at<double>(0, 3) = 1.556009358886184871e-02;
  RT.at<double>(1, 0) = -2.717181149355868408e-02;
  RT.at<double>(1, 1) = 1.499351210954324998e-02;
  RT.at<double>(1, 2) = -9.995183276232504355e-01;
  RT.at<double>(1, 3) = 6.512429988850447493e-02;
  RT.at<double>(2, 0) = 9.986573584916456081e-01;
  RT.at<double>(2, 1) = -4.370828787255359726e-02;
  RT.at<double>(2, 2) = -2.780406268405100079e-02;
  RT.at<double>(2, 3) = 7.613610580158369778e-02;
  RT.at<double>(3, 0) = 0.0;
  RT.at<double>(3, 1) = 0.0;
  RT.at<double>(3, 2) = 0.0;
  RT.at<double>(3, 3) = 1.0;

  P_rect_00.at<double>(0, 0) = 1985.56984;
  P_rect_00.at<double>(0, 1) = 0.000000e+00;
  P_rect_00.at<double>(0, 2) = 1015.90116;
  P_rect_00.at<double>(0, 3) = 0.000000e+00;
  P_rect_00.at<double>(1, 0) = 0.000000e+00;
  P_rect_00.at<double>(1, 1) = 1990.54053;
  P_rect_00.at<double>(1, 2) = 575.9691;
  P_rect_00.at<double>(1, 3) = 0.000000e+00;
  P_rect_00.at<double>(2, 0) = 0.000000e+00;
  P_rect_00.at<double>(2, 1) = 0.000000e+00;
  P_rect_00.at<double>(2, 2) = 1.000000e+00;
  P_rect_00.at<double>(2, 3) = 0.000000e+00;
}

void projection_handler()
{
  uint32_t frame_cnt = 0, total_frames = 0;
  float fps = 0.;

  cv::Mat myImage, overlay;
  cv::VideoCapture capture;

  capture.open(CAM_PORT);
  if(!capture.isOpened()){
      std::cerr << "Error opening video file\n";
      return;
  }

  // yolov4 init
  std::vector<std::string> class_list = load_class_list();
  bool is_cuda = cv::cuda::getCudaEnabledDeviceCount();

  cv::dnn::Net net;
  load_net(net, is_cuda);

  auto model = cv::dnn::DetectionModel(net);
  model.setInputParams(1./255, cv::Size(416, 416), cv::Scalar(), true);


  // pcl to 2d image plane
  cv::Mat P_rect_00(3, 4, cv::DataType<double>::type);  // 3x4 projection matrix after rectification
  //    cv::Mat R_rect_00(4, 4, cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
  cv::Mat RT(4, 4, cv::DataType<double>::type);  // rotation matrix and translation vector
  loadCalibrationData(P_rect_00, RT);

  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  auto start = std::chrono::high_resolution_clock::now();
  while (true)
  {  
    capture.read(myImage);
    if (myImage.empty())
    {  // Breaking the loop if no video frame is detected//
      break;
    }

    cv::resize(myImage, myImage, cv::Size((int)(myImage.cols / SCALE_FACTOR) + 1, (int)(myImage.rows / SCALE_FACTOR) + 1));

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    model.detect(myImage, classIds, confidences, boxes, .2, .4);
    total_frames++;
    frame_cnt++;

    for (auto it = xyzi_vector.begin(); it != xyzi_vector.end(); ++it)
    {
      
      // 1. Convert current Lidar point into homogeneous coordinates and store it in the 4D variable X.
      X.at<double>(0, 0) = it->x;
      X.at<double>(1, 0) = it->y;
      X.at<double>(2, 0) = it->z;
      X.at<double>(3, 0) = 1;

      // 2. Then, apply the projection equation as detailed in lesson 5.1 to map X onto the image plane of the camera.
      // Y = P_rect_00 * R_rect_00 * RT * X;
      Y = P_rect_00 * RT * X;

      // 3. Once this is done, transform Y back into Euclidean coordinates and store the result in the variable pt.
      cv::Point pt;
      pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
      pt.x = (int) (pt.x / SCALE_FACTOR);

      pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
      pt.y = (int) (pt.y / SCALE_FACTOR);
 
      float val = it->x;
      float maxVal = 20.0;
      int red = min(255, (int)(255 * abs((val - maxVal) / maxVal)));
      int green = min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));

      cv::circle(myImage, pt, 1, cv::Scalar(0, green, red), -1);
    }

    int detections = classIds.size();
    for (int i = 0; i < detections; ++i) {

        auto box = boxes[i];
        auto confidence = confidences[i];
        auto classId = classIds[i];
        const auto color = colors[classId % colors.size()];

        // std::cout << box << std::endl;
        cv::rectangle(myImage, box, color, 3);
        cv::rectangle(myImage, cv::Point(box.x, box.y - 20), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
        
        char obj_msg[100];
        sprintf(obj_msg, "%s:%.2f", class_list[classId].c_str(), confidence );
        cv::putText(myImage, obj_msg, cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
    }

    if(frame_cnt >= 30){
        auto end = std::chrono::high_resolution_clock::now();
        fps = frame_cnt * 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        frame_cnt = 0;
        start = std::chrono::high_resolution_clock::now();
    }

    if(fps > 0){
        std::ostringstream fps_label;
        fps_label << std::fixed << std::setprecision(2);
        fps_label << "[FPS] " << fps;
        std::string fps_label_str = fps_label.str();

        cv::putText(myImage, fps_label_str.c_str(), cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    }

    string windowName = "LiDAR data on image overlay";
    cv::namedWindow(windowName, 3);
    cv::imshow(windowName, myImage);
    if(cv::waitKey(1) == 'q'){
        capture.release();
        std::cout << "finished by user\n";
        break;
    }
    
  }
  capture.release();  // Releasing the buffer memory//
}
  using namespace robosense::lidar;
  using namespace pcl::visualization;
  std::shared_ptr<PCLVisualizer> pcl_viewer;

  float leafsize = 0.7;
  float clusterTolerance = 1;  // 1?
  int minClusterSize = 5;
  int maxClusterSize = 100;

  std::mutex mex_viewer;
  void colorize(const pcl::PointCloud<PointXYZIC>& pc, pcl::PointCloud<pcl::PointXYZRGB>& pc_colored,
                const std::vector<int>& color);


  struct PointXYZI  ///< user defined point type
  {
    float x;
    float y;
    float z;
    uint8_t intensity;
  };

  /**
   * @brief The point cloud callback function. This function will be registered to lidar driver.
   *              When the point cloud message is ready, driver can send out messages through this function.
   * @param msg  The lidar point cloud message.
   */

  void pointCloudCallback(const PointCloudMsg<pcl::PointXYZI>& msg)
  {
    // RS_MSG << "msg: " << msg.seq << " point cloud size: " << msg.point_cloud_ptr->size() << RS_REND;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_filtered_z(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_filtered_yz(new pcl::PointCloud<pcl::PointXYZI>);
    //  Visualization *_vs;
    //  pcl::PointCloud<PointXYZIR> msg_returned;
    // pcl::PointCloud<pcl::PointXYZI> new_msg_returned;
    pcl::PointCloud<PointXYZIC> new_msg_returned;

    // pcl::PointCloud<pcl::PointXYZI>::Ptr clustering_msg(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<PointXYZIC>::Ptr clustering_msg(new pcl::PointCloud<PointXYZIC>);

    //  LIDAR_OUT *output;

    /* Note: Please do not put time-consuming operations in the callback function! */
    /* Make a copy of the message and process it in another thread is recommended*/
    //  RS_MSG << "msg: " << msg.seq << " point cloud size: " << msg.point_cloud_ptr->size() << RS_REND;

    pcl_pointcloud->points.assign(msg.point_cloud_ptr->begin(), msg.point_cloud_ptr->end());
    pcl_pointcloud->height = msg.height;
    pcl_pointcloud->width = msg.width;
    pcl_pointcloud->is_dense = false;

    // cout << pcl_pointcloud->points.size() << endl;

    //오리지널 PCL 프로젝션용 vector에 assign
    // for (int i = 0; i < pcl_pointcloud->points.size(); i++)
    // {
    //   xyzi_vector[i].x = pcl_pointcloud->points[i].x;
    //   xyzi_vector[i].y = pcl_pointcloud->points[i].y;
    //   xyzi_vector[i].z = pcl_pointcloud->points[i].z;
    //   xyzi_vector[i].intensity = pcl_pointcloud->points[i].intensity;
    // }

    // passthorugh 로 포인트 클라우드 값 필터
    //  X 축으로 filtering
    //  중앙 부분 추출
    pcl::PassThrough<pcl::PointXYZI> zfilter;
    zfilter.setInputCloud(pcl_pointcloud);
    zfilter.setFilterFieldName("z");
    zfilter.setFilterLimits(minZ_filter, maxZ_filter);
    //  zfilter.setFilterLimits(-1.5, 3);
    zfilter.setFilterLimitsNegative(false);
    zfilter.filter(*pcl_pointcloud_filtered_z);

    pcl::PassThrough<pcl::PointXYZI> yfilter;
    yfilter.setInputCloud(pcl_pointcloud_filtered_z);
    yfilter.setFilterFieldName("y");
    yfilter.setFilterLimits(minY_filter, maxY_filter);
    yfilter.setFilterLimitsNegative(false);
    yfilter.filter(*pcl_pointcloud_filtered_yz);

    // voxel test
    //  pcl::PointCloud<pcl::PointXYZI> laserCloudIn = *pcl_pointcloud;

    // pcl::VoxelGrid<pcl::PointXYZI> vg;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_v2(new pcl::PointCloud<pcl::PointXYZI>);
    // vg.setInputCloud(laserCloudIn.makeShared());
    // vg.setLeafSize(0.9, 0.9, 0.9);  // original 0.5f; the larger it is the more downsampled it gets
    // vg.filter(*cloud_filtered_v2);

    uint16_t obj_cnt =
        lidar_clustering("hello", pcl_pointcloud_filtered_yz, new_msg_returned);  // Lidar clustering  진행

    // colorize
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ori_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    *clustering_msg = new_msg_returned;
    // std::cout << clustering_msg->points.size() << std::endl;
    for (int i = 0; i < clustering_msg->points.size(); i++)
    {
      xyzi_vector[i].x = clustering_msg->points[i].x;
      xyzi_vector[i].y = clustering_msg->points[i].y;
      xyzi_vector[i].z = clustering_msg->points[i].z;
      xyzi_vector[i].intensity = clustering_msg->points[i].intensity;
    }


    // Point cloud XYZ에 RGB 칼라 추가하기
    colorize(*clustering_msg, *tgt_colored, { 255, 0, 0 });
    // colorize1(*pcl_pointcloud, *ori_colored, {255,255,255});

    // pcl::PointCloud<pcl::PointXYZRGB> tgt_colored_test = *tgt_colored;

    //   for (int i=0; i<new_msg_returned.points.size(); i++){
    //     tgt_colored_test.points[i].z += 5;
    //   }

    // *combined_colored += *ori_colored;
    *combined_colored += *tgt_colored;
    // *combined_colored += tgt_colored_test;

    // PointCloudColorHandlerRGBField<pcl::PointXYZRGB> point_color_handle(tgt_colored);
    PointCloudColorHandlerRGBField<pcl::PointXYZRGB> point_color_handle(combined_colored);
    {
      const std::lock_guard<std::mutex> lock(mex_viewer);
      // pcl_viewer->updatePointCloud<pcl::PointXYZRGB>(tgt_colored, point_color_handle, "rslidar");
      pcl_viewer->updatePointCloud<pcl::PointXYZRGB>(combined_colored, point_color_handle, "rslidar");
    }
  }

  /**
   * @brief The exception callback function. This function will be registered to lidar driver.
   * @param code The error code struct.
   */
  void exceptionCallback(const Error& code)
  {
    /* Note: Please do not put time-consuming operations in the callback function! */
    /* Make a copy of the error message and process it in another thread is recommended*/
    RS_WARNING << code.toString() << RS_REND;
  }

  int main(int argc, char* argv[])
  {
    RS_TITLE << "------------------------------------------------------" << RS_REND;
    RS_TITLE << "            RS_Driver Viewer Version: v" << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR
             << "." << RSLIDAR_VERSION_PATCH << RS_REND;
    RS_TITLE << "------------------------------------------------------" << RS_REND;

    if (argc < 2)
    {
      RS_INFOL << "Use 'rs_driver_viewer -h/--help' to check the argument menu..." << RS_REND;
    }
    /*if (checkKeywordExist(argc, argv, "-h") || checkKeywordExist(argc, argv, "--help"))
    {
      printHelpMenu();
      return 0;
    }*/
    pcl_viewer = std::make_shared<PCLVisualizer>("RSPointCloudViewer");

    pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
    pcl_viewer->addCoordinateSystem(1.0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl_viewer->addPointCloud<pcl::PointXYZI>(pcl_pointcloud, "rslidar");
    pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "rslidar");

    LidarDriver<pcl::PointXYZI> driver;   ///< Declare the driver object
    RSDriverParam param;                  ///< Create a parameter object
    param.input_param.read_pcap = false;  ///< Set read_pcap to true
    param.input_param.pcap_repeat = true;
    param.input_param.pcap_path = "/home/sujin/Downloads/rs_driver-main_keyboard/demo/pcap_data/"
                                  "2022-05-27-incheon_right_assist.pcap";  ///< Set the pcap file directory
    param.input_param.msop_port = 6699;  ///< Set the lidar msop port number, the default is 6699
    param.wait_for_difop = false;
    param.input_param.difop_port = 7788;  ///< Set the lidar difop port number, the default is 7788
    param.lidar_type = LidarType::RSM1;   ///< Set the lidar type. Make sure this type is correct
    param.print();

    driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function into the driver
    driver.regRecvCallback(pointCloudCallback);      ///< Register the point cloud callback function into the driver
    if (!driver.init(param))                         ///< Call the init function and pass the parameter
    {
      RS_ERROR << "Driver Initialize Error..." << RS_REND;
      return -1;
    }
    driver.start();  ///< The driver thread will start
    RS_DEBUG << "RoboSense Lidar-Driver Linux online demo start......" << RS_REND;

    thread t1(keyboard_input_handler);
    thread t2(projection_handler);

    while (!pcl_viewer->wasStopped())
    {
      {
        const std::lock_guard<std::mutex> lock(mex_viewer);
        pcl_viewer->spinOnce();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    t1.join();
    t2.join();
    return 0;
  }

  void colorize(const pcl::PointCloud<PointXYZIC>& pc, pcl::PointCloud<pcl::PointXYZRGB>& pc_colored,
                const std::vector<int>& color)
  {
    int N = pc.points.size();
    int c;
    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;
    for (int i = 0; i < N; ++i)
    {
      const auto& pt = pc.points[i];
      pt_tmp.x = pt.x;
      pt_tmp.y = pt.y;
      pt_tmp.z = pt.z;
      switch (pt.c)
      {
        case 1:  // red
          pt_tmp.r = 255;
          pt_tmp.g = 0;
          pt_tmp.b = 0;
          break;

        case 2:  // orange
          pt_tmp.r = 255;
          pt_tmp.g = 128;
          pt_tmp.b = 0;
          break;

        case 3:  // yellow
          pt_tmp.r = 255;
          pt_tmp.g = 255;
          pt_tmp.b = 0;
          break;

        case 4:  // green
          pt_tmp.r = 128;
          pt_tmp.g = 255;
          pt_tmp.b = 0;
          break;

        case 5:  // blue
          pt_tmp.r = 0;
          pt_tmp.g = 255;
          pt_tmp.b = 255;
          break;

        case 6:  // navy
          pt_tmp.r = 0;
          pt_tmp.g = 0;
          pt_tmp.b = 255;
          break;

        case 7:  // purple
          pt_tmp.r = 127;
          pt_tmp.g = 0;
          pt_tmp.b = 255;
          break;

        default:
          pt_tmp.r = 255;
          pt_tmp.g = 255;
          pt_tmp.b = 255;
          break;
      }
      pc_colored.points.emplace_back(pt_tmp);
    }
  }
 