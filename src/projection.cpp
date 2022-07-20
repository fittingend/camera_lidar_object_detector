#include <iostream>
#include <stdio.h>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp> //OpenCV header to use VideoCapture class

#include "projection.h"

using namespace std;

void loadIntrinsic(cv::Mat &P_rect_00)
{
  FILE *ref = fopen(INTRINSIC_PATH.c_str(), "ro");
  char line[1024], c;
  int line_num = 0;

  fseek(ref, 0, SEEK_SET);
  while (1)
  {
    fgets(line, 1024, ref);
    c = strlen(line);
    if (c == 0)
    {
      break;
    }
    line_num++;
    line[0] = '\0';
  }
  fseek(ref, 0, SEEK_SET);

  fscanf(ref, "%lf %lf %lf", &P_rect_00.at<double>(0, 0), &P_rect_00.at<double>(0, 1), &P_rect_00.at<double>(0, 2));
  fscanf(ref, "%lf %lf %lf", &P_rect_00.at<double>(1, 0), &P_rect_00.at<double>(1, 1), &P_rect_00.at<double>(1, 2));
  fscanf(ref, "%lf %lf %lf", &P_rect_00.at<double>(2, 0), &P_rect_00.at<double>(2, 1), &P_rect_00.at<double>(2, 2));

  P_rect_00.at<double>(0, 3) = 0.0;
  P_rect_00.at<double>(1, 3) = 0.0;
  P_rect_00.at<double>(2, 3) = 0.0;

  printf("%lf %lf %lf %lf\n", P_rect_00.at<double>(0, 0), P_rect_00.at<double>(0, 1), P_rect_00.at<double>(0, 2), P_rect_00.at<double>(0, 3));
  printf("%lf %lf %lf %lf\n", P_rect_00.at<double>(1, 0), P_rect_00.at<double>(1, 1), P_rect_00.at<double>(1, 2), P_rect_00.at<double>(1, 3));
  printf("%lf %lf %lf %lf\n", P_rect_00.at<double>(2, 0), P_rect_00.at<double>(2, 1), P_rect_00.at<double>(2, 2), P_rect_00.at<double>(2, 3));
}

void loadExtrinsic(cv::Mat &RT)
{
  FILE *ref = fopen(EXTRINSIC_PATH.c_str(), "ro");
  char line[1024], c;
  int line_num = 0;

  fseek(ref, 0, SEEK_SET);
  while (1)
  {
    fgets(line, 1024, ref);
    c = strlen(line);
    if (c == 0)
    {
      break;
    }
    line_num++;
    line[0] = '\0';
  }
  fseek(ref, 0, SEEK_SET);

  fscanf(ref, "%lf %lf %lf %lf", &RT.at<double>(0, 0), &RT.at<double>(0, 1), &RT.at<double>(0, 2), &RT.at<double>(0, 3));
  fscanf(ref, "%lf %lf %lf %lf", &RT.at<double>(1, 0), &RT.at<double>(1, 1), &RT.at<double>(1, 2), &RT.at<double>(1, 3));
  fscanf(ref, "%lf %lf %lf %lf", &RT.at<double>(2, 0), &RT.at<double>(2, 1), &RT.at<double>(2, 2), &RT.at<double>(2, 3));
  fscanf(ref, "%lf %lf %lf %lf", &RT.at<double>(3, 0), &RT.at<double>(3, 1), &RT.at<double>(3, 2), &RT.at<double>(3, 3));

  printf("%lf %lf %lf %lf\n", RT.at<double>(0, 0), RT.at<double>(0, 1), RT.at<double>(0, 2), RT.at<double>(0, 3));
  printf("%lf %lf %lf %lf\n", RT.at<double>(1, 0), RT.at<double>(1, 1), RT.at<double>(1, 2), RT.at<double>(1, 3));
  printf("%lf %lf %lf %lf\n", RT.at<double>(2, 0), RT.at<double>(2, 1), RT.at<double>(2, 2), RT.at<double>(2, 3));
  printf("%lf %lf %lf %lf\n", RT.at<double>(3, 0), RT.at<double>(3, 1), RT.at<double>(3, 2), RT.at<double>(3, 3));
}