# 개발과정

### 1. 카메라-라이다 캘리브레이션 

#### i) 관련 이론지식

```
https://darkpgmr.tistory.com/32
//에서 발췌 및 요약정리
```
라이다의 3D 공간좌표를 카메라의 2D 영상좌표로 변환하기 위해선 이 사이의 **변환관계**를 설명할수 있는 패러미터를 구해야함
![projection](./img/projection.png)

* (X,Y,Z): 월드 좌표계(world coordinate system)상의 3D 포인트의 좌표
* [R|t] : 월드 좌표계를 카메라 좌표계로 변환시키기 위한 회전(rotational matrix)과 이동변환 행렬 (translational matrix). 이 둘  [R|t] 를합쳐 **카메라 외부 파라미터(extrinsic parameter)**라 부름. 카메라의 설치 높이, 방향(팬, 틸트) 등 카메라와 외부 공간과의 기하학적 관계에 관련된 파라미터
* A: 카메라 내부 패러미터 (intrinsic camera matrix / intrinsic parameter) 로 카메라의 초점 거리, aspect ratio, 중심점 등 카메라 자체의 내부적인 파라미터를 의미
	* a) 초점거리 (focal length): fx, fy
	* b) 주점 (principal point): cx, cy
	* c) 비대칭 계수 (skew coefficient): skew_c = tanα ;  이미지 센서의 cell array의 y축이 기울어진 정도 (α도). 최신 카메라는 대부분 0 값을 가짐. 

#### ii) 캘리브레이션 진행
아래와 같은 Git repo 를 이용해서 진행. 실행 순서에 따라 나열함
```
https://github.com/RoboSense-LiDAR/rslidar_sdk
//RS lidar sdk 를 이용해 real time 으로 RS 라이다 point cloud 를  ROS topic 으로 보내기 

https://github.com/HViktorTsoi/rs_to_velodyne
//RS lidar topic 을 velodyne topic 형태로 바꿔주는 repo 로써 아래ACSC  repo  를 이용한 RS lidar calibration 에 필요 

https://github.com/HViktorTsoi/ACSC
// ROS 환경 구성 및 카메라와 라이다 data topic 이 성공적으로 publish 됐으면 해당 repo를 이용해 캘리브레이션 수행 
```

### 2. Object detector 코드 구성

#### ObjectDetector.cpp
#### main():
RS lidar driver initialize 하고 pointCloudCallback 함수 실행해 RS lidar 에서 받은 raw 포인트클라우드 처리
```
  driver.regRecvCallback(pointCloudCallback);     ///< Register the point cloud callback function into the driver
  ```
#### pointCloudCallback():
Raw 포인트 클라우드인 msg 처리를 다음 순서로 실행
1. 	planar segmentation() 으로 지면 제거
2. xyz 축으로 각각 min&max 값 설정해 관심point cloud만 필터 
3. pcl::VoxelGrid 로 voxel 화. leafsize  가 클수록 voxel 화가 많이 되어 출력 point cloud 개수가 줄어듬
4. Clustering 진행: K-means OR DBSCAN (default) 중 선택 가능. MINIMUM_POINTS 와 EPSILON 등의 parameter 설정 가능 

이렇게 preprocess 된 라이다 포인트 클라우드는 main 함수에서 생성된 projection_handler 쓰레드에서 카메라와 함께 projection 처리 
	
```
  thread t1(keyboard_input_handler);
  thread t2(projection_handler);
 ```
#### projection_handler():
설정에 따라 리얼타임 카메라 영상 OR pre-record 된 비디오 재생.   
 다크넷 프레임워크 (Darknet: 오픈소스 뉴럴네트워크 프레임워크로 C 및  CUDA 로 제작. YOLOv4는 CSPDarknet53 CNN: 53 개의 convolutional layer로 개발됨)를 사용해 Camera 용 딥러닝 인지모델인 YOLOv4 실행 (아래 repo 의 cpp 버전 참조해 코드 디자인)
 ```
 https://github.com/doleron/yolov4-opencv-cpp-python
 ```
 camera extrinsic / intrinsic matrix 가 있는 파일을 불러와 라이다 포인트 클라우드를 3D 에서 2D 벡터로 계산해 변환.  출력값을 카메라 프레임에 원으로 투영 (cv::circle)
 ```
 cv::circle(frame, pt, 5, cv::Scalar(0, green, red), -1);
 ```