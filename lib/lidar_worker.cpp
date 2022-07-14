#include "lidar_worker.h"

#include <time.h>
#include <cmath>

#define PI 3.141592653589793238462643383279502884
#define PI2 PI*2
#define get_azimuth_vlp16(frame,pos) (float(frame[2+pos*100])+float(frame[3+pos*100])*0x0100)/100.0
#define get_block_ptr(frame, n) (&frame[100*n]) //하나의 데이터블록이 100바이트 사이즈
#define get_angle(block) (float(block[2])+float(block[3])*0x0100)/100.0*PI/180.0
#define get_distance(block,ch) ((float)(block[ch*3+4])+(float)(block[ch*3+5]*0x0100))/1000.0*2.0
#define get_intensity(block,ch) (float)(sqrt(float(block[ch*3+6])))

float vlp16_vertical_angle[16] ={-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15};
float vlp16_vertical_corrrection[16] = { 0.0112, -0.0007, 0.0097, -0.0022, 0.0081, -0.0037, 0.0066, -0.0051, 0.0051, -0.0066, 0.0037, -0.0081, 0.0022, -0.0097, 0.0007, -0.0112};
float vlp16_vertical_cos_p[16], vlp16_vertical_sin_p[16];

/* M1 lidar related settings */
#define HEADER_SIZE 32
#define get_azimuth_M1(block,ch) (float(block[9*ch-3]*0x100 + block[9*ch-2]-0x8000)*0.01)
//데이터 블록 내에 7,8번째 바이트는 앵글값
#define get_radius_M1(block, ch) (float(block[9*ch-7]*0x100 + block[9*ch-6])*0.005)
#define get_intensity_M1(block, ch) (float(block[9*ch-1]))
#define get_elevation_M1(block, ch) (float(block[9*ch-5]*0x100 + block[9*ch-4]-0x8000)*0x01)
#define get_block_ptr_M1(frame, dataBlock) &frame[HEADER_SIZE+45*(dataBlock-1)-1] //return the address of the start of the Datablock 1-25
//TO DO: header info: 32 bytes, to be added later? 



void VLP16::MsgParser(uint8_t *frame)
{
    int i,j,k,u;
    uint8_t *block_ptr;
    float angle, angle1, angle_offset, distance, intensity;
    float _angle, _angle1, _angle_offset;
    float xy;

    /*
     lidar blcok내부 2개의 데이터셋의 azimuth angle을 계산..
     block 내부 첫번째 데이터 셋은 angle을 사용하며,
     두번째 데이터 셋은 angle1을 사용하여 PointData로 변환
    */
    block_ptr =get_block_ptr(frame,0); //여기서 frame 은 data block 한 개 내의 2개 frame을 의미
    angle = get_angle(block_ptr);
    block_ptr =get_block_ptr(frame,1); 
    angle1 = get_angle(block_ptr); 
    angle_offset = angle1-angle;

    //앵글 오프셋 (앵글값의 차이값) 은 일정함으로 이 값을 계속 add 해주면 앵글 값 구할 수 있음
    _angle = get_azimuth_vlp16(frame,0);
    _angle1 = get_azimuth_vlp16(frame,1);
    _angle_offset = _angle1-_angle; 
    for(i=0;i<12;i++) //vlp16은 한 packet 당 data block 이 12개가 있다
    {
        block_ptr =get_block_ptr(frame,i); // i 번째 블락 데이터 위치 가져오기 
        angle = get_angle(block_ptr); // 해당 block의 azimuth 데이터를 사용하여 사용하여 실제 측정된 azimuth angle을 계산한다.  
        angle1 = angle+angle_offset;
        if(angle1>=PI2) angle1=angle1-PI2; //360 도는 2pi 이므로 angle1 은 radian 값

        _angle = get_azimuth_vlp16(frame,i);
        _angle1 = _angle+_angle_offset;
        if(_angle1>=360) _angle1-360; //_angle1 은 degree 값

        for(j=0;j<16;j++) //블락 데이터 내의 레이저 채널 수(j) 16개에 해당하는 loop
        {   
            int flag=0;
            // i번째 block 처리 
            pcl::PointXYZI pt;

            // 총 24개의 측정데이터 셋 중 i*2 [0,2,4,..., 22] 번쨰 lidar j ch 의 측정 데이터 셋 처리 
            if((this->*yaw_func)(_angle))
            {
                distance = get_distance(block_ptr,j); 
                pt.z = vlp16_vertical_sin_p[j]*distance+vlp16_vertical_corrrection[j]; //PointData의 z축 게산
                pt.x = vlp16_vertical_cos_p[j]*sin(angle)*distance; //PointData의 x축 게산
                pt.y = vlp16_vertical_cos_p[j]*cos(angle)*distance; //PointData의 y축 게산
                if((this->*x_func)(pt.x)&&(this->*y_func)(pt.y)&&(this->*z_func)(pt.z))
                {
                    pt.intensity = get_intensity(block_ptr,j); //PointData의 intensity 게산
                    pData.push_back(pt);
                }
            }
            
            // 총 24개의 측정데이터 셋 중 i*2+1 [1,3,5,..., 23] 번쨰 lidar j ch 의 측정 데이터 셋 처리
            if((this->*yaw_func)(_angle1))
            {
                k=j+16;
                distance = get_distance(block_ptr,k); 
                pt.z = vlp16_vertical_sin_p[j]*distance+vlp16_vertical_corrrection[j];
                pt.x = vlp16_vertical_cos_p[j]*sin(angle1)*distance;
                pt.y = vlp16_vertical_cos_p[j]*cos(angle1)*distance;
                if((this->*x_func)(pt.x)&&(this->*y_func)(pt.y)&&(this->*z_func)(pt.z))    
                {
                    pt.intensity = get_intensity(block_ptr,k); 
                    pData.push_back(pt);
                }
            }     
        }   
    }
    
}

void M1::MsgParser(uint8_t *frame)
{
//    printf("entering M1 msg parser\n");
    uint8_t *block_ptr;
    float azimuth, elevation, radius, intensity;
    for(int dataBlock=1;dataBlock<26;dataBlock++)
    {
        block_ptr = get_block_ptr_M1(frame, dataBlock); // 프레임 내 1st-25th 블락 데이터 시작 위치 가져오기 

        for(int channel=1;channel<6;channel++) //데이터 블록내 채널 1~5까지의 데이터 
        {              
//          int flag=0;
            pcl::PointXYZI pt;
            azimuth = get_azimuth_M1(block_ptr, channel);
          
            if((this->*yaw_func)(azimuth)) //invoking yaw_range function with the parameter 'azimuth'
            {
                radius = get_radius_M1(block_ptr,channel); 
                elevation = get_elevation_M1(block_ptr, channel);
                //radial distance 구하기
                pt.x = radius * cos(elevation) * sin(azimuth);
                pt.y = radius * cos(elevation) * cos(azimuth);
                pt.z = radius * sin(elevation);
                //vlp16 과 xyz 기준 맞춤 - M1 매뉴얼과는 다르게 x 와 y define
               if((this->*x_func)(pt.x)&&(this->*y_func)(pt.y)&&(this->*z_func)(pt.z)&& radius >= 0.5)
               //xyz 가 main 에서 설정한 range 안에 들어갈때 + radial distance 가 최소값인 0.5m 이상을 만족하는 값만 pData 에 저장 후 처리
                {
                    cout << "For datablock" <<dataBlock<< " and channel " <<channel <<" the azimuth value is " <<azimuth <<endl;
                    cout <<pt.x << ", "<<pt.y << ", "<< pt.z << endl;
                    pt.intensity = get_intensity_M1(block_ptr,channel); //PointData의 intensity 계산
//                    cout << "Intensity is " << pt.intensity <<endl;
//                    cout<< "Radial distance is " << radius << endl;
                    pData.push_back(pt);
                }
            }
            
        }   
    }
    
}

void VLP16::worker(uint8_t *frame)
{
    static struct timespec timea, time_old;
    static uint16_t cnt=0;
    static uint16_t cnt2=0;
    static float old_angle = 0;
    float angle_a, angle_b;
    uint16_t obj_cnt;
    pcl::PointCloud<PointXYZIR> lcr;

    angle_a = get_azimuth_vlp16(frame,0); //수신한 lidar 메시지 데이터의 첫번째 angle 값 계산
    angle_b = get_azimuth_vlp16(frame,11); //수신한 lidar 메시지 데이터의 마지막 angle값 계산
    if((this->*yaw_func)(angle_a)||(this->*yaw_func)(angle_b))
    {
        cnt++;
        MsgParser(frame); // 메시지를 파싱하여 PointXYZI 데이터로 변환함
    }
    else
    {
        if(pData.size()!=0) // 300<azimuth_angle<60의 데이터셋 수집한 후 clustering 진행.
        {
 //           obj_cnt=lidar_clustering(name,pData.makeShared(), lcr); // Lidar clustering  진행.
 //           output->out(lcr,obj_cnt);
 //           pData.clear();
 //           lcr.points.clear();
        }

    }
}

void M1::worker(uint8_t *frame)
{
    printf("M1 worker entered!\n");
    static struct timespec timea, time_old;
    static uint16_t cnt=0;
    static uint16_t cnt2=0;
    static float old_angle = 0;
    float angle_a, angle_b;
    uint16_t obj_cnt;
    pcl::PointCloud<PointXYZIR> lcr;

//    angle_a = get_azimuth_M1(frame,1); //수신한 lidar 메시지 데이터의 첫번째 azimuth - horizontal angle 값 계산
//    angle_b = get_azimuth_M1(frame,5); //수신한 lidar 메시지 데이터의 마지막 azimuth - horizontal angle 값 계산
//    if((this->*yaw_func)(angle_a)||(this->*yaw_func)(angle_b))
 //   {
        printf("처음 if 절 들어옴\n");
        cnt++;
        MsgParser(frame); // 메시지를 파싱하여 PointXYZI 데이터로 변환함
//    }
//    else
//    {
        if(pData.size()!=0) // 한 프레임이 끝난걸 어떻게 알 수 있을까.. 
            printf("clustering 시작\n");
//            obj_cnt=lidar_clustering(name,pData.makeShared(), lcr); // Lidar clustering  진행.
//            output->out(lcr,obj_cnt);
//            pData.clear();
//            lcr.points.clear();
//        }

//    }
}

int VLP16::x_range(float _x)
{
    if((_range.x_low<_x)&&(_x<_range.x_high)) return 1;
    else return 0;
}
int VLP16::x_range_(float _x)
{
    return 1;
}

int VLP16::y_range(float _y)
{
    if((_range.y_low<_y)&&(_y<_range.y_high)) return 1;
    else return 0;
}
int VLP16::y_range_(float _y)
{
    return 1;
}

int VLP16::z_range(float _z)
{
    if((_range.z_low<_z)&&(_z<_range.z_high)) return 1;
    else return 0;
}
int VLP16::z_range_(float _z)
{
    return 1;
}


int VLP16::yaw_range(float _yaw)
{
    if((_range.yaw_low<_yaw)&&(_yaw<_range.yaw_high)) return 1;
    else return 0;    
}
int VLP16::yaw_range_(float _yaw)
{
    if((_yaw<_range.yaw_high)||(_range.yaw_low<_yaw)) return 1;
    else return 0;    
}
int VLP16::yaw_range__(float _yaw)
{
    return 1;
}

int VLP16::pitch_range(float _pitch)
{
    if((_range.pitch_low<_pitch)&&(_pitch<_range.pitch_high)) return 1;
    else return 0;    
}
int VLP16::pitch_range_(float _pitch)
{
    if((_pitch<_range.pitch_high)||(_range.pitch_low<_pitch)) return 1;
    else return 0;
}
int VLP16::pitch_range__(float _pitch)
{
    return 1;
}

int M1::x_range(float _x)
{
    if((_range.x_low<_x)&&(_x<_range.x_high)) return 1;
    else return 0;
}
int M1::x_range_(float _x)
{
    return 1;
}

int M1::y_range(float _y)
{
    if((_range.y_low<_y)&&(_y<_range.y_high)) return 1;
    else return 0;
}
int M1::y_range_(float _y)
{
    return 1;
}

int M1::z_range(float _z)
{
    if((_range.z_low<_z)&&(_z<_range.z_high)) return 1;
    else return 0;
}
int M1::z_range_(float _z)
{
    return 1;
}


int M1::yaw_range(float _yaw)
{
    if((_range.yaw_low<_yaw)&&(_yaw<_range.yaw_high)) return 1;
    else return 0;    
}
int M1::yaw_range_(float _yaw)
{
    if((_yaw<_range.yaw_high)||(_range.yaw_low<_yaw)) return 1;
    else return 0;    
}
int M1::yaw_range__(float _yaw)
{
    return 1;
}

int M1::pitch_range(float _pitch)
{
    if((_range.pitch_low<_pitch)&&(_pitch<_range.pitch_high)) return 1;
    else return 0;    
}
int M1::pitch_range_(float _pitch)
{
    if((_pitch<_range.pitch_high)||(_range.pitch_low<_pitch)) return 1;
    else return 0;
}
int M1::pitch_range__(float _pitch)
{
    return 1;
}

void VLP16::init_meas_range(MEAS_RANGE *_info)
{
    int i;
    float *_ptr = (float *)(&_range);
    memcpy(&_range, _info, sizeof(MEAS_RANGE));
    
    if((_range.x_low!=0)||(_range.x_high!=0))
    {
        x_func=&VLP16::x_range;
    }
    else
    {
        x_func=&VLP16::x_range_;
    }
    if((_range.y_low!=0)||(_range.y_high!=0))
    {
        y_func=&VLP16::y_range;
    }
    else
    {
        y_func=&VLP16::y_range_;
    }
    if((_range.z_low!=0)||(_range.z_high!=0))
    {
        z_func=&VLP16::z_range;
    }
    else
    {
        z_func=&VLP16::z_range_;
    }

    if((_range.yaw_low==0)&&(_range.yaw_high==0))
    {
        yaw_func = &VLP16::yaw_range__;
    }
    else if(_range.yaw_low>_range.yaw_high)
    {   
        yaw_func = &VLP16::yaw_range_;
    }
    else
    {
        yaw_func = &VLP16::yaw_range;
    }

    if((_range.pitch_low==0)&&(_range.pitch_high-=0))
    {
        pitch_func = &VLP16::pitch_range__;
    }
    else if(_range.pitch_low>_range.pitch_high)
    {   
        pitch_func = &VLP16::pitch_range_;
    }
    else
    {
        pitch_func = &VLP16::pitch_range;
    }   

}

void M1::init_meas_range(MEAS_RANGE *_info)
{
    int i;
    float *_ptr = (float *)(&_range);
    memcpy(&_range, _info, sizeof(MEAS_RANGE));
    
    if((_range.x_low!=0)||(_range.x_high!=0))
    {
        cout<< "x range set" <<endl;
        x_func=&M1::x_range;
    }
    else
    {
        cout<< "x range NOT set" <<endl;
        x_func=&M1::x_range_;
    }
    if((_range.y_low!=0)||(_range.y_high!=0))
    {
        y_func=&M1::y_range;
    }
    else
    {
        y_func=&M1::y_range_;
    }
    if((_range.z_low!=0)||(_range.z_high!=0))
    {
        z_func=&M1::z_range;
    }
    else
    {
        z_func=&M1::z_range_;
    }

    if((_range.yaw_low==0)&&(_range.yaw_high==0))
    {
        yaw_func = &M1::yaw_range__;
    }
    else if(_range.yaw_low>_range.yaw_high)
    {   
        yaw_func = &M1::yaw_range_;
    }
    else
    {
        yaw_func = &M1::yaw_range;
    }

    if((_range.pitch_low==0)&&(_range.pitch_high-=0))
    {
        pitch_func = &M1::pitch_range__;
    }
    else if(_range.pitch_low>_range.pitch_high)
    {   
        pitch_func = &M1::pitch_range_;
    }
    else
    {
        pitch_func = &M1::pitch_range;
    }   

}

void VLP16::init(LIDAR_INFO *_info)
{
    output = new LIDAR_OUT(_info->sensor_name);
    init_meas_range(&_info->range);
    init_(_info->port, _info->sensor_name);
}

void M1::init(LIDAR_INFO *_info)
{
    output = new LIDAR_OUT(_info->sensor_name);
    init_meas_range(&_info->range);
    init_(_info->port, _info->sensor_name); //udp_worker 호출
}

LIDAR_WORKER::LIDAR_WORKER(vector<LIDAR_INFO> &_info)
{
    int i;
  
    for(i=0;i<16;i++)
    {
        //lidar ch별 point 계산에 사용되는 값 초기화
        vlp16_vertical_cos_p[i]=cos(vlp16_vertical_angle[i]*PI/180.0); 
        vlp16_vertical_sin_p[i]=sin(vlp16_vertical_angle[i]*PI/180.0);

    }

    for(i=0;i<_info.size();i++)
    {

        if(_info[i].model == LIDAR_WORKER_VLP16)
        {
            VLP16 *ptr = new VLP16();
            ptr->init(&_info[i]);
            vlp16_list.push_back(ptr);
        }
        if(_info[i].model == LIDAR_WORKER_M1)
        {
            printf("LIDAR_WORKER_M1 안에 들어옴\n");
            M1 *ptr = new M1();
            ptr->init(&_info[i]);
            M1_list.push_back(ptr);
            printf("init 끝남\n");
        }

    }
}

LIDAR_WORKER::~LIDAR_WORKER()
{
    int i;
    for(i=0;i<vlp16_list.size();i++)
    {
        VLP16 *ptr = vlp16_list[i];
        delete ptr;
    }
    for(i=0;i<M1_list.size();i++)
    {
        M1 *ptr = M1_list[i];
        delete ptr;
    }
}
