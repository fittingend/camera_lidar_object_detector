#include <iostream>
#include <term.h>
#include <termios.h>  
#include <unistd.h>

#include "key.h"

using namespace std;

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

p       | DBSCAN_MINIMUM_POINTS 를 0.1 단위로 변화
p       | DBSCAN_MINIMUM_POINTS 를 0.1 단위로 변화
e       | DBSCAN_EPSILON 를 0.1 단위로 변화
e       | DBSCAN_EPSILON 를 0.1 단위로 변화
===================================
*/

void keyboard_input_handler(void)
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
    case 'e':
      if (flag)
        EPSILON_INPUT += 0.1;
      else
        EPSILON_INPUT -= 0.1;
      std::cout << "EPSILON_INPUT : " << EPSILON_INPUT << std::endl;
      break;
    case 'p':
      if (flag)
        MINIMUM_POINTS += 10;
      else
        MINIMUM_POINTS -= 10;
      std::cout << "MINIMUM_POINTS : " << MINIMUM_POINTS << std::endl;
      break;
    }
  }
}




