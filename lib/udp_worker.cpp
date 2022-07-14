#include "udp_worker.h"

#define UDP_BUFSIZE 1540

using namespace std;


void UDP_WORKER::worker(uint8_t *frame)
{

}



void UDP_WORKER::udp_worker_thread(void)
{
    int addrlen;
    int retval;
    sockaddr_in clientaddr;
    uint8_t buf[UDP_BUFSIZE+1];

    addrlen=sizeof(clientaddr);	

    
    while(thread_run_flag)
    {
        retval=recvfrom(sock,buf,UDP_BUFSIZE,0,(sockaddr*)&clientaddr,(socklen_t*)&addrlen);
  //      printf("sujin-packet received\n");
  //      cout << buf <<endl;
        worker(buf);
    }
}


void UDP_WORKER::init_(uint16_t nport, string &_name)
{
    int retval, opt=1;
    thread_run_flag=1;
    sockaddr_in serveraddr;
    
    thread_run_flag = UDP_WORKER_THREAD_RUN;
    port = nport;
    name=_name;
    sock=socket(AF_INET, SOCK_DGRAM, 0); 
    if(sock<0)
    {
      cout<<"udp "<< nport<<" port sock init error!!!"<<endl;
    }
    else
    {

    }
    
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
    
    bzero(&serveraddr, sizeof(serveraddr)); 
    serveraddr.sin_family=AF_INET;
    serveraddr.sin_port=htons(port);
    serveraddr.sin_addr.s_addr=inet_addr("255.255.255.255");
    serveraddr.sin_addr.s_addr=INADDR_ANY;
    retval=bind(sock, (sockaddr*)&serveraddr, sizeof(serveraddr));
    if(retval <0)
    {
      cout<<"udp "<< nport<<" port bind error!!!"<<endl;
    }
    else
    {
     
    }
    worker_thread = thread(&UDP_WORKER::udp_worker_thread,this);
}

UDP_WORKER::UDP_WORKER()
{

}

UDP_WORKER::~UDP_WORKER()
{
    thread_run_flag = UDP_WORKER_THREAD_STOP;
}
