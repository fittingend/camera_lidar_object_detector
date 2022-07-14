#ifndef _UDP_WORKER_H_
#define _UDP_WORKER_H_


#include<iostream>

#include<netinet/in.h>
#include<sys/socket.h>
#include<arpa/inet.h>

#include <error.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <thread>

#define UDP_WORKER_THREAD_RUN 1
#define UDP_WORKER_THREAD_STOP 0

using namespace std;

class UDP_WORKER
{
    private:
    
    public:
    string name;
    int thread_run_flag;
    int sock;
    uint16_t port;
    thread worker_thread;
    
    void udp_worker_thread(void);
    virtual void worker(uint8_t *frame);
    void init_(uint16_t nport, string &_name);
    UDP_WORKER();
    ~UDP_WORKER();
};

#endif