#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/select.h>
#include <pthread.h>
#include <semaphore.h>

#define ESC_STATUS_ID 0x125
#define SPEED_FACTOR  0.05625
#define REQUEST_ID    0x7df

pthread_t cansendThread;
sem_t     sem_can;
int       exitThread=0,g_fd_cansend=0;
int       g_speed=0;

char buf[]={0x02, 0x01 ,0x0D, 0x00, 0x00, 0x00, 0x00, 0x00};

void *cansend(void*arg)
{
    float fspeed=0.0;
    unsigned short t;
    unsigned int nbytes;
    struct   can_frame frame;
    frame.can_id=ESC_STATUS_ID;
    frame.can_dlc=8;
    memset(frame.data,0,sizeof(frame.data));
    while(1)
    {
        sem_wait(&sem_can);
        if(exitThread){
            break;
        }  
        fspeed=g_speed/SPEED_FACTOR;
        t=(unsigned short)fspeed;
        printf("[%s]%f\n",__func__,fspeed);
        frame.data[1]=(t>>5)&0xff;
        frame.data[2]=(t&0x1f)<<3;
        nbytes= write(g_fd_cansend, &frame, sizeof(frame)); //发送
        if(nbytes!=sizeof(frame)){
            printf("send frame  data  faild  %d\n",nbytes);
        }       
    }
}
int can_sendInit(char *can)
{
    int s,nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, can);

    ioctl(s, SIOCGIFINDEX, &ifr);
    printf("%s can_ifindex = %x  s= %d\n",can, (int)ifr.ifr_ifindex,s);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    setsockopt(s,SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    if(pthread_create(&cansendThread,NULL,cansend,NULL)<0){
        fprintf(stderr,"<%s>:cannot create cansend thread \n",__func__);
        return -1;
    }   
    sem_init(&sem_can,0,0);
    return s;
}
void can_sendRelease(int s)
{
    exitThread=1;
    sem_post(&sem_can);
    sem_destroy(&sem_can);
    close(s);
}

int obdii_request_speed(int s,struct can_frame frame)
{
    int nbytes= write(s, &frame, sizeof(frame)); //发送
    if(nbytes!=sizeof(frame)){
        printf("[%s] faild \n",__func__);
        return -1;
    }  
    return 0;
}
int obdii_resolver_speed(struct can_frame frame,int *speed)
{
    int datalen=frame.data[0];
    int mode   =frame.data[1];
    int PID    =frame.data[2];
    if(PID!=0x0D){
        printf("[%s] PID not ringht %d\n",__func__,PID);
        return -1;
    }
    *speed  =frame.data[datalen];
    printf("%s speed = %d\n",__func__,*speed);
    return 0;
}


int main(int argc ,char **argv)
{
	int s,nbytes,i=0,ret;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame  frame;
    struct can_frame  request_frame;
    struct can_filter rfilter[2];
    printf("%s %s  %s\n",argv[0],argv[1],argv[2]);
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, argv[1]);
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    rfilter[0].can_id = 0x7e9;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x7e8;
    rfilter[1].can_mask = CAN_SFF_MASK;
    setsockopt(s,SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    memset(&frame,0,sizeof(frame));
    struct timeval tv;
    fd_set r_fds;
    g_fd_cansend=can_sendInit(argv[2]);

    request_frame.can_id=REQUEST_ID;
    request_frame.can_dlc=8;
    memcpy(request_frame.data,buf,sizeof(buf));
    while(1){
        obdii_request_speed(s,request_frame);
        FD_ZERO(&r_fds);
        FD_SET(s,&r_fds);
        tv.tv_sec = 0;
        tv.tv_usec = 20000; 
        ret= select(s + 1,&r_fds,NULL,NULL,&tv);
        switch(ret){
            case -1:
                printf("select faild\n");
                break;
            case 0:
                //printf("time out\n");
                break;
            default:
                nbytes= read(s, &frame, sizeof(frame)); //接收报文
                if(nbytes>0){
                    printf("id = 0x%x dlc = %d \n",frame.can_id,frame.can_dlc);
                    if(!obdii_resolver_speed(frame,&g_speed));
                        sem_post(&sem_can);
                }
                break;
        }       
    }
    close(s);
    can_sendRelease(g_fd_cansend);
}