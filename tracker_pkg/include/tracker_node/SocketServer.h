//
// Created by electricsoul on 2020/8/15.
//

#ifndef TEST_SOCKETSERVER_H
#define TEST_SOCKETSERVER_H

#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<unistd.h>
#include <arpa/inet.h>

using namespace std;
#define MAXLINE 1024

union DataPack{

public:
	// center_x,center_y,width,height,score
	float data_field[10];
	unsigned char pack_field[40];

};


class SocketServer{

public:
    SocketServer()
    {

    }

    SocketServer(string ip_addr,int port)
    {
        if((listenfd = socket(AF_INET,SOCK_STREAM,0)) == -1){
            printf("create socket error: %s(errno:%d)\n",strerror(errno),errno);
            exit(0);
        }
        memset(&servaddr,0,sizeof(servaddr));
        servaddr.sin_family = AF_INET;

        servaddr.sin_addr.s_addr = inet_addr(ip_addr.c_str());
        servaddr.sin_port = htons(port);

        if(bind(listenfd,(struct sockaddr*) &servaddr , sizeof(servaddr)) == -1){
            printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);
            exit(0);
        }

        if(listen(listenfd,10) == -1){
            printf("listen socket error:%s(errno: %d)\n",strerror(errno),errno);
            exit(0);
        }

        printf("========== waiting for client's request =============\n");

        connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);
    }

    void receive_msg()
    {

        memset(buff,0,sizeof(buff)/sizeof(char) );

        if( connfd == -1){
            printf("accept socket error: %s(errno: %d)",strerror(errno),errno);
            return;
        }
        n = recv(connfd,buff,MAXLINE,0);
        buff[n] = '\0';
    }

    void release()
    {
        close(connfd);
        close(listenfd);
    }

public:

    int listenfd;
    int connfd;
    struct sockaddr_in servaddr;
    char buff[MAXLINE];
    int n;

};

#endif //TEST_SOCKETSERVER_H
