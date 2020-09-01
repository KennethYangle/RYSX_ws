//
// Created by electricsoul on 2020/8/15.
//

#ifndef TEST_SOCKETCLIENT_H
#define TEST_SOCKETCLIENT_H

#include<iostream>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>

using namespace std;
#define MAXLINE 1024

class SocketClient{

public:

    SocketClient(){}

    SocketClient(string ip_addr,int port)
    {
        if ( (sockfd = socket(AF_INET,SOCK_STREAM,0)) < 0 ) {
            printf("create socket error: %s(errno: %d)\n", strerror(errno), errno);
            exit(0);
        }

        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(port);

        if ( inet_pton(AF_INET,ip_addr.c_str(),&servaddr.sin_addr) <= 0 ) {
            printf("inet_pton error for %s\n", ip_addr.c_str());
            exit(0);
        }

        if (connect(sockfd,(struct sockaddr*) & servaddr,sizeof(servaddr)) < 0 ) {
            printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
            exit(0);
        }
    }

    void send_msg(char *user_command)
    {
        memset(sendLine, 0, sizeof(sendLine));
	    memcpy(sendLine,user_command,1*sizeof(unsigned char));

        if ( send(sockfd,sendLine,sizeof(sendLine),0) < 0 ) {
            printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
            exit(0);
        }
        if (strcmp(sendLine, "exit\n") == 0)
            return;

    }

    void release()
    {
        close(sockfd);
    }

public:

    int   sockfd, n;
    char recvLine[MAXLINE], sendLine[MAXLINE];
    struct sockaddr_in servaddr;

};

#endif //TEST_SOCKETCLIENT_H
