/**
 * @agautam2_assignment3
 * @author  Abhishek Gautam <agautam2@buffalo.edu>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * This contains the main function. Add further description here....
 */

//  Code to implement Distance Vector Routing Protocol
//  MBP111.0138.B16
//  System Serial: C02P4SP9G3QH
//  main.cpp
//
//  Created by Abhishek Gautam on 4/14/16.
//  agautam2@buffalo.edu
//  University at Buffalo, The State University of New York.
//  Copyright © 2016 Gautam. All rights reserved.

/**
 * main function
 *
 * @param  argc Number of arguments
 * @param  argv The argument list
 * @return 0 EXIT_SUCCESS
 */

#include <stdio.h>
#include <iostream>
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstring>
#include "Serialize.h"


#define MAXBUFLEN 100;
#define INF 65535;

using namespace std;

struct routingEntry
{
    uint16_t destinationRouterPort;
    struct in_addr destinationRouterIP;
    uint16_t routerID;
    uint16_t metricCost;
    uint16_t padding;
};

struct updatePacket
{
    uint16_t entryCount;
    uint16_t localRouterPort;
    struct in_addr localRouterIP;
    routingEntry routingEntries [];
};

struct controlPacket
{
    uint32_t destinationIP;
    uint8_t controlCode;
    uint8_t responseTime;
    uint16_t payloadLength;
};

struct controlResponse
{
    struct in_addr controllerIP;
    uint8_t controlCode;
    uint8_t responseCode;
    uint16_t payloadLength;
};

class Router
{
    
    /*All class-related variables go here*/
    
    char *listenPort;
    int sockfdController, sockfdUpdates, fdMaxNumber, newsockfd, readBytes;
    uint16_t routerPort;
    struct addrinfo hints, *servinfo, *p;
    int rv, returnedInfo;
    int numbytes, clientPort;
    struct sockaddr_storage their_addr, clientAddress;
    char buffer[100];
    unsigned char *controlHeaderBuffer;
    socklen_t addr_len, clientLength;
    char s[INET6_ADDRSTRLEN];
    char storeAddress[INET6_ADDRSTRLEN];
    char service [20];
    char host [1024];
    fd_set masterDescriptor;
    fd_set tempRead_fds;
    int yes;
    
    /*All class-related variables go here*/
    
public: Router()
    {
        printf("Default constructor intialised\n");
    }
    
public: int estalblishRouter(uint16_t controlPort)
    {
        //printf("You choose to use %hu as the router port\n", routerPort);
        printf("You choose to use %hu as the control port\n", controlPort);
        //printf("You choose to use %hu as the data port\n", routerPort);
        
        char *controlPortChar;
        controlPortChar = (char*)malloc(sizeof(int)+1);
        sprintf(controlPortChar, "%d", controlPort);
        //printf("%s\n",controlPortChar);
        
        FD_ZERO(&masterDescriptor);
        FD_ZERO(&tempRead_fds);
        
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_flags = AI_PASSIVE;
        
        /*[PA3]Router will listen for controller messages on this CONTROL PORT(TCP). This will be supplied as an argument when executing the application*/
        returnedInfo = getaddrinfo(NULL, controlPortChar, &hints, &servinfo);
        printf("returned info: %d\n",returnedInfo);
        if(returnedInfo!=0)
        {
            fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(returnedInfo));
            return 1;
        }
        for(p=servinfo; p!=NULL;p=p->ai_next)
        {
            sockfdController = socket(p->ai_family, p->ai_socktype, p->ai_protocol);          //SOCKET
            printf("socket file descriptor :%d\n\n", sockfdController);
            if(sockfdController<0)
            {
                perror("server: socket");
                continue;
            }
            if(setsockopt(sockfdController, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int))==-1)
            {
                perror("setsockopt");
                exit(1);
            }
            if(::bind(sockfdController, p->ai_addr, p->ai_addrlen) < -1)                      //BIND
            {
                close(sockfdController);
                perror("server:bind");
                continue;
            }
            break;
        }
        if(p==NULL)
        {
            fprintf(stderr, "server: port binding failed\n");
        }
        freeaddrinfo(servinfo);
        if(listen(sockfdController,5) == -1)                                                 //LISTEN
        {
            perror("listen");
        }
        else
        {
            printf("listening for incoming connections on CONTROL PORT (TCP)\n");
        }
        /*Router will listen for controller messages on this CONTROL PORT(TCP). This will be supplied as an argument when executing the application[PA3]*/
        
        
        /*[PA3]Router will listen for routing updates on this ROUTER PORT (UDP)*/
        routerPort=9988;
        
        listenPort=(char*)malloc(sizeof(int)+1);
        
        sprintf(listenPort, "%d", routerPort);
        
        if ((rv = getaddrinfo(NULL, listenPort, &hints, &servinfo)) != 0)
        {
            fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
            return 1;
        }
        
        for(p = servinfo; p != NULL; p = p->ai_next)
        {
            if ((sockfdUpdates = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1)
            {
                perror("listener: socket\n");
                continue;
            }
            if(::bind(sockfdUpdates, p->ai_addr, p->ai_addrlen) < -1)
            {
                close(sockfdUpdates);
                perror("server: bind\n");
                continue;
            }
            else
            {
                printf("Router now listening for updates on router port: %s\n", listenPort);
            }
            break;
        }
        if (p == NULL)
        {
            fprintf(stderr, "listener: failed to bind socket\n");
            return -1;
        }
        freeaddrinfo(servinfo);
        printf("listener: waiting to recvfrom...\n");
        
        //        addr_len = sizeof their_addr;
        //        numbytes=recvfrom(sockfdUpdates, buffer, sizeof(buffer), 0, (sockaddr *)&their_addr, &addr_len);
        //        if(numbytes==-1)
        //        {
        //            perror("recvfrom");
        //            exit(1);
        //        }
        //
        //        getpeername(sockfdUpdates, (struct sockaddr *) &clientAddress, &clientLength);
        //
        //        if(clientAddress.ss_family == AF_INET)
        //        {
        //            struct sockaddr_in *peername = (struct sockaddr_in *) &clientAddress;
        //            clientPort = ntohs(peername->sin_port);
        //            inet_ntop(AF_INET, &peername->sin_addr, storeAddress, sizeof (storeAddress));
        //        }
        //        else
        //        {
        //            struct sockaddr_in6 *peername = (struct sockaddr_in6 *) &clientAddress;
        //            clientPort = ntohs(peername->sin6_port);
        //            inet_ntop(AF_INET, &peername->sin6_addr, storeAddress, sizeof (storeAddress));
        //        }
        //        printf("Accepting routing updates from %s: %d on socket %d\n\n", storeAddress, clientPort, sockfdUpdates);
        //        printf("Socket %d is bound to %s\n", sockfdUpdates, storeAddress);
        //        clientLength = sizeof(clientAddress);
        //
        //        printf("listener: packet is %d bytes long\n", numbytes);
        //        buffer[numbytes] = '\0';
        //        printf("listener: packet contains \"%s\"\n", buffer);
        //        close(sockfdUpdates);
        
        /*Router will listen for routing updates on this ROUTER PORT (UDP)[PA3]*/
        
        
        /*SELECT SYSTEM HANDLING*/
        FD_SET(sockfdController, &masterDescriptor);
        FD_SET(sockfdUpdates, &masterDescriptor);
        fdMaxNumber=sockfdController;
        struct timeval selectCallTimer;
        int dataSocket;
        
        for(;;)
        {
            tempRead_fds = masterDescriptor;
            selectCallTimer.tv_sec=1.0;
            selectCallTimer.tv_usec=0.0;
            
            if(select(fdMaxNumber+1, &tempRead_fds, NULL, NULL, &selectCallTimer)==-1)//bad select
            {
                perror("ERROR IN SELECT SYSTEM CALL");
            }
            else    //good select
            {
                for(int i=0;i<=fdMaxNumber+1;i++)
                {
                    if(FD_ISSET(i, &tempRead_fds))
                    {
                        if(i==sockfdController)
                        {
                            printf("found connection from CONTROLLER\n");
                            clientLength=sizeof(clientAddress);
                            newsockfd=accept(sockfdController, (sockaddr *)&clientAddress, &clientLength);
                            if(newsockfd==-1)
                            {
                                perror("ERROR IN ACCEPT");
                            }
                            else    //bad accept
                            {
                                FD_SET(newsockfd, &masterDescriptor);
                                if(newsockfd>fdMaxNumber)
                                {
                                    fdMaxNumber=newsockfd;
                                    printf("fdMaxNumber updated\n");
                                }
                                clientLength = sizeof (clientAddress);
                                getpeername(sockfdController, (struct sockaddr *) &clientAddress, &clientLength);
                                if(clientAddress.ss_family == AF_INET)
                                {
                                    struct sockaddr_in *peername = (struct sockaddr_in *) &clientAddress;
                                    clientPort = ntohs(peername->sin_port);
                                    inet_ntop(AF_INET, &peername->sin_addr, storeAddress, sizeof (storeAddress));
                                }
                                else
                                {
                                    struct sockaddr_in6 *peername = (struct sockaddr_in6 *) &clientAddress;
                                    clientPort = ntohs(peername->sin6_port);
                                    inet_ntop(AF_INET, &peername->sin6_addr, storeAddress, sizeof (storeAddress));
                                }
                                printf("Accepting new connection from %s: %d on socket %d\n\n", storeAddress, clientPort, newsockfd);
                                printf("Socket %d is bound to %s\n", newsockfd, storeAddress);
                                
                                /*handle data on TCP socket here using recv
                                 1. do actions based on control codes
                                 2. handle file transfer if control code is Ox05 and 0x06 */
                                controlHeaderBuffer = static_cast<unsigned char *>(malloc(1024));
                                bzero(controlHeaderBuffer, sizeof(controlHeaderBuffer));
                                if((readBytes = recv(newsockfd,controlHeaderBuffer,sizeof(controlHeaderBuffer),0)) <=0)
                                {
                                    if(readBytes==0)
                                    {
                                        printf("Remote Server Hung Up\n");
                                    }
                                    else
                                    {
                                        perror("ERROR IN RECEIVING");
                                    }
                                    close(i);
                                    FD_CLR(i,&masterDescriptor);
                                }
                                else
                                {
                                    //OK WE HAVE SOME DATA NOW FROM THE CONTROLLER
                                    printf("received %d bytes of data from the CONTROLLER\n", readBytes);
                                    printf("trying to unpack\n");
                                    struct controlPacket *temp =  (struct controlPacket *) malloc(sizeof(struct controlPacket));
//                                    temp->destinationIP=(controlHeaderBuffer[0] << 0) | (controlHeaderBuffer[1] << 8) | (controlHeaderBuffer[2] << 16) | (controlHeaderBuffer[3] << 24);
//                                    temp->controlCode=(controlHeaderBuffer[4] << 8) | controlHeaderBuffer[4];
//                                    temp->responseTime=(controlHeaderBuffer[5] << 8) | controlHeaderBuffer[5];
//                                    temp->payloadLength=(controlHeaderBuffer[6] << 0) | (controlHeaderBuffer[7] << 8);
                                    
                                    
                                    //call to unpack using args as: 32(L), 8(C), 8(C) and 16 (H) followed by payload
                                    
                                    unpack(controlHeaderBuffer, "LCCH", &temp->destinationIP, &temp->controlCode, &temp->responseTime, &temp->payloadLength);
                                    char * str = inet_ntoa(*(struct in_addr *)&temp->destinationIP);
                                    printf("dest IP: %s\n", str);
                                    printf("control code: %u\n", temp->controlCode);
                                    printf("response time: %u\n", temp->responseTime);
                                    printf("payload length: %u\n", temp->payloadLength);
                                    printf("unpack successful\n");
                                    
                                    
                                }
                                
                            }//end of bad accept
                        }
                        //select SET#2
                        else if(FD_ISSET(sockfdUpdates, &tempRead_fds))
                        {
                            //hand UDP routing updates here using recvfrom() and manipulate them for Bellman-Ford
                        }
                        //select SET#3
                        else
                        {
                            
                        }
                    }
                }//for of iterating through connections ends
            }//else of good select ends
        }//for(;;) ends
        
    }// estalblishRouter ends
};// class ends

int main(int argc, char **argv)
{
    /*Start Here*/
    uint16_t controlPort;
    /*http://stackoverflow.com/questions/20019786/safe-and-portable-way-to-convert-a-char-to-uint16-t*/
    controlPort=strtol(argv[1], NULL, 10);
    
    if (argc<1)
    {
        fprintf(stderr,"USAGE: ./<app-name> <control port>");
        exit(1);
    }
    else
    {
        Router router = *new Router();
        router.estalblishRouter(controlPort);
    }
    
    return 0;
}