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
#include <algorithm>
#include "Serialize.h"


#define MAXBUFLEN 100;
#define INF 65535;

//using namespace std;

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

struct controlPacketPayload
{
    uint32_t routerIP[5];
    uint16_t nodes;
    uint16_t updateInterval;
    uint16_t routerID[5];
    uint16_t routerPort[5];
    uint16_t dataPort[5];
    uint16_t metric[5];
    uint16_t sequenceNumber;
    uint8_t TTL;
    uint8_t transferID;
    char fileName[256];
    
};

struct controlResponsePayload
{
    char *dataString;
    uint16_t routerID;
    uint16_t padding;
    uint16_t nextHop;
    uint16_t metric;
    uint8_t transferID;
    uint8_t TTL;
};


struct controlPacketHeader
{
    uint32_t destinationIP;
    uint8_t controlCode;
    uint8_t responseTime;
    uint16_t payloadLength;
};

struct controlResponseHeader
{
    uint32_t controllerIP;
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
    struct sockaddr_in6 *peername6;
    struct sockaddr_in *peername;
    char buffer[100];
    unsigned char *controlBuffer;
    unsigned char *controlResponseBuffer;
    unsigned char *controlResponsePayloadBuffer;
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
                //printf("Router now listening for updates on router port: %s\n", listenPort);
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
                                    peername = (struct sockaddr_in *) &clientAddress;
                                    clientPort = ntohs(peername->sin_port);
                                    inet_ntop(AF_INET, &peername->sin_addr, storeAddress, sizeof (storeAddress));
                                }
                                else
                                {
                                    peername6 = (struct sockaddr_in6 *) &clientAddress;
                                    clientPort = ntohs(peername->sin_port);
                                    inet_ntop(AF_INET, &peername->sin_addr, storeAddress, sizeof (storeAddress));
                                }
                                printf("Accepting new connection from %s: %d on socket %d\n\n", storeAddress, clientPort, newsockfd);
                                printf("Socket %d is bound to %s\n", newsockfd, storeAddress);
                                
                                /*handle data on TCP socket here using recv
                                 1. do actions based on control codes
                                 2. handle file transfer if control code is Ox05 and 0x06 */
                                controlBuffer =  new unsigned char [1024];
                                
                                printf("####clearing controlBuffer####\n");
                                //controlBuffer = (unsigned char *)(malloc(1024));
                                bzero(controlBuffer, sizeof(controlBuffer));
                                printf("size of receive buffer is: %lu\n", sizeof(controlBuffer));
                                
                                //READ DATA FROM CONTROLLER
                                    if((readBytes = recv(newsockfd,controlBuffer,1024,0)) <=0)
                                    {
                                        if(readBytes==0)
                                        {
                                            printf("Remote Server Hung Up\n");
                                            break;
                                        }
                                        else
                                        {
                                            perror("ERROR IN RECEIVING");
                                            break;
                                        }
                                        close(i);
                                        FD_CLR(i,&masterDescriptor);
                                    }
                                
                                
                                if(readBytes>1)
                                {
                                    //OK WE HAVE SOME DATA NOW FROM THE CONTROLLER
                                    
                                    printf("received %d bytes of data from the CONTROLLER\n", readBytes);
                                    printf("trying to unpack\n");
                                    printf("--------HEADER CONTAINS--------\n");
                                    struct controlPacketHeader *cph =  (struct controlPacketHeader *) malloc(sizeof(struct controlPacketHeader));
                                    
                                    //call to unpack using args as: 32(L), 8(C), 8(C) and 16 (H) followed by payload
                                    unpack(controlBuffer, "LCCH", &cph->destinationIP, &cph->controlCode, &cph->responseTime, &cph->payloadLength);
                                    
                                    /*FOR BASIC TESTING-WILL IT BE SAME FOR ALL? - YES HEADER IS SAME. PAYLOAD VARIES ACC TO CONTROL CODE*/
                                    char * str = inet_ntoa(*(struct in_addr *)&cph->destinationIP);
                                    printf("dest IP: %s\n", str);
                                    printf("control code: %u\n", cph->controlCode);
                                    printf("response time: %u\n", cph->responseTime);
                                    printf("payload length: %u\n", cph->payloadLength);
                                    printf("unpack successful\n");
                                    
                                    
                                    /*DECISIONS BASED ON CONTROL CODES NOW*/
                                    if(cph->controlCode==0)
                                    {
                                        //AUTHOR-RESPONSE REQUIRED
                                        //for my testing
                                        char hex[5];
                                        sprintf(hex, "%x", cph->controlCode);
                                        printf("control code %s found. Academic Integrity Response will be generated\n", hex);
                                        
                                        controlResponseBuffer = new unsigned char [1024];
                                        controlResponsePayloadBuffer = new unsigned char [1024];
                                        struct controlResponseHeader *crh = (struct controlResponseHeader *) malloc(sizeof(struct controlResponseHeader));
                                        struct controlResponsePayload *crp = (struct controlResponsePayload *) malloc(sizeof(struct controlResponsePayload));
                                        
                                        crh->controllerIP=static_cast<uint32_t>(peername->sin_addr.s_addr);
                                        crh->controlCode=0;
                                        crh->responseCode=0;
                                        crp->dataString=static_cast<char *>(malloc(256));
                                        strcpy(crp->dataString, "I, agautam2, have read and understood the course academic integrity policy.\0");
                                        crh->payloadLength=sizeof("I, agautam2, have read and understood the course academic integrity policy.")-1;
                                        printf("done writing\n");
                                        fflush(stdout);
                                        
                                        //call to pack using args as: 32(L), 8(C), 8(C) and 16 (H) followed by payload
                                        pack(controlResponseBuffer, "LCCHC", (uint32_t)crh->controllerIP, (uint8_t)crh->controlCode, (uint8_t)crh->responseCode,(uint16_t)crh->payloadLength);
                                        controlResponseBuffer += 8;
                                        memcpy(controlResponseBuffer, crp->dataString, 75);//to make it work
                                        controlResponseBuffer=controlResponseBuffer-8;
                                        
                                        //pack(controlResponsePayloadBuffer, "s", crp->dataString);
                                        
                                        /*To test from sending side:*/
                                        FILE *fp;
                                        fp = fopen("file1.txt", "w");
                                        fwrite(controlResponseBuffer, 83, 1, fp); //(payload (75)+ header(8)
                                        fclose(fp);
                                        
                                        //SEND
                                        int ableToSend1 = send(newsockfd, controlResponseBuffer,83, 0);
                                        
                                        if(ableToSend1<0)
                                        {
                                            printf("failed to send serialized header\n");
                                        }
                                    }
                                    else if(cph->controlCode==1)
                                    {
                                        //INIT-BUILD RT-NO RESPONSE REQUIRED
                                        char hex[5];
                                        sprintf(hex, "%x", cph->controlCode);
                                        printf("control code %s found. Routing Table will be populated\n",hex);
                                        struct controlPacketPayload *cpp = (struct controlPacketPayload *) malloc(sizeof(struct controlPacketPayload));
                                        //controlBuffer +=8;
                                        //call to unpack to extract payload using args as: 16(H), 16(H), 16(H), 16(H), 16(H), 16(H), 32(L)
                                        
                                        
                                        unpack(controlBuffer, "LCCH HH HHHHL HHHHL HHHHL HHHHL HHHHL", &cph->destinationIP, &cph->controlCode, &cph->responseTime, &cph->payloadLength , &cpp->nodes, &cpp->updateInterval, &cpp->routerID[0], &cpp->routerPort[0], &cpp->dataPort[0], &cpp->metric[0], &cpp->routerIP[0], &cpp->routerID[1], &cpp->routerPort[1], &cpp->dataPort[1], &cpp->metric[1], &cpp->routerIP[1], &cpp->routerID[2], &cpp->routerPort[2], &cpp->dataPort[2], &cpp->metric[2], &cpp->routerIP[2], &cpp->routerID[3], &cpp->routerPort[3], &cpp->dataPort[3], &cpp->metric[3], &cpp->routerIP[3], &cpp->routerID[4], &cpp->routerPort[4], &cpp->dataPort[4], &cpp->metric[4], &cpp->routerIP[4]);
                                        
                                        printf("unpacking again for code %s \n", hex);
                                        printf("--------HEADER CONTAINS---------\n");
                                        char * str = inet_ntoa(*(struct in_addr *)&cph->destinationIP);
                                        printf("dest IP: %s\n", str);
                                        printf("control code: %u\n", cph->controlCode);
                                        printf("response time: %u\n", cph->responseTime);
                                        printf("payload length: %u\n", cph->payloadLength);
                                        printf("--------PAYLOAD CONTAINS--------\n");
                                        printf("No. of nodes: %u\n",cpp->nodes);
                                        printf("Update Interval: %u\n",cpp->updateInterval);
                                        for(i=0;i<5;i++)
                                        {
                                        printf("Router ID [%d] : %u\n",i+1, cpp->routerID[i]);
                                        printf("Router Port [%d] : %u\n",i+1,cpp->routerPort[i]);
                                        printf("Data Port [%d] : %u\n",i+1, cpp->dataPort[i]);
                                        printf("Metric [%d] : %u\n",i+1, cpp->metric[i]);
                                        }
                                        char * str1= inet_ntoa(*(struct in_addr *)&cpp->routerIP[0]);
                                        printf("Router IP [1]: %s\n", str1);
                                        char * str2= inet_ntoa(*(struct in_addr *)&cpp->routerIP[1]);
                                        printf("Router IP [2]: %s\n", str2);
                                        char * str3= inet_ntoa(*(struct in_addr *)&cpp->routerIP[2]);
                                        printf("Router IP [3]: %s\n", str3);
                                        char * str4= inet_ntoa(*(struct in_addr *)&cpp->routerIP[3]);
                                        printf("Router IP [4]: %s\n", str4);
                                        char * str5= inet_ntoa(*(struct in_addr *)&cpp->routerIP[4]);
                                        printf("Router IP [5]: %s\n", str5);
                                        
                                        
                                    }
                                    else if(cph->controlCode==2)
                                    {
                                        //ROUTING TABLE-RT RESPONSE REQUIRED
                                        printf("control code 0x02 found. Routing Table requested. Will be sent\n");
                                    }
                                    else if(cph->controlCode==3)
                                    {
                                        //UPDATE-UPDATE RT-NO RESPONSE REQUIRED
                                    
                                        char hex[5];
                                        sprintf(hex, "%x", cph->controlCode);
                                        printf("control code %s found. Routing Table will be populated. Waiting for payload...\n",hex);
                                        struct controlPacketPayload *cpp = (struct controlPacketPayload *) malloc(sizeof(struct controlPacketPayload));
                                        unpack(controlBuffer, "LCCHHH", &cph->destinationIP, &cph->controlCode, &cph->responseTime, &cph->payloadLength , &cpp->routerID[0], &cpp->metric[0]);
                                        printf("unpacking again for code %s \n", hex);
                                        printf("--------HEADER CONTAINS---------\n");
                                        char * str = inet_ntoa(*(struct in_addr *)&cph->destinationIP);
                                        printf("dest IP: %s\n", str);
                                        printf("control code: %u\n", cph->controlCode);
                                        printf("response time: %u\n", cph->responseTime);
                                        printf("payload length: %u\n", cph->payloadLength);
                                        printf("--------PAYLOAD CONTAINS--------\n");
                                        printf("Router ID for which cost is to be updated: %u\n",cpp->routerID[0]);
                                        printf("Cost to be updated to %u\n",cpp->metric[0]);
                                        
                                    }
                                    else if(cph->controlCode==4)
                                    {
                                        //CRASH-TURN OFF ROUTING OPERATIONS-RESPONSE REQUIRED
                                        printf("control code 0x04 found. Router will crash now\n");
                                    }
                                    else if(cph->controlCode==5)
                                    {
                                        //SENDFILE-SEND FILE TO OTHER ROUTER-NO RESPONSE REQUIRED
                                        char hex[5];
                                        sprintf(hex, "%x", cph->controlCode);
                                        printf("control code %s found. File needs to be sent. Waiting for payload...\n",hex);
                                        struct controlPacketPayload *cpp = (struct controlPacketPayload *) malloc(sizeof(struct controlPacketPayload));
                                        printf("size of filename: %d", sizeof(cpp->fileName));
                                        unpack(controlBuffer, "LCCHLCCH256s", &cph->destinationIP, &cph->controlCode, &cph->responseTime, &cph->payloadLength ,&cpp->routerIP[0], &cpp->TTL, &cpp->transferID, &cpp->sequenceNumber, &cpp->fileName);
                                        printf("unpacking again for code %s \n", hex);
                                        printf("--------HEADER CONTAINS---------\n");
                                        char * str = inet_ntoa(*(struct in_addr *)&cph->destinationIP);
                                        printf("dest IP: %s\n", str);
                                        printf("control code: %u\n", cph->controlCode);
                                        printf("response time: %u\n", cph->responseTime);
                                        printf("payload length: %u\n", cph->payloadLength);
                                        printf("--------PAYLOAD CONTAINS--------\n");
                                        char * str1= inet_ntoa(*(struct in_addr *)&cpp->routerIP[0]);
                                        printf("File to be sent: %s\n",cpp->fileName);
                                        printf("Router IP to which file is to be sent: %s\n",str1);
                                        printf("TTL Value: %u\n", cpp->TTL);
                                        printf("Init Seq num: %u\n", cpp->sequenceNumber);
                                        printf("TransferID: %u\n",cpp->transferID);
                                        
                                        
                                    }
                                    else if(cph->controlCode==6)
                                    {
                                        //SENDFILESTATS-RESPONSE REQUIRED
                                        char hex[5];
                                        sprintf(hex, "%x", cph->controlCode);
                                        printf("control code %s found. File STATS need to be sent. Waiting for payload...\n",hex);
                                        struct controlPacketPayload *cpp = (struct controlPacketPayload *) malloc(sizeof(struct controlPacketPayload));
                                        unpack(controlBuffer, "LCCHC", &cph->destinationIP, &cph->controlCode, &cph->responseTime, &cph->payloadLength ,&cpp->transferID);
                                        printf("unpacking again for code %s \n", hex);
                                        printf("--------HEADER CONTAINS---------\n");
                                        char * str = inet_ntoa(*(struct in_addr *)&cph->destinationIP);
                                        printf("dest IP: %s\n", str);
                                        printf("control code: %u\n", cph->controlCode);
                                        printf("response time: %u\n", cph->responseTime);
                                        printf("payload length: %u\n", cph->payloadLength);
                                        printf("--------PAYLOAD CONTAINS--------\n");
                                        printf("Stats needed for ID: %u\n",cpp->transferID);
                                    }
                                    else if(cph->controlCode==7)
                                    {
                                        //LASTDATAPACKET-RESPONSE REQUIRED
                                        printf("control code 0x07 found. Last data packet needs to be sent\n");
                                    }
                                    else if(cph->controlCode==8)
                                    {
                                        //SECONDLASTDATAPACKET-RESPONSE REQUIRED
                                        printf("control code 0x08 found. Second Last data packet needs to be sent\n");
                                    }
                                    
                                    
                                }
                                
                            }//end of bad accept
                        }
                        //select SET#2
                        else if(FD_ISSET(sockfdUpdates, &tempRead_fds))
                        {
                            //handle UDP routing updates here using recvfrom() and manipulate them for Bellman-Ford
                        }
                        //select SET#3
                        else
                        {
                            //is this even neeeded?
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