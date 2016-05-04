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

//  Code to implement a Distance Vector Routing Protocol
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


/*TODO - 27th April
 1. Select call for data port
 2. Bellman ford function
 3. Send update function
 4. Display routing table function
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

// for ROUTING ENTRIES in a ROUTING UPDATE PACKET
struct routingEntry
{
    uint32_t destinationRouterIP;
    uint16_t destinationRouterPort;
    uint16_t padding;
    uint16_t destinationRouterID;
    uint16_t metricCost;
};

/* for ROUTING UPDATE PACKET. A pointer of this struct will be used
to do all the work related to a routing update and sent to other
routers via UDP.
*/
 struct updatePacket
{
    uint16_t numberOfUpdateFields;
    uint16_t sourceRouterPort;
    uint32_t sourceRouterIP;
    routingEntry routingEntries [];
};


/*For routing table at each router also known as LBTT. A PART of it will be sent to the
CONTROLLER. Make an array of struct whenever the info is to be loaded 
into it and use it to store locally.
 
 ----THIS IS DIFFERENT FROM A ROUTING UPDATE STRUCT----*/
struct routingTable
{
    int uptime;
    bool ne;
    //REQD FIELDS
    uint16_t destinationRouterID;
    uint16_t routerSerialNumber;
    uint16_t padding;
    uint16_t nextHopID;
    uint16_t metricCost;
    uint16_t routerPort; //added later. add its functionality in the lower part of the code
    uint16_t dataPort;
    //EXTRA FIELDS
    uint32_t destinationIP;
    uint32_t sourceRouterIP;
    
    bool doesExist;
    bool active;
};

//DUMP FOR ALL DATA RCVD FROM THE CONTROLLER IN THE PAYLOAD
struct controlPacketPayload
{
    uint32_t routerIP[5];
    uint16_t nodes;
    uint16_t updateInterval;
    uint16_t routerID[5];
    uint16_t routerPort[5];
    uint16_t dataPort[5];
    uint16_t metric[5];
    uint16_t changedCost;
    uint16_t changeCostForRID;
    uint32_t sendFileToThisIP;
    uint16_t sequenceNumber;
    uint8_t TTL;
    uint8_t transferID;
    char fileName[256];
    
};

//FOR CONTROL RESPONSE PAYLOAD - 1
struct controlResponsePayload
{
    uint8_t transferID;
    uint8_t TTL;
};

//CONTROL RESPONSE FOR ROUTING TABLE (0x02) - NOT USED AS FAR
struct topologyTable
{
    uint16_t routerID;
    uint32_t destinationIP;
    uint16_t padding;
    uint16_t nextHopRouterID;
    uint16_t metric;
    bool neighbor;
    int holdTime;
};

//FOR CONTROL HEADER
struct controlPacketHeader
{
    uint32_t destinationIP;
    uint8_t controlCode;
    uint8_t responseTime;
    uint16_t payloadLength;
};

//FOR CONTROL RESPONSE HEADER
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
    
    char *listenPortRP, *listenPortDP;
    int iSocket, sockfdController, sockfdUpdates, sockfdData, fdMaxNumber, newsockfd, readBytes, sockFile, fileRecvSocket;
    uint16_t routerPort, dataPort;
    struct addrinfo hints, *servinfo, *p;
    int rv, returnedInfo;
    int numbytes, clientPort;
    struct sockaddr_storage their_addr, clientAddress;
    struct sockaddr_in6 *peername6;
    struct sockaddr_in *peername;
    char buffer[100];   //check this - agautam2
    unsigned char *controlBuffer;
    unsigned char *controlResponseBuffer;
    unsigned char *controlResponsePayloadBuffer;
    unsigned char *updateBuffer;
    socklen_t addr_len, clientLength;
    char s[INET6_ADDRSTRLEN];
    char storeAddress[INET6_ADDRSTRLEN];
    char service [20];
    char host [1024];
    fd_set masterDescriptor;
    fd_set tempRead_fds;
    int yes, selectReturn;
    //#GLOBAL USAGE
    int neighborCount, stopwatch;
    uint16_t whoAmiID;  //stores network order ID (convert whenever to be used)
    uint16_t whoAmiPort; //stores network order ID (convert whenever to be used)
    uint32_t whoAmiIP; //stores network order ID (convert whenever to be used)
    uint16_t updateInterval, nodeCount; // stores network order ID (convert whenever to be used)
    uint16_t dv[10][10];
    unsigned int neReachability[10];
    
    
    
    /*All class-related variables go here*/
    
public: Router()
    {
        printf("Default constructor intialised\n");
        stopwatch=0;
    }

// Pseudo code for algo - Pg 373 Ross Kurose Chapter 4
public: int distanveVectorRoutingAlgorithm(uint16_t dv[][10], uint16_t nodeCount, uint16_t myID, struct routingTable * lbtt)
    {
        for(int i=1; i<=ntohs(nodeCount) ; i++)
        {
            int mySerialNumber;
            int tempNextHopID=-1;
            int comparator=INF;
            int maxHop=65535;
            //lookup for serialnumber of myID
            for(int i=1;i<=ntohs(nodeCount);i++)
            {
                if(ntohs(lbtt[i].destinationRouterID)==ntohs(myID))
                {
                    mySerialNumber=i;
                    printf("#1 mySerialNumber is: [%d]\n", mySerialNumber);
                }
            }
            
            
            if(ntohs(lbtt[i].destinationRouterID)==ntohs(myID))
            {
                continue;
            }
            
            for(int j=1; j<=ntohs(nodeCount); j++)
            {
                if (lbtt[j].ne==true)
                {
                    if(lbtt[j].active==true)
                    {
                        if(ntohs(lbtt[j].destinationRouterID)!=ntohs(myID))
                        {
                            if(comparator > (dv[mySerialNumber][(lbtt[j].routerSerialNumber)]+dv[(lbtt[j].routerSerialNumber)][(lbtt[i].routerSerialNumber)]))
                            {
                                comparator=dv[mySerialNumber][(lbtt[j].routerSerialNumber)]+dv[(lbtt[j].routerSerialNumber)][(lbtt[i].routerSerialNumber)];
                                tempNextHopID=(lbtt[j].destinationRouterID);
                            }
                        }
                    }
                }
            }
            
            dv[mySerialNumber][(lbtt[i].routerSerialNumber)]=comparator;
            lbtt[i].nextHopID=tempNextHopID;
            printf("///////////// nextHopID updated to : [%u]\n", ntohs(lbtt[i].nextHopID));
            lbtt[i].metricCost=comparator;
            if(dv[mySerialNumber][(lbtt[i].routerSerialNumber)]==maxHop)
            {
                lbtt[i].nextHopID=-1;
            }
        }

        return 0;
    }
//sends periodic updates on router port (UDP)
// send nodeCount in network order when calling this method. still to decide on myPort and myIP
public: int transmitRoutingUpdates(struct routingTable lbtt[], uint16_t myPort, uint32_t myIP, uint16_t nodeCount, int s)
    {
        ssize_t sentBytes,size=(2*sizeof(uint16_t))+(sizeof(struct in_addr))+(ntohs(nodeCount)*sizeof(struct routingEntry));
        //updateBuffer = new unsigned char [1024];
        struct updatePacket *updateMessage=(struct updatePacket*)malloc(sizeof(struct updatePacket));
        //send nodeCount, myPort and myIP in network order only
        updateMessage->numberOfUpdateFields=nodeCount;  //2 bytes
        updateMessage->sourceRouterPort=myPort;   //2 bytes
        updateMessage->sourceRouterIP=myIP; //4 bytes
        
        //ROUTING ENTRIES BEGIN NOW
        for(int i=1; i<=ntohs(nodeCount); i++)
        {
            updateMessage->routingEntries[i-1].destinationRouterIP = lbtt[i].destinationIP; //4 bytes
            updateMessage->routingEntries[i-1].destinationRouterPort = lbtt[i].routerPort; //2 bytes
            updateMessage->routingEntries[i-1].padding=0; //2 bytes
            updateMessage->routingEntries[i-1].destinationRouterID = lbtt[i].destinationRouterID;//2 bytes
            updateMessage->routingEntries[i-1].metricCost = htons(lbtt[i].metricCost);// 2bytes
        }
        
//        printf("lbtt[1]:COST  [%u]\n", (lbtt[1].metricCost));
//        printf("lbtt[2]:COST  [%u]\n", (lbtt[2].metricCost));
//        printf("lbtt[3]:COST  [%u]\n", (lbtt[3].metricCost));
//        printf("lbtt[4]:COST  [%u]\n", (lbtt[4].metricCost));
       
        printf("lbtt[1]:NE? (YES=1 NO=0) [%d]\n", lbtt[1].ne);
        printf("lbtt[2]:NE? (YES=1 NO=0) [%d]\n", lbtt[2].ne);
        printf("lbtt[3]:NE? (YES=1 NO=0) [%d]\n", lbtt[3].ne);
        printf("lbtt[4]:NE? (YES=1 NO=0) [%d]\n", lbtt[4].ne);
       
//        printf("lbtt[1]:ACTIVE? (YES=1 NO=0) [%d]\n", lbtt[1].active);
//        printf("lbtt[2]:ACTIVE? (YES=1 NO=0) [%d]\n", lbtt[2].active);
//        printf("lbtt[3]:ACTIVE? (YES=1 NO=0) [%d]\n", lbtt[3].active);
//        printf("lbtt[4]:ACTIVE? (YES=1 NO=0) [%d]\n", lbtt[4].active);
        
    
        for(int i=1;i<=ntohs(nodeCount);i++)
        {
            if(lbtt[i].ne==true)
            {
                //printf("######## i [%d] is true\n", i);
                if(lbtt[i].active==true)
                {
                    struct sockaddr_in dest;
                    dest.sin_family=AF_INET;
                    dest.sin_port=lbtt[i].routerPort;
                    dest.sin_addr= *(struct in_addr *)&lbtt[i].destinationIP;
                    sentBytes=sendto(s, (struct updatePacket*)updateMessage, size, 0,(struct sockaddr*)&dest, sizeof dest);
                    char *ip=inet_ntoa(dest.sin_addr);
                    printf("[PERIODIC UPDATE] - SENDING %zd bytes of routing update to [%d] on port [%u]\n", sentBytes, ntohs(lbtt[i].destinationRouterID), ntohs(lbtt[i].routerPort));
                }
            }
        }
        return 0;
    }
public: int establishRoutingUpdates(uint16_t rp)
    {
        /*[PA3]Router will listen for routing updates on this ROUTER PORT (UDP)*/
        
        //read router port from INIT control payload
        
        routerPort=ntohs(rp);
        printf("LISTENING FOR ROUTING UPDATES ON ROUTER PORT[%u...]\n", routerPort);
        listenPortRP=(char*)malloc(sizeof(int)+1);
        sprintf(listenPortRP, "%d", routerPort);
        
        /*setting up DATAGRAM SOCKET - code from BEEJ NETWORK PRGRAMMING GUIDE*/
        printf("setting families\n");
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_flags = AI_PASSIVE;

        
        if ((rv = getaddrinfo(NULL, listenPortRP, &hints, &servinfo)) != 0)
        {
            fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
            return 1;
        }
        printf("building socket\n");
        for(p = servinfo; p != NULL; p = p->ai_next)
        {
            if ((sockfdUpdates = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1)   //SOCKET
            {
                printf("listener: Could not create socket\n");
                continue;
            }
            if(setsockopt(sockfdUpdates, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int))==-1)
            {
                printf("setsockopt: check SO_REUSEADDR");
                exit(2);
            }
            if(::bind(sockfdUpdates, p->ai_addr, p->ai_addrlen) < -1)   //BIND
            {
                close(sockfdUpdates);
                perror("server: bind\n");
                continue;
            }
            break;
        }
        if (p == NULL)
        {
            fprintf(stderr, "listener: failed to bind socket\n");
            return -1;
        }
        printf("all done\n");
        freeaddrinfo(servinfo);
        printf("freeing servinfo\n");
        printf("listener: waiting to recvfrom on [%s] ...\n", listenPortRP);
        
        FD_SET(sockfdUpdates, &masterDescriptor);
        printf("added sockfdUpdates to masterDescriptor\n");
        if(sockfdUpdates>fdMaxNumber)
        {
            fdMaxNumber=sockfdUpdates;
            printf("fdMaxNumber updated to %d in UDP call control\n", fdMaxNumber);
        }
        return 1;
    }
public: int establisDataTransfer(uint16_t dp)
    {
        dataPort=ntohs(dp);
        
        listenPortDP=(char*)malloc(sizeof(int)+1);
        sprintf(listenPortDP, "%d", dataPort);
        return 1;
        FD_ZERO(&masterDescriptor);
        FD_ZERO(&tempRead_fds);
        
        memset(&hints, 0, sizeof hints);
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        hints.ai_flags = AI_PASSIVE;
        
        /*[PA3]Router will listen for controller messages on this CONTROL PORT(TCP). This will be supplied as an argument when executing the application*/
        returnedInfo = getaddrinfo(NULL, listenPortDP, &hints, &servinfo);
        printf("returned info: %d\n",returnedInfo);
        if(returnedInfo!=0)
        {
            fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(returnedInfo));
            return 1;
        }
        for(p=servinfo; p!=NULL;p=p->ai_next)
        {
            sockfdData = socket(p->ai_family, p->ai_socktype, p->ai_protocol);          //SOCKET
            printf("socket file descriptor :%d\n\n", sockfdData);
            if(sockfdData<0)
            {
                perror("server: socket");
                continue;
            }
            if(setsockopt(sockfdData, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int))==-1)
            {
                perror("setsockopt");
                exit(1);
            }
            if(::bind(sockfdData, p->ai_addr, p->ai_addrlen) < -1)                      //BIND
            {
                close(sockfdData);
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
            printf("LISTENING FOR DATA CONNECTIONS ON DATA PORT[%u...]\n", dataPort);
        }
    }
public: int estalblishRouter(uint16_t controlPort)
    {
        struct routingTable localBaseTopologyTable [6];
        printf("You choose to use %hu as the control port\n", controlPort);
        //printf("You choose to use %hu as the data port\n", routerPort);
        
        char *controlPortChar;
        controlPortChar = (char*)malloc(sizeof(int)+1);
        sprintf(controlPortChar, "%d", controlPort);
        //printf("%s\n",controlPortChar);
        
        /*Setting up TCP SOCKET - code taken from BEEJ NETWORK PROGRAMMING GUIDE*/
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
        /*CONTROL PORT(TCP) CODE ENDS HERE*/
        
        /*SELECT SYSTEM HANDLING*/
        FD_SET(sockfdController, &masterDescriptor);
        //FD_SET(sockfdUpdates, &masterDescriptor);
        fdMaxNumber=sockfdController;
        printf("fdMaxNumber in FD_SET:CONTROLLER is: %d\n",fdMaxNumber);
        struct timeval selectCallTimer;
        
        for(;;)
        {
            tempRead_fds = masterDescriptor;
            selectCallTimer.tv_sec=1.0;
            selectCallTimer.tv_usec=0.0;
            
            //SELECT CALL
            selectReturn=select(fdMaxNumber+1, &tempRead_fds, NULL, NULL, &selectCallTimer);
           
            //CHECK FOR SELECT RETURN CASES
            if(selectReturn==-1)//bad select
            {
                perror("ERROR IN SELECT SYSTEM CALL");
            }
            else if(selectReturn==0)
            {
                /*select with no timely response - HANDLE MISSED UDP UPDATES HERE
                 THIS BLOCK WILL BE FIRED EVERY UNIT TIME AND A TIMER VARIABLE WILL
                 BE USED TO KEEP TRACK*/
                
                int mySerialNumber;
                //lookup for serialnumber of myID
                for(int i=1;i<=ntohs(nodeCount);i++)
                {
                    if(ntohs(localBaseTopologyTable[i].destinationRouterID)==ntohs(whoAmiID))
                    {
                        mySerialNumber=i;
                        printf("#2 mySerialNumber is: [%d]\n", mySerialNumber);
                    }
                }
                
                printf("timer value: %d\n",stopwatch);
                stopwatch++;
                if(stopwatch==ntohs(updateInterval))
                {
//                    printf("sockdfUpdates before sending update is: [%d]\n",sockfdUpdates);
//                    printf("####**** before calling transmitRoutingUpdates value of lbtt[1].ne : %d\n", localBaseTopologyTable[1].ne);
//                    printf("####**** before calling transmitRoutingUpdates value of lbtt[2].ne : %d\n", localBaseTopologyTable[2].ne);
//                    printf("####**** before calling transmitRoutingUpdates value of lbtt[3].ne : %d\n", localBaseTopologyTable[3].ne);
//                    printf("####**** before calling transmitRoutingUpdates value of lbtt[4].ne : %d\n", localBaseTopologyTable[4].ne);
                    transmitRoutingUpdates(localBaseTopologyTable, whoAmiPort, whoAmiIP, nodeCount, sockfdUpdates);
                    stopwatch=0;
                }
                
                for(int i=1; i<=ntohs(nodeCount); i++)
                {
                    if (localBaseTopologyTable[i].ne==true)
                    {
                       if(localBaseTopologyTable[i].doesExist==true)
                       {
                           if(localBaseTopologyTable[i].active==true)
                           {
                               localBaseTopologyTable[i].uptime++;
                        
                                printf("time since update: %d\n", localBaseTopologyTable[i].uptime);
                               
                                if(localBaseTopologyTable[i].uptime==(ntohs(updateInterval)*3))
                                {
                                    localBaseTopologyTable[i].nextHopID=-1;
                                    localBaseTopologyTable[i].active=false;
                                    
                                    dv[mySerialNumber][(localBaseTopologyTable[i].routerSerialNumber)]=INF;
                                        for (int j = 1; j <=ntohs(nodeCount); j++)
                                        {
                                            printf("setting dv to inf\n");
                                            dv[(localBaseTopologyTable[i].routerSerialNumber)][(localBaseTopologyTable[j].routerSerialNumber)]=INF;
                                        }
                                    printf("No update received from Neighbor [%u] for %d secs, setting cost to INF\n",ntohs(localBaseTopologyTable[i].destinationRouterID), ntohs(updateInterval)*3);
                                    //calling bellman-ford INSTANCE #3
                                    distanveVectorRoutingAlgorithm(dv, nodeCount, whoAmiID, localBaseTopologyTable);
                                    //bellman_ford(my_server_id, number_servers, local_table, distance_vector);
                                }
                           }
                       }
                    }
                }
                
            }//end of select with no timely response
            else
            {
                //good select with timely response either from controller, UDP updates or the DATA PORT
                
                
                for(iSocket=0;iSocket<=fdMaxNumber+1;iSocket++)
                {
                    if(FD_ISSET(iSocket, &tempRead_fds))
                    {
                        if(iSocket==sockfdController)
                        {
                            printf("found connection from CONTROLLER, sockfdController value is [%d]\n",sockfdController);
                            clientLength=sizeof(clientAddress);
                            newsockfd=accept(sockfdController, (sockaddr *)&clientAddress, &clientLength);
                            if(newsockfd==-1)
                            {
                                perror("ERROR IN ACCEPT");
                            }
                            else    //good accept
                            {
                                printf("adding newsockfd [%d] to masterDescriptor\n", newsockfd);
                                FD_SET(newsockfd, &masterDescriptor);
                                if(newsockfd>fdMaxNumber)
                                {
                                    fdMaxNumber=newsockfd;
                                    printf("fdMaxNumber updated at accept to %d\n", fdMaxNumber);
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
                                
                                
                            }//end of good accept
                        
                        }
                        else if(iSocket==sockfdUpdates)   //to handle UDP updates
                        {
                            ssize_t receivedBytes,size=(2*sizeof(uint16_t))+(sizeof(struct in_addr))+(ntohs(nodeCount) *sizeof(struct routingEntry));
                            struct updatePacket *updateMessage=(updatePacket*)malloc(size);
                            
                            int mySerialNumber;
                            //lookup for serialnumber of myID
                            for(int i=1;i<=ntohs(nodeCount);i++)
                            {
                                if(ntohs(localBaseTopologyTable[i].destinationRouterID)==ntohs(whoAmiID))
                                {
                                    mySerialNumber=i;
                                    printf("#3 mySerialNumber is: [%d]\n", mySerialNumber);
                                }
                            }
                            
                            receivedBytes=recvfrom(iSocket, (struct updatePacket*)updateMessage, size, 0, NULL, NULL);
                            printf("*******************************************************************\n");
                            printf("[PERIODIC UPDATE] - RECEIVING %zd bytes of routing update on %u\n", receivedBytes, ntohs(whoAmiPort));
                            printf("[PERIODIC UPDATE] - number of update fields in the update: %u\n", ntohs(updateMessage->numberOfUpdateFields));
//                            printf("[PERIODIC UPDATE] - source port in the update: %u\n", ntohs(updateMessage->sourceRouterPort));
//                            char * str = inet_ntoa(*(struct in_addr *)&updateMessage->sourceRouterIP);
//                            printf("[PERIODIC UPDATE] - source ip in the update: %s\n", str);
//                            char * str2 = inet_ntoa(*(struct in_addr *)&updateMessage->routingEntries[0].destinationRouterIP);
//                            printf("[PERIODIC UPDATE] - ROUTER ip 1 in the update: %s\n", str2);
//                            printf("[PERIODIC UPDATE] - ROUTER port 1 in the update: %d\n", ntohs(updateMessage->routingEntries[0].destinationRouterPort));
//                            printf("[PERIODIC UPDATE] - ROUTER ID 1 in the update: %d\n", ntohs(updateMessage->routingEntries[0].destinationRouterID));
//                            printf("[PERIODIC UPDATE] - ROUTER cost 1 in the update: %d\n", ntohs(updateMessage->routingEntries[0].metricCost));
//                            printf("[PERIODIC UPDATE] - Padding 1 in the update: %d\n\n", ntohs(updateMessage->routingEntries[0].padding));
//                            
//                            char * str3 = inet_ntoa(*(struct in_addr *)&updateMessage->routingEntries[1].destinationRouterIP);
//                            printf("[PERIODIC UPDATE] - ROUTER ip 2 in the update: %s\n", str3);
//                            printf("[PERIODIC UPDATE] - ROUTER port 2 in the update: %d\n", ntohs(updateMessage->routingEntries[1].destinationRouterPort));
//                            printf("[PERIODIC UPDATE] - ROUTER ID 2 in the update: %d\n", ntohs(updateMessage->routingEntries[1].destinationRouterID));
//                           printf("[PERIODIC UPDATE] - ROUTER cost 2 in the update: %d\n", ntohs(updateMessage->routingEntries[1].metricCost));
//                            printf("[PERIODIC UPDATE] - Padding 2 in the update: %d\n\n", ntohs(updateMessage->routingEntries[1].padding));
//                            
//                            char * str4 = inet_ntoa(*(struct in_addr *)&updateMessage->routingEntries[2].destinationRouterIP);
//                            printf("[PERIODIC UPDATE] - ROUTER ip 3 in the update: %s\n", str4);
//                            printf("[PERIODIC UPDATE] - ROUTER port 3 in the update: %d\n", ntohs(updateMessage->routingEntries[2].destinationRouterPort));
//                            printf("[PERIODIC UPDATE] - ROUTER ID 3 in the update: %d\n", ntohs(updateMessage->routingEntries[2].destinationRouterID));
//                            printf("[PERIODIC UPDATE] - ROUTER cost 3 in the update: %d\n", ntohs(updateMessage->routingEntries[2].metricCost));
//                            printf("[PERIODIC UPDATE] - Padding 3 in the update: %d\n\n", ntohs(updateMessage->routingEntries[2].padding));
//                            
//                            char * str5 = inet_ntoa(*(struct in_addr *)&updateMessage->routingEntries[3].destinationRouterIP);
//                            printf("[PERIODIC UPDATE] - ROUTER ip 4 in the update: %s\n", str5);
//                            printf("[PERIODIC UPDATE] - ROUTER port 4 in the update: %d\n", ntohs(updateMessage->routingEntries[3].destinationRouterPort));
//                            printf("[PERIODIC UPDATE] - ROUTER ID 4 in the update: %d\n", ntohs(updateMessage->routingEntries[3].destinationRouterID));
//                            printf("[PERIODIC UPDATE] - ROUTER cost 4 in the update: %d\n", ntohs(updateMessage->routingEntries[3].metricCost));
//                            printf("[PERIODIC UPDATE] - Padding 4 in the update: %d\n\n", ntohs(updateMessage->routingEntries[3].padding));
//                            printf("*******************************************************************\n");
                            if(receivedBytes==size)
                            {
                                int advertiserID = 0;
                                int advertiserLocatedAtIndex=0;
                                int numberOfFields = ntohs(updateMessage->numberOfUpdateFields);
                                for(int i=1; i<=ntohs(updateMessage->numberOfUpdateFields); i++)
                                {
                                    //identify the advertiser based on cost he advertises to himself (if 0 he is the remote ID)
                                    int advertiserCost = ntohs(updateMessage->routingEntries[i-1].metricCost);
                                    if(advertiserCost==0)
                                    {
                                        advertiserID=ntohs(updateMessage->routingEntries[i-1].destinationRouterID);
                                        printf("remote server from where update came is: %u\n",advertiserID);
                                    }
                                    //now find the index of the advertiser in LBTT
                                    for(int i=1;i<=ntohs(nodeCount);i++)
                                    {
                                        if(ntohs(localBaseTopologyTable[i].destinationRouterID)==advertiserID)
                                        {
                                            advertiserLocatedAtIndex=i;
                                            printf("advertiser is located at index: [%d]\n", advertiserLocatedAtIndex);
                                        }
                                    }
                                }
                                if (localBaseTopologyTable[advertiserLocatedAtIndex].active==true)
                                {
                                    
                                    if (localBaseTopologyTable[advertiserLocatedAtIndex].doesExist==false || localBaseTopologyTable[advertiserLocatedAtIndex].uptime>(ntohs(updateInterval)*3))
                                    {
                                        localBaseTopologyTable[advertiserLocatedAtIndex].doesExist=true;
                                        dv[mySerialNumber][advertiserLocatedAtIndex]=neReachability[advertiserLocatedAtIndex];
                                    }
                                    //REMOVE THIS PRINT STATEMENT ALONG WITH FOR BLOCK - agautam2
                                    for(int i=1; i<=numberOfFields; i++)
                                    {
                                        printf("%-15d%-15d\n", ntohs(updateMessage->routingEntries[i-1].destinationRouterID), ntohs(updateMessage->routingEntries[i-1].metricCost));
                                    }
                                    for(int i=1; i<=numberOfFields; i++)
                                    {
                                        dv[advertiserLocatedAtIndex][(localBaseTopologyTable[i].routerSerialNumber)]=ntohs(updateMessage->routingEntries[i-1].metricCost);
                                    }
                                    localBaseTopologyTable[advertiserLocatedAtIndex].uptime=0;
                                    printf("timer set to zero again\n");
                                    //calling bellman-ford #INSTANCE 2
                                    distanveVectorRoutingAlgorithm(dv, nodeCount, whoAmiID, localBaseTopologyTable);
                                }
                            }
                            memset((struct updatePacket*)updateMessage,'\0',size);
                            //code to get peer info:
                            getpeername(iSocket, (struct sockaddr *) &clientAddress, &clientLength);
                            
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
                            printf("Accepting %zd Bytes of Routing Update from %s: %d on socket %d\n\n",receivedBytes, storeAddress, clientPort, iSocket);

                        }
                        else if(iSocket==sockfdData)
                        {
                            //handle data port here
                            clientLength = sizeof clientAddress;
                            fileRecvSocket = accept(sockfdData,(struct sockaddr *)&clientAddress,&clientLength);     //ACCEPT
                            if (fileRecvSocket == -1)
                            {
                                perror("accept");
                            }
                            else
                            {
                                printf("adding to master set\n\n");
                                FD_SET(fileRecvSocket, &masterDescriptor); // add to master set
                                if (fileRecvSocket > fdMaxNumber)
                                {    // keep track of the max
                                    fdMaxNumber = fileRecvSocket;
                                    printf("fdMaxNumber after adding to master set is: %d\n", fdMaxNumber);
                                    
                                }
                                clientLength = sizeof (clientAddress);
                                getpeername(sockfdData, (struct sockaddr *) &clientAddress, &clientLength);
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
                                printf("Accepting new connection from %s: %d on socket %d\n\n", storeAddress, clientPort, fileRecvSocket);
                                printf("Socket %d is bound to %s\n", fileRecvSocket, storeAddress);
                                clientLength = sizeof(clientAddress);
                                getnameinfo((struct sockaddr *) &clientAddress, clientLength, host, sizeof(host), service, sizeof(service), 0);
                            
                            }
                        }
                        else
                        {
                            /*handle data on TCP socket here using recv
                             1. do actions based on control codes
                             2. handle file transfer if control code is Ox05 and 0x06 */
                            controlBuffer =  new unsigned char [1024];
                            
                            printf("####clearing controlBuffer####\n");
                            //controlBuffer = (unsigned char *)(malloc(1024));
                            //bzero(controlBuffer, sizeof(controlBuffer));
                            memset(&controlBuffer[0], 0, 1024);
                            printf("size of receive buffer is: %lu\n", sizeof(controlBuffer));
                            
                            //READ DATA FROM CONTROLLER
                            if((readBytes = recv(iSocket,controlBuffer,1024,0)) <=0)
                            {
                                if(readBytes==0)
                                {
                                    printf("Remote Server Hung Up\n");
                                    //break;
                                }
                                else
                                {
                                    perror("ERROR IN RECEIVING");
                                    //break;
                                }
                                close(iSocket);
                                FD_CLR(iSocket,&masterDescriptor);
                            }
                            
                            
                            if(readBytes>1)
                            {
                                //OK WE HAVE SOME DATA NOW FROM THE CONTROLLER
                                
                                printf("received %d bytes of data from the CONTROLLER\n", readBytes);
                                printf("trying to unpack\n");
                                
                                struct controlPacketHeader *cph =  (struct controlPacketHeader *) malloc(sizeof(struct controlPacketHeader));
                                
                                //call to unpack using args as: 32(L), 8(C), 8(C) and 16 (H) followed by payload
                                //unpack(controlBuffer, "LCCH", &cph->destinationIP, &cph->controlCode, &cph->responseTime, &cph->payloadLength);
                                
                                memcpy(&cph->destinationIP, controlBuffer, 4);
                                controlBuffer+=4;
                                whoAmiIP=cph->destinationIP;
                                memcpy(&cph->controlCode, controlBuffer, 1);
                                controlBuffer+=1;
                                memcpy(&cph->responseTime, controlBuffer, 1);
                                controlBuffer+=1;
                                memcpy(&cph->payloadLength, controlBuffer, 2);
                                controlBuffer-=6;
                                
                                /*FOR BASIC TESTING-WILL IT BE SAME FOR ALL? - YES HEADER IS SAME. PAYLOAD VARIES ACC TO CONTROL CODE*/
                                printf("--------HEADER CONTAINS--------\n");
                                char * str = inet_ntoa(*(struct in_addr *)&cph->destinationIP);
                                printf("dest IP: %s\n", str);
                                printf("control code: %u\n", (cph->controlCode));   //don't use ntohs for 8bit
                                printf("response time: %u\n", (cph->responseTime));
                                printf("payload length: %u\n", ntohs(cph->payloadLength));
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
                                    memset(&controlResponseBuffer[0], 0, 1024);
                                    //controlResponsePayloadBuffer = new unsigned char [1024];
                                    struct controlResponseHeader *crh = (struct controlResponseHeader *) malloc(sizeof(struct controlResponseHeader));
                                    //struct controlResponsePayload *crp = (struct controlResponsePayload *) malloc(sizeof(struct controlResponsePayload));
                                    
                                    crh->controllerIP=static_cast<uint32_t>(peername->sin_addr.s_addr);
                                    crh->controlCode=0;
                                    crh->responseCode=0;
                                    char *dataString;
                                    dataString=static_cast<char *>(malloc(256));
                                    strcpy(dataString, "I, agautam2, have read and understood the course academic integrity policy.\0");
                                    crh->payloadLength=sizeof("I, agautam2, have read and understood the course academic integrity policy.")-1;
                                    printf("done writing\n");
                                    //fflush(stdout);
                                    
                                    //call to pack using args as: 32(L), 8(C), 8(C) and 16 (H) followed by payload
                                    pack(controlResponseBuffer, "LCCH", (uint32_t)crh->controllerIP, (uint8_t)crh->controlCode, (uint8_t)crh->responseCode,(uint16_t)crh->payloadLength);
                                    printf("packed now adding payload\n");
                                    controlResponseBuffer += 8;
                                    //printf("shifting buffer\n");
                                    memcpy(controlResponseBuffer, dataString, 75);
                                    //printf("added payload\n");
                                    controlResponseBuffer=controlResponseBuffer-8;
                                    //printf("shifting buffer back\n");
                                    
//                                    /*To test from sending side:*/
//                                    FILE *fp;
//                                    fp = fopen("file1.txt", "w");
//                                    fwrite(controlResponseBuffer, 83, 1, fp); //(payload (75)+ header(8)
//                                    fclose(fp);
                                    
                                    //SEND
                                    printf("trying to send\n");
                                    int ableToSend1 = send(newsockfd, controlResponseBuffer,83, 0);
                                    printf("sent\n");
                                    
                                    if(ableToSend1<0)
                                    {
                                        printf("failed to send serialized header\n");
                                    }
                                }
                                else if(cph->controlCode==1)
                                {
                                    //INIT-BUILD RT-NO RESPONSE REQUIRED EXCEPT HEADER
                                    neighborCount=0;
                                    stopwatch=0;
                                    char hex[5];
                                    sprintf(hex, "%x", cph->controlCode);
                                    printf("control code %s found. Routing Table will be populated\n",hex);
                                    struct controlPacketPayload *cpp = (struct controlPacketPayload *) malloc(sizeof(struct controlPacketPayload));
                                    
                                    printf("--------PAYLOAD CONTAINS--------\n");
                                    controlBuffer+=8;
                                    memcpy(&cpp->nodes, controlBuffer, 2);
                                    controlBuffer+=2;
                                    printf("No. of nodes: %u\n",ntohs(cpp->nodes));
                                    nodeCount = cpp->nodes;
                                    printf("setting nodeCount globally to: %u\n", ntohs(nodeCount));
                                    memcpy(&cpp->updateInterval, controlBuffer, 2);
                                    controlBuffer-=10;
                                    updateInterval=cpp->updateInterval;
                                    printf("setting updateInterval globally to: %u\n", ntohs(updateInterval));
                                    printf("Update Interval: %u\n",ntohs(cpp->updateInterval));
                                   
                                    
                                    //set ne flag to false initially
                                    for(int i=1;i <=ntohs(cpp->nodes); i++)
                                    {
                                        localBaseTopologyTable[i].ne=false;
                                        printf("setting [%d] to [%d]\n\n",i,localBaseTopologyTable[i].ne);
                                    }
                                    //make cases according to number of routers
                                    if(ntohs(cpp->nodes)==1)
                                    {
                                        controlBuffer+=12;
                                        memcpy(&cpp->routerID[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[0], controlBuffer, 4);
                                        controlBuffer-=20;
                                        //find whoAmiID
                                        for(int i=0; i<ntohs(cpp->nodes);i++)
                                        {
                                            if(ntohs(cpp->metric[i])==0)
                                            {
                                                whoAmiID=cpp->routerID[i];
                                                whoAmiPort=cpp->routerPort[i];
                                                establishRoutingUpdates(cpp->routerPort[i]);
                                                //establisDataTransfer(cpp->dataPort[i]);
                                                printf("Sending router port [%u] info to router\n",ntohs(cpp->routerPort[i]));
                                                //printf("Sending data port [%u] info to router\n",ntohs(cpp->dataPort[i]));
                                            }
                                        }
                                        //establish router port for UDP updates
                                        
                                    }
                                    if(ntohs(cpp->nodes)==2)
                                    {
                                        controlBuffer+=12;
                                        memcpy(&cpp->routerID[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[0], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[1], controlBuffer, 4);
                                        controlBuffer-=32;
                                        //find ne count
                                        if(ntohs(cpp->metric[0])!= 65535 && (ntohs(cpp->metric[0])!=0))
                                        {
                                            //printf("##ne count inc CASE 1\n");
                                            neighborCount++;
                                            localBaseTopologyTable[1].ne=true;
                                        }
                                        if(ntohs(cpp->metric[1])!= 65535 && (ntohs(cpp->metric[1])!=0))
                                        {
                                            //printf("##ne count inc CASE 2\n");
                                            neighborCount++;
                                            localBaseTopologyTable[2].ne=true;
                                        }
                                        //find whoAmiID
                                        for(int i=0; i<ntohs(cpp->nodes);i++)
                                        {
                                            if(ntohs(cpp->metric[i])==0)
                                            {
                                                whoAmiID=cpp->routerID[i];
                                                whoAmiPort=cpp->routerPort[i];
                                                establishRoutingUpdates(cpp->routerPort[i]);
                                                //establisDataTransfer(cpp->dataPort[i]);
                                                printf("Sending router port [%u] info to router\n",ntohs(cpp->routerPort[i]));
                                                //printf("Sending data port [%u] info to router\n",ntohs(cpp->dataPort[i]));
                                            }
                                        }
                                        
                                    }
                                    if(ntohs(cpp->nodes)==3)
                                    {
                                        controlBuffer+=12;
                                        memcpy(&cpp->routerID[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[0], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[1], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[2], controlBuffer, 4);
                                        controlBuffer-=44;
                                        
                                        //find ne count
                                        if(ntohs(cpp->metric[0])!= 65535 && (ntohs(cpp->metric[0])!=0))
                                        {
                                            //printf("##ne count inc CASE 1\n");
                                            neighborCount++;
                                            localBaseTopologyTable[1].ne=true;
                                        }
                                        if(ntohs(cpp->metric[1])!= 65535 && (ntohs(cpp->metric[1])!=0))
                                        {
                                            //printf("##ne count inc CASE 2\n");
                                            neighborCount++;
                                            localBaseTopologyTable[2].ne=true;
                                        }
                                        if(ntohs(cpp->metric[2])!= 65535 && (ntohs(cpp->metric[2])!=0))
                                        {
                                            //printf("##ne count inc CASE 3\n");
                                            neighborCount++;
                                            localBaseTopologyTable[3].ne=true;
                                        }
                                        
                                        //find whoAmiID
                                        for(int i=0; i<ntohs(cpp->nodes);i++)
                                        {
                                            if(ntohs(cpp->metric[i])==0)
                                            {
                                                whoAmiID=cpp->routerID[i];
                                                whoAmiPort=cpp->routerPort[i];
                                                establishRoutingUpdates(cpp->routerPort[i]);
                                                //establisDataTransfer(cpp->dataPort[i]);
                                                printf("Sending router port [%u] info to router\n",ntohs(cpp->routerPort[i]));
                                                //printf("Sending data port [%u] info to router\n",ntohs(cpp->dataPort[i]));
                                            }
                                        }
                                        
                                    }
                                    if(ntohs(cpp->nodes)==4)
                                    {
                                        controlBuffer+=12;
                                        memcpy(&cpp->routerID[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[0], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[1], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[2], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[3], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[3], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[3], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[3], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[3], controlBuffer, 4);
                                        controlBuffer-=56;
                                        
                                        //find ne count
                                        if(ntohs(cpp->metric[0])!= 65535 && (ntohs(cpp->metric[0])!=0))
                                        {
                                            //printf("##ne count inc CASE 1\n");
                                            neighborCount++;
                                            localBaseTopologyTable[1].ne=true;
                                            //printf("##ne setting NE TRUE##\n");
                                        }
                                        if(ntohs(cpp->metric[1])!= 65535 && (ntohs(cpp->metric[1])!=0))
                                        {
                                            //printf("##ne count inc CASE 2\n");
                                            neighborCount++;
                                            localBaseTopologyTable[2].ne=true;
                                            //printf("##ne setting NE TRUE##\n");
                                        }
                                        if(ntohs(cpp->metric[2])!= 65535 && (ntohs(cpp->metric[2])!=0))
                                        {
                                            //printf("##ne count inc CASE 3\n");
                                            neighborCount++;
                                            localBaseTopologyTable[3].ne=true;
                                            //printf("##ne setting NE TRUE##\n");
                                        }
                                        if(ntohs(cpp->metric[3])!= 65535 && (ntohs(cpp->metric[3])!=0))
                                        {
                                            //printf("##ne count inc CASE 4\n");
                                            neighborCount++;
                                            localBaseTopologyTable[4].ne=true;
                                            //printf("##ne setting NE TRUE##\n");
                                        }
                                        
                                        //find whoAmiID
                                        for(int i=0; i<ntohs(cpp->nodes);i++)
                                        {
                                            if(ntohs(cpp->metric[i])==0)
                                            {
                                                whoAmiID=cpp->routerID[i];
                                                whoAmiPort=cpp->routerPort[i];
                                                establishRoutingUpdates(cpp->routerPort[i]);
                                                //establisDataTransfer(cpp->dataPort[i]);
                                                printf("Sending router port [%u] info to router\n",ntohs(cpp->routerPort[i]));
                                                //printf("Sending data port [%u] info to router\n",ntohs(cpp->dataPort[i]));
                                            }
                                        }
                                        
                                    }
                                    if(ntohs(cpp->nodes)==5)
                                    {
                                        controlBuffer+=12;
                                        memcpy(&cpp->routerID[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[0], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[0], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[1], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[1], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[2], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[2], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[3], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[3], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[3], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[3], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[3], controlBuffer, 4);
                                        controlBuffer+=4;
                                        
                                        memcpy(&cpp->routerID[4], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerPort[4], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->dataPort[4], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->metric[4], controlBuffer, 2);
                                        controlBuffer+=2;
                                        memcpy(&cpp->routerIP[4], controlBuffer, 4);
                                        controlBuffer-=68;
                                        
                                        //find ne count
                                        if(ntohs(cpp->metric[0])!= 65535 && (ntohs(cpp->metric[0])!=0))
                                        {
                                            neighborCount++;
                                            localBaseTopologyTable[1].ne=true;
                                        }
                                        if(ntohs(cpp->metric[1])!= 65535 && (ntohs(cpp->metric[1])!=0))
                                        {
                                            neighborCount++;
                                            localBaseTopologyTable[2].ne=true;
                                        }
                                        if(ntohs(cpp->metric[2])!= 65535 && (ntohs(cpp->metric[2])!=0))
                                        {
                                            neighborCount++;
                                            localBaseTopologyTable[3].ne=true;
                                        }
                                        if(ntohs(cpp->metric[3])!= 65535 && (ntohs(cpp->metric[3])!=0))
                                        {
                                            neighborCount++;
                                            localBaseTopologyTable[4].ne=true;
                                        }
                                        if(ntohs(cpp->metric[4])!= 65535 && (ntohs(cpp->metric[4])!=0))
                                        {
                                            neighborCount++;
                                            localBaseTopologyTable[5].ne=true;
                                        }
                                        
                                        //find whoAmiID
                                        for(int i=0; i<ntohs(cpp->nodes);i++)
                                        {
                                            if(ntohs(cpp->metric[i])==0)
                                            {
                                                whoAmiID=cpp->routerID[i];
                                                whoAmiPort=cpp->routerPort[i];
                                                establishRoutingUpdates(cpp->routerPort[i]);
                                                //establisDataTransfer(cpp->dataPort[i]);
                                                printf("Sending router port [%u] info to router\n",ntohs(cpp->routerPort[i]));
                                                //printf("Sending data port [%u] info to router\n",ntohs(cpp->dataPort[i]));
                                            }
                                        }
                                    }
                                    //done for 5 routers
                                    
                                    
                                    for(int i=0;i<(ntohs(cpp->nodes));i++)
                                    {
                                        printf("Router ID [%d] : %u\n",i+1, ntohs(cpp->routerID[i]));
                                        printf("Router Port [%d] : %u\n",i+1,ntohs(cpp->routerPort[i]));
                                        printf("Data Port [%d] : %u\n",i+1, ntohs(cpp->dataPort[i]));
                                        printf("Metric [%d] : %u\n",i+1, ntohs(cpp->metric[i]));
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
                                    
                                    controlResponseBuffer = new unsigned char [1024];
                                    struct controlResponseHeader *crh = (struct controlResponseHeader *) malloc(sizeof(struct controlResponseHeader));
                                    printf("trying to pack\n");
                                    crh->controllerIP=static_cast<uint32_t>(peername->sin_addr.s_addr);
                                    //printf("done 1\n");
                                    crh->controlCode=1;
                                    //printf("done 2\n");
                                    crh->responseCode=0;
                                    //printf("done 3\n");
                                    crh->payloadLength=0;
                                    //printf("done 4\n");
                                    //send control response header
                                    pack(controlResponseBuffer, "LCCH", (uint32_t)crh->controllerIP, (uint8_t)crh->controlCode, (uint8_t)crh->responseCode, (uint16_t)crh->payloadLength);
                                    printf("pack successful\n");
                                    int ableToSend1 = send(newsockfd, controlResponseBuffer,8, 0);
                                    
                                    if(ableToSend1<0)
                                    {
                                        printf("failed to send control response header\n");
                                    }
                                    
                                    /*PREPARE LOCAL RT, known as LBTT from here onwards*/
                                    
                                    /*1. find who am i? - ID to which the controller msg is directed - done in loops*/
                                    printf("whoAmiID is - [%u]\n", ntohs(whoAmiID));
                                    char * str = inet_ntoa(*(struct in_addr *)&whoAmiIP);
                                    printf("whoAmiIP is - [%s]\n", str);
                                    printf("whoAmiPort is - [%u]\n", ntohs(whoAmiPort));
                                    
                                    /*
                                     2. find total number of nodes, run loop and POPULATE whatever was read from control message:
                                     a. router IDs
                                     b. next hop router ID
                                     c. cost
                                     d. destination router IP
                                     e. source router IP
                                     */
                                    
                                    for(int i=1; i<=ntohs(cpp->nodes); i++)
                                    {
                                        //ne flag set to false in the starting
                                        //LBTT contains data from 1 onwards and in network order
                                        //cpp-> contains data from 0 onwards
                                        
                                        /*DOUBT - cost for every router(including ne) should be INF? or
                                         for ne the cost should be what the controller sends in init.
                                         Say for a test case in which update interval is 30 and RT is
                                         requested*/
                                        
                                        
                                        localBaseTopologyTable[i].destinationRouterID=cpp->routerID[i-1];
                                        localBaseTopologyTable[i].nextHopID=-1; //for now let it be -1
                                        localBaseTopologyTable[i].metricCost=65535;   // check this - agautam2
                                        localBaseTopologyTable[i].destinationIP=cpp->routerIP[i-1];
                                        localBaseTopologyTable[i].routerPort=cpp->routerPort[i-1];
                                        localBaseTopologyTable[i].dataPort=cpp->dataPort[i-1];
                                        localBaseTopologyTable[i].sourceRouterIP=whoAmiIP;
                                        localBaseTopologyTable[i].active=true;
                                        localBaseTopologyTable[i].uptime=65535;
                                        localBaseTopologyTable[i].doesExist=false;
                                        localBaseTopologyTable[i].padding=0;
                                        localBaseTopologyTable[i].routerSerialNumber=i;
                                    }
                                    
                                    /*3. find number of neighbors and who are ne for which and populate details for them*/
                                    printf("Neighbor count for Router ID[%u] is: %d\n", ntohs(whoAmiID),neighborCount);
                                    
                                    for(int i=1;i<=ntohs(cpp->nodes); i++)
                                    {
                                        printf("[%d] is NE (YES = 1 NO = 0) : %d\n",i, localBaseTopologyTable[i].ne);
                                    }
                                    
                                    /*4. build my topology i.e. I KNOW ABOUT MY NEIGHBORS*/
                                    //WORKING FINE
                                    for(int i=1;i<=ntohs(cpp->nodes);i++)
                                    {
                                        if(localBaseTopologyTable[i].ne==true)
                                        {
                                            neReachability[(localBaseTopologyTable[i].routerSerialNumber)]=ntohs(cpp->metric[i-1]);
                                            
                                            printf("NE [%d] is reachable through me [%u] by a cost of %u\n", ntohs(localBaseTopologyTable[localBaseTopologyTable[i].routerSerialNumber].destinationRouterID), ntohs(whoAmiID), neReachability[(localBaseTopologyTable[i].routerSerialNumber)]);
                                        }
                                    }
                                    
                                    
                                    /*5. update own cost, own next hop id and own uptime*/
                                    for(int i=1;i<=ntohs(nodeCount);i++)
                                    {
                                        if(ntohs(localBaseTopologyTable[i].destinationRouterID)==ntohs(whoAmiID))
                                        {
                                            printf("****** SETTING OWN PARAMETERS FOR [%d]\n", ntohs(localBaseTopologyTable[i].destinationRouterID));
                                        localBaseTopologyTable[i].metricCost=0;
                                        localBaseTopologyTable[i].nextHopID=whoAmiID;
                                            printf("my next hop ID is [%d] me myself\n",ntohs(localBaseTopologyTable[i].nextHopID));
                                        localBaseTopologyTable[i].uptime=0;
                                        }
                                    }
                                    
                                    /*PRINT localBaseTopologyTable for TESTING*/
                                    
                                    printf("-------------------------------------------------------------------\n");
                                    printf("PRINTING LBTT A/I-ACTIVE[1] INACTIVE[0] <INITIAL MATRIX TABLE>\n");
                                    printf("-------------------------------------------------------------------\n");
                                    printf("\n");
                                    
                                    //REMOVE THIS PRINT STATEMENT - agautam2
                                    printf("%-5s%-20s%-15s%-15s%-15s%-15s%-15s%-15s%-15s\n", "S.No","Dest IP", "Dest RID", "Router Port", "Next Hop ID","Metric", "A/I", "TSU", "NE[Y=1|N=0]");
                                    printf("\n");
                                    for(int i=1;i<=ntohs(cpp->nodes);i++)
                                    {
                                        
                                        char * dest= inet_ntoa(*(struct in_addr *)&localBaseTopologyTable[i].destinationIP);
                                        printf("%-5d%-20s%-15u%-15u%-15u%-15u%-15d%-15u%-15d\n", localBaseTopologyTable[i].routerSerialNumber, dest, ntohs(localBaseTopologyTable[i].destinationRouterID), ntohs(localBaseTopologyTable[i].routerPort), ntohs(localBaseTopologyTable[i].nextHopID),ntohs(localBaseTopologyTable[i].metricCost), localBaseTopologyTable[i].active, localBaseTopologyTable[i].uptime,localBaseTopologyTable[i].ne);

                                        
                                    }
                                    /*6. initialiase DV */
                                    for(int i=1; i<=ntohs(cpp->nodes); i++)
                                    {
                                        for(int j=1; j<=ntohs(cpp->nodes); j++)
                                        {
                                            if(i!=j)
                                            {
                                                dv[(localBaseTopologyTable[i].routerSerialNumber)][(localBaseTopologyTable[j].routerSerialNumber)]=INF;
                                            }
                                            else if(i==j)
                                            {
                                                dv[(localBaseTopologyTable[i].routerSerialNumber)][(localBaseTopologyTable[j].routerSerialNumber)]=0;
                                            }
                                        }
                                    }
                                    
                                } //cc1 ends
                                else if(cph->controlCode==2)
                                {
                                    //ROUTING TABLE-RT RESPONSE REQUIRED
                                    printf("control code 0x02 found. Routing Table requested. Will be sent\n");
                                    /*print ROUTING TABLE HERE FOR TESTING PUPOSES*/
                                    
                                    printf("---------PRINTING CURRENT ROUTING TABLE----------------------\n");
                                    printf("%-15s%-15s%-15s%-15s\n", "RID","NEXT HOP ID","METRIC", "DATA PORT");
                                    for (int i = 1; i <= ntohs(nodeCount); ++i)
                                    {
                                        printf("%-15d%-15d%-15d%-15d\n", ntohs(localBaseTopologyTable[i].destinationRouterID),ntohs(localBaseTopologyTable[i].nextHopID),(localBaseTopologyTable[i].metricCost), ntohs(localBaseTopologyTable[i].dataPort));
                                    }
                                    struct controlResponseHeader *crh = (struct controlResponseHeader *) malloc(sizeof(struct controlResponseHeader));
                                    controlResponseBuffer = new unsigned char [1024];
                                    memset(&controlResponseBuffer[0], 0, 1024);
                                    crh->controllerIP=static_cast<uint32_t>(peername->sin_addr.s_addr);
                                    crh->controlCode=3;
                                    crh->responseCode=0;
                                    printf("setting payload length to %u\n", 8*ntohs(nodeCount));
                                    crh->payloadLength=8*ntohs(nodeCount);
                                    printf("trying to pack header for CC=2\n");
                                    pack(controlResponseBuffer, "LCCH", (uint32_t)crh->controllerIP, (uint8_t)crh->controlCode, (uint8_t)crh->responseCode,(uint16_t)crh->payloadLength);
                                    printf("packed Header\n");
                                    printf("shifting buffer\n");
                                    
                                    controlResponseBuffer+=8;
                                    
                                    if(ntohs(nodeCount)==1)
                                    {
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost1=ntohs(localBaseTopologyTable[1].metricCost);
                                        memcpy(controlResponseBuffer, &cost1, 2);
                                        
                                        //shift back
                                        controlResponseBuffer-=14;
                                        printf("trying to send\n");
                                        int ableToSend1 = send(newsockfd, controlResponseBuffer,(8+(8*ntohs(nodeCount))), 0);
                                        printf("sent\n");
                                        
                                        if(ableToSend1<0)
                                        {
                                            printf("failed to send response for CC=2\n");
                                        }
                                        
                                    }
                                    
                                    if(ntohs(nodeCount)==2)
                                    {
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost1=ntohs(localBaseTopologyTable[1].metricCost);
                                        memcpy(controlResponseBuffer, &cost1, 2);
                                        controlResponseBuffer+=2;
                                        
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[2].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[2].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[2].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost2=ntohs(localBaseTopologyTable[1].metricCost);
                                        memcpy(controlResponseBuffer, &cost2, 2);
                                        
                                        //shift back
                                        controlResponseBuffer-=22;
                                        printf("trying to send\n");
                                        int ableToSend1 = send(newsockfd, controlResponseBuffer,(8+(8*ntohs(nodeCount))), 0);
                                        printf("sent\n");
                                        
                                        if(ableToSend1<0)
                                        {
                                            printf("failed to send response for CC=2\n");
                                        }
                                        
                                    }
                                    
                                    if(ntohs(nodeCount)==3)
                                    {
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost1=ntohs(localBaseTopologyTable[1].metricCost);
                                        memcpy(controlResponseBuffer, &cost1, 2);
                                        controlResponseBuffer+=2;
                                        
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[2].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[2].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[2].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost2=ntohs(localBaseTopologyTable[1].metricCost);
                                        memcpy(controlResponseBuffer, &cost2, 2);
                                        controlResponseBuffer+=2;
                                        
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[3].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[3].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[3].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost3=ntohs(localBaseTopologyTable[1].metricCost);
                                        memcpy(controlResponseBuffer, &cost3, 2);
                                        
                                        //shift back
                                        controlResponseBuffer-=30;
                                        printf("trying to send\n");
                                        int ableToSend1 = send(newsockfd, controlResponseBuffer,(8+(8*ntohs(nodeCount))), 0);
                                        printf("sent\n");
                                        
                                        if(ableToSend1<0)
                                        {
                                            printf("failed to send response for CC=2\n");
                                        }
                                        
                                    }
                                    
                                    if(ntohs(nodeCount)==4)
                                    {
                                        //printf("trying to pack payload for %d nodes \n", ntohs(nodeCount));
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[1].destinationRouterID, 2);
                                    controlResponseBuffer+=2;
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[1].padding, 2);
                                    controlResponseBuffer+=2;
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[1].nextHopID, 2);
                                    controlResponseBuffer+=2;
                                        //printf("copying into response buffer, values: NHID[1] - [%u]\n", ntohs(localBaseTopologyTable[1].nextHopID));
                                        uint16_t cost1=ntohs(localBaseTopologyTable[1].metricCost);
                                    memcpy(controlResponseBuffer, &cost1, 2);
                                    controlResponseBuffer+=2;
                                        //printf("copying into response buffer, values: cost[1] - [%u]\n", cost1);
                                    
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[2].destinationRouterID, 2);
                                    controlResponseBuffer+=2;
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[2].padding, 2);
                                    controlResponseBuffer+=2;
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[2].nextHopID, 2);
                                    controlResponseBuffer+=2;
                                        //printf("copying into response buffer, values: NHID[2] - [%u]\n", ntohs(localBaseTopologyTable[2].nextHopID));
                                        uint16_t cost2=ntohs(localBaseTopologyTable[2].metricCost);
                                    memcpy(controlResponseBuffer, &cost2, 2);
                                    controlResponseBuffer+=2;
                                        //printf("copying into response buffer, values: cost[2] - [%u]\n", cost2);
                                    
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[3].destinationRouterID, 2);
                                    controlResponseBuffer+=2;
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[3].padding, 2);
                                    controlResponseBuffer+=2;
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[3].nextHopID, 2);
                                    controlResponseBuffer+=2;
                                        //printf("copying into response buffer, values: NHID[3] - [%u]\n", ntohs(localBaseTopologyTable[3].nextHopID));
                                        uint16_t cost3=ntohs(localBaseTopologyTable[3].metricCost);
                                    memcpy(controlResponseBuffer, &cost3, 2);
                                    controlResponseBuffer+=2;
                                        //printf("copying into response buffer, values: cost[3] - [%u]\n", cost3);
                                        
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[4].destinationRouterID, 2);
                                    controlResponseBuffer+=2;
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[4].padding, 2);
                                    controlResponseBuffer+=2;
                                    memcpy(controlResponseBuffer, &localBaseTopologyTable[4].nextHopID, 2);
                                    controlResponseBuffer+=2;
                                        //printf("copying into response buffer, values: NHID[4] - [%u]\n", ntohs(localBaseTopologyTable[4].nextHopID));
                                        uint16_t cost4=ntohs(localBaseTopologyTable[4].metricCost);
                                    memcpy(controlResponseBuffer, &cost4, 2);
                                        //printf("copying into response buffer, values: cost[4] - [%u]\n", cost4);
                                        
                                    
                                    //shift back
                                    controlResponseBuffer-=38;
                                    
                                        printf("trying to send\n");
                                        int ableToSend1 = send(newsockfd, controlResponseBuffer,(8+(8*ntohs(nodeCount))), 0);
                                        printf("sent\n");
                                        
                                        if(ableToSend1<0)
                                        {
                                            printf("failed to send response for CC=2\n");
                                        }
                                    }
                                    
                                    if(ntohs(nodeCount)==5)
                                    {
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[1].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost1=ntohs(localBaseTopologyTable[1].metricCost);
                                        memcpy(controlResponseBuffer, &cost1, 2);
                                        controlResponseBuffer+=2;
                                        
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[2].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[2].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[2].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost2=ntohs(localBaseTopologyTable[2].metricCost);
                                        memcpy(controlResponseBuffer, &cost2, 2);
                                        controlResponseBuffer+=2;
                                        
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[3].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[3].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[3].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost3=ntohs(localBaseTopologyTable[3].metricCost);
                                        memcpy(controlResponseBuffer, &cost3, 2);
                                        controlResponseBuffer+=2;
                                        
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[4].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[4].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[4].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost4=ntohs(localBaseTopologyTable[4].metricCost);
                                        memcpy(controlResponseBuffer, &cost4, 2);
                                        controlResponseBuffer+=2;
                                        
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[5].destinationRouterID, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[5].padding, 2);
                                        controlResponseBuffer+=2;
                                        memcpy(controlResponseBuffer, &localBaseTopologyTable[5].nextHopID, 2);
                                        controlResponseBuffer+=2;
                                        uint16_t cost5=ntohs(localBaseTopologyTable[5].metricCost);
                                        memcpy(controlResponseBuffer, &cost5, 2);
                                        
                                        //shift back
                                        controlResponseBuffer-=46;
                                        printf("trying to send\n");
                                        int ableToSend1 = send(newsockfd, controlResponseBuffer,(8+(8*ntohs(nodeCount))), 0);
                                        printf("sent\n");
                                        
                                        if(ableToSend1<0)
                                        {
                                            printf("failed to send response for CC=2\n");
                                        }
                                        
                                    }
                                    
                                    printf("cc2 ends\n");
                        
                                    
                                    }//cc2 ends
                                else if(cph->controlCode==3)
                                {
                                    //UPDATE-UPDATE RT-NO RESPONSE REQUIRED EXCEPT HEADER
                                    
                                    /*The controller uses this to change/update the link cost between between the router receiving this message and a NEIGHBORING router. This control message will always be sent in pairs to both the routers involved in a link.*/
                                    
                                    char hex[5];
                                    int index=0;
                                    sprintf(hex, "%x", cph->controlCode);
                                    printf("control code %s found. Routing Table will be populated. Waiting for payload...\n",hex);
                                    struct controlPacketPayload *cpp = (struct controlPacketPayload *) malloc(sizeof(struct controlPacketPayload));
                                    //unpack(controlBuffer, "LCCHHH", &cph->destinationIP, &cph->controlCode, &cph->responseTime, &cph->payloadLength , &cpp->routerID[0], &cpp->metric[0]);
                                    
                                    controlBuffer+=8;
                                    memcpy(&cpp->changeCostForRID, controlBuffer, 2);
                                    controlBuffer+=2;
                                    memcpy(&cpp->changedCost, controlBuffer, 2);
                                    controlBuffer-=6;
                                    
                                    printf("--------PAYLOAD CONTAINS--------\n");
                                    printf("Router ID for which cost is to be updated: %u\n",ntohs(cpp->changeCostForRID));
                                    printf("Cost to be updated to %u\n",ntohs(cpp->changedCost));
                                    
                                    
                                    controlResponseBuffer = new unsigned char [1024];
                                    struct controlResponseHeader *crh = (struct controlResponseHeader *) malloc(sizeof(struct controlResponseHeader));
                                    printf("constructing control response header for cc=3\n");
                                    crh->controllerIP=static_cast<uint32_t>(peername->sin_addr.s_addr);
                                    printf("done #1\n");
                                    crh->controlCode=3;
                                    printf("done #2\n");
                                    crh->responseCode=0;
                                    printf("done #3\n");
                                    crh->payloadLength=0;
                                    printf("done #4\n");
                                    printf("Trying to pack\n");
                                    pack(controlResponseBuffer, "LCCH", (uint32_t)crh->controllerIP, (uint8_t)crh->controlCode, (uint8_t)crh->responseCode, (uint16_t)crh->payloadLength);
                                    printf("pack successful\n");
                                    int ableToSend1 = send(newsockfd, controlResponseBuffer,8, 0);
                                    printf("control response with cc 3 sent\n");
                                    if(ableToSend1<0)
                                    {
                                        printf("failed to send control response header\n");
                                    }
                                    
                                    
                                    /*DO THE FOLLOWING:
                                     1. Read router ID1 and router ID2 and the cost to be updated. (ID1 and ID2 will always be neighbors separated by a link)
                                     2. Check if cost is INF or something else
                                     3. if ID1 = whoAmiID and ID2 is a ne for whoAmiID in the LBTT, update neReaachability with the new cost
                                     4. Set doesExist false.
                                     5. Set DV
                                     6. Call Bellman-Ford instance-1
                                     7. Args for Bellman-Ford:
                                     a. pointer of routing table (algo will run on this table)
                                     b. whoAmiID
                                     c. no. of nodes
                                     d. DV
                                     */
                                    uint16_t costIdentifier;
                                    if(ntohs(cpp->changedCost)==65535)
                                    {
                                        costIdentifier=INF;
                                        printf("cost set to INF\n");
                                    }
                                    else
                                    {
                                        printf("cost set to otherwise\n");
                                        costIdentifier=ntohs(cpp->changedCost);
                                    }
                                    for(int i=1;i<ntohs(nodeCount);i++)
                                    {
                                        if(localBaseTopologyTable[i].destinationRouterID==cpp->changeCostForRID)
                                        index=i;
                                            
                                    }
                                    
                                    if(localBaseTopologyTable[index].ne==true)
                                    {
                                        printf("NE is true, neReachability will be updated\n");
                                        neReachability[index]=costIdentifier;
                                        
                                        for(int i=1; i<=ntohs(nodeCount); i++)
                                        {
                                            printf("setting doesExist to false for all\n");
                                            //wrong. change it to router ID at [i]
                                            localBaseTopologyTable[i].doesExist=false;
                                            printf("setting DV\n");
                                            for(int j=1; j<=ntohs(nodeCount); j++)
                                            {
                                                if(i==j)
                                                {
                                                    //this is fine
                                                    printf("setting DV to 0\n");
                                                    dv[(localBaseTopologyTable[i].routerSerialNumber)][(localBaseTopologyTable[j].routerSerialNumber)]=0;
                                                }
                                                else
                                                {
                                                    //this is fine
                                                    printf("setting DV to INF\n");
                                                    dv[(localBaseTopologyTable[i].routerSerialNumber)][(localBaseTopologyTable[j].routerSerialNumber)]=INF;
                                                }
                                            }
                                        }
                                        //calling bellman-ford INSTANCE #1
                                        printf("calling bellman ford INSTANCE #1\n");
                                        distanveVectorRoutingAlgorithm(dv, nodeCount, whoAmiID, localBaseTopologyTable);
                                    }
                                    
                                }//cc3 ends
                                else if(cph->controlCode==4)
                                {
                                    //CRASH-TURN OFF ROUTING OPERATIONS- NO RESPONSE REQUIRED EXCEPT HEADER
                                    printf("control code 0x04 found. Router [%d] will crash now\n", ntohs(whoAmiID));
                                    //generate response before exiting
                                    //localBaseTopologyTable[ntohs(whoAmiID)].active=false;
                                    
                                    controlResponseBuffer = new unsigned char [1024];
                                    struct controlResponseHeader *crh = (struct controlResponseHeader *) malloc(sizeof(struct controlResponseHeader));
                                    printf("trying to pack\n");
                                    crh->controllerIP=static_cast<uint32_t>(peername->sin_addr.s_addr);
                                    //printf("done 1\n");
                                    crh->controlCode=4;
                                    //printf("done 2\n");
                                    crh->responseCode=0;
                                    //printf("done 3\n");
                                    crh->payloadLength=0;
                                    //printf("done 4\n");
                                    pack(controlResponseBuffer, "LCCH", (uint32_t)crh->controllerIP, (uint8_t)crh->controlCode, (uint8_t)crh->responseCode, (uint16_t)crh->payloadLength);
                                    printf("pack successful\n");
                                    int ableToSend1 = send(newsockfd, controlResponseBuffer,8, 0);
                                    
                                    if(ableToSend1<0)
                                    {
                                        printf("failed to send control response header\n");
                                    }
                                    printf("response sent to controller\n");
                                    //system exit and all operations stop
                                    exit(0);
                                }//cc4 ends
                                else if(cph->controlCode==5)
                                {
                                    //SENDFILE-SEND FILE TO OTHER ROUTER-NO RESPONSE REQUIRED EXCEPT HEADER
                                    
                                    /*1. Read from the file name specified by the controller. That file will reside in the same directory.
                                     2. Packetize the file in the specified format and send it to the data port of the next hop router (found after table lookup)
                                     3. Destination router will read this and reconstruct it and write to disk
                                     4. Compute MD5 to check for proper reassembly at destination*/
                                    
                                    char hex[5];
                                    sprintf(hex, "%x", cph->controlCode);
                                    printf("control code %s found. File needs to be sent. Waiting for payload...\n",hex);
                                    struct controlPacketPayload *cpp = (struct controlPacketPayload *) malloc(sizeof(struct controlPacketPayload));
                                    
                                    controlBuffer+=8;
                                    memcpy(&cpp->sendFileToThisIP, controlBuffer, 4);
                                    controlBuffer+=4;
                                    memcpy(&cpp->TTL, controlBuffer, 1);
                                    controlBuffer+=1;
                                    memcpy(&cpp->transferID, controlBuffer, 1);
                                    controlBuffer+=1;
                                    memcpy(&cpp->sequenceNumber, controlBuffer, 2);
                                    controlBuffer+=2;
                                    memcpy(&cpp->fileName, controlBuffer, (cph->payloadLength-8));      //make it equal to payload size - agautam2
                                    controlBuffer-=16;
                                    
                                    printf("--------PAYLOAD CONTAINS--------\n");
                                    char * str1= inet_ntoa(*(struct in_addr *)&cpp->sendFileToThisIP);
                                    printf("File to be sent: %s\n",cpp->fileName);
                                    printf("Router IP to which file is to be sent: %s\n",str1);
                                    printf("TTL Value: %u\n", cpp->TTL);
                                    printf("Init Seq num: %u\n", ntohs(cpp->sequenceNumber));
                                    printf("TransferID: %u\n",cpp->transferID);
                                    
                                    printf("Trying to pack\n");
                                    controlResponseBuffer = new unsigned char [1024];
                                    struct controlResponseHeader *crh = (struct controlResponseHeader *) malloc(sizeof(struct controlResponseHeader));
                                    printf("trying to pack\n");
                                    crh->controllerIP=static_cast<uint32_t>(peername->sin_addr.s_addr);
                                    //printf("done 1\n");
                                    crh->controlCode=5;
                                    //printf("done 2\n");
                                    crh->responseCode=0;
                                    //printf("done 3\n");
                                    crh->payloadLength=0;
                                    //printf("done 4\n");
                                    pack(controlResponseBuffer, "LCCH", (uint32_t)crh->controllerIP, (uint8_t)crh->controlCode, (uint8_t)crh->responseCode, (uint16_t)crh->payloadLength);
                                    printf("pack successful\n");
                                    
                                    for(int i=1;i<=ntohs(nodeCount);i++)
                                    {
                                     if(localBaseTopologyTable[i].destinationIP==cpp->sendFileToThisIP)
                                     {
                                         printf("File is to be sent to: RID[%d]\n", ntohs(localBaseTopologyTable[i].destinationRouterID));
                                         char * nextHopIP = inet_ntoa(*(struct in_addr *)&localBaseTopologyTable[i].destinationIP);
                                         printf("File will be routed to next hop[%d] which has IP[%s] on port[%d]\n", localBaseTopologyTable[i].nextHopID, nextHopIP, ntohs(localBaseTopologyTable[i].dataPort));
                                         char * dpConnect;
                                         dpConnect=(char*)malloc(sizeof(int)+1);
                                         sprintf(dpConnect, "%d", ntohs(localBaseTopologyTable[i].dataPort));
                                         
                                         //variables for connecting to remote router
                                         //CODE FROM BEEJ
                                         struct addrinfo hints, *servinfo, *p;
                                         
                                         if ((rv = getaddrinfo(nextHopIP, dpConnect, &hints, &servinfo)) != 0)
                                         {
                                             fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
                                             return 1;
                                         }
                                         for(p = servinfo; p != NULL; p = p->ai_next)
                                         {
                                             if ((sockFile = socket(p->ai_family, p->ai_socktype,p->ai_protocol)) == -1)
                                             {
                                                 perror("client: socket");
                                                 continue;
                                             }
                                             if (connect(sockFile, p->ai_addr, p->ai_addrlen) == -1)
                                             {
                                                 close(sockFile);
                                                 perror("client: connect"); //connect to remote router on nextHopIP and dataPort
                                                 continue;
                                             }
                                             break;
                                         }
                                         if (p == NULL)
                                         {
                                             fprintf(stderr, "client: failed to connect\n");
                                             return 2;
                                         }
                                         
                                         
                                         memset(&hints, 0, sizeof hints);
                                         hints.ai_family = AF_INET;
                                         hints.ai_socktype = SOCK_STREAM;
                                         
                                         
                                     }
                                    }
                                    
                                    
                                    //send response to controller
                                    int ableToSend1 = send(newsockfd, controlResponseBuffer,8, 0);
                                    
                                    if(ableToSend1<0)
                                    {
                                        printf("failed to send control response header\n");
                                    }
                                    
                                }//cc5 ends
                                else if(cph->controlCode==6)
                                {
                                    //SENDFILESTATS-RESPONSE REQUIRED
                                    char hex[5];
                                    sprintf(hex, "%x", cph->controlCode);
                                    printf("control code %s found. File STATS need to be sent. Waiting for payload...\n",hex);
                                    struct controlPacketPayload *cpp = (struct controlPacketPayload *) malloc(sizeof(struct controlPacketPayload));
                                    //unpack(controlBuffer, "LCCHC", &cph->destinationIP, &cph->controlCode, &cph->responseTime, &cph->payloadLength ,&cpp->transferID);
                                    controlBuffer+=8;
                                    memcpy(&cpp->transferID, controlBuffer, 1);
                                    
                                    
                                    printf("--------PAYLOAD CONTAINS--------\n");
                                    printf("Stats needed for ID: %u\n",cpp->transferID);
                                }//cc6 ends
                                else if(cph->controlCode==7)
                                {
                                    //LASTDATAPACKET-RESPONSE REQUIRED
                                    printf("control code 0x07 found. Last data packet needs to be sent\n");
                                }//cc7 ends
                                else if(cph->controlCode==8)
                                {
                                    //SECONDLASTDATAPACKET-RESPONSE REQUIRED
                                    printf("control code 0x08 found. Second Last data packet needs to be sent\n");
                                }//cc8 ends
                                
                            } //readBytes manipulation ends

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