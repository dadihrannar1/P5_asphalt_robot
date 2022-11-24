#include "include/UDP_Com.h"
#include <iostream>
#include <string.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>


void UDP_Com::EncodeMessage(){
    UDP_Com::JSON_Message = Message.dump(3);
}

void UDP_Com::DecodeMessage(std::string JSON_message){
    Message = nlohmann::json::parse(JSON_message);
}

#pragma region Update And Extract
void UDP_Com::UpdatePosition(float posx, float posy){
    Message["Position"]["X"] = posx;
    Message["Position"]["Y"] = posy;
    EncodeMessage();
}

void UDP_Com::UpdateTime(float timeDet, float timeClock){
    Message["Time"]["Detected"] = timeDet;
    Message["Time"]["TimeSet"] = timeClock;
    EncodeMessage();
}

void UDP_Com::UpdateCrackDet(float crackDet){
    Message["Crack"]["DetectionIndex"] = crackDet;
    EncodeMessage();
}

int *UDP_Com::ExtractPosition(){
    int *PosXY = new int[2];
    PosXY[0] = Message["Position"]["X"];
    PosXY[1] = Message["Position"]["Y"];
    return PosXY;
}

float *UDP_Com::ExtractTime(){
    float *Time = new float[2];
    Time[0] = Message["Time"]["Detected"];
    return Time;
}

float *UDP_Com::ExtractCrackDet(){
    float *CrackDet = new float[2];
    CrackDet[0] = Message["Crack"]["DetectionIndex"];
    return CrackDet;
}

#pragma endregion

void UDP_Com::SendMessage(){  //CLIENT, send message to server
    char buffer[MAXLINE];
    char Mesg[] = "0";
    strcpy(Mesg,JSON_Message.c_str());
      
    sendto(Client_sockfd, (const char *)Mesg, strlen(Mesg),
        MSG_CONFIRM, (const struct sockaddr *) &servaddr, 
            sizeof(servaddr));

    if(debug == true){
        std::cout << Mesg << "\n Message sent to server\n" << std::endl;
    }
    
    //close(Client_sockfd);

}

void UDP_Com::ReceiveMessage(){  //SERVER, receive message from client
    char buffer[MAXLINE];
    int len, n;
    len = sizeof(cliaddr);  //len is value/resuslt
    
    if(debug == true){
        std::cout << "Waiting for new message\n" << std::endl;
    }

    n = recvfrom(Server_sockfd, (char *)buffer, MAXLINE, 
                MSG_WAITALL, ( struct sockaddr *) &cliaddr,
                (socklen_t*)&len);
    buffer[n] = '\0';
    
    //Convert buffer char array to string and decode
    std::string RawStringMsg = convertToString(buffer, sizeof(buffer)); 

    if(debug == true){
        std::cout << RawStringMsg << "\nMessage received from client\n" << std::endl;
    }

    DecodeMessage(RawStringMsg);

    if(debug == true){
        std::cout << "\nMessage decoded\n" << std::endl;
    }
    
}

std::string UDP_Com::convertToString(char* a, int size){
    std::string s = a;
    return s;
}

void UDP_Com::PrintMessage(){
    std::cout << "UDP Printing" << Message.dump(3) << std::endl;
}

void UDP_Com::ToggleDebug(bool status){
    debug = status;
}

void UDP_Com::InitiateServer(){
    // Creating socket file descriptor
        if ( (Server_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
            perror("socket creation failed");
            exit(EXIT_FAILURE);
        }
        int enable = 1;
        if (setsockopt(Server_sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0){
            perror("setsockopt(SO_REUSEADDR) failed");
        }
        
        memset(&servaddr, 0, sizeof(servaddr));
        memset(&cliaddr, 0, sizeof(cliaddr));
        
        // Filling server information
        servaddr.sin_family    = AF_INET; // IPv4
        servaddr.sin_addr.s_addr = INADDR_ANY;
        servaddr.sin_port = htons(PORT);
        
        // Bind the socket with the server address
        if ( bind(Server_sockfd, (const struct sockaddr *)&servaddr, 
                sizeof(servaddr)) < 0 )
        {
            perror("bind failed");
        }
}
 
void UDP_Com::InitiateClient(){
      
    // Creating socket file descriptor
    if ( (Client_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }
  
    memset(&servaddr, 0, sizeof(servaddr));
      
    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(PORT);
    servaddr.sin_addr.s_addr = INADDR_ANY;
}


