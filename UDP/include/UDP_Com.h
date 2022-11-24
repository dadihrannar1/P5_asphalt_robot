#pragma once

#include "json.h"
#include <stdlib.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>

class UDP_Com
{
private:
    nlohmann::json Message;
    std::string JSON_Message;
    int Server_sockfd, Client_sockfd;
    struct sockaddr_in servaddr, cliaddr;
    #define PORT 21350
    #define MAXLINE 1024

    void EncodeMessage(); //Genererer string ud fra Message.
    bool debug = false;

public:
    UDP_Com()
    {
        Message =
        {
            {"Position", 
                {
                    {"X", 0.00}, 
                    {"Y", 0.00} 
                }
            },
            {"Time",
                {
                    {"Detected", 0.00}
                }
            },
            {"Crack", 
                {
                    {"DetectionIndex", 0.00}
                }
            }
        }; 

        
    }


    //konstant flow af positioner fra generede trajectory, fartøj hastighed og om den position er crack eller ej (bitumenflow)
    void UpdatePosition(float posx, float posy);
    void UpdateTime(float timeDet, float timeClock);
    void UpdateCrackDet(float crackDet);
    // void UpdateVelocity(float velx, float vely);
    // void UpdateAcceleration(float accx, float accy);
    // void UpdateBitumenFlow(int BitFlow);
    int *ExtractPosition();
    float *ExtractTime();
    float *ExtractCrackDet();
    // float *ExtractVelocity();
    // float *ExtractAcceleration();
    // int ExtractBitumenFlow();
    
    void InitiateServer();      //Initiate Server to enable use of the ReceiveMessage() function
    void InitiateClient();      //Initiate Client to enable use of the SendMessage() function
    void SendMessage();         //Send message to Server. Remember to InitiateClient()!
    void ReceiveMessage();      //ReceiveMessage from Client. Remember to InitiateServer()!
    void PrintMessage();        //Prints Message at its current state
    void ToggleDebug(bool status);  //Enables/Disables printing of Debug information
    void DecodeMessage(std::string JSON_message); //Genererer Message ud fra string

    std::string convertToString(char* a, int size);

    

};
