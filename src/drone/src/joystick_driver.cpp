#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include "ros/ros.h"
#include "drone/Joystick.h"
#include "std_msgs/String.h"

using namespace std;

unsigned int constrain(unsigned int value, unsigned int min_value, unsigned int max_value)
{
 if (value < min_value)
 {
  return min_value;
 }
 if (value > max_value)
 {
  return max_value;
 }
 return value;
}

void error( char *msg)
{
 perror(msg);
 exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "joystick_driver_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<drone::Joystick>("joystick_state", 1000);
    ros::Rate loop_rate(1000);

    int sockfd;
    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    struct sockaddr_in serv,client;

    serv.sin_family = AF_INET;
    serv.sin_port = htons(1234);
    //  serv.sin_addr.s_addr = inet_addr("192.168.1.84");
    serv.sin_addr.s_addr = inet_addr("192.168.88.155");


     bind(sockfd,(struct sockaddr *)&serv,sizeof(serv));

     unsigned char esp8266_data[128];
     socklen_t l = sizeof(client);
     socklen_t m = sizeof(serv);
     int len = 128;

    cout << "Client started" << endl;


     while(ros::ok())
     {
        recvfrom(sockfd,esp8266_data,len,0,(struct sockaddr *)&client,&l);

        drone::Joystick msg;
        if (esp8266_data[0] == 0x77 and esp8266_data[1])
        {
            msg.left_x = constrain(esp8266_data[4] | esp8266_data[5] << 8, 0, 1023);
            msg.left_y = constrain(esp8266_data[6] | esp8266_data[7] << 8, 0, 1023);
            msg.right_x = constrain(esp8266_data[8] | esp8266_data[9] << 8, 0, 1023);
            msg.right_y = constrain(esp8266_data[10] | esp8266_data[11] << 8, 0, 1023);

            msg.Short.A = esp8266_data[14];
            msg.Short.B = esp8266_data[15];
            msg.Short.C = esp8266_data[16];
            msg.Short.D = esp8266_data[17];
            msg.Short.Lsw = esp8266_data[18];
            msg.Short.Rsw = esp8266_data[19];
            msg.Short.HOLD = esp8266_data[20];
            msg.Short.LED = esp8266_data[21];
            msg.Short.L = esp8266_data[22];
            msg.Short.R = esp8266_data[23];

            msg.Long.A = esp8266_data[24];
            msg.Long.B = esp8266_data[25];
            msg.Long.C = esp8266_data[26];
            msg.Long.D = esp8266_data[27];
            msg.Long.Lsw = esp8266_data[28];
            msg.Long.Rsw = esp8266_data[29];
            msg.Long.HOLD = esp8266_data[30];
            msg.Long.LED = esp8266_data[31];
            msg.Long.L = esp8266_data[32];
            msg.Long.R = esp8266_data[33];

            msg.pressed_buttons = esp8266_data[34] | esp8266_data[35] << 8;
            msg.State = esp8266_data[36];
            msg.Battery = esp8266_data[12] | esp8266_data[13] << 8;
            msg.SpeedMode = esp8266_data[37];
            msg.LightMode = esp8266_data[38];

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
     }
     return 0;
 }

