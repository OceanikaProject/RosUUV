#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>
#include "ros/ros.h"
#include "drone/Teleop.h"

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

int convert_to_new_range(int x, int in_min, int in_max, int out_min, int out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
    ros::Publisher pub = n.advertise<drone::Teleop>("joystick_state", 1);
    ros::Rate loop_rate(100);

    int sockfd;
    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    struct sockaddr_in serv,client;

    serv.sin_family = AF_INET;
    serv.sin_port = htons(1234);
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

        drone::Teleop msg;
        if (esp8266_data[0] == 0x77 and esp8266_data[1] == 0x66)
        {
            msg.lx = convert_to_new_range(constrain(esp8266_data[4] | esp8266_data[5] << 8, 0, 1023), 0, 1023, -100, 100);
            msg.ly = convert_to_new_range(constrain(esp8266_data[6] | esp8266_data[7] << 8, 0, 1023), 0, 1023, -100, 100);
            msg.rx = convert_to_new_range(constrain(esp8266_data[8] | esp8266_data[9] << 8, 0, 1023), 0, 1023, -100, 100);
            msg.ry = convert_to_new_range(constrain(esp8266_data[10] | esp8266_data[11] << 8, 0, 1023), 0, 1023, -100, 100);

            unsigned char hand = 1;
            if ((esp8266_data[34] | esp8266_data[35] << 8) & 1) 
            {
                hand = 0;
            }
            else if (((esp8266_data[34] | esp8266_data[35] << 8) >> 3) & 1)
            {
                hand = 2;
            }

            msg.status |= (esp8266_data[36] & 16) >> 4;
            msg.status |= ((esp8266_data[36] & 8) >> 3) << 1;
            msg.status |= hand << 2;
            msg.status |= esp8266_data[37] << 4;
            msg.status |= esp8266_data[38] << 6;
            msg.status |= !(esp8266_data[36] & 1) << 8;
            msg.status |= ((esp8266_data[36] & 2) >> 1) << 9;

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}

