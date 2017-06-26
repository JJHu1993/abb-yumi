#include <iostream>
#include <assert.h>
#include <string>

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"

#define JOINT_NUMBER 14
#define END_LINK_NUM 2
using namespace std;

//format:{v1_x,v1_y,v1_z,v2_x,v2_y,v2_z}
double cartVelArray[] = {-0.02,0,0,0,0,0};

int main(int argc, char **argv) {
	ros::init(argc, argv, "yumi_example");
	ros::NodeHandle n;
    ros::Publisher cartVelPublisher = n.advertise<std_msgs::Float64MultiArray> ("/yumi/ikSloverVel_controller/command", 1);

	ros::Rate rate(2); //Hz
    std_msgs::Float64MultiArray armCommand;
    armCommand.data.resize(3*END_LINK_NUM);
	while (n.ok()) {

		cout << "sending command ...\n[v1_x v1_y v1_z v2_x v2_y v2_z]:" << endl;
        for(int i = 0;i < 3*END_LINK_NUM;i++){
            armCommand.data[i] = cartVelArray[i];
            if(argc != 1) armCommand.data[i] = 0.0;
            cout << armCommand.data[i] << "  ";
        }
		cartVelPublisher.publish(armCommand);
		cout << "\n--------------------" << endl;
        rate.sleep();
	}

	return 0;
}

/* EOF */
