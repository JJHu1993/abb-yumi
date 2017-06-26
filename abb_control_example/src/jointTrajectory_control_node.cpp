#include <iostream>
#include <assert.h>
#include <string>

#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#define JOINT_NUMBER 14
using namespace std;

string joint_names[JOINT_NUMBER] = {
  "yumi_joint_1_l",
  "yumi_joint_1_r",
  "yumi_joint_2_l",
  "yumi_joint_2_r",
  "yumi_joint_3_l",
  "yumi_joint_3_r",
  "yumi_joint_4_l",
  "yumi_joint_4_r",
  "yumi_joint_5_l",
  "yumi_joint_5_r",
  "yumi_joint_6_l",
  "yumi_joint_6_r",
  "yumi_joint_7_l",
  "yumi_joint_7_r"};
//open [0.5, -0.906876266002655, 0.5, 0.2693699300289154, -1.795966386795044, -1.3672288656234741, 0.5, -2.345722198486328, 0.5, 0.10665600001811981, 0.5, -0.2191559225320816, 0.5, 0.26214897632598877]
//close {0.9220358729362488, -1.3639558553695679, 0.2633530795574188, 0.3935069143772125, -0.4229700267314911, 0.11999057978391647, 4.527664661407471, -3.3252062797546387, -0.046134576201438904, 0.2501060366630554, 0.5979139804840088, -0.8899979591369629, -0.7876819968223572, 1.207590937614441}
  float joint_positions_up[JOINT_NUMBER] = {-0.004894050769507885, 0.01121285930275917, -0.7481237053871155, -0.7881973385810852, 0.39989161491394043, 0.45870086550712585, -1.0020086765289307, -2.4347167015075684, 0.9538304805755615, -0.7194674015045166, 0.5497622489929199, -0.2827472388744354, 0.15741610527038574, 0.007038898766040802};
  float joint_positions_down[JOINT_NUMBER] = {-1.5901633501052856, 2.758988380432129, -0.776835024356842, -0.7566481828689575, -1.2274471521377563, -1.1534804105758667, -3.3565382957458496, -3.377692699432373, 1.941343903541565, 1.8748698234558105, 1.8207812309265137, 1.2813187837600708, -0.5231171250343323, -0.42509302496910095};

  int main(int argc, char **argv) {

	ros::init(argc, argv, "yumi_example");
	ros::NodeHandle n;
	ros::Publisher armJointTrajectoryPublisher;

	armJointTrajectoryPublisher = n.advertise<trajectory_msgs::JointTrajectory > ("/yumi/joint_trajectory_pos_controller/command", 1);

	ros::Rate rate(2); //Hz
	double readValue;
  int count = 0;
	while (n.ok()) {
    if(count==2) break;
		trajectory_msgs::JointTrajectory armCommand;
		trajectory_msgs::JointTrajectoryPoint desiredConfiguration;

		desiredConfiguration.positions.resize(JOINT_NUMBER);  //5 arm joints + 1 gripper joint
		armCommand.joint_names.resize(JOINT_NUMBER);

		for (int i = 0; i < JOINT_NUMBER; ++i) {
			desiredConfiguration.positions[i] = joint_positions_up[i];
			armCommand.joint_names[i] = joint_names[i];
		};


		armCommand.header.stamp = ros::Time::now();
		armCommand.header.frame_id = "yumi_body";
		armCommand.points.resize(1); // only one point so far
		armCommand.points[0] = desiredConfiguration;
        	armCommand.points[0].time_from_start = ros::Duration(0.5); // 1 ns

		cout << "sending command ..." << endl;
		armJointTrajectoryPublisher.publish(armCommand);
		cout << "--------------------" << endl;
    rate.sleep();
    count++;


	}

	return 0;
}

/* EOF */
