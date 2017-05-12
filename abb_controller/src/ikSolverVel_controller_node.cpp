#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/tree.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treeiksolvervel_wdls.hpp>
#include <kdl/treeiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "trajectory_msgs/JointTrajectory.h"

#include <stdio.h>
#include <iostream>
#include <string>

#define IK_VEL
#define JOINT_NUMBER 14
#define LAMBDA 0.1
using namespace KDL;

std::string velocityTopicNames[] = {
    "/yumi/joint_1_l_velocity_controller/command",
    "/yumi/joint_2_l_velocity_controller/command",
    "/yumi/joint_7_l_velocity_controller/command",
    "/yumi/joint_3_l_velocity_controller/command",
    "/yumi/joint_4_l_velocity_controller/command",
    "/yumi/joint_5_l_velocity_controller/command",
    "/yumi/joint_6_l_velocity_controller/command",
    "/yumi/joint_1_r_velocity_controller/command",
    "/yumi/joint_2_r_velocity_controller/command",
    "/yumi/joint_7_r_velocity_controller/command",
    "/yumi/joint_3_r_velocity_controller/command",
    "/yumi/joint_4_r_velocity_controller/command",
    "/yumi/joint_5_r_velocity_controller/command",
    "/yumi/joint_6_r_velocity_controller/command"};

//map joint order 1 1 2 2 3 3 ... to 1 2 7 3....,1 2 7 3...
int map_index[]={0,2,12,4,6,8,10,1,3,13,5,7,9,11};
JntArray q_in;
//callback to get joint state
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    JntArray q_in_temp(msg->position.size());
    for(int i = 0;i < msg->position.size();i++){
        q_in_temp(i) = msg->position[map_index[i]];
    }
    //ordered by {joint_1_l,2,7,3,4,5,6,joint_1_r,2,7,3,4,5,6}
    q_in = q_in_temp;
    // for (int i = 0; i < q_in.rows(); i++) {
    //     std::cout << q_in(i) << " ";
    // }
    // std::cout << "\n";

}


Twists v_in;
std::string eeLinks[] = {
    "yumi_link_7_l",
    "yumi_link_7_r"};
void cartVelCallback(const std_msgs::Float64MultiArray& msg)
{
    std::cout << "received desired cartesian: ";
    for (int i = 0; i < msg.data.size(); i+=3) {
        v_in[eeLinks[i/3]] = Twist(Vector(msg.data[i],msg.data[i+1],msg.data[i+2]),Vector(0,0,0));
        std::cout << eeLinks[i] << " v_x:" << msg.data[i] << " v_y:" << msg.data[i+1] << " v_z:" << msg.data[i+2] << " m/s ";
    }
    std::cout << "\n";

}

int main(int argc, char** argv)
{



    ros::init(argc, argv, "abb_controller", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    //subscrib topic /yumi/joint_state to get joint state
    ros::Subscriber subJointStates = node.subscribe("/yumi/joint_states", 1000, jointStatesCallback);
    ros::Subscriber subCartVelocities = node.subscribe("/yumi/ikSloverVel_controller/command", 1000, cartVelCallback);

    std::vector <ros::Publisher> velocityPublishers;
    ros::Publisher velocityPublisher;
    for(int i = 0;i < JOINT_NUMBER;i++){
        velocityPublisher = node.advertise<std_msgs::Float64> (velocityTopicNames[i], 1);
        velocityPublishers.push_back(velocityPublisher);
     }


    std::string robot_desc_string;
    KDL::Tree tree;
    node.param("robot_description",robot_desc_string,std::string());
    if(!kdl_parser::treeFromString(robot_desc_string,tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }
    if(tree.getNrOfJoints() != JOINT_NUMBER){
        ROS_ERROR("Wrong Joint Number:%d\n",tree.getNrOfJoints());
        return false;
    }

    // std::cout <<(*((*((*rootSegment).second.children[0])).second.children[1])).first<< '\n';

    JntArray q_init(JOINT_NUMBER);
    JntArray q_out(JOINT_NUMBER);
    JntArray qdot_out(JOINT_NUMBER);

    std::vector<std::string>endpoints;
    endpoints.push_back("yumi_link_7_l");
    endpoints.push_back("yumi_link_7_r");
    TreeFkSolverPos_recursive fkSolver(tree);
    TreeIkSolverVel_wdls ikSolverVel(tree,endpoints);
    ikSolverVel.setLambda(LAMBDA);


    //initialize v_in
    v_in[eeLinks[0]] = Twist(Vector(0,0,0),Vector(0,0,0));
    v_in[eeLinks[1]] = Twist(Vector(0,0,0),Vector(0,0,0));

    std_msgs::Float64 joint_velocity;
    bool velocity_limit = false;
    ros::Rate rate(2);
    while(ros::ok()){
        //solve velocity when joint state was known
        double ret = -666;
        if(q_in.rows()>0)
            ret = ikSolverVel.CartToJnt(q_in,v_in,qdot_out);
        if(ret>=0){
            std::cout << "Listening to topic /yumi/ikSloverVel_controller/command\n";
            std::cout << "Send Joint Velocities:" << "  ";
            for(int i=0;i<JOINT_NUMBER;i++){
                joint_velocity.data = qdot_out(i);
                if(!velocity_limit && (joint_velocity.data > 0.1 || joint_velocity.data < -0.1)){
                    velocity_limit = true;
                    std::cout << "Joint Velocity reached the limit!" << '\n';
                }
                //reset velocity
                if(argc != 1 || velocity_limit) joint_velocity.data = 0;
                velocityPublishers[i].publish(joint_velocity);
                std::cout <<joint_velocity.data<<" ";
            }
            std::cout << '\n';

        }
        else if(ret == -666){
            std::cout << "waiting for joint states data[Please Wait]" << '\n';
        }

        std::cout << "------------------------" << '\n';
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}






//   #ifdef debug
//     //Creation of the chain:
//     KDL::Chain chain_l;
//     chain_l.addSegment(Segment("11",Joint(Joint::None),Frame(Vector(0.0,-2.0,0))));
//     chain_l.addSegment(Segment("12",Joint(Joint::RotX),Frame(Vector(0.0,0.0,2))));
//     chain_l.addSegment(Segment("13",Joint(Joint::RotX),Frame(Vector(0.0,0.0,1))));
//     KDL::Chain chain_r;
//     chain_r.addSegment(Segment("21",Joint(Joint::None),Frame(Vector(0.0,2.0,0))));
//     chain_r.addSegment(Segment("22",Joint(Joint::RotX),Frame(Vector(0.0,0.0,2))));
//     chain_r.addSegment(Segment("23",Joint(Joint::RotX),Frame(Vector(0.0,0.0,1))));
//     KDL::Tree tree;
//     tree.addSegment(Segment("base_segment",Joint(Joint::None),Frame(Vector(0.0,0.0,0))),"root");
//
//
//     tree.addChain(chain_l,"base_segment");
//     tree.addChain(chain_r,"base_segment");
//
//
//     // //Creation of the solvers:
//     // ChainFkSolverPos_recursive fksolver1(chain);//Forward position solver
//     // ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver
//     // ChainIkSolverPos_NR iksolver1(chain,fksolver1,iksolver1v,100000,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
//     //
//     std::vector<std::string>endpoints;
//     endpoints.push_back("13");
//     endpoints.push_back("23");
//     JntArray q_min(tree.getNrOfJoints()),q_max(tree.getNrOfJoints());
//     q_min(0) = -180;
//     q_min(1) = -180;
//     q_min(2) = -180;
//     q_min(3) = -180;
//     q_max(0) = 180;
//     q_max(1) = 180;
//     q_max(2) = 180;
//     q_max(3) = 180;
//
//     TreeFkSolverPos_recursive fkSolver(tree);
//     TreeIkSolverVel_wdls ikSolverVel(tree,endpoints);
//     TreeIkSolverPos_NR_JL ikSolverPos(tree,endpoints,q_min,q_max,fkSolver,ikSolverVel,100,1e-6);
//
//     //Creation of jntarrays:
//     JntArray q_in((tree.getNrOfJoints()));
//     JntArray q_init((tree.getNrOfJoints()));
//     JntArray q_out((tree.getNrOfJoints()));
//     JntArray qdot_out(tree.getNrOfJoints());
//
//
//
//     // //Set destination frame
//     // Twist v_in(Vector(0,0,1),Vector(0,0,0));
//     // int ret = iksolver1v.CartToJnt(q_in,v_in,qdot_out);
//     // if(ret == SolverI::E_NOERROR)
//     //   std::cout << "q:"<<qdot_out(0)*180/3.141592653<<" "<<qdot_out(1)*180/3.141592653<<'\n';
// #ifdef IK_POS
//     Frames p_dest;
//     p_dest["13"] = Frame(Vector(0,0,1));
//     p_dest["23"] = Frame(Vector(0,2,3));
//     int ret = ikSolverPos.CartToJnt(q_init,p_dest,q_out);
//     // int ret = fkSolver.JntToCart(q_in,p_out,"23");
//     if(ret == SolverI::E_NOERROR)
//       std::cout <<q_out(0)*180/3.14159265<<" "<<q_out(1)*180/3.14159265<<" "
//                 <<q_out(2)*180/3.14159265<<" "<<q_out(3)*180/3.14159265<<'\n';
//
// #endif
//
// #ifdef IK_VEL
//
//     Twists v_in;
//     v_in["13"] = Twist(Vector(0,-0.5,-0.1),Vector(0,0,0));
//     v_in["23"] = Twist(Vector(0,0,0),Vector(0,0,0));
//     int ret = ikSolverVel.CartToJnt(q_in,v_in,qdot_out);
//
//     if(ret == SolverI::E_NOERROR)
//       std::cout <<qdot_out(0)<<" "<<qdot_out(1)<<" "
//                 <<qdot_out(2)<<" "<<qdot_out(3)<<'\n';
//
// #endif
//
// #endif
