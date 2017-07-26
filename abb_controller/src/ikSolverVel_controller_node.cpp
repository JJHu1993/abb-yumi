#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
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
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include "trajectory_msgs/JointTrajectory.h"

#include <stdio.h>
#include <iostream>
#include <string>

#define IK_VEL
#define JOINT_NUMBER 14
#define LAMBDA 0.0001
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
//for IK_Vel use
JntArray q_in;
JntArray q_in_l;
JntArray q_in_r;
//for FK_Vel use
JntArrayVel qdot_in_l;
JntArrayVel qdot_in_r;
//callback to get joint state
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    if(msg->position.size() != JOINT_NUMBER){
        std::cout << "wrong joint size" << '\n';
        return ;
    }
    JntArray q_in_temp(JOINT_NUMBER);
    JntArray q_in_temp_l(JOINT_NUMBER/2);
    JntArray q_in_temp_r(JOINT_NUMBER/2);
    JntArray qdot_in_temp_l(JOINT_NUMBER/2);
    JntArray qdot_in_temp_r(JOINT_NUMBER/2);
    for(int i = 0;i <JOINT_NUMBER;i++){
        q_in_temp(i) = msg->position[map_index[i]];
        if(i<JOINT_NUMBER/2){
            q_in_temp_l(i) = msg->position[map_index[i]];
            qdot_in_temp_l(i) = msg->velocity[map_index[i]];
        }
        else{
            q_in_temp_r(i - JOINT_NUMBER/2) = msg->position[map_index[i]];
            qdot_in_temp_r(i - JOINT_NUMBER/2) = msg->velocity[map_index[i]];
        }
    }
    //ordered by {joint_1_l,2,7,3,4,5,6,joint_1_r,2,7,3,4,5,6}
    q_in = q_in_temp;
    q_in_l = q_in_temp_l;
    q_in_r = q_in_temp_r;
    qdot_in_l = JntArrayVel(q_in_temp_l,qdot_in_temp_l);
    qdot_in_r = JntArrayVel(q_in_temp_r,qdot_in_temp_r);
    // qdot_in = qdot_in_temp;
    // for (int i = 0; i < qdot_in.rows(); i++) {
    //     std::cout << qdot_in(i) << " ";
    // }
    // std::cout << "\n";

}


Twists v_in;
Twist v_in_l;
Twist v_in_r;
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
    v_in_l = Twist(Vector(msg.data[0],msg.data[1],msg.data[2]),Vector(0,0,0));
    v_in_r = Twist(Vector(msg.data[3],msg.data[4],msg.data[5]),Vector(0,0,0));
    std::cout << "\n";

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "abb_controller", ros::init_options::NoSigintHandler);
    ros::NodeHandle node;

    //subscrib topic /yumi/joint_state to get joint state
    ros::Subscriber subJointStates = node.subscribe("/yumi/joint_states", 1000, jointStatesCallback);
    ros::Subscriber subCartVelocities = node.subscribe("/yumi/ikSloverVel_controller/command", 1000, cartVelCallback);
    //cartesian velocity of ee publisher
    ros::Publisher cart_vPublisher = node.advertise<std_msgs::Float64MultiArray> ("/yumi/ikSloverVel_controller/state", 1);
    ros::Publisher cart_pPublisher = node.advertise<std_msgs::Float64MultiArray> ("/yumi/ikSloverVel_controller/ee_cart_position", 1);
    ros::Publisher cart_pVisPublisher = node.advertise<geometry_msgs::PoseArray> ("/yumi/ikSloverVel_controller/ee_cart_position_vis", 1);
    std::vector <ros::Publisher> velocityPublishers;
    ros::Publisher velocityPublisher;
    for(int i = 0;i < JOINT_NUMBER;i++){
        velocityPublisher = node.advertise<std_msgs::Float64> (velocityTopicNames[i], 1);
        velocityPublishers.push_back(velocityPublisher);
     }


    std::string robot_desc_string;
    KDL::Tree tree;
    Chain chain_l,chain_r;  //for FK_Vel use
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
    JntArray qdot_out_l(JOINT_NUMBER/2);
    JntArray qdot_out_r(JOINT_NUMBER/2);

    //tree IK_Vel
    std::vector<std::string>endpoints;
    endpoints.push_back("yumi_link_7_l");
    endpoints.push_back("yumi_link_7_r");
    TreeFkSolverPos_recursive fkSolver(tree);
    TreeIkSolverVel_wdls ikSolverVel(tree,endpoints);
    ikSolverVel.setLambda(LAMBDA);

    //for FK_Vel use
    SegmentMap::const_iterator rootSegment= tree.getRootSegment();
    std::cout << "root name:" <<(*rootSegment).first<<'\n';
    tree.getChain((*rootSegment).first,"yumi_link_7_l",chain_l);
    tree.getChain((*rootSegment).first,"yumi_link_7_r",chain_r);
    chain_l.addSegment(Segment("gripper_l",Joint(Joint::None),Frame(Vector(0.0,0.0,0.136))));
    chain_r.addSegment(Segment("gripper_r",Joint(Joint::None),Frame(Vector(0.0,0.0,0.136))));
    ChainFkSolverVel_recursive  fkSolverVel_l(chain_l);
    ChainFkSolverVel_recursive  fkSolverVel_r(chain_r);
    
    //chain FK_Pos
    ChainFkSolverPos_recursive  fkSolverPos_l(chain_l);
    ChainFkSolverPos_recursive  fkSolverPos_r(chain_r);
    

    //chain IK_VEL
    ChainIkSolverVel_pinv  ikSolverVel_l(chain_l);
    ChainIkSolverVel_pinv  ikSolverVel_r(chain_r);

    //initialize v_in
    v_in[eeLinks[0]] = Twist(Vector(0,0,0),Vector(0,0,0));
    v_in[eeLinks[1]] = Twist(Vector(0,0,0),Vector(0,0,0));

    std_msgs::Float64 joint_velocity;
    bool velocity_limit = false;
    ros::Rate rate(30);
    while(ros::ok()){
        //solve velocity when joint state was known
        // double ret = -666;
        // if(q_in.rows()>0)
        //     ret = ikSolverVel.CartToJnt(q_in,v_in,qdot_out);
        // if(ret>=0){
        int ret_l0 = -666,ret_r0 = -666;
        if(q_in_l.rows()>0 && q_in_r.rows()>0){
            ret_l0 = ikSolverVel_l.CartToJnt(q_in_l,v_in_l,qdot_out_l);
            ret_r0 = ikSolverVel_r.CartToJnt(q_in_r,v_in_r,qdot_out_r);
        }
        if(ret_l0 == SolverI::E_NOERROR && ret_r0 == SolverI::E_NOERROR){
            std::cout << "Listening to topic /yumi/ikSloverVel_controller/command\n";
            std::cout << "Send Joint Velocities:" << "  ";
            for(int i=0;i<JOINT_NUMBER;i++){
                //tree method
                // joint_velocity.data = qdot_out(i);
                //chain method
                if(i<JOINT_NUMBER/2)
                    joint_velocity.data = qdot_out_l(i);
                else
                    joint_velocity.data = qdot_out_r(i-7);

                if(!velocity_limit && (joint_velocity.data > 0.8 || joint_velocity.data < -0.8)){
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
        //tree method
        //else if(ret == -666 ){
        //chain method
        else if(ret_l0 == -666 || ret_r0 == -666){
            std::cout << "waiting for joint states data[Please Wait]" << '\n';
        }
        std::cout << "------------------------" << '\n';

        //--------------------------------------------------------------------------------
        //compute FK Vel and publish
        FrameVel cart_v_real_l,cart_v_real_r;
        int ret_l= -1,ret_r = -1;
        if(qdot_in_l.qdot.rows()>0 && qdot_in_r.qdot.rows()>0){
            ret_l = fkSolverVel_l.JntToCart(qdot_in_l,cart_v_real_l);
            ret_r = fkSolverVel_r.JntToCart(qdot_in_r,cart_v_real_r);
        }
        if(ret_l == SolverI::E_NOERROR && ret_r == SolverI::E_NOERROR){
            std_msgs::Float64MultiArray cart_v_real;
            cart_v_real.data.resize(2*3);
            for(int i = 0;i < 2*3;i++){
                if(i<3)
                    cart_v_real.data[i] = cart_v_real_l.p.v[i];
                else
                    cart_v_real.data[i] = cart_v_real_r.p.v[i-3];
            }
            cart_vPublisher.publish(cart_v_real);
        }
        
        //compute FK Pos and publish
        Frame cart_p_real_l,cart_p_real_r;
        int ret_p_l = -1,ret_p_r = -1;
        if(q_in_l.rows()>0 && q_in_r.rows()>0){
            ret_p_l = fkSolverPos_l.JntToCart(q_in_l,cart_p_real_l);
            ret_p_r = fkSolverPos_r.JntToCart(q_in_r,cart_p_real_r);
        }
        if(ret_p_l == SolverI::E_NOERROR && ret_p_r == SolverI::E_NOERROR){
            std_msgs::Float64MultiArray cart_p_real;
            geometry_msgs::PoseArray cart_p_real_vis;
            cart_p_real_vis.header.frame_id = "world";
            geometry_msgs::Pose cart_p_real_vis_l;
            geometry_msgs::Pose cart_p_real_vis_r;
            cart_p_real.data.resize(2*3);
            for(int i = 0;i < 2*3;i++){
                if(i<3)
                    cart_p_real.data[i] = cart_p_real_l.p[i];
                else
                    cart_p_real.data[i] = cart_p_real_r.p[i-3];
            }
            cart_p_real_vis_l.position.x = cart_p_real_l.p[0];
            cart_p_real_vis_l.position.y = cart_p_real_l.p[1];
            cart_p_real_vis_l.position.z = cart_p_real_l.p[2];
            cart_p_real_vis_r.position.x = cart_p_real_r.p[0];
            cart_p_real_vis_r.position.y = cart_p_real_r.p[1];
            cart_p_real_vis_r.position.z = cart_p_real_r.p[2];
            cart_p_real_vis.poses.push_back(cart_p_real_vis_l);
            cart_p_real_vis.poses.push_back(cart_p_real_vis_r);
            cart_pVisPublisher.publish(cart_p_real_vis);
            cart_pPublisher.publish(cart_p_real);
        }


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
