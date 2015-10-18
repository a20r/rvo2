
#include <RVO.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "rvo2/utility.hpp"

using namespace std;

void q0_pose_callback(geometry_msgs::PoseStamped);
void q1_pose_callback(geometry_msgs::PoseStamped);
void q2_pose_callback(geometry_msgs::PoseStamped);

geometry_msgs::PoseStamped q0_pose, q1_pose, q2_pose;
bool q0_pos_set = false, q1_pos_set = false, q2_pos_set = false;
ros::Publisher q0_cmd_vel, q1_cmd_vel, q2_cmd_vel;
ros::Subscriber q0_sub, q1_sub, q2_sub;
RVO::Vector3 q0_vel, q1_vel, q2_vel;
ros::Time lt_0, lt_1, lt_2;

void q0_pose_callback(geometry_msgs::PoseStamped p_q0) {
    if (!q0_pos_set) {
        q0_pose = p_q0;
        q0_pos_set = true;
        lt_0 = ros::Time::now();
        q0_vel = RVO::Vector3(0, 0, 0);
    } else{
        float now = ros::Time::now().toSec();
        float dt = now - lt_0.toSec();
        q0_vel = (pose_to_vector(p_q0) - pose_to_vector(q0_pose)) /
            (dt + 0.0001);
        lt_0 = ros::Time::now();
    }
    q0_pose = p_q0;
}

void q1_pose_callback(geometry_msgs::PoseStamped p_q1) {
    if (!q1_pos_set) {
        q1_pose = p_q1;
        q1_pos_set = true;
        lt_1 = ros::Time::now();
        q1_vel = RVO::Vector3(0, 0, 0);
    } else{
        float now = ros::Time::now().toSec();
        float dt = now - lt_1.toSec();
        q1_vel = (pose_to_vector(p_q1) - pose_to_vector(q1_pose)) /
            (dt + 0.0001);
        lt_1 = ros::Time::now();
    }
    q1_pose = p_q1;
}

void q2_pose_callback(geometry_msgs::PoseStamped p_q2) {
    if (!q2_pos_set) {
        q2_pose = p_q2;
        q2_pos_set = true;
        lt_2 = ros::Time::now();
        q2_vel = RVO::Vector3(0, 0, 0);
    } else{
        float now = ros::Time::now().toSec();
        float dt = now - lt_2.toSec();
        q2_vel = (pose_to_vector(p_q2) - pose_to_vector(q2_pose)) /
            (dt + 0.0001);
        lt_2 = ros::Time::now();
    }
    q2_pose = p_q2;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rvo2_example");
    ros::NodeHandle n;
    q0_sub = n.subscribe("/q0/ground_truth_to_tf/pose", 0, q0_pose_callback);
    q1_sub = n.subscribe("/q1/ground_truth_to_tf/pose", 0, q1_pose_callback);
    q2_sub = n.subscribe("/q2/ground_truth_to_tf/pose", 0, q2_pose_callback);
    // q0_cmd_vel = n.advertise<geometry_msgs::Twist>("/q0/cmd_vel", 0);
    // q1_cmd_vel = n.advertise<geometry_msgs::Twist>("/q1/cmd_vel", 0);
    // q2_cmd_vel = n.advertise<geometry_msgs::Twist>("/q2/cmd_vel", 0);
    q0_cmd_vel = n.advertise<geometry_msgs::Twist>("/q0/pref_vel", 0);
    q1_cmd_vel = n.advertise<geometry_msgs::Twist>("/q1/pref_vel", 0);
    q2_cmd_vel = n.advertise<geometry_msgs::Twist>("/q2/pref_vel", 0);


    while (ros::ok()) {
        ros::spinOnce();
        if (q0_pos_set and q1_pos_set and q2_pos_set) {
            RVO::RVOSimulator *sim = new RVO::RVOSimulator();
            sim->setTimeStep(1.0 / 30);
            sim->setAgentDefaults(3, 2, 10, 3, 1);
            sim->addAgent(pose_to_vector(q0_pose));
            sim->addAgent(pose_to_vector(q1_pose));
            sim->addAgent(pose_to_vector(q2_pose));
            sim->setAgentPrefVelocity(0, RVO::Vector3(1, 1, 1));
            sim->setAgentPrefVelocity(1, RVO::Vector3(-1, -1, 1));
            sim->setAgentPrefVelocity(2, RVO::Vector3(-1, 1, 1));
            ros::Rate r(30);
            ros::Time st = ros::Time::now();
            while (ros::ok()) {
                sim->globalTime_ = ros::Time::now().toSec();
                sim->setAgentPosition(0, pose_to_vector(q0_pose));
                sim->setAgentPosition(1, pose_to_vector(q1_pose));
                sim->setAgentPosition(2, pose_to_vector(q2_pose));
                sim->setAgentVelocity(0, q0_vel);
                sim->setAgentVelocity(1, q1_vel);
                sim->setAgentVelocity(2, q2_vel);
                sim->doStep();

                // Sends commands to the quads
                // q0_cmd_vel.publish(vector_to_twist(sim->getAgentVelocity(0)));
                // q1_cmd_vel.publish(vector_to_twist(sim->getAgentVelocity(1)));
                // q2_cmd_vel.publish(vector_to_twist(sim->getAgentVelocity(2)));
                q0_cmd_vel.publish(vector_to_twist(sim->getAgentPrefVelocity(0)));
                q1_cmd_vel.publish(vector_to_twist(sim->getAgentPrefVelocity(1)));
                q2_cmd_vel.publish(vector_to_twist(sim->getAgentPrefVelocity(2)));

                r.sleep();
                ros::spinOnce();
            }
        }
    }
    return 0;
}
