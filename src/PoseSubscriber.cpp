
#include <RVO.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "rvo2/PoseSubscriber.hpp"
#include "rvo2/utility.hpp"

PoseSubscriber::PoseSubscriber(ros::NodeHandle *n, RVO::RVOSimulator *sim,
        string topic) {
    this->sim = sim;
    this->topic = topic;
    this->pos_set = false;
    this->pref_vel = RVO::Vector3(0, 0, 0);
    this->sub = n->subscribe(topic, 0, &PoseSubscriber::callback, this);
}

void PoseSubscriber::callback(geometry_msgs::PoseStamped ps) {
    if (!pos_set) {
        pos_set = true;
        id = sim->addAgent(pose_to_vector(ps));
        sim->setAgentPrefVelocity(id, pref_vel);
        sim->setAgentVelocity(id, pref_vel);
        time = ros::Time::now().toSec();
    } else {
        float cur_time = ros::Time::now().toSec();
        float dt = cur_time - time + EPS;
        RVO::Vector3 vel = (pose_to_vector(ps) - pos) / dt;
        sim->setAgentVelocity(id, vel);
        time = cur_time;
    }
    sim->globalTime_ = time;
    pos = pose_to_vector(ps);
}

void PoseSubscriber::set_pref_vel(RVO::Vector3 vel) {
    sim->setAgentPrefVelocity(id, vel);
    this->pref_vel = vel;
}

void PoseSubscriber::set_pref_vel(geometry_msgs::Twist vel) {
    this->set_pref_vel(twist_to_vector(vel));
}

int PoseSubscriber::get_id() {
    return this->id;
}
