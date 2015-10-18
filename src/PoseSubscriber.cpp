
#include <RVO.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "rvo2/PrefVelSubscriber.hpp"
#include "rvo2/utility.hpp"
#include "rvo2/PoseSubscriber.hpp"

PoseSubscriber::PoseSubscriber(ros::NodeHandle *n, RVO::RVOSimulator *sim,
        string topic, PrefVelSubscriber *pv_sub) {
    this->n = n;
    this->sim = sim;
    this->topic = topic;
    this->pos_set = false;
    this->pv_sub = pv_sub;
    this->pref_vel = RVO::Vector3(0, 0, 0);
    this->sub = n->subscribe(topic, 0, &PoseSubscriber::callback, this);
}

void PoseSubscriber::start() {
    this->sub = n->subscribe(topic, 0, &PoseSubscriber::callback, this);
}

void PoseSubscriber::callback(geometry_msgs::PoseStamped ps) {
    float cur_time = ros::Time::now().toSec();
    RVO::Vector3 new_pos = pose_to_vector(ps);
    if (!pos_set) {
        pos_set = true;
        id = sim->addAgent(new_pos);
        pv_sub->set_id(id);
        sim->setAgentPrefVelocity(id, pref_vel);
        sim->setAgentVelocity(id, pref_vel);
    } else {
        float dt = cur_time - time + EPS;
        RVO::Vector3 vel = (new_pos - pos);
        sim->setAgentVelocity(id, vel);
    }
    sim->setAgentPosition(id, new_pos);
    time = cur_time;
    sim->globalTime_ = time;
    pos = new_pos;
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
