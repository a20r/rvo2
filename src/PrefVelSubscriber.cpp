
#include <RVO.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "rvo2/utility.hpp"
#include "rvo2/PrefVelSubscriber.hpp"

PrefVelSubscriber::PrefVelSubscriber(ros::NodeHandle *n,
        RVO::RVOSimulator *sim, string pref_vel_topic,
        string cmd_vel_topic) {
    this->n = n;
    this->sim = sim;
    this->pref_vel_topic = pref_vel_topic;
    this->cmd_vel_topic = cmd_vel_topic;
    this->id = -1;
    this->time = ros::Time::now().toSec();
}

void PrefVelSubscriber::start() {
    this->pub = n->advertise<geometry_msgs::Twist>(cmd_vel_topic, 0);
    this->sub = n->subscribe(pref_vel_topic, 0,
            &PrefVelSubscriber::callback, this);
}

void PrefVelSubscriber::set_id(int id) {
    this->id = id;
}

int PrefVelSubscriber::get_id() {
    return this->id;
}

RVO::Vector3 PrefVelSubscriber::get_pref_vel() {
    return this->pref_vel;
}

ros::Publisher PrefVelSubscriber::get_pub() {
    return this->pub;
}

void PrefVelSubscriber::callback(geometry_msgs::Twist pref_vel) {
    if (id >= 0) {
        float cur_time = ros::Time::now().toSec();
        // sim->setAgentPrefVelocity(id, twist_to_vector(pref_vel));
        time = cur_time;
        this->pref_vel = twist_to_vector(pref_vel);
        // RVO::Vector3 vel = sim->compute_new_velocity(id);
        // sim->setAgentVelocity(id, vel);
        // pub.publish(vector_to_twist(vel));
    }
}
