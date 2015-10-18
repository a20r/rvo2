
#include <RVO.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "rvo2/utility.hpp"
#include "rvo2/PrefVelSubscriber.hpp"

PrefVelSubscriber::PrefVelSubscriber(ros::NodeHandle *n,
        RVO::RVOSimulator *sim, string pref_vel_topic,
        string cmd_vel_topic, int id) {
    this->n = n;
    this->sim = sim;
    this->pref_vel_topic = pref_vel_topic;
    this->cmd_vel_topic = cmd_vel_topic;
    this->id = id;
}

void PrefVelSubscriber::start() {
    this->pub = n->advertise<geometry_msgs::Twist>(cmd_vel_topic, 0);
    this->sub = n->subscribe(pref_vel_topic, 0,
            &PrefVelSubscriber::callback, this);
}

void PrefVelSubscriber::callback(geometry_msgs::Twist pref_vel) {
    sim->setAgentPrefVelocity(id, twist_to_vector(pref_vel));
    RVO::Vector3 vel = sim->compute_new_velocity(id);
    pub.publish(vector_to_twist(vel));
}
