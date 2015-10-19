
#include <RVO.h>
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "rvo2/utility.hpp"
#include "rvo2/VelocitySubscriber.hpp"

VelocitySubscriber::VelocitySubscriber(ros::NodeHandle *n,
        RVO::RVOSimulator *sim, string vel_topic) {
    this->n = n;
    this->sim = sim;
    this->vel_topic = vel_topic;
    this->id = -1;
}

void VelocitySubscriber::start() {
    this->sub = n->subscribe(vel_topic, 0,
            &VelocitySubscriber::callback, this);
}

void VelocitySubscriber::set_id(int id) {
    this->id = id;
}

void VelocitySubscriber::callback(geometry_msgs::Vector3Stamped vel) {
    if (id >= 0) {
        // sim->setAgentVelocity(id, vector_to_vector(vel));
    }
}
