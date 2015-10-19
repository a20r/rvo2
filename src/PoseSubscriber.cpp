
#include <RVO.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "rvo2/utility.hpp"
#include "rvo2/PrefVelSubscriber.hpp"
#include "rvo2/VelocitySubscriber.hpp"
#include "rvo2/PoseSubscriber.hpp"

PoseSubscriber::PoseSubscriber(ros::NodeHandle *n, RVO::RVOSimulator *sim,
        string topic, PrefVelSubscriber *pv_sub, VelocitySubscriber *v_sub) {
    this->n = n;
    this->sim = sim;
    this->topic = topic;
    this->pos_set = false;
    this->pv_sub = pv_sub;
    this->v_sub = v_sub;
}

void PoseSubscriber::start() {
    this->sub = n->subscribe(topic, 0, &PoseSubscriber::callback, this);
}

void PoseSubscriber::callback(geometry_msgs::PoseStamped ps) {
    RVO::Vector3 new_pos = pose_to_vector(ps);
    if (!pos_set) {
        pos_set = true;
        id = sim->addAgent(new_pos);
        pv_sub->set_id(id);
        v_sub->set_id(id);
    }
    pos = new_pos;
    sim->setAgentPosition(id, new_pos);
}

int PoseSubscriber::get_id() {
    return this->id;
}
