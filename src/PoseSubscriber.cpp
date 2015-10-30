
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
    this->time = ros::Time::now().toSec();
}

void PoseSubscriber::callback(geometry_msgs::PoseStamped ps) {
    RVO::Vector3 new_pos = pose_to_vector(ps);
    float cur_time = ros::Time::now().toSec();
    float dt = cur_time - time + EPS;
    // if (dt > 0) {
        if (!pos_set) {
            pos_set = true;
            pos = new_pos;
            pv_sub->set_id(0);
            v_sub->set_id(0);
        }
        vel = (new_pos - pos) / dt;
        pos = new_pos;
        time = cur_time;
        // sim->setAgentPosition(id, new_pos);
    // }
}

RVO::Vector3 PoseSubscriber::get_vel() {
    return this->vel;
}

RVO::Vector3 PoseSubscriber::get_pos() {
    return this->pos;
}

bool PoseSubscriber::is_pos_set() {
    return this->pos_set;
}

int PoseSubscriber::get_id() {
    return this->id;
}
