
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
}

void PoseSubscriber::start() {
    this->sub = n->subscribe(topic, 0, &PoseSubscriber::callback, this);
    this->time = ros::Time::now().toSec();
}

void PoseSubscriber::callback(geometry_msgs::PoseStamped ps) {
    RVO::Vector3 new_pos = pose_to_vector(ps);
    float cur_time = ros::Time::now().toSec();
    if (cur_time - time > 0) {
        if (!pos_set) {
            pos_set = true;
            id = sim->addAgent(new_pos);
            pv_sub->set_id(id);
        } else {
            float dt = cur_time - time;
            time = cur_time;
            RVO::Vector3 vel = (new_pos - pos);
            sim->setAgentVelocity(id, vel);
            cout << sim->getAgentVelocity(id) << endl;
        }
        pos = new_pos;
        sim->setAgentPosition(id, new_pos);
    }
}

int PoseSubscriber::get_id() {
    return this->id;
}
