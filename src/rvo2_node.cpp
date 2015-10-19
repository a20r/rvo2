
#include <RVO.h>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "boost/bind.hpp"
#include "rvo2/utility.hpp"
#include "rvo2/PoseSubscriber.hpp"
#include "rvo2/PrefVelSubscriber.hpp"
#include "rvo2/VelocitySubscriber.hpp"

using namespace std;

RVO::RVOSimulator *sim;
vector<PoseSubscriber *> pose_subs;
vector<PrefVelSubscriber *> pref_vel_subs;
vector<VelocitySubscriber *> velocity_subs;
ros::NodeHandle *n;

RVO::RVOSimulator *init_sim() {
    RVO::RVOSimulator *rvo_sim = new RVO::RVOSimulator();
    // This parameter is fucking stupid
    float dist = 2;
    float radius = 1;
    float speed = 1;
    rvo_sim->setAgentDefaults(dist, 2, 10, radius, speed, RVO::Vector3(0, 0, 0));
    rvo_sim->setTimeStep(1.2);
    return rvo_sim;
}

int main(int argc, char *argv[]) {
    // Initializing ROS
    ros::init(argc, argv, "rvo2");
    n = new ros::NodeHandle();
    ros::MultiThreadedSpinner spinner(8);

    // Setting up parameters
    vector<string> p_topics, cv_topics, pv_topics, v_topics;
    ros::param::get("~position_topics", p_topics);
    ros::param::get("~cmd_vel_topics", cv_topics);
    ros::param::get("~pref_vel_topics", pv_topics);
    ros::param::get("~velocity_topics", v_topics);
    sim = init_sim();

    for (int i = 0; i < p_topics.size(); i++) {
        pref_vel_subs.push_back(new PrefVelSubscriber(n, sim,
                    pv_topics[i], cv_topics[i]));
        velocity_subs.push_back(new VelocitySubscriber(n, sim,
                    v_topics[i]));
        pose_subs.push_back(new PoseSubscriber(n, sim, p_topics[i],
                    pref_vel_subs[i], velocity_subs[i]));
        pose_subs[i]->start();
        pref_vel_subs[i]->start();
        velocity_subs[i]->start();
    }

    spinner.spin();
}
