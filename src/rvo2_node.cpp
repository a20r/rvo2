
#include <RVO.h>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "boost/bind.hpp"
#include "rvo2/utility.hpp"
#include "rvo2/PoseSubscriber.hpp"
#include "rvo2/PrefVelSubscriber.hpp"

using namespace std;

RVO::RVOSimulator *sim;
vector<PoseSubscriber *> pose_subs;
vector<PrefVelSubscriber *> pref_vel_subs;
ros::NodeHandle *n;

RVO::RVOSimulator *init_sim() {
    RVO::RVOSimulator *rvo_sim = new RVO::RVOSimulator();
    rvo_sim->setTimeStep(1.0 / 30);
    rvo_sim->setAgentDefaults(2, 2, 100, 2, 1);
    return rvo_sim;
}

int main(int argc, char *argv[]) {
    // Initializing ROS
    ros::init(argc, argv, "rvo2");
    n = new ros::NodeHandle();
    ros::MultiThreadedSpinner spinner(8);

    // Setting up parameters
    vector<string> p_topics, cv_topics, pv_topics;
    ros::param::get("~position_topics", p_topics);
    ros::param::get("~cmd_vel_topics", cv_topics);
    ros::param::get("~pref_vel_topics", pv_topics);
    sim = init_sim();

    for (int i = 0; i < p_topics.size(); i++) {
        pref_vel_subs.push_back(new PrefVelSubscriber(n, sim,
                    pv_topics[i], cv_topics[i]));
        pose_subs.push_back(new PoseSubscriber(n, sim, p_topics[i],
                    pref_vel_subs[i]));
        pose_subs[i]->start();
        pref_vel_subs[i]->start();
    }

    spinner.spin();
}
