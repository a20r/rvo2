
#include <RVO.h>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "boost/bind.hpp"
#include "rvo2/utility.hpp"
#include "rvo2/PoseSubscriber.hpp"

using namespace std;

RVO::RVOSimulator *sim;
ros::Publisher cmd_vel_pub;
ros::Subscriber pref_vel_sub;
vector<PoseSubscriber *> pose_subs;
PoseSubscriber pose_sub;
ros::NodeHandle *n;

RVO::RVOSimulator *init_sim() {
    RVO::RVOSimulator *rvo_sim = new RVO::RVOSimulator();
    rvo_sim->setTimeStep(1.0 / 30);
    rvo_sim->setAgentDefaults(2, 2, 10, 2, 0.5);
    return rvo_sim;
}

void pref_vel_callback(geometry_msgs::Twist pref_vel) {
    pose_sub.set_pref_vel(pref_vel);
    sim->doStep();
    RVO::Vector3 vel = sim->getAgentVelocity(pose_sub.get_id());
    cmd_vel_pub.publish(vector_to_twist(vel));
}

int main(int argc, char *argv[]) {
    // Initializing ROS
    ros::init(argc, argv, "rvo2");
    n = new ros::NodeHandle();

    // Setting up parameters
    vector<string> p_topics;
    string cmd_vel_topic, pref_vel_topic, pose_topic;
    ros::param::get("~position_topics", p_topics);
    ros::param::get("~pose_topic", pose_topic);
    ros::param::get("~cmd_vel_topic", cmd_vel_topic);
    ros::param::get("~pref_vel_topic", pref_vel_topic);
    sim = init_sim();

    for (int i = 0; i < p_topics.size(); i++) {
        pose_subs.push_back(new PoseSubscriber(n, sim, p_topics[i]));
    }

    pose_sub = PoseSubscriber(n, sim, pose_topic);
    pref_vel_sub = n->subscribe(pref_vel_topic, 0, pref_vel_callback);
    cmd_vel_pub = n->advertise<geometry_msgs::Twist>(cmd_vel_topic, 0);

    ros::spin();
}
