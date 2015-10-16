
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

RVO::RVOSimulator *init_sim() {
    RVO::RVOSimulator *rvo_sim = new RVO::RVOSimulator();
    rvo_sim->setTimeStep(1.0 / 30);
    rvo_sim->setAgentDefaults(1.5, 2, 10, 1, 0.5);
    return rvo_sim;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "rvo2");
    ros::NodeHandle n;
    sim = init_sim();
    vector<string> p_topics;
    ros::param::get("~position_topics", p_topics);
    cout << p_topics.size() << endl;
    PoseSubscriber pose_sub(n, sim, "/q0/ground_truth_to_tf/pose");
    pose_sub.start();
    ros::spin();
}
