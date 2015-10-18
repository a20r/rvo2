
#ifndef RVO2_POSE_SUBSCRIBER_H
#define RVO2_POSE_SUBSCRIBER_H

#define EPS 0.001
#include "rvo2/PrefVelSubscriber.hpp"

using namespace std;

class PoseSubscriber {

    protected:
        RVO::RVOSimulator *sim;
        ros::NodeHandle *n;
        ros::Subscriber sub;
        PrefVelSubscriber *pv_sub;
        string topic;
        RVO::Vector3 pos, pref_vel;
        bool pos_set;
        int id;
        float time;

    public:
        ~PoseSubscriber() {};
        PoseSubscriber() {};
        PoseSubscriber(ros::NodeHandle *, RVO::RVOSimulator *, string,
                PrefVelSubscriber *);
        void start();
        void callback(geometry_msgs::PoseStamped ps);
        void set_pref_vel(RVO::Vector3);
        void set_pref_vel(geometry_msgs::Twist);
        int get_id();
};

#endif
