
#ifndef RVO2_POSE_SUBSCRIBER_H
#define RVO2_POSE_SUBSCRIBER_H

#define EPS 0.001

#include "rvo2/PrefVelSubscriber.hpp"
#include "rvo2/VelocitySubscriber.hpp"

using namespace std;

class PoseSubscriber {

    protected:
        RVO::RVOSimulator *sim;
        ros::NodeHandle *n;
        ros::Subscriber sub;
        PrefVelSubscriber *pv_sub;
        VelocitySubscriber *v_sub;
        string topic;
        RVO::Vector3 pos, pref_vel;
        bool pos_set;
        int id;

    public:
        ~PoseSubscriber() {};
        PoseSubscriber() {};
        PoseSubscriber(ros::NodeHandle *, RVO::RVOSimulator *, string,
                PrefVelSubscriber *, VelocitySubscriber *);
        void start();
        void callback(geometry_msgs::PoseStamped ps);
        int get_id();
};

#endif
