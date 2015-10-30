
#ifndef RVO2_POSE_SUBSCRIBER_H
#define RVO2_POSE_SUBSCRIBER_H

#define EPS 0.0005

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
        RVO::Vector3 pos, vel;
        float time;
        bool pos_set;
        int id;

    public:
        ~PoseSubscriber() {};
        PoseSubscriber() {};
        PoseSubscriber(ros::NodeHandle *, RVO::RVOSimulator *, string,
                PrefVelSubscriber *, VelocitySubscriber *);
        void start();
        void callback(geometry_msgs::PoseStamped ps);
        RVO::Vector3 get_vel();
        RVO::Vector3 get_pos();
        bool is_pos_set();
        int get_id();
};

#endif
