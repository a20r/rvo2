
#ifndef RVO2_PREF_VEL_SUBSCRIBER_H
#define RVO2_PREF_VEL_SUBSCRIBER_H

using namespace std;

class PrefVelSubscriber {

    protected:
        RVO::RVOSimulator *sim;
        ros::NodeHandle *n;
        ros::Subscriber sub;
        ros::Publisher pub;
        string pref_vel_topic, cmd_vel_topic;
        int id;

    public:
        ~PrefVelSubscriber() {};
        PrefVelSubscriber() {};
        PrefVelSubscriber(ros::NodeHandle *, RVO::RVOSimulator *,
                string, string, int);
        void start();
        void callback(geometry_msgs::Twist vel);
};

#endif
