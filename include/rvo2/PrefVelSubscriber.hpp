
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
        float time;
        RVO::Vector3 pref_vel;

    public:
        ~PrefVelSubscriber() {};
        PrefVelSubscriber() {};
        PrefVelSubscriber(ros::NodeHandle *, RVO::RVOSimulator *,
                string, string);
        RVO::Vector3 get_pref_vel();
        void start();
        void set_id(int);
        ros::Publisher get_pub();
        int get_id();
        void callback(geometry_msgs::Twist);
};

#endif
