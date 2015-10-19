
#ifndef RVO2_VEL_SUBSCRIBER_H
#define RVO2_VEL_SUBSCRIBER_H

using namespace std;

class VelocitySubscriber {

    protected:
        RVO::RVOSimulator *sim;
        ros::NodeHandle *n;
        ros::Subscriber sub;
        string vel_topic;
        int id;

    public:
        ~VelocitySubscriber() {};
        VelocitySubscriber() {};
        VelocitySubscriber(ros::NodeHandle *, RVO::RVOSimulator *, string);
        void start();
        void set_id(int);
        void callback(geometry_msgs::Vector3Stamped);
};

#endif
