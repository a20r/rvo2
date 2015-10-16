
#ifndef RVO2_UTILITY_H
#define RVO2_UTILITY_H

#include <RVO.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

inline RVO::Vector3 pose_to_vector(geometry_msgs::PoseStamped pose) {
    float x = pose.pose.position.x;
    float y = pose.pose.position.y;
    float z = pose.pose.position.z;
    return RVO::Vector3(x, y, z);
}

inline geometry_msgs::Twist vector_to_twist(RVO::Vector3 vec) {
    geometry_msgs::Twist twist;
    twist.linear.x = vec.x();
    twist.linear.y = vec.y();
    twist.linear.z = vec.z();
    return twist;
}

inline RVO::Vector3 twist_to_vector(geometry_msgs::Twist twist) {
    float x, y, z;
    x = twist.linear.x;
    y = twist.linear.y;
    z = twist.linear.z;
    return RVO::Vector3(x, y, z);
}

#endif
