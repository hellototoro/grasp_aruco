#ifndef ARUCO_GRASP_H
#define ARUCO_GRASP_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>

class ArucoGrasp
{
public:
    ArucoGrasp(const std::string& planning_group);
    ~ArucoGrasp();

    bool getArucoMarkerPose(const std::string& marker_frame, geometry_msgs::Pose& pose);
    bool moveToPose(const geometry_msgs::Pose& target_pose);

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    moveit::planning_interface::MoveGroupInterface move_group_;
};

#endif // ARUCO_GRASP_H
