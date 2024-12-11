#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_grasp");
    ros::NodeHandle nh;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Wait for the transform to be available
    try {
        listener.waitForTransform("panda_link0", "aruco_marker_frame", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("panda_link0", "aruco_marker_frame", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::shutdown();
        return -1;
    }

    // Convert the transform to a PoseStamped message
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "panda_link0";
    target_pose.header.stamp = ros::Time::now();
    target_pose.pose.position.x = transform.getOrigin().x();
    target_pose.pose.position.y = transform.getOrigin().y();
    target_pose.pose.position.z = transform.getOrigin().z();
    target_pose.pose.orientation.x = transform.getRotation().x();
    target_pose.pose.orientation.y = transform.getRotation().y();
    target_pose.pose.orientation.z = transform.getRotation().z();
    target_pose.pose.orientation.w = transform.getRotation().w();

    std::cout << "Target pose: " << target_pose << std::endl;


    // // Initialize MoveIt
    // moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    // move_group.setPoseReferenceFrame("panda_link0");
    // move_group.setPoseTarget(target_pose);

    // // Plan and execute the motion
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // if (success) {
    //     move_group.move();
    //     ROS_INFO("Grasping action executed successfully.");
    // } else {
    //     ROS_ERROR("Failed to plan the grasping action.");
    // }

    ros::shutdown();
    return 0;
}
