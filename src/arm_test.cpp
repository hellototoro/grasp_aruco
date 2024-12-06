#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_test");
    ros::NodeHandle node_handle;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Wait for the transform to be available
    try {
        listener.waitForTransform("panda_link0", "aruco_marker_frame", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("panda_link0", "aruco_marker_frame", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::shutdown();
        return -1;
    }

    static const std::string PLANNING_GROUP = "panda_arm";
    static const std::string ROBOT_DESCRIPTION = "panda/robot_description";

    // moveit::planning_interface::MoveGroupInterface move_group_interface(moveit::planning_interface::MoveGroupInterface::Options{PLANNING_GROUP, ROBOT_DESCRIPTION});
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    /* RViz可视化 */
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.;
    visual_tools.publishText(text_pose, "grasp aruco marker", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* 获取基本信息 */
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = transform.getOrigin().x();
    target_pose1.position.y = transform.getOrigin().y();
    target_pose1.position.z = transform.getOrigin().z();// - 0.03/2
    target_pose1.orientation.x = transform.getRotation().x();
    target_pose1.orientation.y = transform.getRotation().y();
    target_pose1.orientation.z = transform.getRotation().z();
    target_pose1.orientation.w = transform.getRotation().w();

    move_group_interface.setPoseTarget(target_pose1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit_msgs::CollisionObject object_to_attach;
    object_to_attach.id = "aruco_marker";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.06;
    primitive.dimensions[primitive.BOX_Y] = 0.06;
    primitive.dimensions[primitive.BOX_Z] = 0.03;

    object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
    geometry_msgs::Pose grab_pose;
    grab_pose.position.x = transform.getOrigin().x();
    grab_pose.position.y = transform.getOrigin().y();
    grab_pose.position.z = transform.getOrigin().z();// - 0.03/2
    grab_pose.orientation.x = transform.getRotation().x();
    grab_pose.orientation.y = transform.getRotation().y();
    grab_pose.orientation.z = transform.getRotation().z();
    grab_pose.orientation.w = transform.getRotation().w();

    object_to_attach.primitives.push_back(primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    move_group_interface.attachObject(object_to_attach.id, "panda_hand", { "panda_leftfinger", "panda_rightfinger" });

    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

    // Replan, but now with the object in hand.
    move_group_interface.setStartStateToCurrentState();
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (move around cuboid with cylinder) %s", success ? "" : "FAILED");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the plan is complete");

    // Now, let's detach the cylinder from the robot's gripper.
    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
    move_group_interface.detachObject(object_to_attach.id);

    // Show text in RViz of status
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Object detached from robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the new object is detached from the robot");

    // Now, let's remove the objects from the world.
    ROS_INFO_NAMED("tutorial", "Remove the objects from the world");
    std::vector<std::string> object_ids;
    // object_ids.push_back(collision_object.id);
    object_ids.push_back(object_to_attach.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    // Show text in RViz of status
    visual_tools.publishText(text_pose, "Objects removed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    /* Wait for MoveGroup to receive and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disappears");

    ros::shutdown();
    return 0;
}
