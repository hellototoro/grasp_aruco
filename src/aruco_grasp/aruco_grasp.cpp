// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::Pose place_pose_;
constexpr char LOGNAME[] = "moveit_task_constructor_demo";

using namespace moveit::task_constructor;

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& psi);

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_place_task");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface psi;
    addCollisionObjects(psi);

    moveit::task_constructor::Task task;
    task.stages()->setName("pick_place_task");
    task.loadRobotModel();

    auto sampling_planner = std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>();
    sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
    sampling_planner->setMaxVelocityScalingFactor(0.05);

    auto interpolation_planner = std::make_shared<moveit::task_constructor::solvers::JointInterpolationPlanner>();

    //create a cartesian planner
    auto cartesian_planner = std::make_shared<moveit::task_constructor::solvers::CartesianPath>();
    cartesian_planner->setMaxVelocityScalingFactor(0.05);
    cartesian_planner->setMaxAccelerationScalingFactor(0.05);
    cartesian_planner->setStepSize(.01);

    const std::string arm_group_name_ = "panda_arm";
    const std::string eef_name_ = "panda_hand";
    const std::string hand_group_name_ = "panda_hand";
    const std::string hand_frame_ = "panda_link8";
    const std::string object_name_ = "object";
    const std::string object = object_name_;

    const std::string hand_open_pose_ = "open";
    const std::string hand_close_pose_ = "close";
    const std::string arm_home_pose_ = "ready";

    const std::string world_frame_ = "panda_link0";
    const std::string table_reference_frame_ = "panda_link0";
    const std::string object_reference_frame_ = "panda_link0";
    const std::string surface_link_ = "table1";
    std::vector<std::string> support_surfaces_;
    support_surfaces_ = { surface_link_ };

    const double approach_object_min_dist_ = 0.1;
    const double approach_object_max_dist_ = 0.15;
    const double lift_object_min_dist_ = 0.01;
    const double lift_object_max_dist_ = 0.1;
    const double place_surface_offset_ = 0.0001;

    Eigen::Isometry3d grasp_frame_transform_ = Eigen::Isometry3d::Identity();
    grasp_frame_transform_.translation() = Eigen::Vector3d(0, 0, 0.1);
    grasp_frame_transform_.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
    // grasp_frame_transform_.rotate(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));

    task.setProperty("group", arm_group_name_);
    task.setProperty("eef", eef_name_);
    task.setProperty("hand", hand_group_name_);
    task.setProperty("hand_grasping_frame", hand_frame_);
    task.setProperty("ik_frame", hand_frame_);

    //current state
    {
        auto current_state = std::make_unique<stages::CurrentState>("current state");

        // Verify that object is not attached
        auto applicability_filter =
            std::make_unique<stages::PredicateFilter>("applicability test", std::move(current_state));
        applicability_filter->setPredicate([object](const SolutionBase& s, std::string& comment) {
            if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
                comment = "object with id '" + object + "' is already attached and cannot be picked";
                return false;
            }
            return true;
        });
        task.add(std::move(applicability_filter));
    }
    /****************************************************
     *                                                  *
     *               Open Hand                          *
     *                                                  *
     ***************************************************/
    Stage* initial_state_ptr = nullptr;
    {
        auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);//interpolation_planner
        stage->setGroup(hand_group_name_);
        stage->setGoal(hand_open_pose_);
        initial_state_ptr = stage.get();  // remember start state for monitoring grasp pose generator
        task.add(std::move(stage));
    }

    /****************************************************
     *                                                  *
     *               Move to Pick                       *
     *                                                  *
     ***************************************************/
    // Connect initial open-hand state with pre-grasp pose defined in the following
    {
        auto stage = std::make_unique<stages::Connect>(
            "move to pick", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(Stage::PARENT);
        task.add(std::move(stage));
    }

    /****************************************************
     *                                                  *
     *               Pick Object                        *
     *                                                  *
     ***************************************************/
    Stage* pick_stage_ptr = nullptr;
    {
        // A SerialContainer combines several sub-stages, here for picking the object
        auto grasp = std::make_unique<SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), { "eef", "hand", "group", "ik_frame" });
        grasp->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group", "ik_frame" });

        /****************************************************
  ---- *               Approach Object                    *
         ***************************************************/
        {
            // Move the eef link forward along its z-axis by an amount within the given min-max range
            auto stage = std::make_unique<stages::MoveRelative>("approach object", cartesian_planner);
            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", hand_frame_);  // link to perform IK for
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });  // inherit group from parent stage
            stage->setMinMaxDistance(approach_object_min_dist_, approach_object_max_dist_);

            // Set hand forward direction
            geometry_msgs::Vector3Stamped vec;
            vec.header.frame_id = hand_frame_;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        /****************************************************
  ---- *               Generate Grasp Pose                *
         ***************************************************/
        {
            // Sample grasp pose candidates in angle increments around the z-axis of the object
            auto stage = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
            stage->properties().configureInitFrom(Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setPreGraspPose(hand_open_pose_);
            stage->setObject(object);  // object to sample grasps for
            stage->setAngleDelta(M_PI / 12);
            stage->setMonitoredStage(initial_state_ptr);  // hook into successful initial-phase solutions

            // Compute IK for sampled grasp poses
            auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(8);  // limit number of solutions
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);  // define virtual frame to reach the target_pose
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });  // inherit properties from parent
            wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                                    { "target_pose" });  // inherit property from child solution
            grasp->insert(std::move(wrapper));
        }

        /****************************************************
  ---- *               Allow Collision (hand object)   *
         ***************************************************/
        {
            // Modify planning scene (w/o altering the robot's pose) to allow touching the object for picking
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (hand,object)");
            stage->allowCollisions(
                object, task.getRobotModel()->getJointModelGroup(hand_group_name_)->getLinkModelNamesWithCollisionGeometry(),
                true);
            grasp->insert(std::move(stage));
        }

        /****************************************************
  ---- *               Close Hand                      *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);//interpolation_planner
            stage->setGroup(hand_group_name_);
            // stage->setGoal(hand_close_pose_);
            stage->setGoal({{"panda_finger_joint1", 0.033}, {"panda_finger_joint2", 0.033}});
            // stage->properties().set("tolerance", 0.01);  // 设置误差范围（允许+/- 0.002m的偏差）
            grasp->insert(std::move(stage));
        }

        /****************************************************
  .... *               Attach Object                      *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
            stage->attachObject(object, hand_frame_);  // attach object to hand_frame_
            grasp->insert(std::move(stage));
        }

        /****************************************************
  .... *               Allow collision (object support)   *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("allow collision (object,support)");
            stage->allowCollisions({ object }, support_surfaces_, true);
            grasp->insert(std::move(stage));
        }

        /****************************************************
  .... *               Lift object                        *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
            stage->setIKFrame(hand_frame_);
            stage->properties().set("marker_ns", "lift_object");

            // Set upward direction
            geometry_msgs::Vector3Stamped vec;
            vec.header.frame_id = world_frame_;
            vec.vector.z = 1.0;
            stage->setDirection(vec);
            grasp->insert(std::move(stage));
        }

        /****************************************************
  .... *               Forbid collision (object support)  *
         ***************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (object,support)");
            stage->allowCollisions({ object }, support_surfaces_, false);
            grasp->insert(std::move(stage));
        }

        pick_stage_ptr = grasp.get();  // remember for monitoring place pose generator

        // Add grasp container to task
        task.add(std::move(grasp));
    }

    /******************************************************
     *                                                    *
     *          Move to Place                             *
     *                                                    *
     *****************************************************/
    {
        // Connect the grasped state to the pre-place state, i.e. realize the object transport
        auto stage = std::make_unique<stages::Connect>(
            "move to place", stages::Connect::GroupPlannerVector{ { arm_group_name_, sampling_planner } });
        stage->setTimeout(5.0);
        stage->properties().configureInitFrom(Stage::PARENT);
        task.add(std::move(stage));
    }

    /******************************************************
     *                                                    *
     *          Place Object                              *
     *                                                    *
     *****************************************************/
    // All placing sub-stages are collected within a serial container again
    {
        auto place = std::make_unique<SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), { "eef", "hand", "group" });
        place->properties().configureInitFrom(Stage::PARENT, { "eef", "hand", "group" });

        /******************************************************
  ---- *          Lower Object                              *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
            stage->properties().set("marker_ns", "lower_object");
            stage->properties().set("link", hand_frame_);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(.03, .13);

            // Set downward direction
            geometry_msgs::Vector3Stamped vec;
            vec.header.frame_id = world_frame_;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        /******************************************************
  ---- *          Generate Place Pose                       *
         *****************************************************/
        {
            // Generate Place Pose
            auto stage = std::make_unique<stages::GeneratePlacePose>("generate place pose");
            stage->properties().configureInitFrom(Stage::PARENT, { "ik_frame" });
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject(object);

            // Set target pose
            geometry_msgs::PoseStamped p;
            p.header.frame_id = object_reference_frame_;
            p.pose = place_pose_;
            p.pose.position.z += 0.5 * 0.03 + place_surface_offset_; //0.03是方块的高
            stage->setPose(p);
            stage->setMonitoredStage(pick_stage_ptr);  // hook into successful pick solutions

            // Compute IK
            auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
            wrapper->setMaxIKSolutions(2);
            wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
            wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
            wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
            place->insert(std::move(wrapper));
        }

        /******************************************************
  ---- *          Open Hand                              *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);//interpolation_planner
            stage->setGroup(hand_group_name_);
            stage->setGoal(hand_open_pose_);
            place->insert(std::move(stage));
        }

        /******************************************************
  ---- *          Forbid collision (hand, object)        *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("forbid collision (hand,object)");
            stage->allowCollisions(object_name_, *task.getRobotModel()->getJointModelGroup(hand_group_name_), false);
            place->insert(std::move(stage));
        }

        /******************************************************
  ---- *          Detach Object                             *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
            stage->detachObject(object_name_, hand_frame_);
            place->insert(std::move(stage));
        }

        /******************************************************
  ---- *          Retreat Motion                            *
         *****************************************************/
        {
            auto stage = std::make_unique<stages::MoveRelative>("retreat after place", cartesian_planner);
            stage->properties().configureInitFrom(Stage::PARENT, { "group" });
            stage->setMinMaxDistance(.12, .25);
            stage->setIKFrame(hand_frame_);
            stage->properties().set("marker_ns", "retreat");
            geometry_msgs::Vector3Stamped vec;
            vec.header.frame_id = hand_frame_;
            vec.vector.z = -1.0;
            stage->setDirection(vec);
            place->insert(std::move(stage));
        }

        // Add place container to task
        task.add(std::move(place));
    }

    /******************************************************
     *                                                    *
     *          Move to Home                              *
     *                                                    *
     *****************************************************/
    {
        auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
        stage->properties().configureInitFrom(Stage::PARENT, { "group" });
        stage->setGoal(arm_home_pose_);
        stage->restrictDirection(stages::MoveTo::FORWARD);
        task.add(std::move(stage));
    }

    // prepare Task structure for planning
    try {
        task.init();
        if (task.plan(10)) {
            ROS_INFO_NAMED(LOGNAME, "Planning succeded");
            if (true) {
                moveit_msgs::MoveItErrorCodes execute_result;
                execute_result = task.execute(*task.solutions().front());
                if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
                    ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_result.val);
                }
                ROS_INFO_NAMED(LOGNAME, "Execution complete");
            } else {
                ROS_INFO_NAMED(LOGNAME, "Execution disabled");
            }
        } else {
            ROS_INFO_NAMED(LOGNAME, "Planning failed");
        }
    } catch (InitStageException& e) {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
        return 0;
    }

    // Keep introspection alive
    ros::waitForShutdown();
    return 0;
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& psi)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;

    // Wait for the transform to be available
    try {
        listener.waitForTransform("panda_link0", "aruco_marker_frame", ros::Time(0), ros::Duration(1000.0));
        listener.lookupTransform("panda_link0", "aruco_marker_frame", ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::shutdown();
        exit(1);
    }
    geometry_msgs::Pose target_pose;
    target_pose.position.x = transform.getOrigin().x();
    target_pose.position.y = transform.getOrigin().y();
    target_pose.position.z = transform.getOrigin().z() - 0.03/2;
    // target_pose.orientation.x = transform.getRotation().x();
    // target_pose.orientation.y = transform.getRotation().y();
    // target_pose.orientation.z = transform.getRotation().z();
    // target_pose.orientation.w = transform.getRotation().w();

    // Adjust the orientation to make the z-axis perpendicular to the ground
    tf::Quaternion q = transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    q.setRPY(0, 0, yaw);  // Set roll and pitch to 0 to make z-axis perpendicular to the ground
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(2);

    // Add the first table where the cube will originally be keptask.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.8;
    collision_objects[0].primitives[0].dimensions[1] = 0.8;
    collision_objects[0].primitives[0].dimensions[2] = 0.1;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0] = target_pose;
    collision_objects[0].primitive_poses[0].position.z -= 0.03/2 + 0.1/2;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[1].header.frame_id = "panda_link0";
    collision_objects[1].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.06;
    collision_objects[1].primitives[0].dimensions[1] = 0.06;
    collision_objects[1].primitives[0].dimensions[2] = 0.03;

    /* Define the pose of the objectask. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0] = target_pose;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    place_pose_ = target_pose;
    place_pose_.position.x -= 0.2;
    place_pose_.position.y -= 0.2;

    psi.applyCollisionObjects(collision_objects);
}
