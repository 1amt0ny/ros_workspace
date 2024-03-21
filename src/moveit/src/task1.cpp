#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>

using namespace std;

// Plan each trajectory
void multi_trajectory(
        moveit::planning_interface::MoveGroupInterface &dofbot,
        const vector<double> &pose,
        moveit_msgs::RobotTrajectory &trajectory) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const robot_state::JointModelGroup *joint_model_group;
    // obtain the start state
    moveit::core::RobotStatePtr start_state(dofbot.getCurrentState());
    joint_model_group = start_state->getJointModelGroup(dofbot.getName());
    dofbot.setJointValueTarget(pose);
    dofbot.plan(plan);
    start_state->setJointGroupPositions(joint_model_group, pose);
    dofbot.setStartState(*start_state);
    trajectory.joint_trajectory.joint_names = plan.trajectory_.joint_trajectory.joint_names;
    for (size_t j = 0; j < plan.trajectory_.joint_trajectory.points.size(); j++) {
        trajectory.joint_trajectory.points.push_back(plan.trajectory_.joint_trajectory.points[j]);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_avoidance_path");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);	// set thread
    spinner.start();			// open thread
    
    moveit_msgs::RobotTrajectory trajectory;
    // Initialize the arm motion planning group
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    moveit_visual_tools::MoveItVisualTools tool(dofbot.getPlanningFrame());
    tool.deleteAllMarkers();

    dofbot.allowReplanning(true);
        
    // Planning time (in seconds)
    dofbot.setPlanningTime(5);
    dofbot.setNumPlanningAttempts(10);
    
    // Set allowable target angle error tolerance
    dofbot.setGoalJointTolerance(0.01);
    dofbot.setGoalPositionTolerance(0.01);
    dofbot.setGoalOrientationTolerance(0.01);
    dofbot.setGoalTolerance(0.01);
    
    // Set allowable max vel and accel
    dofbot.setMaxVelocityScalingFactor(1.0);
    dofbot.setMaxAccelerationScalingFactor(1.0);
    
    // set arm initial position
    dofbot.setNamedTarget("up");
    dofbot.move();
    sleep(0.1);
    
    // Get the name of the framework the arm is planning
    string frame = dofbot.getPlanningFrame();
    // Create scene instance
    moveit::planning_interface::PlanningSceneInterface scene;
    
    //////////////// Add obstacles ////////////////
    vector<string> object_ids;
    scene.removeCollisionObjects(object_ids);
    // Create a detection object container
    vector<moveit_msgs::CollisionObject> objects;
    // Create Collision Detection Objects
    moveit_msgs::CollisionObject obj;
    // Set the ID of the obstacle
    obj.id = "obj";
    object_ids.push_back(obj.id);
    // the status of the obstacle
    obj.operation = obj.ADD;
    // Set the header information of the obstacle
    obj.header.frame_id = frame;
    shape_msgs::SolidPrimitive primitive;
    // Set the obstacle type
    primitive.type = primitive.BOX;
    // Set Obstacle Dimensions
    primitive.dimensions.resize(3);
    // Set the length, width and height of obstacles
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.1;
    obj.primitives.push_back(primitive);
    geometry_msgs::Pose pose;
    // Set the position information of the obstacle [x,y,z]
    pose.position.x = 0;
    pose.position.y = 0.2;
    pose.position.z = 0.15;
    // Set the orientation information of obstacles
    tf::Quaternion quaternion;
    // R,P,Y are in degrees
    double Roll = 0.0;
    double Pitch = 0.0;
    double Yaw = 90.0;
    // RPY to Quaternion
    quaternion.setRPY(Roll * M_PI / 180, Pitch * M_PI / 180, Yaw * M_PI / 180);
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    // Set the pose information of obstacles
    obj.primitive_poses.push_back(pose);
    objects.push_back(obj);
    
    //////////////////// Set the color of the obstacle ////////////////////
    // Create a color container for detected objects
    std::vector<moveit_msgs::ObjectColor> colors;
    // Create a color instance
    moveit_msgs::ObjectColor color;
    // Add the id that needs to set the color
    color.id = "obj";
    // Set RGBA value in range [0~1]
    color.color.r = 0;
    color.color.g = 1.0;
    color.color.b = 0;
    color.color.a = 0.5;
    colors.push_back(color);
    // Add these settings to the scene
    scene.applyCollisionObjects(objects, colors);
    // Set target point
    dofbot.setNamedTarget("down");
    // start moving
    dofbot.move();
    sleep(0.5);
    
    
    
    
    
    // Set three reachable target points
    vector<vector<double>> poses{
            {1.34,  -1.0,  -0.61, 0.2,   0},
            {0,     0,     0,     0,     0},
            {-1.16, -0.97, -0.81, -0.79, 3.14}
    };
    for (int i = 0; i < poses.size(); ++i) {
        multi_trajectory(dofbot, poses.at(i), trajectory);
    }

    // Trajectory merge
    moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
    robot_trajectory::RobotTrajectory rt(dofbot.getCurrentState()->getRobotModel(), "dofbot");
    rt.setRobotTrajectoryMsg(*dofbot.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 1, 1);
    rt.getRobotTrajectoryMsg(trajectory);
    joinedPlan.trajectory_ = trajectory;

    // show trajectory
    tool.publishTrajectoryLine(joinedPlan.trajectory_, dofbot.getCurrentState()->getJointModelGroup("dofbot"));
    tool.trigger();

    // Execute trajectory planning
    if (!dofbot.execute(joinedPlan)) {
        ROS_ERROR("Failed to execute plan");
        return false;
    }
    sleep(1);
    ROS_INFO("Finished");
    ros::shutdown();
    return 0;
}

