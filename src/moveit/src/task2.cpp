#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "moveit_revise_trajectory_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

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

    // return the arm to initial position
    dofbot.setNamedTarget("down");
    dofbot.move();

    // Set three reachable target points
    vector<vector<double>> poses{
    	{0.5, -0.5, 0.5, -0.5, 0},    // A moderately high, forward-reaching pose
    	{-0.5, 0.5, -0.5, 0.5, 0},    // A mirror pose of the first, reaching to the opposite side
    	{0.25, -0.25, 0, 0.25, -0.25} // A compact pose, more centered and slightly lower
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
