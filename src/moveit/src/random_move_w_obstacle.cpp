#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dofbot_random_obstacle_cpp");
    ros::NodeHandle n;
    // set thread
    ros::AsyncSpinner spinner(1);
    // open thread
    spinner.start();
    
    // Initialize the arm motion planning group
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    
    dofbot.allowReplanning(true);
    // (unit: second)
    dofbot.setPlanningTime(5);
    dofbot.setNumPlanningAttempts(10);
    // (unit: m)
    dofbot.setGoalPositionTolerance(0.01);
    // (unit: radian)
    dofbot.setGoalOrientationTolerance(0.01);
    // set maximum speed
    dofbot.setMaxVelocityScalingFactor(1.0);
    // set maximum acceleration
    dofbot.setMaxAccelerationScalingFactor(1.0);
    // set target point
    dofbot.setNamedTarget("up");
    // start moving
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
    while (true){
        // Set random target points
        dofbot.setRandomTarget();
        // start moving
        dofbot.move();
        sleep(0.5);
    }
    // blocking process
    ros::waitForShutdown();
    return 0;
}

