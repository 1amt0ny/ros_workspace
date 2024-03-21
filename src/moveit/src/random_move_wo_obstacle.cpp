#include <iostream>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "dofbot_random_move_cpp");
    ros::NodeHandle n;
    // set thread
    ros::AsyncSpinner spinner(1);
    // open thread
    spinner.start();
    
    // Initialize the robotic arm motion planning group
    moveit::planning_interface::MoveGroupInterface dofbot("dofbot");
    
    
    // set target point
//    dofbot.setNamedTarget("down");
//    // start moving
//    dofbot.move();
//    dofbot.asyncMove();
//    sleep(0.1);
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
