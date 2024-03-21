#include "dofbot_kinemarics.h"
#include <iostream>
using namespace KDL;
using namespace std;
Dofbot dofbot = Dofbot();
// rad to deg
const float RA2DE = 180.0f / M_PI;
// deg to rad
const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/src/moveit/urdf/dofbot.urdf";

/*
 * This is the code for the inverse kinematics solution of the manipulator, not involving the gripper.
 * 
 * Note: The joint angle obtained by the IK solution is only numerical data, so it may be out of bounds.
 * 
 */
int main(int argc, char **argv) {
    // gripper pose
    double Roll = -135;
    double Pitch = 0;
    double Yaw = 0;
    // find end-effector position
    double x = 0;
    double y = 5.5;
    double z = 17.375;
    // End-effector position (unit: m)
    double xyz[]{x, y, z};
    // End-effector orientation (unit: rad)
    double rpy[]{Roll , Pitch, Yaw };
    // Create output angular container
    vector<double> outjoints;
    // Create end-effector position container
    vector<double> targetXYZ;
    // Create end-effector orientation container
    vector<double> targetRPY;
    // End-effector position unit conversion, from cm to m
    for (int k = 0; k < 3; ++k) targetXYZ.push_back(xyz[k] / 100);
    // End-effector orientation unit conversion, from degree to radian
    for (int l = 0; l < 3; ++l) targetRPY.push_back(rpy[l] * DE2RA);
    // IK solve the angle of each joint to reach the target point
    dofbot.dofbot_getIK(urdf_file, targetXYZ, targetRPY, outjoints);
    // Print the IK solution result
    cout <<fixed<< "IK kinematics result : " << endl;
    for (int i = 0; i < 5; i++) cout << outjoints.at(i) * RA2DE + 90 << "\t";
    cout << endl;
    return 0;
}
