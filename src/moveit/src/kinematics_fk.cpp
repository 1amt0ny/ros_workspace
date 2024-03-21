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
 * This is the code to find the forward kinematics solution of the manipulator, not involving the gripper.
 * 
 */
int main(int argc, char **argv) {
    // The current angular velocity of each joint
//    double joints[]{90, 90, 90, 90, 90};
    double joints[]{90, 135, 0, 0, 90};
    // Define the target joint angle container
    vector<double> initjoints;
    // Define the pose container
    vector<double> initpos;
    // Target joint angle unit conversion, from deg [0, 180] to rad [-1.57, 1.57]
    for (int i = 0; i < 5; ++i) initjoints.push_back((joints[i] - 90) * DE2RA);
    // Call the FK solution function to get the current pose
    dofbot.dofbot_getFK(urdf_file, initjoints, initpos);
    cout <<fixed<< "FK kinematics result : " << endl;
    cout << "Xcoordinate (cm)： " << initpos.at(0) * 100 << "\t"
         << "Ycoordinate (cm)： " << initpos.at(1) * 100 << "\t"
         << "Zcoordinate (cm)： " << initpos.at(2) * 100 << endl;
    cout << "Roll  (°)： " << initpos.at(3) * RA2DE << "\t"
         << "Pitch (°)： " << initpos.at(4) * RA2DE << "\t"
         << "Yaw   (°)： " << initpos.at(5) * RA2DE << endl;
    return 0;
}
