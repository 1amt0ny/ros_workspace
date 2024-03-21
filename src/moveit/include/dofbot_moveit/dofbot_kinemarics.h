#ifndef ARM_MOVEIT_ARM_KINEMARICS_H
#define ARM_MOVEIT_ARM_KINEMARICS_H

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/segment.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <iostream>

using namespace KDL;
using namespace std;
class Dofbot{
public:
    /**
     * Forward kinematics calculation 
     * Obtain the current pose according to the revolute joint angle
     * @param urdf_file   model file path
     * @param joints      current joint angle
     * @param cartpos     current end-effector pose
     */
    bool dofbot_getFK(const char *urdf_file, vector<double> &joints, vector<double> &currentPos);

    /**
     * Inverse kinematics calculation 
     * Obtain the angle that each joint needs to rotate to reach the target pose
     * @param urdf_file      model file path
     * @param targetXYZ      target location
     * @param targetRPY      target pose
     * @param outjoints      target pose joint angle
     */
    bool dofbot_getIK(const char *urdf_file, vector<double> &targetXYZ, vector<double> &targetRPY, vector<double> &outjoints);
};

#endif //ARM_MOVEIT_ARM_KINEMARICS_H
