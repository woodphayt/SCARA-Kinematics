/* Author: K.N. Chan
   Email: dauker@outlook.com
   Desc: A class of scara robot model, including its links length and forward
         and inverse kinematics solving algorithms
*/

#ifndef SCARA_H
#define SCARA_H

#include <vector>

#define PI 3.14159265

namespace scara {
// Data structure of robotic position
typedef struct {
  double x;
  double y;
  double z;
  double roll;
  double pitch = 180;
  double yaw = 0;
} Pose;

// Data structure of joint angles
typedef struct {
  double joint_1_angle;
  double joint_2_angle;
  double joint_3_value;
  double joint_4_angle;
} JointAngles;

class Scara
{
public:
  // Set the length of link_1 and link_2 in millimeters
  void SetLinkLength(const double &link_1_length, const double &link_2_length);

  // Convert degree to radians
  double Degree2Radian(const double &degree);

  // Convert radians to degree
  double Radian2Degree(const double &radian);

  // Forward kinematics calculator
  Pose ForwardKinematicsCalculator(const JointAngles &joint_angle);

  // Inverse kinematics calculator
  // If handcoor is set to 1, it returns right-handed coordinates
  // If handcoor is set to 0, it returns left-handed coordinates
  JointAngles InverseKinematicsCalculator(const Pose &pose, const int &handcoor);

private:
  double link_1_length_;
  double link_2_length_;
};

} // namespace scara

#endif