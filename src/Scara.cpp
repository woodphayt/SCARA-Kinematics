/* Author: K.N. Chan
   Email: dauker@outlook.com
*/

#include "Scara.h"
#include <cmath>

namespace scara
{
inline double Scara::Degree2Radian(const double &degree) {
  return degree * PI / 180;
}

inline double Scara::Radian2Degree(const double &radian) {
  return radian * 180 / PI;
}

void Scara::SetLinkLength(const double &link_1_length, const double &link_2_length) {
  link_1_length_ = link_1_length;
  link_2_length_ = link_2_length;
}

// Forward kinematics calculator
Pose Scara::ForwardKinematicsCalculator(const JointAngles &joint_angle) {
  // Calculate the projection of link_1 and link_2 on the x axis
  double link_1_x_projection = link_1_length_ * cos(Degree2Radian(joint_angle.joint_1_angle));
  double link_2_x_projection = link_2_length_ * cos(Degree2Radian(joint_angle.joint_1_angle + joint_angle.joint_2_angle));
  double x = link_1_x_projection + link_2_x_projection;

  // Calculate the projection of link_1 and link_2 on the y axis
  double link_1_y_projection = link_1_length_ * sin(Degree2Radian(joint_angle.joint_1_angle));
  double link_2_y_projection = link_2_length_ * sin(Degree2Radian(joint_angle.joint_1_angle + joint_angle.joint_2_angle));
  double y = link_1_y_projection + link_2_y_projection;

  double z = -joint_angle.joint_3_value;

  // Determine the left-haned and right-handed
  double roll_temp = joint_angle.joint_4_angle - joint_angle.joint_1_angle - joint_angle.joint_2_angle;
  double roll = (joint_angle.joint_2_angle < 0) ? roll_temp - 180 : roll_temp + 180;

  Pose pose;
  pose.x = x;
  pose.y = y;
  pose.z = z;
  pose.roll = roll;
  return pose;
}

// Inverse kinematics calculator
// If handcoor is set to 1, it returns right-handed coordinates
// If handcoor is set to 0, it returns left-handed coordinates
JointAngles Scara::InverseKinematicsCalculator(const Pose &pose, const int &handcoor) {
  double cosine_joint_2 = (pose.x * pose.x + pose.y * pose.y - link_1_length_ * link_1_length_ - link_2_length_ * link_2_length_) / (2 * link_1_length_ * link_2_length_);
  double sine_joint_2_absolute = sqrt(1 - (cosine_joint_2 * cosine_joint_2));
  double sine_joint_2 = (handcoor == 1) ? sine_joint_2_absolute : -sine_joint_2_absolute;
  double joint_2_angle = Radian2Degree(atan2(sine_joint_2, cosine_joint_2));

  double alpha = atan2(pose.y, pose.x);
  double beta = atan2(link_2_length_ * sine_joint_2, link_1_length_ + link_2_length_ * cosine_joint_2);
  double joint_1_angle = Radian2Degree(alpha - beta);

  double joint_3_value = -pose.z;

  double joinit_4_temp = pose.roll + joint_1_angle + joint_2_angle;
  double joint_4_angle = (handcoor == 1) ? joinit_4_temp - 180 : joinit_4_temp + 180;

  JointAngles joint_angle;
  joint_angle.joint_1_angle = joint_1_angle;
  joint_angle.joint_2_angle = joint_2_angle;
  joint_angle.joint_3_value = joint_3_value;
  joint_angle.joint_4_angle = joint_4_angle;
  return joint_angle;
}

} // namespace scara