/* Author: K.N. Chan
   Email: dauker@outlook.com
   Desc: Forward and inverse kinematics demo of AH3 robot
*/

#include "Scara.h"
#include <iostream>

int main(int argc, char* argv[])
{
  // Instantiate an object of the AH3 robot with link lengths of 225mm and 175mm
  scara::Scara ah3_robot;
  ah3_robot.SetLinkLength(225.0, 175.0);

  // Forward kinematics solution demo
  scara::JointAngles joint_angle = {
    joint_1_angle: -38.653,
    joint_2_angle: 88.345,
    joint_3_value: 82.497,
    joint_4_angle: -23.416
  };
  scara::Pose pose_converted = ah3_robot.ForwardKinematicsCalculator(joint_angle);
  std::cout << "[Joint Space]" << " "
            << "J1: " << joint_angle.joint_1_angle << "  "
            << "J2: " << joint_angle.joint_2_angle << "  "
            << "J3: " << joint_angle.joint_3_value << "  "
            << "J4: " << joint_angle.joint_4_angle << "  "
            << "\n"
            << "[Cartesian Space]" << " "
            << "x: " << pose_converted.x << "  "
            << "y: " << pose_converted.y << "  "
            << "z: " << pose_converted.z << "  "
            << "yaw: " << pose_converted.yaw << "  "
            << "pitch: " << pose_converted.pitch << "  "
            << "roll:" << pose_converted.roll << "  "
            << "\n"
            << std::endl;

  // Inverse kinematics solution demo(right handed)
  scara::Pose pose = {
    x: 300.255,
    y: -26.563,
    z: -82.497,
    roll: -150.555
  };
  scara::JointAngles joint_converted = ah3_robot.InverseKinematicsCalculator(pose, 0);
  std::cout << "[Cartesian Space]" << "  "
            << "x: " << pose.x << "  "
            << "y: " << pose.y << "  "
            << "z: " << pose.z << "  "
            << "yaw: " << pose.yaw << "  "
            << "pitch: " << pose.pitch << "  "
            << "roll :" << pose.roll << "  "
            << "\n"
            << "[Joint Space]" << " "
            << "J1: " << joint_converted.joint_1_angle << "  "
            << "J2: " << joint_converted.joint_2_angle << "  "
            << "J3: " << joint_converted.joint_3_value << "  "
            << "J4: " << joint_converted.joint_4_angle << "  "
            << std::endl;
}