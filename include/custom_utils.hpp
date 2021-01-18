#pragma once
#include "colors.hpp"
#include <iostream>

class Pose {
  public:
    double x;
    double y;
    double z;
    double roll;   // arround x.
    double pitch;  // arround y.
    double yaw;    // arround z.

    // Constructors
    Pose() : Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0){};

    Pose(const double x_, const double y_, const double z_, const double roll_,
         const double pitch_, const double yaw_)
        : x(x_), y(y_), z(z_), roll(roll_), pitch(pitch_), yaw(yaw_){};

    Pose(const Pose &pose)
        : Pose(pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw){};

    // Operators
    friend std::ostream &operator<<(std::ostream &stream, const Pose &pose);
    friend bool operator==(const Pose &pose1, const Pose &pose2);
    friend bool operator!=(const Pose &pose1, const Pose &pose2);
    friend Pose operator+(const Pose &pose1, const Pose &pose2);
};
