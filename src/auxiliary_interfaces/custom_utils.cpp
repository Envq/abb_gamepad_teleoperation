#include "custom_utils.hpp"


// Pose =================================================================================
std::ostream &operator<<(std::ostream &stream, const Pose &pose) {
    stream << colorize("Position:\t", Colors::BOLD) << std::endl;
    stream << "- x: " << pose.x << std::endl;
    stream << "- y: " << pose.y << std::endl;
    stream << "- z: " << pose.z << std::endl;
    stream << colorize("Orientation:\t", Colors::BOLD) << std::endl;
    stream << "- roll:  " << pose.roll << std::endl;
    stream << "- pitch: " << pose.pitch << std::endl;
    stream << "- yaw:   " << pose.yaw << std::endl << std::endl;
    return stream;
}

bool operator==(const Pose &pose1, const Pose &pose2) {
    if (pose1.x != pose2.x)
        return false;
    if (pose1.y != pose2.y)
        return false;
    if (pose1.z != pose2.z)
        return false;
    if (pose1.roll != pose2.roll)
        return false;
    if (pose1.pitch != pose2.pitch)
        return false;
    if (pose1.yaw != pose2.yaw)
        return false;
    return true;
}

bool operator!=(const Pose &pose1, const Pose &pose2) {
    return !(pose1 == pose2);
}

Pose operator+(const Pose &pose1, const Pose &pose2) {
    return Pose{pose1.x + pose2.x,         pose1.y + pose2.y,
                pose1.z + pose2.z,         pose1.roll + pose2.roll,
                pose1.pitch + pose2.pitch, pose1.yaw + pose2.yaw};
}
