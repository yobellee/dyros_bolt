#include <iostream>
#include <vector>
#include <cmath>

struct Joint {
    double positionX; // Joint position (angle around X-axis in radians)
    double positionY; // Joint position (angle around Y-axis in radians)
    double positionZ; // Joint position (angle around Z-axis in radians)
    double velocityX; // Joint velocity (angular velocity around X-axis in rad/s)
    double velocityY; // Joint velocity (angular velocity around Y-axis in rad/s)
    double velocityZ; // Joint velocity (angular velocity around Z-axis in rad/s)
    double accelerationX; // Joint acceleration (angular acceleration around X-axis in rad/s^2)
    double accelerationY; // Joint acceleration (angular acceleration around Y-axis in rad/s^2)
    double accelerationZ; // Joint acceleration (angular acceleration around Z-axis in rad/s^2)
    double torqueX; // Joint torque around X-axis
    double torqueY; // Joint torque around Y-axis
    double torqueZ; // Joint torque around Z-axis
};

struct Link {
    double lengthX; // Link length in X-axis direction
    double lengthY; // Link length in Y-axis direction
    double lengthZ; // Link length in Z-axis direction
    double mass; // Link mass
    double center_of_massX; // Distance from the joint to the center of mass in X-axis direction
    double center_of_massY; // Distance from the joint to the center of mass in Y-axis direction
    double center_of_massZ; // Distance from the joint to the center of mass in Z-axis direction
};

class RoboticArm {
public:
    RoboticArm(const std::vector<double>& jointPositions) : joints_(jointPositions.size()), torques_(jointPositions.size()) {
        for (size_t i = 0; i < jointPositions.size(); ++i) {
            joints_[i].positionX = jointPositions[i];
            joints_[i].positionY = 0.0;
            joints_[i].positionZ = 0.0;
            joints_[i].velocityX = 0.0;
            joints_[i].velocityY = 0.0;
            joints_[i].velocityZ = 0.0;
            joints_[i].accelerationX = 0.0;
            joints_[i].accelerationY = 0.0;
            joints_[i].accelerationZ = 0.0;
            torques_[i].torqueX = 0.0;
            torques_[i].torqueY = 0.0;
            torques_[i].torqueZ = 0.0;
        }
    }

    void setJointAccelerations(const std::vector<double>& jointAccelerations) {
        for (size_t i = 0; i < jointAccelerations.size() && i < joints_.size(); ++i) {
            joints_[i].accelerationX = jointAccelerations[i];
        }
    }

    void calculateInverseDynamics(const std::vector<Link>& links, int linkIndex) {
        if (linkIndex < 0) {
            return; // Base case, we reached the end of the arm
        }

        // Retrieve current joint and link information
        Joint& joint = joints_[linkIndex];
        const Link& link = links[linkIndex];

        // Calculate joint torques using recursive Newton-Euler algorithm
        double inertiaX = link.mass * link.center_of_massY * link.center_of_massY + link.mass * link.center_of_massZ * link.center_of_massZ;
        double inertiaY = link.mass * link.center_of_massX * link.center_of_massX + link.mass * link.center_of_massZ * link.center_of_massZ;
        double inertiaZ = link.mass * link.center_of_massX * link.center_of_massX + link.mass * link.center_of_massY * link.center_of_massY;

        joint.torqueX = inertiaX * joint.accelerationX;
        joint.torqueY = inertiaY * joint.accelerationY;
        joint.torqueZ = inertiaZ * joint.accelerationZ;

        // Proceed to the previous link (if available)
        calculateInverseDynamics(links, linkIndex - 1);
    }

    const std::vector<Joint>& getJoints() const {
        return joints_;
    }

    const std::vector<Joint>& getTorques() const {
        return torques_;
    }

private:
    std::vector<Joint> joints_;
    std::vector<Joint> torques_;
};

int main() {
    // Example setup: a 3D robotic arm with three revolute joints
    std::vector<double> jointPositions = {0.5, 1.2, 0.7};
    RoboticArm arm(jointPositions);

    std::vector<double> jointAccelerations = {0.3, 0.1, 0.2};
    arm.setJointAccelerations(jointAccelerations);

    std::vector<Link> links = { {0.5, 0.3, 0.2, 1.0, 0.1, 0.15, 0.2},
                                {1.0, 0.2, 0.5, 2.0, 0.2, 0.3, 0.4},
                                {0.8, 0.5, 1.0, 1.5, 0.2, 0.4, 0.6} };

    // Call the inverse dynamics function for the last joint (end effector)
    int endEffectorIndex = arm.getJoints().size() - 1;
    arm.calculateInverseDynamics(links, endEffectorIndex);

    // Display the computed joint torques
    const std::vector<Joint>& torques = arm.getTorques();
    for (const Joint& joint : torques) {
        std::cout << "Joint Torques (X, Y, Z): " << joint.torqueX << " Nm, "
                                                << joint.torqueY << " Nm, "
                                                << joint.torqueZ << " Nm" << std::endl;
    }

    return 0;
}
