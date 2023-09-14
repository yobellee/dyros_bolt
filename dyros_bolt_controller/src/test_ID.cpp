#include <iostream>
#include <vector>
#include <cmath>

struct Joint {
    double position; // Joint position (angle in radians)
    double velocity; // Joint velocity (angular velocity in rad/s)
    double acceleration; // Joint acceleration (angular acceleration in rad/s^2)
    double torque; // Joint torque
};

struct Link {
    double length; // Link length
    double mass; // Link mass
    double center_of_mass; // Distance from the joint to the center of mass of the link
};

class RoboticArm {
public:
    RoboticArm(const std::vector<double>& jointPositions) : joints_(jointPositions.size()), torques_(jointPositions.size()) {
        for (size_t i = 0; i < jointPositions.size(); ++i) {
            joints_[i].position = jointPositions[i];
            joints_[i].velocity = 0.0;
            joints_[i].acceleration = 0.0;
            torques_[i].torque = 0.0;
        }
    }

    void setJointVelocities(const std::vector<double>& jointVelocities) {
        for (size_t i = 0; i < jointVelocities.size() && i < joints_.size(); ++i) {
            joints_[i].velocity = jointVelocities[i];
        }
    }

    void setJointAccelerations(const std::vector<double>& jointAccelerations) {
        for (size_t i = 0; i < jointAccelerations.size() && i < joints_.size(); ++i) {
            joints_[i].acceleration = jointAccelerations[i];
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
        double inertia = link.mass * link.center_of_mass * link.center_of_mass;
        joint.torque = inertia * joint.acceleration;

        // Add gravity contribution
        const double g = 9.81; // acceleration due to gravity
        joint.torque += link.mass * g * link.center_of_mass * std::cos(joint.position);

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
    // Example setup: a 2D planar robotic arm with two revolute joints
    std::vector<double> jointPositions = {0.5, 1.2};
    RoboticArm arm(jointPositions);

    std::vector<double> jointVelocities = {0.1, 0.2};
    arm.setJointVelocities(jointVelocities);

    std::vector<double> jointAccelerations = {0.3, 0.1};
    arm.setJointAccelerations(jointAccelerations);

    std::vector<Link> links = { {0.5, 1.0, 0.25}, {1.0, 2.0, 0.5} };

    // Call the inverse dynamics function for the last joint (end effector)
    int endEffectorIndex = arm.getJoints().size() - 1;
    arm.calculateInverseDynamics(links, endEffectorIndex);

    // Display the computed joint torques
    const std::vector<Joint>& torques = arm.getTorques();
    for (const Joint& joint : torques) {
        std::cout << "Joint Torque: " << joint.torque << " Nm" << std::endl;
    }

    return 0;
}
