#include <iostream>
#include <vector>
#include <cmath>

#define g 9.81

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

// Function to calculate inverse kinematics for a 3D robotic arm with three revolute joints
std::vector<Joint> inverseKinematics(const std::vector<Link>& links, const std::vector<double>& desiredPositions) {
    std::vector<Joint> joints(3);

    // Assuming each joint moves around one axis (X, Y, Z), and the end effector is specified in XYZ coordinates.
    joints[0].positionX = desiredPositions[0];
    joints[1].positionY = desiredPositions[1];
    joints[2].positionZ = desiredPositions[2];

    return joints;
}

// Function to calculate inverse dynamics for a 3D robotic arm with three revolute joints
std::vector<Joint> inverseDynamics(const std::vector<Link>& links, const std::vector<Joint>& joints, const std::vector<double>& desiredAccelerations) {
    std::vector<Joint> torques(3);

    // Assuming simple dynamics model, we consider only joint torques, neglecting gravity and other external forces.
    for (size_t i = 0; i < links.size(); ++i) {
        double gravitational_torque = -links[i].mass * g * links[i].center_of_massZ;
        torques[i].torqueX = links[i].mass * links[i].center_of_massY * links[i].center_of_massZ * desiredAccelerations[i] + gravitational_torque;
        torques[i].torqueY = links[i].mass * links[i].center_of_massX * links[i].center_of_massZ * desiredAccelerations[i] + gravitational_torque;
        torques[i].torqueZ = links[i].mass * links[i].center_of_massX * links[i].center_of_massY * desiredAccelerations[i] + gravitational_torque;
    }
    
    return torques;
}

int main() {
    // Example setup: a 3D robotic arm with three revolute joints
    std::vector<Link> links = { {0.5, 0.3, 0.2, 1.0, 0.1, 0.15, 0.2},
                                {1.0, 0.2, 0.5, 2.0, 0.2, 0.3, 0.4},
                                {0.8, 0.5, 1.0, 1.5, 0.2, 0.4, 0.6} };

    // Desired trajectory for end effector
    std::vector<double> desiredPositions = {0.8, 1.2, 0.5}; // Desired end effector positions (angles in radians)
    std::vector<double> desiredVelocities = {0.0, 0.0, 0.0}; // Desired end effector velocities (angular velocities in rad/s)
    std::vector<double> desiredAccelerations = {0.0, 0.0, 0.0}; // Desired end effector accelerations (angular accelerations in rad/s^2)

    // Calculate inverse kinematics
    std::vector<Joint> joints = inverseKinematics(links, desiredPositions);

    // Set joint velocities and accelerations based on desired trajectory (for simplicity, assuming constant velocity and acceleration)
    for (size_t i = 0; i < joints.size(); ++i) {
        joints[i].velocityX = desiredVelocities[i];
        joints[i].velocityY = desiredVelocities[i];
        joints[i].velocityZ = desiredVelocities[i];
        joints[i].accelerationX = desiredAccelerations[i];
        joints[i].accelerationY = desiredAccelerations[i];
        joints[i].accelerationZ = desiredAccelerations[i];
    }

    // Calculate inverse dynamics
    std::vector<Joint> torques = inverseDynamics(links, joints, desiredAccelerations);

    // Display the computed joint torques
    for (size_t i = 0; i < torques.size(); ++i) {
        std::cout << "Joint " << i + 1 << " Torques (X, Y, Z): " << torques[i].torqueX << " Nm, "
                                                                << torques[i].torqueY << " Nm, "
                                                                << torques[i].torqueZ << " Nm" << std::endl;
    }

    return 0;
}
