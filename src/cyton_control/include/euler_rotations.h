#pragma once

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>


class EulerRotations
{
public:
    // Quaternions struct
    struct Quaternions { double w, x, y, z; };

    // Eulear angles struct
    struct Euler{ double phi, theta, psi; };

    /** Euler angles to quaternions. Euler angles follow the post multiply
        sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
        and "phi" around x (roll) **/
    static Quaternions euler_to_quaternions(double phi, double
        theta, double psi);

    /** Quaternions to Euler Angles. Euler angles follow the post multiply
        sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
        and "phi" around x (roll) **/
    static Euler quaternions_to_euler(double w, double x, double y, 
        double z);

public:
    // Basic rotation matrix wrt x axis
    static Eigen::Matrix3d basic_rotation_x(double x);

    // Basic rotation matrix wrt y axis
    static Eigen::Matrix3d basic_rotation_y(double x);

    // Basic rotation matrix wrt z axis
    static Eigen::Matrix3d basic_rotation_z(double x);

    // Euler rotation matrix z-y'-x''
    static Eigen::Matrix3d rotation(double phi, double theta, double psi);
    static Eigen::Matrix3d rotation(Eigen::Vector3d euler_angles);
    static Eigen::Matrix3d rotation(std::vector<double> euler_angles);
    static Eigen::Matrix3d rotation(Euler euler_angles);

    // Euler angles derivative to body anglular velocity mapping (w = G * theta_dot)
    static Eigen::Matrix3d G(double phi, double theta, double psi);
    static Eigen::Matrix3d G(Eigen::Vector3d euler_angles);
    static Eigen::Matrix3d G(std::vector<double> euler_angles);
    static Eigen::Matrix3d G(Euler euler_angles);

    // Euler angles second derivative to body anglular acceleration mapping
    static Eigen::Matrix3d G_dot(Eigen::Vector3d euler_angles,
        Eigen::Vector3d euler_angles_dot);

    static Eigen::Matrix3d G_dot(std::vector<double> euler_angles,
        std::vector<double> euler_angles_dot);
};