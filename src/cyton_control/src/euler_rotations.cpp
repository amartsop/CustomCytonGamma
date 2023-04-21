#include "../include/euler_rotations.h"


// Basic rotation matrix wrt x axis
Eigen::Matrix3d EulerRotations::basic_rotation_x(double x)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    m << 1.0f, 0.0f, 0.0f, 
        0.0f, cos(x), -sin(x), 
        0.0f, sin(x), cos(x);
    return m;
}


// Basic rotation matrix wrt y axis
Eigen::Matrix3d EulerRotations::basic_rotation_y(double x)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    m << cos(x), 0.0f, sin(x), 
        0.0f, 1.0f, 0.0f, 
        -sin(x), 0.0f, cos(x);
    return m;
}


// Basic rotation matrix wrt z axis
Eigen::Matrix3d EulerRotations::basic_rotation_z(double x)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    m << cos(x), -sin(x), 0.0f, 
        sin(x), cos(x), 0.0f, 
        0.0f, 0.0f, 1.0f;
    return m;
}


// Euler rotation matrix z-y'-x''
Eigen::Matrix3d EulerRotations::rotation(double phi, double theta, double psi)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    // Basic rotations
    Eigen::Matrix3d rotx =  basic_rotation_x(phi);
    Eigen::Matrix3d roty = basic_rotation_y(theta);
    Eigen::Matrix3d rotz = basic_rotation_z(psi);

    // Total rotation matrix
    m = rotz * roty * rotx;

    return m;
}


// Euler rotation matrix z-y'-x''
Eigen::Matrix3d EulerRotations::rotation(Eigen::Vector3d euler_angles)
{
    return rotation(euler_angles(0), euler_angles(1), euler_angles(2));
}


// Euler rotation matrix z-y'-x''
Eigen::Matrix3d EulerRotations::rotation(std::vector<double> euler_angles)
{
    return rotation(euler_angles[0], euler_angles[1], euler_angles[2]);
}

// Euler rotation matrix z-y'-x''
Eigen::Matrix3d EulerRotations::rotation(Euler euler_angles)
{
    return rotation(euler_angles.phi, euler_angles.theta, euler_angles.psi);
}


// Body anglular velocity to euler angles derivative mapping (w = G * theta_dot)
Eigen::Matrix3d EulerRotations::G(double phi, double theta, double psi)
{
    // Matrix initialization
    Eigen::Matrix3d m;

    m << 1.0, 0.0, -sin(theta), 
        0.0, cos(phi), cos(theta) * sin(phi), 
        0.0, -sin(phi), cos(phi) * cos(theta);

    return m;
}


// Body anglular velocity to euler angles derivative mapping (w = G * theta_dot)
Eigen::Matrix3d EulerRotations::G(Eigen::Vector3d euler_angles)
{
    return G(euler_angles(0), euler_angles(1), euler_angles(2));
}


// Body anglular velocity to euler angles derivative mapping (w = G * theta_dot)
Eigen::Matrix3d EulerRotations::G(std::vector<double> euler_angles)
{
    return G(euler_angles[0], euler_angles[1], euler_angles[2]);
}


// Body anglular velocity to euler angles derivative mapping (w = G * theta_dot)
Eigen::Matrix3d EulerRotations::G(Euler euler_angles)
{
    return G(euler_angles.phi, euler_angles.theta, euler_angles.psi);
}


// Euler angles second derivative to body anglular acceleration mapping
Eigen::Matrix3d EulerRotations::G_dot(Eigen::Vector3d euler_angles,
    Eigen::Vector3d euler_angles_dot)
{
    double phi = euler_angles(0);
    double theta = euler_angles(1);

    double phi_dot = euler_angles_dot(0);
    double theta_dot = euler_angles_dot(1);

    // Matrix initialization
    Eigen::Matrix3d m;

    m << 0.0, 0.0, - cos(theta) * theta_dot,
        0.0, - sin(phi) * phi_dot, cos(theta) * cos(phi) * phi_dot - 
        sin(theta) * sin(phi) * theta_dot,
        0.0, - cos(phi) * phi_dot, -sin(phi) * cos(theta) * phi_dot - 
        cos(phi) * sin(theta) * theta_dot;

    return m;
}


// Euler angles second derivative to body anglular acceleration mapping
Eigen::Matrix3d EulerRotations::G_dot(std::vector<double> euler_angles,
        std::vector<double> euler_angles_dot)
{
    double phi = euler_angles[0];
    double theta = euler_angles[1];

    double phi_dot = euler_angles_dot[0];
    double theta_dot = euler_angles_dot[1];

    // Matrix initialization
    Eigen::Matrix3d m;

    m << 0.0, 0.0, - cos(theta) * theta_dot,
        0.0, - sin(phi) * phi_dot, cos(theta) * cos(phi) * phi_dot - 
        sin(theta) * sin(phi) * theta_dot,
        0.0, - cos(phi) * phi_dot, -sin(phi) * cos(theta) * phi_dot - 
        cos(phi) * sin(theta) * theta_dot;

    return m;
}

/** Euler angles to quaternions. Euler angles follow the post multiply
    sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
    and "phi" around x (roll) **/
EulerRotations::Quaternions EulerRotations::euler_to_quaternions(double phi,
    double theta, double psi)
{
    EulerRotations::Quaternions quatern;

    double cy = cos(psi * 0.5); double sy = sin(psi * 0.5);
    double cp = cos(theta * 0.5); double sp = sin(theta * 0.5);
    double cr = cos(phi * 0.5); double sr = sin(phi * 0.5);

    quatern.w = cr * cp * cy + sr * sp * sy;
    quatern.x = sr * cp * cy - cr * sp * sy;
    quatern.y = cr * sp * cy + sr * cp * sy;
    quatern.z = cr * cp * sy - sr * sp * cy;

    // Return Quaternions
    return quatern;
}


/** Quaternions to Euler Angles. Euler angles follow the post multiply
    sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
    and "phi" around x (roll) **/
EulerRotations::Euler EulerRotations::quaternions_to_euler(double w, double x,
    double y, double z)
{
    EulerRotations::Euler euler_angles;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler_angles.phi = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0)
    {
        euler_angles.theta = std::copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
    }
    else
    {
        euler_angles.theta = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler_angles.psi = std::atan2(siny_cosp, cosy_cosp);

    // Return Euler angles
    return euler_angles;
}