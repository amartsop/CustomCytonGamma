#pragma once 

#include <iostream>
#include <vector>
#include "robot_control.h"

class PathPlanning
{
    public:
        PathPlanning();
        
        // Quaternions struct
        struct Quaternions { double w, x, y, z; };

        // Eulear angles struct
        struct Euler{ double phi, theta, psi; };

        // Generate trajectory
        std::vector<cmsgs::Cartesian> generate_trajectory(const
            cmsgs::Cartesian& cur_pose);

        // Generate trajectory
        std::vector<cmsgs::Cartesian> generate_const_trajectory(const
            cmsgs::Cartesian& cur_pose);
    private:

        // Trajectory initial and final time 
        double m_t0 = 0.0; // Initial time (s) 
        double m_tf = 8.0; // Final time (s)

        // Trajectory sampling frequency
        double m_fs = 100.0;  // Sampling frequency (Hz)
        double m_ts = 1.0 / m_fs; // Sampling period (s)
        unsigned int m_pnum; // Number of sampling points
};