#pragma once 

#include <iostream>
#include <vector>

// Custom msgs namespace 
namespace cmsgs {

    // Cartesian space path struct
    struct Cartesian {
        double time = 0.0;
        std::vector<double> position = {0.0, 0.0, 0.0};
        std::vector<double> velocity = {0.0, 0.0, 0.0};
        std::vector<double> acceleration = {0.0, 0.0, 0.0};
        std::vector<double> euler_position = {0.0, 0.0, 0.0};
        std::vector<double> euler_velocity = {0.0, 0.0, 0.0};
        std::vector<double> euler_acceleration = {0.0, 0.0, 0.0};
    };



    // Joint space path struct
    struct Joint {
        double time;
        std::vector<double> position;
        std::vector<double> velocity;
        std::vector<double> acceleration;
    };

    // Force torque data 
    struct FTData {
        double time = 0.0;
        std::vector<double> forces = {0.0, 0.0, 0.0};
        std::vector<double> torques = {0.0, 0.0, 0.0};
    };
}