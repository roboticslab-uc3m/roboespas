#ifndef IIWA_ST
#define IIWA_ST

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>

namespace IiwaTrajectoryGeneration
{
    static std::vector<Eigen::VectorXd> TrapezoidalVelocityProfile(Eigen::VectorXd q_curr, Eigen::VectorXd x_goal, double control_step_size, double velocity, Eigen::VectorXd &timestamps, std::vector<Eigen::VectorXd> &xdot, std::vector<Eigen::VectorXd> &xdotdot)
    {
        //First calculate cartesian velocities for a high time, so you can estimate which will be the joint velocities generated for that time. Then rescale that time to meet maximum/velocity percentage of the maximum joint velocity
        double time_first_approach = 20;
        
    }
    static ParameterizedTrapezoidalVelocityProfileTrajectory(double ttotal, double distance, double a_max, double step_size, double &a, double &tacc, double &tflat)
    {
        ttotal = 
    }
}
#endif
