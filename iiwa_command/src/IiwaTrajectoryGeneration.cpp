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
    static void ParameterizedTrapezoidalVelocityProfileTrajectory(double ttotal, double dtotal, double a_max, double step_size, double &a, double &tacc, double &tflat)
    {
        ttotal = ceil(ttotal/step_size) * step_size;
        //TODO: Explain where does this formulae come from
        double a_min = (4*dtotal)/(ttotal*ttotal);
        if (ttotal*ttotal-4*dtotal/a_max) < 0
        {
            //The sqrt will return error
            ROS_ERROR("Planning to calculate the sqrt of a negative number");
            return;
        }
        double vflat_1 = ttotal*a_max/2 + a_max*sqrt(ttotal*ttotal-4*dtotal/a_max)/2;
        double vflat_2 = ttotal*a_max/2 - a_max*sqrt(ttotal*ttotal-4*dtotal/a_max)/2;
        double tflat_1 = ttotal - 2*vflat_1/a_max;
        double tflat_2 = ttotal - 2*vflat_2/a_max;
        double tacc_1 = vflat_1/a_max;
        double tacc_2 = vflat_2/a_max;
        bool sol_1 = vflat_1>0 & tflat_1>0 & tacc_1>0;
        bool sol_2 = vflat_2>0 & tflat_1>0 & tacc_1>0;
        double vflat, tflat, tacc;
        if (sol_1)
        {
            vflat = vflat_1;
            tflat = tflat_1;
            tacc = tacc_1;
        }
        else if (sol_2 & !sol_1)
        {
            vflat = vflat_2;
            tflat = tflat_2;
            tacc = tacc_2;
        }
        else
        {
            ROS_ERROR("No positive ttotal as a solution");
        }
        //Fit the values to the discrete system
        tacc = ceil(tacc/step_size)*step_size;
        tflat = ttotal -2*tacc;
        a = dtotal/(tacc*tflat+tacc*tacc);
        vflat = tacc*a;
        return;
    }
}
#endif
