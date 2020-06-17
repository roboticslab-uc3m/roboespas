#ifndef IIWA_TG
#define IIWA_TG

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>
#include "IiwaTrajectory.cpp"
#include "IiwaScrewTheory.cpp"

namespace IiwaTrajectoryGeneration
{
    static std::vector<Eigen::VectorXd> TrapezoidalVelocityProfile(Eigen::VectorXd q_curr, Eigen::VectorXd x_goal, double control_step_size, double velocity, Eigen::VectorXd &timestamps, std::vector<Eigen::VectorXd> &xdot, std::vector<Eigen::VectorXd> &xdotdot)
    {
        //First calculate cartesian velocities for a high time, so you can estimate which will be the joint velocities generated for that time. Then rescale that time to meet maximum/velocity percentage of the maximum joint velocity
        double time_first_approach = 20;

    }
    static double GetAccelerationFromTrapezoidalVelocityTrajectory(double dtotal, double tacc, double tflat)
    {
        return dtotal/(tacc*tflat+tacc*tacc);
    }
    static void ParameterizedTrapezoidalVelocityTrajectory(double ttotal, double dtotal, double a_max, double step_size, double &a, double &tacc, double &tflat)
    {
        ttotal = ceil(ttotal/step_size) * step_size;
        //TODO: Explain where does this formulae come from
        double a_min = (4*dtotal)/(ttotal*ttotal);
        if ((ttotal*ttotal-4*dtotal/a_max) < 0.0)
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
        bool sol_2 = vflat_2>0 & tflat_2>0 & tacc_2>0;
        double vflat;
        if (sol_1)
        {
            vflat = vflat_1;
            tflat = tflat_1;
            tacc = tacc_1;
        }
        else if (sol_2)
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
        a = GetAccelerationFromTrapezoidalVelocityTrajectory (dtotal, tacc, tflat);
        vflat = tacc*a;
        return;
    }
    static IiwaTrajectory DeparameterizeTrapezoidalVelocityTrajectory(Eigen::VectorXd x_ini, double tacc, double tflat, double a_tras, Eigen::Vector3d axis_tras, double a_rot, Eigen::Vector3d axis_rot, double step_size)
    {
        Eigen::VectorXd a_screw(6);
        a_screw.head(3) = a_tras * axis_tras;
        a_screw.tail(3) = a_rot * axis_rot;

        double ttotal = tacc*2 + tflat;
        int npoints = ttotal/step_size+ 1 ;

        IiwaTrajectory traj(npoints, step_size);
        traj.xdotdot.col(0) = Eigen::VectorXd::Zero(6);
        traj.xdot.col(0) = Eigen::VectorXd::Zero(6);
        traj.x.col(0) = x_ini;

        int id_acc_start = 1;
        int id_acc_end = round(tacc/step_size);
        int id_flat_start = id_acc_end;
        int id_flat_end = round((tacc+tflat)/step_size);
        int id_dec_start = id_flat_end;
        int id_dec_end = round(ttotal/step_size);

        for (int i=id_acc_start; i<=id_acc_end; i++)
        {
            traj.xdotdot.col(i) = a_screw;
            traj.xdot.col(i) = a_screw * traj.t[i];
            traj.x.col(i) = IiwaScrewTheory::TransformFrame(x_ini, 0.5 *a_screw*traj.t[i]*traj.t[i]);
        }
        for (int i=id_flat_start; i<=id_flat_end; i++)
        {
            //Time since started flat part
            double t = traj.t[i] - traj.t[id_flat_start];
            traj.xdotdot.col(i) = Eigen::VectorXd::Zero(6);
            traj.xdot.col(i) = traj.xdot.col(id_flat_start);
            traj.x.col(i) = IiwaScrewTheory::TransformFrame(traj.x.col(id_flat_start), traj.xdot.col(id_flat_start)*t);
        }
        for (int i=id_dec_start; i<=id_dec_end; i++)
        {
            //Time since started deacceleration part
            double t = traj.t[i] - traj.t[id_dec_start];
            traj.xdotdot.col(i) = -a_screw;
            traj.xdot.col(i) = traj.xdot.col(id_dec_start) - a_screw*t;
            traj.x.col(i) = IiwaScrewTheory::TransformFrame(traj.x.col(id_dec_start), traj.xdot.col(id_dec_start) * t - 0.5 * a_screw * t * t);
        }
        return traj;
    }

}
#endif
