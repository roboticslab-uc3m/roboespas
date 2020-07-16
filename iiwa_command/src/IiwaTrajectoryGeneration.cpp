#ifndef IIWA_TG
#define IIWA_TG

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>
#include "IiwaTrajectory.cpp"
#include "IiwaScrewTheory.cpp"

using namespace Eigen;
using namespace std;

namespace IiwaTrajectoryGeneration
{
    namespace
    {
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
                //cout << "ttotal: " << ttotal << endl;
                //cout << "dtotal: " << dtotal << endl;
                //cout << "a_max: " << a_max << endl;
                ROS_ERROR("Planning to calculate the sqrt of a negative number, changed total time to minimum time");
                double min_time = sqrt(4*dtotal/a_max);
                ttotal= min_time;
                //cout << "min_time: " << ttotal << endl;
                //return;
            }
            ttotal = ceil(ttotal/step_size) * step_size;
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
        static bool FillJointVelocityFromCartesianVelocity(IiwaTrajectory traj, VectorXd q_ini, VectorXd q_max, IiwaTrajectory &traj_output)
        {
            traj_output = IiwaTrajectory(traj.npoints, traj.step_size);
            traj_output.x = traj.x;
            traj_output.xdot = traj.xdot;
            traj_output.xdotdot = traj.xdotdot;
            traj_output.q.col(0) = q_ini;
            for (unsigned int i=0; i<traj.x.cols()-1; i++)
            {
                VectorXd xdot_S = IiwaScrewTheory::TransformScrew_A2S(traj_output.xdot.col(i), traj_output.x.col(i));
                traj_output.qdot.col(i) = IiwaScrewTheory::InverseDifferentialKinematicsPoint(traj_output.q.col(i), xdot_S);
                //TODO: Check qdot max
                traj_output.q.col(i+1) = traj_output.q.col(i) + traj_output.qdot.col(i)*(traj_output.t[i+1] -traj_output.t[i]);
            }
            bool out_of_range=false;
            for (unsigned int i=0; i<traj_output.npoints; i++)
            {
                bool compare = (traj_output.q.col(i).array()>q_max.array()).any();
                if (compare)
                {
                    out_of_range=true;
                }
            }
            if (out_of_range)
            {
                return false;
            }
            return true;
        }
        static IiwaTrajectory DeparameterizeTrapezoidalVelocityTrajectory(VectorXd x_ini, double tacc, double tflat, double a_tras, Vector3d axis_tras, double a_rot, Vector3d axis_rot, double step_size)
        {
            VectorXd a_screw(6);
            a_screw.head(3) = a_tras * axis_tras;
            a_screw.tail(3) = a_rot * axis_rot;

            double ttotal = tacc*2 + tflat;
            int npoints = ttotal/step_size+ 1 ;

            IiwaTrajectory traj(npoints, step_size);
            traj.xdotdot.col(0) = VectorXd::Zero(6);
            traj.xdot.col(0) = VectorXd::Zero(6);
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
                traj.xdotdot.col(i) = VectorXd::Zero(6);
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
        static bool TrapezoidalVelocityTrajectory(double dist_tras, double angle_rot, VectorXd axis_rot, VectorXd axis_tras, VectorXd q_ini, double time, double a_cart_max, VectorXd q_max, double step_size, IiwaTrajectory &traj_q)
        {
            double a_tras, a_rot, tacc, tflat;
            IiwaTrajectoryGeneration::ParameterizedTrapezoidalVelocityTrajectory(time, dist_tras, a_cart_max, step_size, a_tras, tacc, tflat);
            a_rot = IiwaTrajectoryGeneration::GetAccelerationFromTrapezoidalVelocityTrajectory(angle_rot, tacc, tflat);
            //Deparameterize this trajectory and transform it into cartesian positions
            VectorXd x_ini = IiwaScrewTheory::ForwardKinematics(q_ini);
            IiwaTrajectory traj= IiwaTrajectoryGeneration::DeparameterizeTrapezoidalVelocityTrajectory(x_ini, tacc, tflat, a_tras, axis_tras, a_rot, axis_rot, step_size);
            bool possible = IiwaTrajectoryGeneration::FillJointVelocityFromCartesianVelocity(traj, q_ini, q_max, traj_q);
            return possible;
        }
        static double GetMinimumTime(IiwaTrajectory traj, double velocity, VectorXd qdot_max)
        {
            IiwaTrajectory traj_faster = IiwaTrajectory(traj.npoints, traj.step_size);
            VectorXd max_qdot(7);
            for (unsigned int j=0; j<traj.qdot.rows(); j++)
            {
                max_qdot[j] = traj.qdot.row(j).cwiseAbs().maxCoeff();
            }
            VectorXd percentage_qdot = max_qdot.array()/qdot_max.array();
            double max_percentage = percentage_qdot.maxCoeff();
            double time_previous = traj.t[traj.t.size()-1];
            double time_minimum = time_previous * (max_percentage/0.9);
            double time_new = time_minimum/velocity;
            time_new = ceil(time_new/traj.step_size)*traj.step_size;
            return time_new;
        }
    }
    static bool TrapezoidalVelocityProfileTrajectory(VectorXd q_ini, VectorXd x_goal, double control_step_size, double velocity, double a_cart_max, VectorXd q_max, VectorXd qdot_max, IiwaTrajectory &traj_output)
    {
        VectorXd x_ini = IiwaScrewTheory::ForwardKinematics(q_ini);
        //Calculate the needed increment from q_curr to x_goal
        VectorXd x_inc_A = IiwaScrewTheory::ScrewA2B_A(x_ini, x_goal);
        //Parametrize the trajectory as two increments along two axes
        double angle_rot = x_inc_A.tail(3).norm();
        Vector3d axis_rot = x_inc_A.tail(3)/angle_rot;
        double dist_tras = x_inc_A.head(3).norm();
        Vector3d axis_tras = x_inc_A.head(3)/dist_tras;
        //First calculate cartesian velocities for a high time, so you can estimate which will be the joint velocities generated for that time. Then rescale that time to meet maximum/velocity percentage of the maximum joint velocity
        double time_inicial = 20;
        IiwaTrajectory traj_inicial(0, control_step_size);
        bool possible = TrapezoidalVelocityTrajectory(dist_tras, angle_rot, axis_rot, axis_tras, q_ini, time_inicial, a_cart_max, q_max, control_step_size, traj_inicial);
        if (!possible)
        {
            return possible;
        }
        //Calculate the minimum time to cover that distance
        double min_time = GetMinimumTime(traj_inicial, velocity, qdot_max);
        //Get the trapezoidal trajectory for that time
        possible = TrapezoidalVelocityTrajectory(dist_tras, angle_rot, axis_rot, axis_tras, q_ini, min_time, a_cart_max, q_max, control_step_size, traj_output);
        return possible;
    }
    static bool CircumferenceTrajectory(VectorXd q_ini, double radius, int plane)
    {
        return true;
    }
}
#endif
