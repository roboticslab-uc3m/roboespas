#ifndef IIWA_TRAJ
#define IIWA_TRAJ
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Twist.h"
#include <Eigen/Dense>
#include <math.h>
#include <cmath>
#include "IiwaScrewTheory.cpp"

using namespace Eigen;
using namespace std;

class IiwaTrajectory
{
    public:
    int npoints = 0;
    double step_size;
    VectorXd t;
    MatrixXd q;
    MatrixXd x;
    MatrixXd qdot;
    MatrixXd xdot;
    MatrixXd qdotdot;
    MatrixXd xdotdot;
    IiwaTrajectory(int npoints_, double step_size_): npoints(npoints_), t(npoints_), q(7, npoints_), x(6, npoints_), qdot(7, npoints_), xdot(6, npoints_), qdotdot(7, npoints_), xdotdot(6, npoints_)
    {
        step_size=step_size_;
        if (npoints>1)
        {
            t[0] = 0.0;
            for (int i = 1; i<npoints; i++)
            {
                t[i] = t[i-1] + step_size;
            }
            q = MatrixXd::Zero(7, npoints);
            x = MatrixXd::Zero(6, npoints);
            qdot = MatrixXd::Zero(7, npoints);
            xdot = MatrixXd::Zero(6, npoints);
            qdotdot = MatrixXd::Zero(7, npoints);
            xdotdot = MatrixXd::Zero(6, npoints);
        }
    }
};
#endif
