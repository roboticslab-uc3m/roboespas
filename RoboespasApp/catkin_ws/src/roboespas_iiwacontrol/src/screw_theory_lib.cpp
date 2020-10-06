#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <thread>         // this_thread::sleep_for
//std::this_thread::sleep_for(std::chrono::milliseconds(x));
#include <chrono>         // chrono::seconds
#include <geometry_msgs/Twist.h>
#include <math.h>
#include "conversions_lib.cpp"
#include "filter_lib.cpp"

using namespace Eigen;
using namespace std;

//|----------------------| Library that offers 2 main functions and some others to make these work
//| SCREW THEORY LIBRARY | related with screw theory calculations
//|----------------------|
// - ForwardKinematics: Compute the cartesian position corresponding to some joint_position
// - InverseDifferentialKinematics: Compute next joint position to follow a given cartesian trajectory. It calculates the needed velocity to reach expected cartesian position from current cartesian position, and adds it to the given cartesian velocity needed to follow the trajectory, ponderating the error by a k factor. Then with the geometric jacobian obtain the joint velocity corresponding to that cartesian velocity.

namespace ScrewTheory
{
    //Global variables
    Matrix4d Hst0 = Matrix4d::Identity(4,4);
    MatrixXd Twists(6,7);
    VectorXd q_limits;
    VectorXd qdot_limits;
    MatrixXd CentersOfMass(3,7);
    MatrixXd Inertias(3,7);
    VectorXd Masses(7);
    MAFilter::MovingAverageFilterSixVars filter_error(10);
    //Parameter initialization
    void Init(vector<double> Hst0_t, vector<double> Hst0_axang, vector<vector<double>> twists, vector<double> q_limits_std, vector<double> qdot_limits_std, vector<vector<double>> centersofmass, vector<vector<double>> inertias, vector<double> masses);
    //Forward Kinematics
    VectorXd ForwardKinematics(VectorXd joint_pos);
    vector<double> ForwardKinematics(vector<double> joint_pos);
    //Inverse Differential Kinematics
    vector<double> InverseDifferentialKinematics(vector<double> q_curr, vector<double> q_exp, geometry_msgs::Twist x_exp, geometry_msgs::Twist xdot_exp, double t, double k, vector<double> &qdot_commanded, geometry_msgs::Twist &error_xdot, geometry_msgs::Twist &error_xdot_filtered, geometry_msgs::Twist &xdot_commanded, geometry_msgs::Twist &x_commanded);
    VectorXd InverseDifferentialKinematics(VectorXd q_curr, VectorXd q_exp, VectorXd x_exp, VectorXd xdot_exp, double t, double k, VectorXd &qdot_commanded, VectorXd &error_xdot, VectorXd &error_xdot_filtered, VectorXd &xdot_commanded, VectorXd &x_commanded);
    //Product of Exponentials
    Matrix4d ProductOfExponentials(VectorXd joint_pos);
    Matrix4d ProductOfExponentials(VectorXd joint_pos, int i, int j);
    //Geometric Jacobian in Spatial Frame
    MatrixXd GeoJacobianS(VectorXd joint_pos);
    //Other general functions
    VectorXd ComputeCartesianVel(VectorXd x_curr, VectorXd x_des, double time);
    Matrix3d ToMatrixForm(Vector3d w);
    Matrix4d twist2tform(VectorXd twist, double theta);
    Matrix3d axis2skew(Vector3d x);
    MatrixXd tform2adjoint(Matrix4d T);
    Matrix4d trvec2tform(Vector3d trvec);
    VectorXd ToTwistForm(VectorXd vec_xyz, Vector3d p_xyz);
    VectorXd LimitJointVel(VectorXd joint_velocity);
    VectorXd LimitJointPos(VectorXd joint_position);
    //------------------------------------------------------------------------------------------
    //PARAMETER INITIALIZATION
    void Init(vector<double> Hst0_t, vector<double> Hst0_axang, vector<vector<double>> twists, vector<double> q_limits_std, vector<double> qdot_limits_std, vector<vector<double>> centersofmass_std, vector<vector<double>> inertias_std, vector<double> masses_std)
    {
        //Function that initializates the screw theory library, stablishing the values for the parameters used in other functions:
        //Hst0: Robot end effector pose in home position
        Vector4d Hst0_axang_eig=Conversions::StdVectorToEigenVector(Hst0_axang);
        Matrix3d R=AngleAxisd(Hst0_axang_eig[3], Hst0_axang_eig.block(0,0,3,1)).toRotationMatrix();
        Hst0.block(0,0,3,3)=R;
        Hst0.block(0,3,3,1)=Conversions::StdVectorToEigenVector(Hst0_t);
        //Twists: Twists for the robot in home position
        for (int i=0; i<twists.size(); i++)
        {
            VectorXd t=Conversions::StdVectorToEigenVector(twists[i]);
            Twists.block(0,i,6,1)=t.head(6);
        }
        //Limits: Joint position and velocity limits for the real robot
        q_limits=Conversions::StdVectorToEigenVector(q_limits_std);
        qdot_limits=Conversions::StdVectorToEigenVector(qdot_limits_std);
        //CentersOfMass: Centers of mass for each of the links of the real robot
        for (int i=0; i<centersofmass_std.size(); i++)
        {
            VectorXd com=Conversions::StdVectorToEigenVector(centersofmass_std[i]);
            CentersOfMass.block(0,i,3,1)=com.head(3);
        }
        //Inertias: Inertia moments for each of the links of the real robot
        for (int i=0; i<inertias_std.size(); i++)
        {
            VectorXd inertia=Conversions::StdVectorToEigenVector(inertias_std[i]);
            Inertias.block(0,i,3,1)=inertia.head(3);
        }
        //Masses: Masses of each of the links of the real robot
        Masses=Conversions::StdVectorToEigenVector(masses_std);
        return;
    }
    //------------------------------------------------------------------------------------------
    //FORWARD KINEMATICS
    //Compute the cartesian position corresponding to some joint_position using the Twists and Hst0 (configuration at home position) parameters.
    //FK function to access from outside the library without needing Eigen
    vector<double> ForwardKinematics(vector<double> q)
    {
        VectorXd q_eig=Conversions::StdVectorToEigenVector(q);
        VectorXd x_eig=ForwardKinematics(q_eig);
        vector<double> x=Conversions::EigenVectorToStdVector(x_eig);
        return x;
    }
    //FK function with Eigen vectors
    VectorXd ForwardKinematics(VectorXd q)
    {
        //First get the product of exponentials form for the joint pos given
        Matrix4d HstR=ProductOfExponentials(q);
        //Then multiply by Hst0 (the home pos in homogeneous transformation form), to get the homogeneous trasnformation form of the robot after moving to the given joint_pos
        Matrix4d Hst=HstR*Hst0;
        //Obtain the rotation in euler angles and linear component of the transformation
        Matrix3d R=Hst.block(0,0,3,3);
        Vector3d eul=R.eulerAngles(0,1,2);
        Vector3d t=Hst.block(0,3,3,1);
        //Compose the twist vector with the linear and angular component (v, w) = (t, eul)
        VectorXd twist(6);
        twist.head(3)=t;
        twist.tail(3)=eul;
        return twist;
    }
    //------------------------------------------------------------------------------------------
    //INVERSE DIFFERENTIAL KINEMATICS
    //These functions compute next joint position to follow a given cartesian trajectory. It calculates the needed velocity to reach expected cartesian position from current cartesian position, and adds it to the cartesian velocity needed to follow the trajectory, ponderating the error by a k factor. Then with the geometric jacobian obtain the joint velocity corresponding to that cartesian velocity. There are two different functions, one that transforms the input variables from std::vector<double> or geometry_msgs::Twist to Eigen::VectorXd, and the other one that actually computes the IDK. Both have the following input/output variables:
    // - xdot_exp -> Desired cartesian velocity (theoretical, offline calculation), should be calculated as (x_next-x_curr)/t
    // - Variables to calculate the error:
    //     * x_exp -> Expected cartesian position
    //     * q_curr -> After applying FK will be the real current cartesian position
    //     * t -> Time used for computing the offline cartesian velocity, that will be used to calculate the velocity needed to correct the existing error:
    //          error_xdot=(x_traj[i]-FK(q_curr))/(stamp-previous_stamp)
    //     * k -> The calculated error will be added up to the theoretical cartesian velocity multiplied by a factor k, which will be a small number and is received as a parameter in the service.
    // - Output variables:
    //     * qdot_commanded -> Computed joint velocity to fulfill the input cartesian velocity with the error correction
    //     * error_xdot -> Computed error explained before
    //     * error_xdot_filtered -> Used error for the calculation, that reduces the noise introduced by the current iiwa position variable, which has noise as it is a direct measure from the real robot
    //     * xdot_commanded -> Cartesian velocity that mixes the input desired cartesian velocity with the computed cartesian velocity error.
    //     * q_commanded -> Output of the function, expresses the next joint position to follow the trajectory. It may be used to command the robot in position instead of velocity, and to know where the robot probably is after commanding qdot_commanded for time t.

    //IDK function to access from outside the library without needing Eigen
    vector<double> InverseDifferentialKinematics(vector<double> q_curr, vector<double> q_exp, geometry_msgs::Twist x_exp, geometry_msgs::Twist xdot_exp, double t, double k, vector<double> &qdot_commanded, geometry_msgs::Twist &error_xdot, geometry_msgs::Twist &error_xdot_filtered, geometry_msgs::Twist &xdot_commanded, geometry_msgs::Twist &x_commanded)
    {
        //Transform inputs into eigen vectors
        VectorXd q_curr_eig=Conversions::StdVectorToEigenVector(q_curr);
        VectorXd q_exp_eig=Conversions::StdVectorToEigenVector(q_exp);
        VectorXd x_exp_eig=Conversions::TwistToEigenVector(x_exp);
        VectorXd xdot_exp_eig=Conversions::TwistToEigenVector(xdot_exp);
        //Create output joint/cartesian variables
        VectorXd qdot_commanded_eig(7);
        VectorXd error_xdot_eig(6), error_xdot_filtered_eig(6), xdot_commanded_eig(6), x_commanded_eig(6);
        VectorXd q_commanded_eig=InverseDifferentialKinematics(q_curr_eig, q_exp_eig, x_exp_eig, xdot_exp_eig, t, k, qdot_commanded_eig, error_xdot_eig, error_xdot_filtered_eig, xdot_commanded_eig, x_commanded_eig);
        //Transform outputs into vector<double> and geometry_msgs::Twist types
        qdot_commanded=Conversions::EigenVectorToStdVector(qdot_commanded_eig);
        error_xdot=Conversions::EigenVectorToTwist(error_xdot_eig);
        error_xdot_filtered=Conversions::EigenVectorToTwist(error_xdot_filtered_eig);
        xdot_commanded=Conversions::EigenVectorToTwist(xdot_commanded_eig);
        x_commanded=Conversions::EigenVectorToTwist(x_commanded_eig);
        vector<double> q_commanded=Conversions::EigenVectorToStdVector(q_commanded_eig);
        return q_commanded;
    }
    //IDK function with Eigen vectors
    VectorXd InverseDifferentialKinematics(VectorXd q_curr, VectorXd q_exp, VectorXd x_exp, VectorXd xdot_exp, double t, double k, VectorXd &qdot_commanded, VectorXd &error_xdot, VectorXd &error_xdot_filtered, VectorXd &xdot_commanded, VectorXd &x_commanded)
    {
        //Compute the needed velocity to move from current position to expected position
        VectorXd x_curr = ForwardKinematics(q_curr);
        error_xdot = ComputeCartesianVel(x_curr, x_exp, t);
        //Filter this error because it has noise from the robot measurement
        error_xdot_filtered=filter_error.Filter(error_xdot);
        //Calculate cartesian velocity with theoretical xdot + error xdot
        xdot_commanded = (1-k)*xdot_exp+k*error_xdot_filtered;
        //Compute the geometric jacobian with the expected joint position
        MatrixXd Jst=GeoJacobianS(q_curr);
        //Also, the cartesian vel must be transformed into screw form
        VectorXd screw_vel=ToTwistForm(xdot_commanded, x_exp.head(3));
        //Compute the pseudo inverse of the jacobian using the Moore-Penrose formula
        MatrixXd pinv_Jst=Jst.transpose()*(Jst*Jst.transpose()).inverse();
        //Finally compute the joint velocity asociated to that screw velocity
        qdot_commanded=pinv_Jst*screw_vel;
        //Limit this velocity just in case it returns erroneous or too high numbers
        qdot_commanded=LimitJointVel(qdot_commanded);
        //Calculate the q_commanded taking into account q_curr, the velocity q_dot and the time
        VectorXd q_commanded = q_exp + qdot_commanded*t; //q_curr+q_dot*time;
        //And limit this position just in case it goes out of bounds
        q_commanded=LimitJointPos(q_commanded);
        x_commanded=x_curr+xdot_commanded*t;
        return q_commanded;
    }
    //------------------------------------------------------------------------------------------
    //PRODUCT OF EXPONENTIALS
    //These two functions compute the product of exponentials taking into account the variable "Twists", which contain the twists for the robot at home position, and some joint_position. If you only give the function the joint_position, it will compute the PoE for all joints. If you give two integers, there's also a function that may calculate the PoE just for some joints, for example from joint 2 to 4, you would have to ask for ProductOfExponentials(q, 1, 3), as indices in C++ start from 0.
    //Calculate Product of Exponentials 4 by 4 matrix from index i to index j for a given joint_pos using the Twists parameters
    Matrix4d ProductOfExponentials(VectorXd q, int i, int j)
    {
        //First build a 4 by 4 identity matrix
        Matrix4d PoE=Matrix4d::Identity(4,4);
        for (int k=i; k<=j; k++)
        {
            //For each joint, get the twists parameter
            VectorXd twist=Twists.block(0,k,6,1);
            //Calculate the transformation matrix for each screw axis and joint magnitude
            Matrix4d T=twist2tform(twist, q[k]);
            //Multiply by the previous solution (when i=0, multiply by identity matrix)
            PoE=PoE*T;
        }
        return PoE;
    }
    //Calculate Product Of Exponentials 4 by 4 matrix for all joints using the Twists parameters
    Matrix4d ProductOfExponentials(VectorXd q)
    {
        int i=0;
        int j=q.size()-1;
        MatrixXd HstR=ProductOfExponentials(q, i, j);
        return HstR;
    }
    //------------------------------------------------------------------------------------------
    //GEOMETRIC JACOBIAN
    //Function that computes the geometric jacobian of the robot, given the Twists parameter, with the twists of the robot at home position, and given the current joint position.
    MatrixXd GeoJacobianS(VectorXd q)
    {
        //Compute the geometric jacobian given the current joint pos
        MatrixXd JstS(6,7);
        //Initialize the product of exponentials
        MatrixXd PoE=MatrixXd::Identity(4,4);
        for (int i=0; i<7; i++)
        {
            //For each twist
            VectorXd twist=Twists.block(0,i,6,1);
            //Fill up the Jst column with the adjoint of the Product of Exponentials multiplied by the twist
            JstS.block(0,i,6,1)=tform2adjoint(PoE)*twist;
            //And keep calculating the Product of Exponentials for next iteration
            PoE=PoE*twist2tform(twist, q[i]);
        }
        return JstS;
    }
    //------------------------------------------------------------------------------------------
    //OTHER FUNCTIONS
    //Twist to Transformation Matrix Form: 6by1 twist + float theta --> 4by4 tform
    //Converts a 6by1 twist, containing a linear component v and some angular component w into a 4by4 transformation matrix, with some inner rotation 3by3 matrix R and linear translation vector t
    Matrix4d twist2tform(VectorXd twist, double theta)
    {
        Vector3d v=twist.head(3);
        Vector3d w=twist.tail(3);
        //Calculate homogeneous transformation expression for a screw vector (v, w) with a certain magnitude theta such that
        //T=[R, p; 0 0 0 1];
        // where p=(R-I)*(wxv)
        //First calculate the rotation matrix R given the axis and the angle of rotation
        Matrix3d R=AngleAxisd(theta, w).toRotationMatrix(); //similar to func expAxAng
        //Precalculate the 3by3 identity matrix
        Matrix3d I3=Matrix3d::Identity();
        //And calculate the p vector
        RowVector3d p=(I3-R)*(w.cross(v));
        //Finally, compound the 4by4 matrix
        Matrix4d T=MatrixXd::Identity(4,4); //Initializate as a 4by4 identity matrix
        T.block<3,3>(0,0)=R; //Introduce a 3 rows by 3 columns block in row 0, column 0 containing R.
        T.block<3,1>(0,3)=p; //Introduce a 3 rows by 1 columns block in row 0, column 3 containing p.
        return T;
    }
    //The linear component of the twist will be the linear component of the original vector minus the angular component cross-multiplied by the current tool position. This will transform some 6by1 vector from tool frame to space frame, but mantaining the vector origin at the origin of the tool frame.
    VectorXd ToTwistForm(VectorXd vec, Vector3d p_tool)
    {
        Vector3d v=vec.head(3);
        Vector3d w=vec.tail(3);
        Vector3d v_twist=v-w.cross(p_tool);
        Vector3d w_twist=w;
        VectorXd vec_twist(6);
        vec_twist.head(3)=v_twist;
        vec_twist.tail(3)=w_twist;
        return vec_twist;
    }
    //Obtain the adjoint form of a transformation matrix, which express both rotation and translation as 3 by 3 matrices and joins them. It contains the rotation matrix in the upper left and lower right corner, the skew-symmetric form of the translation vector multiplied by the rotation matrix in the upper right corner, and a 3by3 zero matrix in the lower left corner.
    MatrixXd tform2adjoint(Matrix4d T)
    {
        Matrix3d R=T.block(0,0,3,3);
        Vector3d p=T.block(0,3,3,1);
        MatrixXd Ad(6,6);
        Ad.block(0,0,3,3)=R;
        Ad.block(0,3,3,3)=axis2skew(p)*R;
        Ad.block(3,0,3,3)=Matrix3d::Zero();
        Ad.block(3,3,3,3)=R;
        return Ad;
    }
    //Obtain the skew-symmetric form of a rotation vector, which expresses the rotation as a 3by3 matrix as follows:
    Matrix3d axis2skew(Vector3d v)
    {
        //Skew symmetric form of a vector such that:
        //matrix=[   0    -v_z    v_y;
        //      v_z     0    -v_x;
        //     -v_y    v_x     0   ]
        Matrix3d skew_sym_form=Matrix3d::Zero();
        skew_sym_form(0,1)=-v[2];
        skew_sym_form(0,2)= v[1];
        skew_sym_form(1,0)= v[2];
        skew_sym_form(1,2)=-v[0];
        skew_sym_form(2,0)=-v[1];
        skew_sym_form(2,1)= v[0];
        return skew_sym_form;
    }
    //Obtain the 3by3 matrix form of a rotation vector, multiplying each angle by the unit rotation in that angle (different from skew-symmetric form)
    Matrix3d ToMatrixForm(Vector3d w)
    {
        Matrix3d R=(AngleAxisd (w[0], Vector3d::UnitX()) * AngleAxisd(w[1], Vector3d::UnitY()) * AngleAxisd (w[2], Vector3d::UnitZ()) ).matrix();
        return R;
    }
    //Compute some cartesian velocity given two cartesian position, both of them composed by a 3by1 linear component and a 3by1 angular component. Linear component of the velocity may be computed just by substraction and division by the time, but angular component should be computed by first transforming both orientations into matrix form, then calculating the rotation matrix needed to transform the first orientation into the second, and transforming that computed rotation matrix back again into euler angles form. Finally, divide by the time to obtain the angular velocity using two orientations.
    VectorXd ComputeCartesianVel(VectorXd x_curr, VectorXd x_des, double time)
    {
        VectorXd xdot(6);
        //The linear component of the cartesian vel is just the desired pos minus de current one divided by the time
        xdot.head(3)=(x_des.head(3)-x_curr.head(3))/time;
        //However, the angular component is harder to compute. First, convert both orientations (current and desired) into matrix form. This matrices express the transformation from the space frame to each of the frames
        Vector3d w_curr=x_curr.tail(3);
        Vector3d w_des=x_des.tail(3);
        Matrix3d R_S_curr=ToMatrixForm(w_curr);
        Matrix3d R_S_des=ToMatrixForm(w_des);
        //R_AB = R_AS*R_SB, and R_AS=R_SA'
        Matrix3d R_curr_des=R_S_curr.transpose()*R_S_des;
        //Express this transformation in axis angle notation (axis+angle)
        AngleAxisd axang_curr_des(R_curr_des);
        //Multiply the axis by the angle to get a single 3-vector
        Vector3d axis_curr_des=axang_curr_des.axis()*axang_curr_des.angle();
        //This rotation is still expressed in the current frame, so multiply by R_s_curr to express it in the space frame
        Vector3d axis_S_curr_des=R_S_curr*axis_curr_des;
        //Finally, calculate the angular vel dividing by the time
        xdot.tail(3)=axis_S_curr_des/time;
        return xdot;
    }
    //Returns the limited joint velocity taking into account the velocity limits which are saved as global variables and should be set previously (in the Init function)
    VectorXd LimitJointVel(VectorXd qdot)
    {
        VectorXd qdot_out=qdot;
        for (int i=0; i<qdot.size(); i++)
        {
            if (qdot[i]>qdot_limits[i])
            {
                qdot_out[i]=qdot_limits[i];
				cout << "Max positive vel in axis " << i << endl;
            }
            else if(qdot[i]<-qdot_limits[i])
            {
                qdot_out[i]=-qdot_limits[i];
				cout << "Max negative vel in axis " << i << endl;
            }
        }
        return qdot_out;
    }
    //Returns the limited joint position taking into account the joint position limits which are saved as global variables and should be set previously (in the Init function)
    VectorXd LimitJointPos(VectorXd q)
    {
        VectorXd q_out=q;
        for (int i=0; i<q.size(); i++)
        {
            if (q[i]>q_limits[i])
            {
                q_out[i]=q_limits[i];
                cout << "Max joint pos in axis " << i << endl;
            }
            else if(q[i]<-q_limits[i])
            {
                q[i]=-q_limits[i];
                cout << "Min joint pos in axis " << i << endl;
            }
        }
        return q_out;
    }
}


/* TODO: FUNCIONES DE DINÁMICA NO UTILIZADAS DE MOMENTO

    MatrixXd MInertiaAij(VectorXd joint_position);
    MatrixXd LinkInertia(Vector3d inertia, double mass);
    MatrixXd LinkInertiaS(Matrix4d Hsli0, Vector3d inertia, double mass);
    //1by3 translation vector --> 4by4 tform
    Matrix4d trvec2tform(Vector3d trvec)
    {
        Matrix4d T=MatrixXd::Identity(4,4);
        T.block<3,1>(0,3)=trvec;
        return T;
    }
    //float i + float j + float theta + twists (data) --> 6by6 adjoint form
    MatrixXd Aij2adjoint(int i, int j, VectorXd joint_position)
    {
        cout << "Aij2adjoint from " << i << " to " << j << endl;
        MatrixXd adjoint;
        if (i<j)
        {
            adjoint=MatrixXd::Zero(6,6);
        }
        else if (i==j)
        {
            adjoint=MatrixXd::Identity(6,6);
        }
        else
        {
            Matrix4d PoE=ProductOfExponentials(joint_position, j+1, i);
            adjoint=tform2adjoint(PoE).inverse();
        }
        return adjoint;
    }

    MatrixXd LinkInertia(Vector3d inertia, double mass)
    {
        //General form for the link inertia matrix containing the mass of the link and the moments of inertia for some link. Receives the 3by1 inertia tensor and the mass of the link and returns a 6by6 matrix.
        MatrixXd I=MatrixXd::Zero(6,6);
        I(0,0)=mass;
        I(1,1)=mass;
        I(2,2)=mass;
        I(3,3)=inertia(0);
        I(4,4)=inertia(1);
        I(5,5)=inertia(2);
        return I;
    }
    MatrixXd LinkInertiaS(MatrixXd Hsli0, Vector3d inertia, double mass)
    {
        //Calculates the link inertia matrix expressed in the S frame.
        // First calculate the generalised link inertia matrix for this link given the inertia tensor and the mass of the link
        MatrixXd I=LinkInertia(inertia, mass);
        // Then transform this into the space frame. For this aim, the 4by4 matrix Hsli0 containing the information about the center of mass of the link is used. First, calculate its adjoint form, and invert this matrix. Hsli0 expresses the center of mass of the link in the space frame. By inverting its adjoint, you get the adjoint form of the space frame expressed in this link frame.
        // By right-multiplying the generalised link inertia matrix by the inverse of the adjoint form of Hsli0 and left-multiplying by its transposed, you get the generalised link inertia matrix reflected into the space frame.
        MatrixXd ad_inv=tform2adjoint(Hsli0).inverse();
        MatrixXd I_S=ad_inv.transpose()*I*ad_inv;
        return I_S;
    }
    MatrixXd MInertiaAij(VectorXd joint_position)
    {
        //Calculus of the inertia matrix for an open-chain manipulator.
        // Each row of this matrix represent the inertia induced in the base of the robot by each link. The inertia matrix should be symmetric
        MatrixXd Mt=MatrixXd::Zero(7,7);
        int n=joint_position.size();
        for (int i=0; i<n; i++)
        {
            for (int j=0; j<n; j++)
            {
                int k=max(i,j);
                //For each i,j of the M matrix, take into account the further joint (max(i,j)) form the base and calculate the inertia from the base to that joint. k represents the further joint from the base for that index i,j
                //[ 0   1   2   3   4   5   6;
                //  1   1   2   3   4   5   6;
                //  2   2   2   3   4   5   6;
                //  3   3   3   3   4   5   6;
                //  4   4   4   4   4   5   6;
                //  5   5   5   5   5   5   6;
                //  6   6   6   6   6   6   6]
                for (int l=k; l<n; l++)
                {
                    //l=
                    //[ 0-6   1-6   2-6   3-6   4-6   5-6   6-6;
                    //  1-6   1-6   2-6   3-6   4-6   5-6   6-6;
                    //  2-6   2-6   2-6   3-6   4-6   5-6   6-6;
                    //  3-6   3-6   3-6   3-6   4-6   5-6   6-6;
                    //  4-6   4-6   4-6   4-6   4-6   5-6   6-6;
                    //  5-6   5-6   5-6   5-6   5-6   5-6   6-6;
                    //  6-6   6-6   6-6   6-6   6-6   6-6   6-6]
                    // For each term, it will be taken into account the inertia induced by link i and link j on link l
                    // Center of mass of link l
                    MatrixXd Hsli0=trvec2tform(CentersOfMass.block(0,l,3,1));
                    // Inertia of link l
                    Vector3d inertia=Inertias.block(0,l,3,1);
                    // Mass of link l
                    double mass=Masses[l];
                    // Link transformed inertia matrix expressed in the space frame
                    MatrixXd I_S=LinkInertiaS(Hsli0, inertia, mass);
                    // Twist corresponding to link i
                    VectorXd twisti=Twists.block(0,i,6,1);
                    // Twist corresponding to link j
                    VectorXd twistj=Twists.block(0,j,6,1);

                    //Inertia induced by link i in link l
                    MatrixXd Ali=Aij2adjoint(l,i,joint_position);
                    //Inertia induced by link j in link l
                    MatrixXd Alj=Aij2adjoint(l,j,joint_position);
                    //Calculate the twisti'*Ali'*Il_S*Alj*twist
                    VectorXd MISij=twisti.transpose()*Ali.transpose();
                    cout << "MISij: " << endl << MISij << endl;
                    VectorXd MISij2=MISij.transpose()*I_S;
                    cout << "MISij2: " << endl << MISij2 << endl;
                    double MISij3=MISij2.transpose()*Alj*twistj;
                    Mt(i,j)=Mt(i,j)+MISij3;
                }
            }
        }
        cout << "Mt: " << endl << Mt << endl;
        cout << "jpos: " << endl << joint_position << endl;
        return Mt;
    }
    /*Explicacion: Tienes una matrix de 7x7 vacía, de ceros. Para cada término vas a tener en cuenta la inercia que se ejerce desde el link más alejado hasta el final. Es decir, la l para cada i,j quedaría así:
                        [ 0-6   1-6   2-6   3-6   4-6   5-6   6-6;
                          1-6   1-6   2-6   3-6   4-6   5-6   6-6;
                          2-6   2-6   2-6   3-6   4-6   5-6   6-6;
                          3-6   3-6   3-6   3-6   4-6   5-6   6-6;
                          4-6   4-6   4-6   4-6   4-6   5-6   6-6;
                          5-6   5-6   5-6   5-6   5-6   5-6   6-6;
                          6-6   6-6   6-6   6-6   6-6   6-6   6-6]
    Ahora, para cada una de esas ls, por ejemplo en el primer término, estás teniendo en cuenta todas las inercias, porque el maximo de 0,0 es 0 entonces tendrías que hacer sumatorio de 0 a 6, todos los links, porque todos están influenciando en esa articulación. Bueno, entonces, iteras de 0 a 6, y para cada iteración, tienes en cuenta la inercia que ejerce el link l en el link i (Ali) y la inercia que ejerce el link l en el link j (Alj). Cuando l sea 0, como i es 0, Ali será la identidad porque no se genera inercia a si mismo y lo deja igual. Alj también será la identidad.
    Entonces para l=0, i=0, j=0, usas la formulica esa que coge y multiplica el twist_i'*Ali'*inercia_l*Alj*twist_j, y básicamente como tanto Ali como Alj son la identidad, simplemente multiplicará twist_i'*inercia_l*twist_j, y eso lo que hace es quedarse con la parte de la inercia_l que esté en la misma dirección que el twist i y que el twist j. Por ejemplo, si el twist_0 es una rotación positiva en z, se quedará con la parte de la matriz de inercia positivo en z. Es decir, con el componente z del tensor de inercia, pero sudará de la masa del link porque no se influye a si mismo en absoluto.
    Siendo i,j todavía 0, cuando l sea 1, es decir, cuando midas la inercia que ejerce el segundo link en el primero, Ali será la influencia del link 1 en el link 0, que se calcula con el adjunto del producto de exponenciales de 0 a 1 (de las exponenciales de los dos primeros twists)*/
