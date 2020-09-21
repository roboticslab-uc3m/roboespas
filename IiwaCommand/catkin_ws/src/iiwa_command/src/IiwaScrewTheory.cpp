#ifndef IIWA_ST
#define IIWA_ST

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <math.h>

using namespace Eigen;
namespace IiwaScrewTheory
{
    MatrixXd IiwaTwists(6,7);
    Matrix4d Hst0 = Matrix4d::Identity();
    VectorXd qdot_max(7);

//-- Function declarations
    void SetParameters(MatrixXd IiwaTwists_, Matrix4d Hst0_, VectorXd qdot_max_);
    static Vector3d Rotm2Eul(Matrix3d R);
    static Matrix3d Eul2Rotm(Vector3d eul);
    static Vector4d Rotm2AxisAngle(Matrix3d R);
    static Vector3d Rotm2Axis(Matrix3d R);
    static Matrix3d AxisAngle2Rotm(Vector3d axis, double angle);
    static Matrix3d Axis2Skew(Vector3d axis);
    static Matrix4d Screw2Tform(VectorXd screw, double angle);
    static MatrixXd Tform2Adjoint(Matrix4d T);
    static Vector3d RotOrientation(Vector3d ori_A, Vector3d axang_A);
    static VectorXd TransformScrew_A2S(VectorXd screw_A, VectorXd frame_SA);
    static VectorXd TransformFrame(VectorXd frame_A, VectorXd screw_A);
    static Matrix3d AxisAngleExponential(Vector3d om, double mag);
    static Matrix4d ScrewExponential(VectorXd screw, double mag);
    static Vector3d AxisAngleA2B_A(Vector3d axang_A, Vector3d axang_B);
    static VectorXd ScrewA2B_A(VectorXd x_A, VectorXd x_B);
    static MatrixXd GeoJacobianS(VectorXd q_curr);
    static VectorXd ForwardKinematics(VectorXd q);
    static VectorXd InverseDifferentialKinematicsPoint(VectorXd q_curr, VectorXd xdot);

    void SetParameters(MatrixXd IiwaTwists_, Matrix4d Hst0_, VectorXd qdot_max_)
    {
        IiwaTwists = IiwaTwists_;
        Hst0 = Hst0_;
        qdot_max = qdot_max_;
    }
//--Notation transformations
    static Vector3d Rotm2Eul(Matrix3d R)
    {
        //Euler taken in XYZ order
        return R.eulerAngles(0,1,2);
    }
    static Matrix3d Eul2Rotm(Vector3d eul)
    {
        //Euler taken in XYZ order
        AngleAxisd rollAngle(eul[0], Vector3d::UnitX());
        AngleAxisd yawAngle(eul[1], Vector3d::UnitY());
        AngleAxisd pitchAngle(eul[2], Vector3d::UnitZ());
        Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
        Matrix3d R = q.matrix();
        return R;
    }
    static Vector4d Rotm2AxisAngle(Matrix3d R)
    {
        //TODO: Add description to all functions in this file
        AngleAxisd axang(R);
        Vector4d axang_4;
        axang_4.head(3) = axang.axis();
        axang_4[3] = axang.angle();
        return axang_4;
    }
    static Vector3d Rotm2Axis(Matrix3d R)
    {
        AngleAxisd axang(R);
        Vector3d axang_3;
        axang_3 = axang.axis() * axang.angle();
        return axang_3;
    }
    static Matrix3d AxisAngle2Rotm(Vector3d axis, double angle)
    {
        if ((axis.norm()-1)>0.001)
        {
            ROS_ERROR("IiwaScrewTheory::AxisAngle2Rotm: Received a non-unit axis.");
        }
        axis = axis/axis.norm();
        AngleAxisd axang(angle, axis);
        return axang.toRotationMatrix();
    }
    static Matrix3d Axis2Skew(Vector3d axis)
    {
        Matrix3d mat;
        mat << 0.0, -axis[2], axis[1], axis[2], 0.0, -axis[0], -axis[1], axis[0], 0.0;
        return mat;
    }
    static Matrix4d Screw2Tform(VectorXd screw, double mag)
    {
        Matrix4d T = MatrixXd::Identity(4,4);
        Matrix3d R = Matrix3d::Identity(3,3);
        if (screw.size()!=6)
        {
            ROS_ERROR("Screw2Tform: Wrong screw size");
            ros::Duration(4).sleep();
            return T;
        }
        Vector3d vee = screw.head(3);
        Vector3d om = screw.tail(3);

        R = AxisAngle2Rotm(om, mag);
        Vector3d t;
        if (vee.norm() ==0)
        {
            t = (Matrix3d::Identity() - R) * om.cross(vee);
        }
        else if (om.norm() ==0)
        {
            t = vee * mag;
        }
        else
        {
            t = (Matrix3d::Identity() - R) * om.cross(vee) + om*om.transpose()*vee*mag;
        }
        T.block(0,0,3,3)=R;
        T.block(0,3,3,1) = t;
        return T;
    }
    static MatrixXd Tform2Adjoint(Matrix4d T)
    {
        Eigen::MatrixXd Ad(6,6);
        Eigen::Matrix3d R = T.block(0,0,3,3);
        Eigen::Vector3d t = T.block(0,3,3,1);
        Ad.block(0,0,3,3) = R;
        Ad.block(0,3,3,3) = Axis2Skew(t)*R;
        Ad.block(3,0,3,3) = Matrix3d::Zero();
        Ad.block(3,3,3,3) = R;
        return Ad;
    }
//Frame transformations
    static Vector3d RotOrientation(Vector3d ori_A, Vector3d axang_A)
    {
        Vector3d ori_B = ori_A;
        double angle = axang_A.norm();
        if (angle !=0)
        {
            Matrix3d R_AB = AxisAngle2Rotm(axang_A/angle, angle);
            Matrix3d R_SA = Eul2Rotm(ori_A);
            Matrix3d R_SB = R_SA*R_AB;
            ori_B = Rotm2Eul(R_SB);
        }
        return ori_B;
    }
    static VectorXd TransformScrew_A2S(VectorXd screw_A, VectorXd frame_SA)
    {
        VectorXd screw_S(6);
        screw_S.tail(3) = Eul2Rotm(frame_SA.tail(3))*screw_A.tail(3);
        Eigen::Vector3d pos_frame_SA = frame_SA.head(3);
        Eigen::Vector3d ori_screw_S = screw_S.tail(3);
        screw_S.head(3) = screw_A.head(3) - ori_screw_S.cross(pos_frame_SA);
        return screw_S;
    }
    static VectorXd TransformFrame(VectorXd frame_A, VectorXd screw_A)
    {
        VectorXd frame_B(6);
        frame_B.head(3) = frame_A.head(3) + screw_A.head(3);
        frame_B.tail(3) = RotOrientation(frame_A.tail(3), screw_A.tail(3));
        return frame_B;
    }
//--Exponentials
    static Matrix3d AxisAngleExponential(Vector3d om, double mag)
    {
        Matrix3d om_ss = Axis2Skew(om);
        Matrix3d R = Matrix3d::Identity() + om_ss*sin(mag)+ om_ss*om_ss*(1-cos(mag));
        return R;
    }
    static Matrix4d ScrewExponential(VectorXd screw, double mag)
    {
        Matrix4d H = MatrixXd::Identity(4,4);
        Matrix3d R = Matrix3d::Identity(3,3);
        if (screw.size()!=6)
        {
            ROS_ERROR("ScrewExponential: Wrong screw size");
            ros::Duration(4).sleep();
            return H;
        }
        Vector3d vee = screw.head(3);
        Vector3d om = screw.tail(3);

        R = AxisAngleExponential(om, mag);
        Vector3d t;
        if (vee.norm() ==0)
        {
            t = (Matrix3d::Identity() - R) * om.cross(vee);
        }
        else if (om.norm() ==0)
        {
            t = vee * mag;
        }
        else
        {
            t = (Matrix3d::Identity() - R) * om.cross(vee) + om*om.transpose()*vee*mag;
        }
        H.block(0,0,3,3)=R;
        H.block(0,3,3,1) = t;
        return H;
    }
//New frame calculation
    static Vector3d AxisAngleA2B_A(Vector3d axang_A, Vector3d axang_B)
    {
        Matrix3d R_SA = Eul2Rotm(axang_A);
        Matrix3d R_SB = Eul2Rotm(axang_B);
        Matrix3d R_AS = R_SA.transpose();
        Matrix3d R_AB = R_AS * R_SB;
        Vector3d axang_3 = Rotm2Axis(R_AB);
        return axang_3;
    }
    static VectorXd ScrewA2B_A(VectorXd x_A, VectorXd x_B)
    {
        VectorXd screw(6);
        screw.head(3) = x_B.head(3)-x_A.head(3);
        screw.tail(3) = AxisAngleA2B_A(x_A.tail(3), x_B.tail(3));
        return screw;
    }
//--
    static MatrixXd GeoJacobianS(VectorXd q_curr)
    {
        MatrixXd JstS = IiwaTwists;
        Matrix4d PoE = Matrix4d::Identity();
        for (int i=0; i<q_curr.size(); i++)
        {
            JstS.col(i) = Tform2Adjoint(PoE) * IiwaTwists.col(i);
            PoE = PoE*Screw2Tform(IiwaTwists.col(i), q_curr[i]);
        }
        return JstS;
    }
//--Kinematics
    static VectorXd ForwardKinematics(VectorXd q)
    {
        VectorXd x(6);
        if (q.size() != 7)
        {
            ROS_ERROR("Forward kinematics: Wrong joint position size");
            return x;
        }
        VectorXd twist(6);
        Matrix4d PoE = Matrix4d::Identity();
        for (int i=0; i<q.size(); i++)
        {
            PoE = PoE*ScrewExponential(IiwaTwists.col(i), q[i]);
        }
        Matrix4d Hst = PoE * Hst0;
        x.head(3) = Hst.block(0,3,3,1);
        Matrix3d R = Hst.block(0,0,3,3);
        x.tail(3) = Rotm2Eul(R);
        return x;
    }
//--Differential Kinematics
    static VectorXd InverseDifferentialKinematicsPoint(VectorXd q_curr, VectorXd xdot_S)
    {
        MatrixXd JstS = GeoJacobianS(q_curr);
        MatrixXd JstS_inv = JstS.transpose()*(JstS*JstS.transpose()).inverse();
        VectorXd qdot = JstS_inv*xdot_S;//.transpose();
        return qdot;
    }
}
#endif
