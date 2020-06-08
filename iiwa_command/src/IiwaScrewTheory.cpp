#ifndef IIWA_ST
#define IIWA_ST

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>

using namespace Eigen;
namespace IiwaScrewTheory
{
    double prueba;
    MatrixXd IiwaTwists(6,7);
    Matrix4d Hst0 = Matrix4d::Identity();
    void SetParameters(MatrixXd IiwaTwists_, Matrix4d Hst0_)
    {
        IiwaTwists = IiwaTwists_;
        Hst0 = Hst0_;
    }
//--Rotation transformations
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
    static Matrix3d AxisToSkew(Vector3d axis)
    {
        Matrix3d mat;
        mat << 0.0, -axis[2], axis[1], axis[2], 0.0, -axis[0], -axis[1], axis[0], 0.0;
        return mat;
    }
//--Exponentials
    static Matrix3d AxangExponential(Vector3d om, double mag)
    {
        Matrix3d om_ss = AxisToSkew(om);
        Matrix3d R = Matrix3d::Identity() + om_ss*sin(mag)+ om_ss*om_ss*(1-cos(mag));
        return R;        
    }
    static Matrix4d ScrewExponential(VectorXd screw, double mag)
    {
        Matrix4d H = MatrixXd::Identity(4,4);
        Matrix3d R = Matrix3d::Identity(3,3);
        if (screw.size()!=6)
        {
            std::cout << screw << std::endl;
            ROS_ERROR("ScrewExponential: Wrong screw size");
            ros::Duration(4).sleep();
            return H;
        }
        Vector3d vee = screw.head(3);
        Vector3d om = screw.tail(3);
        //std::cout << "vee: " << vee.transpose()  << " om: " << om.transpose() << " mag: " << mag << std::endl;

        R = AxangExponential(om, mag);
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
//Orientations operations
    static Vector3d AxangA2B_A(Vector3d axang_A, Vector3d axang_B)
    {
        Matrix3d R_SA = Eul2Rotm(axang_A);
        Matrix3d R_SB = Eul2Rotm(axang_B);
        Matrix3d R_AS = R_SA.transpose();
        Matrix3d R_AB = R_AS * R_SB;
        Vector3d axang_3 = Rotm2Axis(R_AB);
        return axang_3;
    }
//Screw operations
    static VectorXd ScrewA2B_A(VectorXd x_A, VectorXd x_B)
    {
        VectorXd screw(6);
        screw.head(3) = x_B.head(3)-x_A.head(3);
        screw.tail(3) = AxangA2B_A(x_A.tail(3), x_B.tail(3));
        return screw;   
    }
}
#endif
