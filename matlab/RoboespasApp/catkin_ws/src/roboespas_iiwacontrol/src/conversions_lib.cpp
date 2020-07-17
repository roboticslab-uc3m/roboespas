#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointVelocity.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
namespace Conversions
{
    iiwa_msgs::JointQuantity StdVectorToIiwaJointQuantity(vector<double> jq);
    iiwa_msgs::JointPosition StdVectorToIiwaJointPosition(vector<double> q);
    iiwa_msgs::JointQuantity StdVectorToIiwaJointVelocityy(vector<double> qdot);
    string StdVectorToCartesianString(vector<double> x);
    Eigen::VectorXd StdVectorToEigenVector(vector<double> std_vector);

    geometry_msgs::Twist EigenVectorToTwist(Eigen::VectorXd eigen_vector);
    vector<double> EigenVectorToStdVector(Eigen::VectorXd eigen_vector);

    vector<double> JointPositionToStdVector(iiwa_msgs::JointPosition q);

    string JointQuantityToString(const iiwa_msgs::JointQuantity q);

    Eigen::VectorXd TwistToEigenVector(geometry_msgs::Twist twist_vector);

    // From std::vector to ...
    iiwa_msgs::JointQuantity StdVectorToIiwaJointQuantity(vector<double> jq)
    {
        iiwa_msgs::JointQuantity jq_iiwa;
        jq_iiwa.a1=jq[0];
        jq_iiwa.a2=jq[1];
        jq_iiwa.a3=jq[2];
        jq_iiwa.a4=jq[3];
        jq_iiwa.a5=jq[4];
        jq_iiwa.a6=jq[5];
        jq_iiwa.a7=jq[6];
        return jq_iiwa;
    }
    iiwa_msgs::JointPosition StdVectorToIiwaJointPosition(vector<double> q)
    {
        iiwa_msgs::JointPosition q_iiwa;
        q_iiwa.position=StdVectorToIiwaJointQuantity(q);
        return q_iiwa;
    }
    iiwa_msgs::JointVelocity StdVectorToIiwaJointVelocity(vector<double> qdot)
    {
        iiwa_msgs::JointVelocity qdot_iiwa;
        qdot_iiwa.velocity=StdVectorToIiwaJointQuantity(qdot);
        return qdot_iiwa;
    }
    string StdVectorToCartesianString(vector<double> x)
    {
        stringstream ss;
        ss << fixed << setprecision(9);
        ss << "x: " << x[0] << endl;
        ss << "y: " << x[1] << endl;
        ss << "z: " << x[2] << endl;
        ss << "rx: " << x[3] << endl;
        ss << "ry: " << x[4] << endl;
        ss << "rz: " << x[5] << endl;
        string ss_str=ss.str();
        return ss_str;
    }
    Eigen::VectorXd StdVectorToEigenVector(vector<double> std_vector)
    {
        Eigen::VectorXd eigen_vector(std_vector.size());
        eigen_vector=Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(std_vector.data(), std_vector.size());
        return eigen_vector;
    }
    vector<double> AddStdVectorToStdVector(vector<double> jq_vec, vector<double> vec)
    {
        jq_vec.push_back(vec[0]);
        jq_vec.push_back(vec[1]);
        jq_vec.push_back(vec[2]);
        jq_vec.push_back(vec[3]);
        jq_vec.push_back(vec[4]);
        jq_vec.push_back(vec[5]);
        jq_vec.push_back(vec[6]);
        return jq_vec;
    }
    
    //From Eigen::Eigen::VectorXd to ...
    geometry_msgs::Twist EigenVectorToTwist(Eigen::VectorXd eigen_vector)
    {
        geometry_msgs::Twist twist;
        twist.linear.x=eigen_vector[0];
        twist.linear.y=eigen_vector[1];
        twist.linear.z=eigen_vector[2];
        twist.angular.x=eigen_vector[3];
        twist.angular.y=eigen_vector[4];
        twist.angular.z=eigen_vector[5];
        return twist;
    }
    vector<double> EigenVectorToStdVector(Eigen::VectorXd eigen_vector)
    {
        vector<double> std_vector(eigen_vector.data(), eigen_vector.data()+eigen_vector.rows()*eigen_vector.cols());
        return std_vector;
    }

    //JointPosition to...
    vector<double> JointPositionToStdVector(iiwa_msgs::JointPosition q)
    {
	    vector<double> q_std;
	    q_std.push_back(q.position.a1);
	    q_std.push_back(q.position.a2);
	    q_std.push_back(q.position.a3);
	    q_std.push_back(q.position.a4);
	    q_std.push_back(q.position.a5);
	    q_std.push_back(q.position.a6);
	    q_std.push_back(q.position.a7);
	    return q_std;
    }

    //JointQuantity to...    
    string ToJointQuantityString(const iiwa_msgs::JointQuantity q)
    {
	    stringstream ss;
	    ss << fixed << setprecision(9);
	    ss << "a1: " << q.a1 << endl;
	    ss << "a2: " << q.a2 << endl;
	    ss << "a3: " << q.a3 << endl;
	    ss << "a4: " << q.a4 << endl;
	    ss << "a5: " << q.a5 << endl;
	    ss << "a6: " << q.a6 << endl;
	    ss << "a7: " << q.a7 << endl;
	    string ss_str=ss.str();
	    return ss_str;
    }

    //geometry_msgs::Twist to...
    Eigen::VectorXd TwistToEigenVector(geometry_msgs::Twist twist_vector)
    {
        Eigen::VectorXd eigen_vector(6);
        eigen_vector[0]=twist_vector.linear.x;
        eigen_vector[1]=twist_vector.linear.y;
        eigen_vector[2]=twist_vector.linear.z;
        eigen_vector[3]=twist_vector.angular.x;
        eigen_vector[4]=twist_vector.angular.y;
        eigen_vector[5]=twist_vector.angular.z;
        return eigen_vector;
    }
    vector<double> AddTwistToStdVector(vector<double> x_vec, geometry_msgs::Twist x)
    {
        x_vec.push_back(x.linear.x);
        x_vec.push_back(x.linear.y);
        x_vec.push_back(x.linear.z);
        x_vec.push_back(x.angular.x);
        x_vec.push_back(x.angular.y);
        x_vec.push_back(x.angular.z);
        return x_vec;
    }

}
