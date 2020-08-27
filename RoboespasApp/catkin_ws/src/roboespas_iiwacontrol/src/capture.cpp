#include <cstdlib>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <string>
#include "ros/ros.h"

#include "roboespas_iiwacontrol/ClearCapture.h"
#include "roboespas_iiwacontrol/StopCapture.h"
#include "roboespas_iiwacontrol/StartCapture.h"
#include "roboespas_iiwacontrol/SaveCapture.h"
#include "roboespas_iiwacontrol/LoadCapture.h"
#include "roboespas_iiwacontrol/NewCapture.h"
#include "roboespas_iiwacontrol/ListCapture.h"

#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointVelocity.h"
#include "iiwa_msgs/JointTorque.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "capture_lib.cpp"


using namespace std;

//|--------------|
//| CAPTURE NODE | Node that offers 5 services related with data capture
//|--------------|
// - /roboespas/capture/start -> Starts the capture of the position, velocity and torque of the robot, saving the info into some temporary files
// - /roboespas/capture/stop -> Stops the capture, but does not clear the files
// - /roboespas/capture/save -> Copy those temporary files into some given folder
// - /roboespas/capture/clear -> Clears those temporary files
// - /roboespas/capture/load -> Reads the files inside some folder and returns all the information about the trajectory found in that folder in the form of different ros messages


//Global variables:
//Boolean to start/stop the capture of data
bool capture=false;
//String that will contain the place to save the data after setting it in the initialization (read from .yaml config file)
string dataFilePath="";
//File names
string q_filename="jointPosition.txt";
string x_filename="cartesianPosition.txt";
string t_q_filename="stamps.txt";
string q_torque_filename="jointTorque.txt";
string t_q_torque_filename="stampsJointTorque.txt";
string q_dot_filename="jointVelocity.txt";
string x_dot_filename="cartesianVelocity.txt";
string pp_filename="polynomials.txt";
double last_time_stamp=-1.0;


void q_Callback(const iiwa_msgs::JointPosition::ConstPtr& msg)
{
	//Callback that saves the joint position received message from the topic /iiwa/state/JointPosition in a temporary file inside /src/roboespas/data folder, only if the global variable "capture" is true
	if (capture==true)
	{
		//Create filepaths to save the message info
        string q_filepath = dataFilePath + q_filename;
        string x_filepath = dataFilePath + x_filename;
        string t_q_filepath = dataFilePath + t_q_filename;
        //Save stamp
		ofstream fs_t;
		fs_t.open (t_q_filepath, fstream::out | fstream::app);
		double stamp=msg->header.stamp.toSec();
        if (stamp!=last_time_stamp)
        {
            last_time_stamp=stamp;
		    stringstream ss;
		    ss << fixed << setprecision(9) << stamp;
		    string mystring = ss.str();
		    fs_t << mystring;
		    fs_t << endl;
		    fs_t << endl;
		    fs_t.close();

            //Save joint position
		    ofstream fs_q;
		    fs_q.open (q_filepath, fstream::out | fstream::app);
		    fs_q << Conversions::ToJointQuantityString(msg->position);
		    fs_q << endl;
		    fs_q.close();

		    //Save cartesian position
		    ofstream fs_x;
		    fs_x.open (x_filepath, fstream::out | fstream::app);
		    vector<double> x = ScrewTheory::ForwardKinematics(Conversions::JointPositionToStdVector(*msg));
		    string x_str = Conversions::StdVectorToCartesianString(x);
		    fs_x << x_str;
		    fs_x << endl;
		    fs_x.close();
        }
	}
}

void q_dot_Callback(const iiwa_msgs::JointVelocity::ConstPtr& msg)
{
	//Callback that saves the joint velocity received message from the topic /iiwa/state/JointVelocity in a temporary file inside /src/roboespas/data folder, only if the global variable "capture" is true
	if (capture==true)
	{
		//Create filepath to save the message info
        string q_dot_filepath = dataFilePath + q_dot_filename;
        //Save joint velocity info
		ofstream fs_q_dot;
		fs_q_dot.open (q_dot_filepath, fstream::out | fstream::app);
		fs_q_dot << msg->velocity;
		fs_q_dot << endl;
		fs_q_dot.close();
	}
}

void q_torque_Callback(const iiwa_msgs::JointTorque::ConstPtr& msg)
{
    //Callback that saves the joint torque received message from the topic /iiwa/state/JointTorque in a temporary file inside /src/roboespas/data folder, only if the global variable "capture" is true
	if (capture==true)
	{
        //Create filepath to save the message info
        string q_torque_filepath = dataFilePath + q_torque_filename;
        string t_q_torque_filepath = dataFilePath + t_q_torque_filename;
		//Save torque
		ofstream fs_q_torque;
		fs_q_torque.open (q_torque_filepath, fstream::out | fstream::app);
		fs_q_torque << Conversions::ToJointQuantityString(msg->torque);
		fs_q_torque << endl;
		fs_q_torque.close();
		//Save stamp in temporary file
		ofstream fs_t;
		fs_t.open (t_q_torque_filepath, fstream::out | fstream::app);
		//Save the current stamp in seconds with double precission
		double t=msg->header.stamp.toSec();
		stringstream ss;
		ss << fixed << setprecision(9) << t;
		string t_str = ss.str();
		fs_t << t_str;
		fs_t << endl;
		fs_t << endl;
		fs_t.close();
		
	}
}

bool captureClear(roboespas_iiwacontrol::ClearCapture::Request &req, roboespas_iiwacontrol::ClearCapture::Response &res)
{
	//Service function that remove the temporary files in the /src/roboespas/data folder
    Capture::Clear(dataFilePath);
	res.success=true;
	ROS_INFO("Capture clear");
	return true;
}

bool captureStart(roboespas_iiwacontrol::StartCapture::Request &req, roboespas_iiwacontrol::StartCapture::Response &res)
{
	//Service function that sets the global variable "capture" as true, so all callbacks start copying the received messages from /iiwa/state/JointPosition, JointVelocity, JointTorque are saved in some temporary files inside /src/roboespas/data folder.
    capture=true;
	ROS_INFO("Capture start");
	res.success=true;
	return true;
}

bool captureStop(roboespas_iiwacontrol::StopCapture::Request &req, roboespas_iiwacontrol::StopCapture::Response &res)
{
    //Service function that sets the global variable "capture" as false, so all callbacks stop copying the received messages in the temporary files.
	capture=false;
	ROS_INFO("Capture stop");
	res.success=true;
	return true;
}

bool captureSave(roboespas_iiwacontrol::SaveCapture::Request &req, roboespas_iiwacontrol::SaveCapture::Response &res)
{
    //Service function that copies the temporary files inside /src/roboespas/req.data/req.name/folder directory.
    Capture::Save(dataFilePath, req.name, req.folder);
	res.success=true;
	ROS_INFO("Capture saved in %s", (dataFilePath+req.name+"/"+req.folder+"/").c_str());
	return true;
}

bool captureLoad(roboespas_iiwacontrol::LoadCapture::Request &req, roboespas_iiwacontrol::LoadCapture::Response &res)
{
    //Service function that loads the files inside the /src/roboespas/req.data/req.name folder and returns the info inside them in the following variables: 
    //- joint_trajectory -> Joint position trajectory + Time stamps for each of them
    //- cartesian_trajectory -> Cartesian pose trajectory
    //- xdot_traj -> Cartesian velocity trajectory if that info is found in the folder
    //- tKnot -> If the info about the polynomials is available, tKnot is a vector of doubles with the timestamps that divide each segment from the following
    //- q_coefs -> Coefficients for the joint trajectory polynomials ordered as follows: [c_1_joint1_point1, c_2_joint1_point1, c_3_joint1_point1, c_4_joint1_point1, c_1_joint2_point1, ..., c_1_joint1_point2, ...]. As ros cannot they are all in a list but ordered in some specific way
    //- qdot_coefs -> Same as before but for joint velocity trajectory polynomials taking into account that this time there are only 3 coefficients per joint and point instead of 4.
    //- qdotdot_coefs -> Same as before but this time with 2 coefficients per polynomial

    //First read the files
	string name=req.name;
	string folder=req.folder;
	string q_filepath=dataFilePath+name+"/"+folder+"/"+q_filename;
	string t_filepath=dataFilePath+name+"/"+folder+"/"+t_q_filename;
	ifstream q_file(q_filepath);
	ifstream t_file(t_filepath);
	//Check if the files are not empty
	if (!q_file.is_open())
   {
		res.success=false;
		ROS_INFO("Cannot load %s", q_filepath.c_str());
		return false;
	}
	if (!t_file.is_open())
	{
		res.success=false;
		ROS_INFO("Cannot load %s", t_filepath.c_str());
		return false;
	}
    //Call the function and fill the output   
    Capture::LoadVectors(name, folder, dataFilePath, res.t, res.q, res.x, res.qdot, res.xdot, res.tKnot, res.q_coefs, res.qdot_coefs, res.qdotdot_coefs, res.qtorque, res.ttorque);
	res.success=true;
	ROS_INFO("Capture loaded");
	return true;
}

bool captureList(roboespas_iiwacontrol::ListCapture::Request &req, roboespas_iiwacontrol::ListCapture::Response &res)
{
    std::vector<std::string> names;
    if (!boost::filesystem::exists(dataFilePath))
    {
	cout << dataFilePath << endl;
	ROS_ERROR("dataFilePath does not exist");
	res.names = names;
        return false;
    }
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end

    for (boost::filesystem::directory_iterator itr(dataFilePath); itr != end_itr; ++itr)
    {
        if (is_directory(itr->status()))
        {
            std::stringstream path_ss;
            path_ss << itr->path();
            std::string path=path_ss.str();
            std::string name=path.erase(0,dataFilePath.size()+1);
            name=name.erase(name.size()-1, name.size());
            names.push_back(name);
        }
    }
    std::sort(names.begin(), names.end());
    res.names=names;
    return true;
}

bool captureNew(roboespas_iiwacontrol::NewCapture::Request &req, roboespas_iiwacontrol::NewCapture::Response &res)
{
    //Service that receives some ordered vectors corresponding to:
    // - t: Stamps for the whole trajectory
    // - q: Joint trajectory ordered such that q=[point1_q1, point1_q2, ... point1_q7, ... pointn_q1, ... pointn_q7]
    // - qdot: Joint velocity trajectory ordered same way as q
    // - x: Cartesian trajectory ordered such that x=[p1_x p1_y, ... p1_rz, ..., pn_x, ... pn_rz]
    // - xdot: Cartesian velocity trajectory same way as x
    // - tKnot: Time stamps for the knots that divide the trajectory in polynomials
    // - q_coefs: Coefficients for all the polynomials describing the joint trajectory such that q_coefs=[p1_q1_c1 p1_q1_c2 p1_q1_c3 p1_q1_c4 p1_q2_c1 ... pn_q7_c4] (4 coefficients/polynomial, npoints polynomials for each of the 7 joints)
    // - qdot_coefs: Coefficients for the polynomials describing the joint velocity trajectory, same way as q_coefs but taking into account that just 3 coefficients per polynomial are saved.
    // - qdotdot_coefs: Coefficients for the polynomials describing the joint acceleration trajectory, same as other coefs but taking into account that just 2 coefficients per polynomial are saved
    // This service saves all this information in some files inside dataFilePath/name/folder, inside some .txt files
    string dir_filepath=dataFilePath+req.name;
    string folder_filepath=dataFilePath+req.name+"/"+req.folder;
    boost::filesystem::create_directory(dir_filepath);
    ros::Duration(0.01).sleep();
    boost::filesystem::create_directory(folder_filepath);
    ros::Duration(0.01).sleep();

    //Build the paths for the filenames where the data is saved
	string t_filepath=dataFilePath+req.name+"/"+req.folder+"/"+t_q_filename;
	string q_filepath=dataFilePath+req.name+"/"+req.folder+"/"+q_filename;
	string qdot_filepath=dataFilePath+req.name+"/"+req.folder+"/"+q_dot_filename;
	string x_filepath=dataFilePath+req.name+"/"+req.folder+"/"+x_filename;
	string xdot_filepath=dataFilePath+req.name+"/"+req.folder+"/"+x_dot_filename;
    string poly_filepath=dataFilePath+req.name+"/"+req.folder+"/"+pp_filename;

    //Remove if exist
	remove(t_filepath.c_str());
	remove(q_filepath.c_str());
	remove(qdot_filepath.c_str());
	remove(x_filepath.c_str());
	remove(xdot_filepath.c_str());
	remove(poly_filepath.c_str());
    //Create input file streams for each of those files
    
	ofstream t_file;
	ofstream q_file;
	ofstream qdot_file;
    ofstream x_file;
    ofstream xdot_file;
    ofstream poly_file;
    //Open files
    t_file.open(t_filepath, fstream::out | fstream::app);
    q_file.open(q_filepath, fstream::out | fstream::app);
    qdot_file.open(qdot_filepath, fstream::out | fstream::app);
    x_file.open(x_filepath, fstream::out | fstream::app);
    xdot_file.open(xdot_filepath, fstream::out | fstream::app);
    poly_file.open(poly_filepath, fstream::out | fstream::app);
    int nPoints=req.t.size();
    //Stamps file
    for (int idPoint=0; idPoint<nPoints; idPoint++)
    {
        stringstream ss;
        ss << fixed << setprecision(9) << req.t[idPoint];
        t_file << ss.str() << endl;
        t_file << endl;
    }
    t_file.close();
    //Joint trajectory file
    int n=0;
    for (int idPoint=0; idPoint<nPoints; idPoint++)
    {
        for (int idJoint=0; idJoint<7; idJoint++)
        {
            stringstream ss;
            ss << fixed << setprecision(9) << req.q[n];
            n=n+1;
            q_file << "a" << to_string(idJoint+1) << ": " << ss.str() << endl;
        }
        q_file << endl;
    }
    q_file.close();
    //Cartesian trajectory file
    n=0;
    for (int idPoint=0; idPoint<nPoints; idPoint++)
    {
        for (int idCoord=0; idCoord<6; idCoord++)
        {
            if (idCoord==0)
                x_file << "x: ";
            else if (idCoord==1)
                x_file << "y: ";
            else if (idCoord==2)
                x_file << "z: ";
            else if (idCoord==3)
                x_file << "rx: ";
            else if (idCoord==4)
                x_file << "ry: ";
            else if (idCoord==5)
                x_file << "rz: ";
            stringstream ss;
            ss << fixed << setprecision(9) << req.x[n];
            n=n+1;
            x_file << ss.str() << endl;
        }
        x_file << endl; 
    }
    x_file.close();
    //Joint velocity trajectory file
    n=0;
    for (int idPoint=0; idPoint<nPoints; idPoint++)
    {
        for (int idJoint=0; idJoint<7; idJoint++)
        {
            stringstream ss;
            ss << fixed << setprecision(9) << req.qdot[n];
            n=n+1;
            qdot_file << "a" << to_string(idJoint+1) << ": " << ss.str() << endl;
        }
        qdot_file << endl;
    }
    qdot_file.close();
    //Cartesian velocity trajectory file
    if (req.xdot.size()!=0)
    {
        n=0;
        for (int idPoint=0; idPoint<nPoints; idPoint++)
        {
            for (int idCoord=0; idCoord<6; idCoord++)
            {
                if (idCoord==0)
                    xdot_file << "x: ";
                else if (idCoord==1)
                    xdot_file << "y: ";
                else if (idCoord==2)
                    xdot_file << "z: ";
                else if (idCoord==3)
                    xdot_file << "rx: ";
                else if (idCoord==4)
                    xdot_file << "ry: ";
                else if (idCoord==5)
                    xdot_file << "rz: ";
                stringstream ss;
                ss << fixed << setprecision(9) << req.xdot[n];
                n=n+1;
                xdot_file << ss.str() << endl;
            }
            xdot_file << endl; 
        }
    }
    xdot_file.close();
    //Polynomials file
    if (req.tKnot.size()!=0)
    {
        int nSeg=req.tKnot.size()-1;
        poly_file << "nSeg: " << to_string(nSeg) << endl;
        poly_file << endl;
        poly_file << "breaks: " << endl;
        for (int idKnot=0; idKnot<req.tKnot.size(); idKnot++)
        {
            stringstream ss;
            ss << fixed << setprecision(15) << req.tKnot[idKnot];
            poly_file << ss.str() << endl;
        }
        poly_file << endl;
        poly_file << "coefs_q:" << endl;
        n=0;
        for (int idSeg=0; idSeg<nSeg; idSeg++)
        {
            for (int idJoint=0; idJoint<7; idJoint++)
            {
                poly_file << "q" << to_string(idJoint+1) << ":" << endl;
                for (int idCoef=0; idCoef<4; idCoef++)
                {
                    stringstream ss;
                    ss << fixed << setprecision(15) << req.q_coefs[n];
                    n=n+1;
                    poly_file << ss.str() << endl;
                }
            }
            poly_file << endl;
        }
        poly_file << "coefs_qdot:" << endl;
        n=0;
        for (int idSeg=0; idSeg<nSeg; idSeg++)
        {
            for (int idJoint=0; idJoint<7; idJoint++)
            {
                poly_file << "qdot" << to_string(idJoint+1) << ":" << endl;
                for (int idCoef=0; idCoef<3; idCoef++)
                {
                    stringstream ss;
                    ss << fixed << setprecision(15) << req.qdot_coefs[n];
                    n=n+1;
                    poly_file << ss.str() << endl;
                }
            }
            poly_file << endl;
        }
        poly_file << "coefs_qdotdot:" << endl;
        n=0;
        for (int idSeg=0; idSeg<nSeg; idSeg++)
        {
            for (int idJoint=0; idJoint<7; idJoint++)
            {
                poly_file << "qdotdot" << to_string(idJoint+1) << ":" << endl;
                for (int idCoef=0; idCoef<2; idCoef++)
                {
                    stringstream ss;
                    ss << fixed << setprecision(15) << req.qdotdot_coefs[n];
                    n=n+1;
                    poly_file << ss.str() << endl;
                }
            }
            poly_file << endl;
        }
        poly_file.close();
    }
	ROS_INFO("New capture created");
    res.success=true;    
    return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "capture");
	ros::NodeHandle n;
	ROS_INFO("Node registered as /capture");
	//----------------------------------------------------------------------------------------------------------
    //LOAD PARAMETERS
	//----------------------------------------------------------------------------------------------------------
    n.getParam("/roboespas/dataPath", dataFilePath);
	//----------------------------------------------------------------------------------------------------------
    //INITIALIZATE SCREW THEORY LIBRARY
	//----------------------------------------------------------------------------------------------------------  
	//Load Twists
    vector<vector<double>> twists;
    for (unsigned int i=1; i<=7; i++)
    {
        vector<double> twist;
        string str="/roboespas/iiwa/twists/t"+to_string(i);
        n.getParam(str, twist);
        twists.push_back(twist);

    }
    //Load Hst0
    vector<double> Hst0_axang;
    vector<double> Hst0_t;
    n.getParam("/roboespas/iiwa/Hst0/axang", Hst0_axang);
    n.getParam("/roboespas/iiwa/Hst0/t", Hst0_t);
    
    //Load joint_pos_limits and joint_vel_limits;
    vector<double> joint_vel_limits(7);
    n.getParam("/roboespas/iiwa/limits/joint_vel", joint_vel_limits);
    vector<double> joint_pos_limits(7);
    n.getParam("/roboespas/iiwa/limits/joint_pos", joint_pos_limits);

    //Load CM, IT, mass
    vector<vector<double>> centersofmass;
    vector<vector<double>> inertias;
    vector<double> masses;
    for (unsigned int i=1; i<=7; i++)
    {
        vector<double> com;
        string strcom="/roboespas/iiwa/links/com/com"+to_string(i);
        n.getParam(strcom, com);
        centersofmass.push_back(com);
        vector<double> inertia;
        string stri="/roboespas/iiwa/links/inertias/i"+to_string(i);
        n.getParam(stri, inertia);
        inertias.push_back(inertia);
        double m;
        string strm="/roboespas/iiwa/links/masses/m"+to_string(i);
        n.getParam(strm, m);
        masses.push_back(m);
    }
	//----------------------------------------------------------------------------------------------------------
    //INITIALIZATE LIBRARY
	//----------------------------------------------------------------------------------------------------------  
    ScrewTheory::Init(Hst0_t, Hst0_axang, twists, joint_pos_limits, joint_vel_limits, centersofmass, inertias, masses);
	//----------------------------------------------------------------------------------------------------------
	//SUSCRIBERS
	//----------------------------------------------------------------------------------------------------------
	//iiwa/state/JointPosition
	ros::Subscriber sub_position = n.subscribe("iiwa/state/JointPosition", 1, q_Callback);
	//iiwa/state/JointVelocity	
	ros::Subscriber sub_velocity = n.subscribe("iiwa/state/JointVelocity", 1, q_dot_Callback);
	//iiwa/state/JointTorque
	ros::Subscriber sub_torque = n.subscribe("iiwa/state/JointTorque", 1, q_torque_Callback);
	//----------------------------------------------------------------------------------------------------------
	//SERVICES OFFERED
	//----------------------------------------------------------------------------------------------------------
	//roboespas/capture/clear
	ros::ServiceServer clearService=n.advertiseService("/roboespas/capture/clear", captureClear);
  	ROS_INFO("Service offered /roboespas/capture/clear");
	//roboespas/capture/stop
    ros::ServiceServer stopService=n.advertiseService("/roboespas/capture/stop", captureStop);
  	ROS_INFO("Service offered /roboespas/capture/stop");
	//roboespas/capture/start
    ros::ServiceServer startService=n.advertiseService("/roboespas/capture/start", captureStart);
  	ROS_INFO("Service offered /roboespas/capture/start");
	//roboespas/capture/save
    ros::ServiceServer saveService=n.advertiseService("/roboespas/capture/save", captureSave);
  	ROS_INFO("Service offered /roboespas/capture/save");
	//roboespas/capture/load
    ros::ServiceServer loadService=n.advertiseService("/roboespas/capture/load", captureLoad);
  	ROS_INFO("Service offered /roboespas/capture/load");
	//roboespas/capture/new
    ros::ServiceServer newService=n.advertiseService("/roboespas/capture/new", captureNew);
  	ROS_INFO("Service offered /roboespas/capture/new");
    //roboespas/capture/list
    ros::ServiceServer listService=n.advertiseService("/roboespas/capture/list", captureList);
    ROS_INFO("Service offered /roboespas/capture/list");
	//----------------------------------------------------------------------------------------------------------
	//Loop
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
	return 0;
}


