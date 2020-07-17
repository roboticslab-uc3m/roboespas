#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include "screw_theory_lib.cpp"

using namespace std;

//|-----------------| Library that offers 3 main functions related with captured .txt files with
//| CAPTURE LIBRARY | trajectory info.
//|-----------------|
// - Save: Copy the files from atemporary folder to a selected folder
// - Clear: Deletes the files inside the temporary folder
// - Load: Reads the trajectory files inside some folders and returns several data structures containing the trajectory information

namespace Capture
{
    //Global variables
    string q_filename="jointPosition.txt";
    string x_filename="cartesianPosition.txt";
    string t_q_filename="stamps.txt";
    string q_torque_filename="jointTorque.txt";
    string t_q_torque_filename="stampsJointTorque.txt";
    string q_dot_filename="jointVelocity.txt";
    string x_dot_filename="cartesianVelocity.txt";
    string pp_filename="polynomials.txt";
    //Save
    void Save(string dataFilePath, string name, string folder);
    //Clear
    void Clear(string dataFilePath);
    //Load
    trajectory_msgs::JointTrajectory Load(string name, string folder, string dataFilePath, vector<geometry_msgs::Twist> &cartesian_trajectory_out,vector<geometry_msgs::Twist> &cartesian_velocities_out, vector<double> &tKnot_out, vector<double> &q_coefs_out, vector<double> &qdot_coefs_out, vector<double> &qdotdot_coefs_out);
    void LoadVectors(string name, string folder, string dataFilePath, vector<double> &t, vector<double> &q, vector<double> &x, vector<double> &qdot, vector<double> &xdot, vector<double> &tKnot, vector<double> &q_coefs, vector<double> &qdot_coefs, vector<double> &qdotdot_coefs, vector<double> &qtorque, vector<double> &ttorque);
    //LoadInterpolateSave
    trajectory_msgs::JointTrajectory LoadInterpolateSave(string name_input, string folder_input, string folder_output, string dataFilePath);
    //Other functions
    bool ReadDoubleFromFile(ifstream &file, int start, int end, double &ans);
    void copyFile(string fromFilename, string toFilename);
    //------------------------------------------------------------------------------------------
    //SAVE
    //Function that copies all the temporary files from the dataFilePath folder to dataFilePath/name/folder
    void Save(string dataFilePath, string name, string folder)
    {
        //Filenames of the txtfiles where the data is currently saved
        string q_filepath = dataFilePath+q_filename;
        string x_filepath = dataFilePath + x_filename;
	    string t_q_filepath = dataFilePath + t_q_filename;
    	string q_torque_filepath = dataFilePath + q_torque_filename;
    	string t_q_torque_filepath = dataFilePath + t_q_torque_filename;
    	string q_dot_filepath = dataFilePath + q_dot_filename;
        //Create the new directory and subdirectory
    	string dir = dataFilePath+name+"/";
    	boost::filesystem::create_directory(dir);
    	string subdir = dir + folder + "/";  
	    boost::filesystem::create_directory(subdir);
        //Filenames of the txtfiles where the data will be saved
    	string q_new_filepath = subdir + q_filename;
    	string x_new_filepath = subdir + x_filename;
    	string t_q_new_filepath = subdir + t_q_filename;
    	string q_torque_new_filepath = subdir + q_torque_filename;
    	string t_q_torque_new_filepath = subdir + t_q_torque_filename;
    	string q_dot_new_filepath = subdir + q_dot_filename;
        //Copy the files where they were previously to the new location
    	copyFile(q_filepath, q_new_filepath);
    	copyFile(x_filepath, x_new_filepath);
    	copyFile(t_q_filepath, t_q_new_filepath);
    	copyFile(q_torque_filepath, q_torque_new_filepath);
    	copyFile(t_q_torque_filepath, t_q_torque_new_filepath);
    	copyFile(q_dot_filepath, q_dot_new_filepath );
    }
    //------------------------------------------------------------------------------------------
    //CLEAR
    //Function that removes all the txt files in the temporary folder
    void Clear(string dataFilePath)
    {
        //Filenames of the txtfiles where the data is currently saved
        string q_filepath = dataFilePath + q_filename;
        string x_filepath = dataFilePath + x_filename;
	    string t_q_filepath = dataFilePath + t_q_filename;
    	string q_torque_filepath = dataFilePath + q_torque_filename;
    	string t_q_torque_filepath = dataFilePath + t_q_torque_filename;
    	string q_dot_filepath = dataFilePath + q_dot_filename;
        //Remove them
	    remove(q_filepath.c_str());
	    remove(x_filepath.c_str());
	    remove(t_q_filepath.c_str());
	    remove(q_torque_filepath.c_str());
	    remove(t_q_torque_filepath.c_str());
	    remove(q_dot_filepath.c_str());
    }
    //------------------------------------------------------------------------------------------
    //LOAD
    //Function that access dataFilePath/name/folder and extracts the information inside the files:
    // - q_traj: joint position trajectory saved in jointPosition.txt. Also contains the stamps contained in stamps.txt
    // - x_traj_out: cartesian position trajectory obtained from cartesianPosition.txt file
    // - xdot_traj_out: cartesian velocity trajectory obtained from cartesianVelocity.txt file. This file is not always read, as it does not always exist, just for some trajectories
    // - tKnot_out: vector with the time stamps where the knots for the interpolation polynomials are placed, extracted from the polynomials.txt, which does not always exist.
    // - q_coefs_out -> Coefficients for the joint trajectory polynomials ordered as follows: [c_1_joint1_point1, c_2_joint1_point1, c_3_joint1_point1, c_4_joint1_point1, c_1_joint2_point1, ..., c_1_joint1_point2, ...]. As ros cannot they are all in a list but ordered in some specific way
    // - qdot_coefs_out -> Same as before but for joint velocity trajectory polynomials taking into account that this time there are only 3 coefficients per joint and point instead of 4.
    // - qdotdot_coefs_out -> Same as before but this time with 2 coefficients per polynomial
    trajectory_msgs::JointTrajectory Load(string name, string folder, string dataFilePath, vector<geometry_msgs::Twist> &x_traj_out, vector<geometry_msgs::Twist> &xdot_traj_out, vector<double> &tKnot_out, vector<double> &q_coefs_out, vector<double> &qdot_coefs_out, vector<double> &qdotdot_coefs_out)
    {
        //Build the paths for the filenames where the data is saved
	    string newjointPosFilename=dataFilePath+name+"/"+folder+"/"+q_filename;
	    string newjointVelFilename=dataFilePath+name+"/"+folder+"/"+q_dot_filename;
	    string newcartesianPosFilename=dataFilePath+name+"/"+folder+"/"+x_filename;
	    string newstampsFilename=dataFilePath+name+"/"+folder+"/"+t_q_filename;
	    string newcartesianVelFilename=dataFilePath+name+"/"+folder+"/"+x_dot_filename;
        string newPolynomialsFilename=dataFilePath+name+"/"+folder+"/"+pp_filename;
        //Create input file streams for each of those files
	    ifstream jointPosFile(newjointPosFilename);
	    ifstream jointVelFile(newjointVelFilename);
	    ifstream stampsFile(newstampsFilename);
	    ifstream cartesianPosFile(newcartesianPosFilename);
	    ifstream cartesianVelFile(newcartesianVelFilename);
        ifstream polynomialsFile(newPolynomialsFilename);
	    //Count the total positions containing the positions file (which should be the same as the stamps file as it was captured at the same time
      	double lines= count(istreambuf_iterator<char>(jointPosFile), istreambuf_iterator<char>(), '\n');
        double npos = ceil(lines/8);
        //Clear the file and go to the beginning
	    jointPosFile.clear();
	    jointPosFile.seekg (0, ios::beg);
	    //1. Joint trajectory + Stamps + Velocities
        //Create joint trajectory to fill and add the joint names
	    trajectory_msgs::JointTrajectory joint_trajectory;
	    vector<string> jointnames={"a1", "a2", "a3", "a4", "a5", "a6", "a7"};
	    joint_trajectory.joint_names=jointnames;
        string aux_str;
	    double firststamp;
	    for (int i=0; i<npos; i++)
	    {
            //Variable to load each point
		    trajectory_msgs::JointTrajectoryPoint joint_point;
		    //A. Read positions from jointPosFile
		    for (int j=0; j<7; j++)
		    {
                double q_j;
                //Save the double from the file in the variable q_j, check if it is possible
                if (!ReadDoubleFromFile(jointPosFile, 4, 12, q_j))
                {
                    cout << "Not enough positions for the amount of points" << endl;
                }
                //Add q_j to the JointTrajectoryPoint created before inside the positions field
			    joint_point.positions.push_back(q_j);
		    }
		    //Clear the blank line after every set of joint coordinate positions
		    getline(jointPosFile, aux_str); 
		    //B. Read stamp from stampsFile
            double stamp;
            //Save the double from the stampsFile in the variable stamp, check if it is not empty
            if (!ReadDoubleFromFile(stampsFile, 0, 20, stamp))
            {
                cout << "Not enough stamps for the amount of points" << endl;
            }
		    //If it is the first point, save the stamp in the variable firststamp and set time_from_start to 0
		    if (i==0)
		    {
			    joint_point.time_from_start=ros::Duration(0);
			    firststamp=stamp;
		    }
		    else //In any other case the time_from_start is the difference between the timestamp and the first timestamp
		    {
			    double diff=stamp-firststamp;
			    joint_point.time_from_start=ros::Duration(diff);
		    }
		    //Clear the blank line after every stamp
		    getline(stampsFile, aux_str);
            //C. Read velocities from jointVelFile
            for (int j=0; j<7; j++)
		    {
                double qdot_j;
                //Save the double from the file in the variable qdot_j, check if it is possible
                if (!ReadDoubleFromFile(jointVelFile, 4, 12, qdot_j))
                {
                    if (i==0) //Print just once
                    {
                        cout << "No joint velocities found" << endl;
                    }
                }
			    joint_point.velocities.push_back(qdot_j);
		    }
            //Clear the blank space
		    getline(jointVelFile, aux_str); 
		    //Add the built JointTrajectoryPoint to the whole Trajectory
		    joint_trajectory.points.push_back(joint_point);

	    }
	    //2. Cartesian trajectory
        //Create a vector of twists to save the cartesian trajectory
	    vector<geometry_msgs::Twist> x_traj;
	    for (int i=0; i<npos; i++)
	    {
		    geometry_msgs::Twist x;
            //Save the double from the file in the variable x, while check if there are cartesian positions left
            if (ReadDoubleFromFile(cartesianPosFile, 3, 12, x.linear.x))
            {
		        ReadDoubleFromFile(cartesianPosFile, 3, 12, x.linear.y);
		        ReadDoubleFromFile(cartesianPosFile, 3, 12, x.linear.z);
		        ReadDoubleFromFile(cartesianPosFile, 4, 12, x.angular.x);
		        ReadDoubleFromFile(cartesianPosFile, 4, 12, x.angular.y);
		        ReadDoubleFromFile(cartesianPosFile, 4, 12, x.angular.z);
            }
            else
            {
                if (i==0)
                {
                    //Print just once
                    cout << "No cartesian poses found" << endl;
                }
            }
        	//Clear the blank line after every set of joint coordinate positions
		    getline(cartesianPosFile, aux_str); 
		    //Add the built Twist to the whole Trajectory
		    x_traj.push_back(x);
	    }
        //Fill output variable
        x_traj_out=x_traj;
        //3. Cartesian velocity
        //Create a vector of twists to save the cartesian velocity trajectory
	    vector<geometry_msgs::Twist> xdot_traj;
	    for (int i=0; i<npos-1; i++)
	    {
		    geometry_msgs::Twist xdot;
            //Save the double from the file in the variable xdot, while check if there are cartesian velocities left
            if (ReadDoubleFromFile(cartesianVelFile, 3, 12, xdot.linear.x))
            {
		        ReadDoubleFromFile(cartesianVelFile, 3, 12, xdot.linear.y);
		        ReadDoubleFromFile(cartesianVelFile, 3, 12, xdot.linear.z);
		        ReadDoubleFromFile(cartesianVelFile, 4, 12, xdot.angular.x);
		        ReadDoubleFromFile(cartesianVelFile, 4, 12, xdot.angular.y);
		        ReadDoubleFromFile(cartesianVelFile, 4, 12, xdot.angular.z);
            }
            else
            {
                if (i==0)
                {
                    //Print just once
                    cout << "No cartesian velocities found" << endl;
                }
            }
        	//Clear the blank line after every set of joint coordinate positions
		    getline(cartesianVelFile, aux_str); 
		    //Add the built Twist to the whole Trajectory
		    xdot_traj.push_back(xdot);
	    }
        //Fill output variable
        xdot_traj_out=xdot_traj;
        //4. Polynomials
        double nSeg;
        vector<double> tKnot;
        vector<double> q_coefs; //All in one vector, ordered, first by segment then by joint, and then by number of coefficient. 
        //c1(s1, j1), c2(s1, j1), c3(s1, j1), c4(s1, j1)
        //c1(s1, j2) ,... 
        //...
        //c1(s2, j1), ...
        vector<double> qdot_coefs; //All in one vector, ordered, first by segment then by joint, and then by number of coefficient. 
        vector<double> qdotdot_coefs; //All in one vector, ordered, first by segment then by joint, and then by number of coefficient.
        //Save the double from the file in the variable nSeg
        if (ReadDoubleFromFile(polynomialsFile, 6,12, nSeg))
        {
            //Clear space
            getline(polynomialsFile, aux_str);
            //Clear title
            getline(polynomialsFile, aux_str);
            //Save the list of time values in the file in the vector<double> tKnot
            for (int i=0; i<nSeg+1; i++)
            {
                double t;
                ReadDoubleFromFile(polynomialsFile, 0, 20, t);
                tKnot.push_back(t);
            }
            //Clear space
            getline(polynomialsFile, aux_str);
            //Clear title coefs_q
            getline(polynomialsFile, aux_str);
            //Clear title q1
            getline(polynomialsFile, aux_str);
            //Save the list of coefficients in the variable q_coefs
            for (int i=0; i<nSeg; i++)
            {
                for (int j=0; j<7; j++)
                {
                    double c1, c2, c3, c4;
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c1);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c2);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c3);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c4);
                    //Clear title q2, q3, q4, q5, q6, q7, " ", depending on j
                    getline(polynomialsFile, aux_str); 
                    q_coefs.push_back(c1);
                    q_coefs.push_back(c2);
                    q_coefs.push_back(c3);
                    q_coefs.push_back(c4);
                }
                //Clear title q1, coefs_qdot
                getline(polynomialsFile, aux_str); 
            }
            //Clear title qdot1
            getline(polynomialsFile, aux_str); 
            //Save the list of coefficients in the variable qdot_coefs
            for (int i=0; i<nSeg; i++)
            {
                for (int j=0; j<7; j++)
                {
                    double c1, c2, c3;
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c1);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c2);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c3);
                    //Clear title qdot2, qdot3, qdot4, qdot5, qdot6, qdot7, " ", depending on j
                    getline(polynomialsFile, aux_str); 
                    qdot_coefs.push_back(c1);
                    qdot_coefs.push_back(c2);
                    qdot_coefs.push_back(c3);
                }
                getline(polynomialsFile, aux_str); //Clear title qdot1, coefs_qdotdot
            }
            getline(polynomialsFile, aux_str); //Clear title qdotdot1
            //Save the list of coefficients in the variable qdotdot_coefs.
            for (int i=0; i<nSeg; i++)
            {
                for (int j=0; j<7; j++)
                {
                    double c1, c2;
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c1);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c2);
                    getline(polynomialsFile, aux_str); //Clear title qdot2, qdot3, qdot4, qdot5, qdot6, qdot7, " "
                    qdotdot_coefs.push_back(c1);
                    qdotdot_coefs.push_back(c2);
                }
                getline(polynomialsFile, aux_str); //Clear title qdot1, coefs_qdotdot
            }
        }
        //Fill output variables for the interpolation polynomials
        tKnot_out=tKnot;
        q_coefs_out=q_coefs;
        qdot_coefs_out=qdot_coefs;
        qdotdot_coefs_out=qdotdot_coefs;
        return joint_trajectory;
    }
    void LoadVectors(string name, string folder, string dataFilePath, vector<double> &t, vector<double> &q, vector<double> &x, vector<double> &qdot, vector<double> &xdot, vector<double> &tKnot, vector<double> &q_coefs, vector<double> &qdot_coefs, vector<double> &qdotdot_coefs, vector<double> &qtorque, vector<double> &ttorque)
    {
        //Clear output variables
        t.clear();
        q.clear();
        x.clear();
        qdot.clear();
        xdot.clear();
        tKnot.clear();
        q_coefs.clear();
        qdot_coefs.clear();
        qdotdot_coefs.clear();
        ttorque.clear();
        qtorque.clear();
        //Build the paths for the filenames where the data is saved
	    string jointPosFilename=dataFilePath+name+"/"+folder+"/"+q_filename;
	    string jointVelFilename=dataFilePath+name+"/"+folder+"/"+q_dot_filename;
	    string cartesianPosFilename=dataFilePath+name+"/"+folder+"/"+x_filename;
	    string stampsFilename=dataFilePath+name+"/"+folder+"/"+t_q_filename;
	    string cartesianVelFilename=dataFilePath+name+"/"+folder+"/"+x_dot_filename;
        string polynomialsFilename=dataFilePath+name+"/"+folder+"/"+pp_filename;
        string jointTorqueFilename=dataFilePath+name+"/"+folder+"/"+q_torque_filename;
        string jointTorqueStampsFilename=dataFilePath+name+"/"+folder+"/"+t_q_torque_filename;
        //Create input file streams for each of those files
	    ifstream jointPosFile(jointPosFilename);
	    ifstream jointVelFile(jointVelFilename);
	    ifstream stampsFile(stampsFilename);
	    ifstream cartesianPosFile(cartesianPosFilename);
	    ifstream cartesianVelFile(cartesianVelFilename);
        ifstream polynomialsFile(polynomialsFilename);
        ifstream jointTorqueFile(jointTorqueFilename);
        ifstream jointTorqueStampsFile(jointTorqueStampsFilename);
	    //Count the total positions containing the positions file (which should be the same as the stamps file as it was captured at the same time
      	double lines= count(istreambuf_iterator<char>(jointPosFile), istreambuf_iterator<char>(), '\n');
        double npos = ceil(lines/8);
        string aux_str;
        //Clear the file and go to the beginning
	    jointPosFile.clear();
	    jointPosFile.seekg (0, ios::beg);
        //1. Stamps
	    double firststamp;
	    for (int i=0; i<npos; i++)
	    {
            double stamp;
            //Save the double from the stampsFile in the variable stamp, check if it is not empty
            if (!ReadDoubleFromFile(stampsFile, 0, 20, stamp))
            {
                if (i==0)
                    cout << "Not enough stamps for the amount of points" << endl;
            }
		    //If it is the first point, save the stamp in the variable firststamp
		    if (i==0)
		    {
			    firststamp=stamp;
		    }
 	        stamp=stamp-firststamp;
		    //Clear the blank line after every stamp
		    getline(stampsFile, aux_str);
		    t.push_back(stamp);
	    }
	    //2. Joint trajectory
	    for (int i=0; i<npos; i++)
	    {
		    for (int j=0; j<7; j++)
		    {
                double q_point_joint;
                //Save the double from the file
                if (!ReadDoubleFromFile(jointPosFile, 4, 12, q_point_joint))
                {
                    if (i==0)
                        cout << "Not enough positions for the amount of points" << endl;
                }
                //Add q_point_joint to the vector q_point
			    q.push_back(q_point_joint);
		    }//q=[q qpoint_1 qpoint_2 qpoint_3 qpoint_4 qpoint_5 qpoint_6 qpoint_7] 
		    //Clear the blank line after every set of joint coordinate positions
		    getline(jointPosFile, aux_str);
	    }
        //3. Cartesian trajectory
	    for (int i=0; i<npos; i++)
	    {
		    for (int lin_coord=0; lin_coord<3; lin_coord++)
		    {
                double x_point_coord;
                //Save the double from the file
                if (!ReadDoubleFromFile(cartesianPosFile, 3, 12, x_point_coord))
                {
                    if (i==0)
                        cout << "Not enough cartesian positions for the amount of points" << endl;
                }
                //Add q_j to the JointTrajectoryPoint created before inside the positions field
			    x.push_back(x_point_coord);
		    }//x=[x xpoint_x xpoint_y xpoint_z]
		    for (int ang_coord=0; ang_coord<3; ang_coord++)
		    {
                double x_point_coord;
                //Save the double from the file
                if (!ReadDoubleFromFile(cartesianPosFile, 4, 12, x_point_coord)) //Starts at 4, not at 3, because 'rx' has on more character than 'x'
                {
                    if (i==0)
                        cout << "Not enough cartesian positions for the amount of points" << endl;
                }
                //Add q_j to the JointTrajectoryPoint created before inside the positions field
			    x.push_back(x_point_coord);
		    }//x=[x xpoint_rx xpoint_ry xpoint_rz]
		    //Clear the blank line after every set of joint velocities
		    getline(cartesianPosFile, aux_str);
	    }
        //4. Joint velocity trajectory
	    for (int i=0; i<npos; i++)
	    {
		    for (int j=0; j<7; j++)
		    {
                double qdot_point_joint;
                //Save the double from the file
                if (!ReadDoubleFromFile(jointVelFile, 4, 12, qdot_point_joint))
                {
                    if (i==0)
                        cout << "Not enough velocities for the amount of points" << endl;
                }
                //Add q_j to the JointTrajectoryPoint created before inside the positions field
			    qdot.push_back(qdot_point_joint);
		    }//qdot=[qdot qdotpoint_1 qdotpoint_2 qdotpoint_3 qdotpoint_4 qdotpoint_5 qdotpoint_6 qdotpoint_7] 
		    //Clear the blank line after every set of joint velocities
		    getline(jointVelFile, aux_str);
	    }

        //5. Joint torque stamps
      	double lines_torque = count(istreambuf_iterator<char>(jointTorqueFile), istreambuf_iterator<char>(), '\n');
        double ntorques = ceil(lines_torque/8);
        std::cout << ntorques << std::endl;
        //Clear the file and go to the beginning
	    jointTorqueFile.clear();
	    jointTorqueFile.seekg (0, ios::beg);

        double firsttorquestamp;
        if (ReadDoubleFromFile(jointTorqueStampsFile, 0, 20, firsttorquestamp))
        {
            double stamp_torque;
  	        stamp_torque=stamp_torque-firsttorquestamp;
        	getline(jointTorqueStampsFile, aux_str);
            ttorque.push_back(stamp_torque);
    	    for (int i=0; i<ntorques-1; i++)
	        {
                if (!ReadDoubleFromFile(jointTorqueStampsFile, 0, 20, stamp_torque))
                {
                    if (i==0)
                        cout << "Not enough stamps for the amount of torque points" << endl;
                }
		        //If it is the first point, save the stamp in the variable firststamp
     	        stamp_torque=stamp_torque-firsttorquestamp;
		        //Clear the blank line after every stamp
		        getline(jointTorqueStampsFile, aux_str);
		        ttorque.push_back(stamp_torque);
	        }
        }

        //6. Joint torques
        if (ntorques!=0)
        {
	        for (int i=0; i<ntorques; i++)
	        {
		        for (int j=0; j<7; j++)
		        {
                    double torque_point_joint;
                    //Save the double from the file
                    if (!ReadDoubleFromFile(jointTorqueFile, 4, 12, torque_point_joint))
                    {   
                        std::cout << torque_point_joint << std::endl;
                        if (i==0)
                            cout << "Not enough torques for the amount of points" << endl;
                    }
			        qtorque.push_back(torque_point_joint);
		        }//torque=[torque torquepoint_1 torquepoint_2 torquepoint_3 torquepoint_4 torquepoint_5 torquepoint_6 torquepoint_7] 
		        //Clear the blank line after every set of joint velocities
		        getline(jointTorqueFile, aux_str);
	        }
        }
        //7. Cartesian velocity
	    for (int i=0; i<npos; i++)
	    {
            vector<double> xdot_point;
		    for (int lin_coord=0; lin_coord<3; lin_coord++)
		    {
                double xdot_point_coord;
                //Save the double from the file
                if (!ReadDoubleFromFile(cartesianVelFile, 3, 12, xdot_point_coord))
                {
                    if (i==0)
                        cout << "Not enough cartesian positions for the amount of points" << endl;
                }
                //Add q_j to the JointTrajectoryPoint created before inside the positions field
			    xdot.push_back(xdot_point_coord);
		    }//xdot=[xdot xdotpoint_x xdotpoint_y xdotpoint_z]
		    for (int ang_coord=0; ang_coord<3; ang_coord++)
		    {
                double xdot_point_coord;
                //Save the double from the file
                if (!ReadDoubleFromFile(cartesianVelFile, 4, 12, xdot_point_coord)) //Starts at 4, not at 3, because 'rx' has on more character than 'x'
                {
                    if (i==0)
                        cout << "Not enough cartesian positions for the amount of points" << endl;
                }
                //Add q_j to the JointTrajectoryPoint created before inside the positions field
			    xdot.push_back(xdot_point_coord);
		    }//xdot=[xdot xdotpoint_rx xdotpoint_ry xdotpoint_rz]
		    //Clear the blank line after every set of joint velocities
		    getline(cartesianVelFile, aux_str);
	    }
        //8. Polynomials
        double nSeg;
        //Coefficients are all in one vector, ordered, first by segment then by joint, and then by number of coefficient. 
        //c1(s1, j1), c2(s1, j1), c3(s1, j1), c4(s1, j1)
        //c1(s1, j2) ,... 
        //...
        //c1(s2, j1), ...
        //Save the double from the file in the variable nSeg
        if (ReadDoubleFromFile(polynomialsFile, 6,12, nSeg))
        {
            //Clear space
            getline(polynomialsFile, aux_str);
            //Clear title
            getline(polynomialsFile, aux_str);
            //Save the list of time values in the file in the vector<double> tKnot
            for (int i=0; i<nSeg+1; i++)
            {
                double t;
                ReadDoubleFromFile(polynomialsFile, 0, 20, t);
                tKnot.push_back(t);
            }
            //Clear space
            getline(polynomialsFile, aux_str);
            //Clear title coefs_q
            getline(polynomialsFile, aux_str);
            //Clear title q1
            getline(polynomialsFile, aux_str);
            //Save the list of coefficients in the variable q_coefs
            for (int i=0; i<nSeg; i++)
            {
                for (int j=0; j<7; j++)
                {
                    double c1, c2, c3, c4;
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c1);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c2);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c3);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c4);
                    //Clear title q2, q3, q4, q5, q6, q7, " ", depending on j
                    getline(polynomialsFile, aux_str); 
                    q_coefs.push_back(c1);
                    q_coefs.push_back(c2);
                    q_coefs.push_back(c3);
                    q_coefs.push_back(c4);
                }//q_coefs=[q_coefs q_point_joint_c1 q_point_joint_c2 q_point_joint_c3 q_point_joint_c4] x 7joints
                //Clear title q1, coefs_qdot
                getline(polynomialsFile, aux_str); 
            }
            //Clear title qdot1
            getline(polynomialsFile, aux_str); 
            //Save the list of coefficients in the variable qdot_coefs
            for (int i=0; i<nSeg; i++)
            {
                for (int j=0; j<7; j++)
                {
                    double c1, c2, c3;
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c1);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c2);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c3);
                    //Clear title qdot2, qdot3, qdot4, qdot5, qdot6, qdot7, " ", depending on j
                    getline(polynomialsFile, aux_str); 
                    qdot_coefs.push_back(c1);
                    qdot_coefs.push_back(c2);
                    qdot_coefs.push_back(c3);
                }//qdot_coefs=[qdot_coefs qdot_point_joint_c1 qdot_point_joint_c2 qdot_point_joint_c3] x 7joints
                getline(polynomialsFile, aux_str); //Clear title qdot1, coefs_qdotdot
            }
            getline(polynomialsFile, aux_str); //Clear title qdotdot1
            //Save the list of coefficients in the variable qdotdot_coefs.
            for (int i=0; i<nSeg; i++)
            {
                for (int j=0; j<7; j++)
                {
                    double c1, c2;
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c1);
                    ReadDoubleFromFile(polynomialsFile, 0, 20, c2);
                    getline(polynomialsFile, aux_str); //Clear title qdot2, qdot3, qdot4, qdot5, qdot6, qdot7, " "
                    qdotdot_coefs.push_back(c1);
                    qdotdot_coefs.push_back(c2);
                }//qdotdot_coefs=[qdotdot_coefs qdotdot_point_joint_c1 qdotdot_point_joint_c2] x 7joints
                getline(polynomialsFile, aux_str); //Clear title qdot1, coefs_qdotdot
            }
        }
    }
    //------------------------------------------------------------------------------------------
    //LOAD, INTERPOLATE AND SAVE SOMEWHERE
    trajectory_msgs::JointTrajectory LoadInterpolateSave(string name_input, string folder_input, string folder_output, string dataFilePath)
    {
        
    }
    //------------------------------------------------------------------------------------------
    //OTHER FUNCTIONS
    //Function that reads a double from a file given the starting and end characters to read
    bool ReadDoubleFromFile(ifstream &file, int start, int end, double &ans)
    {
	    string str;
	    string::size_type sz;
        if (getline(file, str))
        {
            str=str.substr(start,end);
            ans=stod(str, &sz);
            return true;
        }
        else 
        {
            return false;
        }
    }
    //Function that copies the content from one file to another given the filenames
    void copyFile(string fromFilename, string toFilename)
    {
        //Create input file stream and output filestream from the fromFilename to the toFilename
	    ifstream src(fromFilename, ios::binary);
	    ofstream dst(toFilename, ios::binary);
        //Pass info from one stream to the other
	    dst << src.rdbuf();
    }
}

