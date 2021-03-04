/**
 * \file
 * \brief 
 * \author 
 * \version 0.1
 * \date 
 * 
 * \param[in] 
 * 
 * Subscribes to: <BR>
 *    ° 
 * 
 * Publishes to: <BR>
 *    ° 
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//Cpp (to read .csv file)
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

//ROS
#include "ros/ros.h"
//include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/eigen_mav_msgs.h>


// Creating a class that will help store the waypoints at each iteration
class WaypointWithTime {
public:
    WaypointWithTime()
    : waiting_time(0), yaw(0.0){

    }
    WaypointWithTime(double t, float x, float y, float z, float _yaw)
    : position(x,y,z), yaw(_yaw), waiting_time(t) {

    }
    Eigen::Vector3d position;
    double yaw;
    double waiting_time;
};

// Include here the ".h" files corresponding to the topic type you use.
// ...

// You may have a number of globals here.
//geometry_msgs::Point Position;
bool sim_running = false;
static const int64_t kNanoSecondsInSeconds = 1000000000;



// Callback functions...



void callback(const sensor_msgs::ImuPtr &msg)
{
    sim_running = true;
}

// function to read and publish current trajectory

//void generateTrajectory(std::vector<float> vectPos , geometry_msgs::Point &pos);
void build_reference(float &x, float &y, float &z, std::vector<float> fvect);

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "trajectory_generation_node");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob;
    ros::NodeHandle nh_loc("~");

    ROS_INFO("Started waypoint publisher!");

    // Read the node parameters if any

    // Declare your node's subscriptions and service clients
    ros::Subscriber sub = nh_glob.subscribe("imu", 10, &callback);

    // Declare you publishers and service servers
    ros::Publisher pubTraj = nh_glob.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY,10);

    // Reading the .csv file
            
    std::ifstream file;
    file.open("/home/elie/catkin_ws/src/trajectory_generation/src/traj.csv");

    //float x,y,z;
    std::vector<WaypointWithTime> waypoints; // creating vector to store all the waypoints
    const float DEG_2_RAD = M_PI /180; // Creating constant variable to convert degrees to radians

    if (file.is_open())
    {
        // Creating string to store the line that is read line by line
        std::string line;
        std::vector<std::string> data;

        // read the text line by line
        while(getline(file, line)){ // While there are still rows to read

            
            // store line in the vector
            data.push_back(line);
        }

        // scanning the vector line by line
        for (size_t i = 0; i < data.size(); ++i){
            // get length of i-th line

            std::string tempString; // temporary variable to hold the element that will be converted to float at each iteration
            std::vector< float > fData; // temporary vector to store the float converted variables at each iteration
            std::istringstream iss(data[i]); // object to manipulate the string row at each iteration

            while(!iss.eof()) // While we are not at the end of the data[i] string
            { 
                std::getline(iss, tempString, ','); // Read the next element
                fData.push_back(stof(tempString)); // push back the element into fData
            }
            
            ROS_INFO_STREAM("End of string");
            //ros::Duration(2.0).sleep();
            
            //generateTrajectory(fData, Position); // update x,y and z with the float elements in fData
            //pubTraj.publish(Position);
            //ROS_INFO_STREAM("Position published!");
            //ros::Duration(0.2).sleep();
            
            
            ROS_INFO_STREAM("--------------------------------------------");
            ROS_INFO_STREAM("t = "<<fData[0]);
            ROS_INFO_STREAM("x = "<<fData[1]);
            ROS_INFO_STREAM("y = "<<fData[2]);
            ROS_INFO_STREAM("z = "<<fData[3]);
            ROS_INFO_STREAM("psi = "<<fData[4]);
            waypoints.push_back(WaypointWithTime(fData[0],fData[1],fData[2],fData[3],fData[4]));
            ROS_INFO_STREAM("------------------PUSHED--------------------");
            
        }
        ROS_INFO("Read %d waypoints.", (int) waypoints.size());
        file.close();
        //ROS_INFO_STREAM("TRAJECTORY PUBLISHED!");
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open file!");
        return 1;
    }

ROS_INFO("Wait for the simulation to become ready...");


while( !sim_running && ros::ok() ){
    ros::spinOnce();
    ros::Duration(0.1).sleep();
}

ROS_INFO("The simulation has started!");

// Wait for 30s so that everything can settel and the crazyflie 2.0 starts to hover
ros::Duration(10).sleep();

ROS_INFO("Publishing waypoints!");

trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
msg->header.stamp = ros::Time::now();
msg->points.resize(waypoints.size());
msg->joint_names.push_back("base_link");
int64_t time_from_start_ns = 0;
for (size_t i = 0; i < waypoints.size();++i){
    WaypointWithTime &wp = waypoints[i];

    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_point.position_W = wp.position;
    trajectory_point.setFromYaw(wp.yaw);
    trajectory_point.time_from_start_ns = time_from_start_ns;

    time_from_start_ns += static_cast<int64_t>(wp.waiting_time * kNanoSecondsInSeconds);

    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
}
ROS_INFO_STREAM("Publishing msg: " << &msg);
pubTraj.publish(msg);
ROS_INFO_STREAM("Done!");

ros::spinOnce();
ros::shutdown();

return 0;

}

