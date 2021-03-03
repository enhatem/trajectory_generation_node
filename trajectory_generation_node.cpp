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
#include <geometry_msgs/Point.h>


// Include here the ".h" files corresponding to the topic type you use.
// ...

// You may have a number of globals here.
geometry_msgs::Point Position;



// Callback functions...

void myFirstCallback(/* Define here the variable which hold the message */){
    // ... Callback function code
}

// function to read and publish current trajectory

void generateTrajectory(std::vector<float> vectPos , geometry_msgs::Point &pos);
void build_reference(float &x, float &y, float &z, std::vector<float> fvect);

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "trajectory_generation_node");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob;
    ros::NodeHandle nh_loc("~");

    // Read the node parameters if any

    // Declare your node's subscriptions and service clients
    // ...

    // Declare you publishers and service servers
    ros::Publisher pubTraj = nh_glob.advertise<geometry_msgs::Point>("/trajectory",1);

    // Reading the .csv file
            
    std::ifstream file;
    file.open("/home/elie/catkin_ws/src/trajectory_generation/src/simple.csv");

    //float x,y,z;

    if (file.is_open())
    {
        // sleep for 5 seconds
        ros::Duration(5.0).sleep();

        // Creating string to store the line that is read line by line
        std::string line;

        // read the text line by line
        while(getline(file, line)){ // While there are still rows to read

            std::string tempString; // temporary variable to hold the element that will be converted to float at each iteration
            std::vector< float > fData; // temporary vector to store the float converted variables at each iteration
            std::istringstream iss(line); // object to manipulate the string row at each iteration

            while(!iss.eof())
            { // While we are not at the end of the file

                std::getline(iss,tempString,','); // Read the next element
                fData.push_back(stof(tempString)); // push bash the element into fData

            }

            ROS_INFO_STREAM("End of string");

            generateTrajectory(fData, Position); // update x,y and z with the float elements in fData
            pubTraj.publish(Position);
            ROS_INFO_STREAM("Position published!");
            ros::Duration(5.0).sleep();
        }

        file.close();
        ros::Duration(5.0).sleep();
        ROS_INFO_STREAM("TRAJECTORY PUBLISHED!");

        
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open file!");
        return 1;
    }
}


void generateTrajectory(std::vector<float> vectPos , geometry_msgs::Point &pos)
{
    pos.x = vectPos[0];
    pos.y = vectPos[1];
    pos.z = vectPos[2];

    ROS_INFO_STREAM("The data in x is: " << pos.x);
    ROS_INFO_STREAM("The data in y is: " << pos.y);
    ROS_INFO_STREAM("The data in z is: " << pos.z);

}
