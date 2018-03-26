#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"


#include <vector>
#include <string>
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <unistd.h>
#include <termios.h>




struct stat st = {0};

enum hand{
    left= 0,
    right=1,
};

int sRate=100;                                                   // set the sample rate (Hz)
int trialCounter=0;                                              // counter for the trials
double startTime;



/*-- Variables related to the mocap system --*/


double lookTW=0.1;	                                            // the timewindow to look back for the average velocity (in seconds)
double velThreshold=0.018;                                      // velocity threshold for destinguish the motion or no=motion of the hand
int mocapRate=250;                                              // the sample rate of the motion capture system

unsigned int handCounter=0;                                     // counter for hand-related messages from the mocap system
std::vector<double> handPosition(3,0);                          // vector for the position of the hand (in)
std::vector<double> handOrientation(4,0);                       // vector for the orientation of the hand (in quaternions)
std::vector<double> handVelocity(3,0);                          // vector for the velocity of the hand (in m/s)

unsigned int elbowCounter=0;                                     // counter for elbow-related messages from the mocap system
std::vector<double> elbowPosition(3,0);                          // vector for the position of the elbow (in)
std::vector<double> elbowOrientation(4,0);                       // vector for the orientation of the elbow (in quaternions)
std::vector<double> elbowVelocity(3,0);                          // vector for the velocity of the elbow (in m/s)

unsigned int shoulderCounter=0;                                 // counter for shoulder-related messages from the mocap system
std::vector<double> shoulderPosition(3,0);						// vector for the position of the shoulder
std::vector<double> shoulderOrientation(4,0);					// vector for the orientation of the shoulder (in quaternions)
std::vector<double> shoulderVelocity(3,0);						// vector for the velocity of the shoulder (in m/s)



std::vector<double> mocapTime;                                   // timestamp for the mocap system

std::vector< std::vector<double> > handRelPos;
std::vector< double > _omegad(3,0); 					// Desired angular velocity [rad/s] (3x1)



/*-- functions for the system --*/

// void saveRecordings();                                           // a function to save the data after each trial
// int getch_();                                                    // a function to catch a key press asynchronously



/*-- Callback functions --*/


void handListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

    handPosition[0]=mocapmsg.pose.position.x;
    handPosition[1]=mocapmsg.pose.position.y;
    handPosition[2]=mocapmsg.pose.position.z;

    handOrientation[0]=mocapmsg.pose.orientation.x;
    handOrientation[1]=mocapmsg.pose.orientation.y;
    handOrientation[2]=mocapmsg.pose.orientation.z;
    handOrientation[3]=mocapmsg.pose.orientation.w;


    //mocapTime.push_back((ros::Time::now().toSec())-startTime);

    handCounter++;
    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}



void elbowListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

    elbowPosition[0]=mocapmsg.pose.position.x;
    elbowPosition[1]=mocapmsg.pose.position.y;
    elbowPosition[2]=mocapmsg.pose.position.z;

    elbowOrientation[0]=mocapmsg.pose.orientation.x;
    elbowOrientation[1]=mocapmsg.pose.orientation.y;
    elbowOrientation[2]=mocapmsg.pose.orientation.z;
    elbowOrientation[3]=mocapmsg.pose.orientation.w;


    //mocapTime.push_back((ros::Time::now().toSec())-startTime);

    elbowCounter++;
    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}

void shoulderListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

    shoulderPosition[0]=mocapmsg.pose.position.x;
    shoulderPosition[1]=mocapmsg.pose.position.y;
    shoulderPosition[2]=mocapmsg.pose.position.z;

    shoulderOrientation[0]=mocapmsg.pose.orientation.x;
    shoulderOrientation[1]=mocapmsg.pose.orientation.y;
    shoulderOrientation[2]=mocapmsg.pose.orientation.z;
    shoulderOrientation[3]=mocapmsg.pose.orientation.w;


    // mocapTime.push_back((ros::Time::now().toSec())-startTime);

 

    shoulderCounter++;
    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}



std::vector<double> handRV(std::vector< std::vector<double> > handrpos){

	/*-- a function to calculate the velocity of thand with respect to the shoulder --*/

	std::vector<double> hrv(3,0);				// the relative velocity of the hand

	// std::cout<<"size of handrpos: " << handrpos.size() <<"\n";



	for(int i=1;i<(int)handrpos.size();i++){

		// std::cout<<"rpos: " << handrpos[i][0] << ", " << handrpos[i][1] << ", " << handrpos[i][2] << "\n";

		hrv[0]=hrv[0]+((handrpos[i][0]-handrpos[i-1][0])*sRate);
		hrv[1]=hrv[1]+((handrpos[i][1]-handrpos[i-1][1])*sRate);
		hrv[2]=hrv[2]+((handrpos[i][2]-handrpos[i-1][2])*sRate);

	}

	// std::cout<<"accummulative velocity: " << hrv[0] <<", " << hrv[1] << ", " << hrv[2] <<"\n";

	hrv[0]=hrv[0]/(int)handrpos.size();
	hrv[1]=hrv[1]/(int)handrpos.size();
	hrv[2]=hrv[2]/(int)handrpos.size();
	
	return hrv;
}

std::vector<double> handRP(std::vector<double> handPos, std::vector<double> shoulderPos){


/*-- a function to calculate the velocity of thand with respect to the shoulder --*/

	std::vector<double> hrp(3,0);				// the relative velocity of the hand

	hrp[0]=handPos[0]-shoulderPos[0];
	hrp[1]=handPos[1]-shoulderPos[1];
	hrp[2]=handPos[2]-shoulderPos[2];

	// std::cout<<"hand rel pos: " << hrp[0] << " " << hrp[1] << " " << hrp[2]<<"\n";

	return hrp;
}



int main(int argc, char **argv)
{



    // initialize the node
    ros::init(argc, argv, "handtracker");

    ros::NodeHandle n;


    // set the publishers for the allegro hand

    //ros::Publisher allegorRight_pub = n.advertise<sensor_msgs::JointState>("rhand/velocity", 100);



    // set the subscribers to listen the classification outcome from the windows machine and the position of the hand

    ros::Subscriber handSub=n.subscribe("hand/pose", 10, handListener);

    ros::Subscriber elbowSub=n.subscribe("elbow/pose", 10, elbowListener);

    ros::Subscriber shoulderSub=n.subscribe("shoulder/pose", 10, shoulderListener);


	// ros::Subscriber _subRealPose;						// Subscribe to robot current pose
    // ros::Subscriber _subRealTwist;          				// Subscribe to robot current pose
	
    ros::Publisher _pubDesiredOrientation=n.advertise<geometry_msgs::Quaternion>("/lwr_test/joint_controllers/passive_ds_command_orient", 1);  				// Publish desired orientation to the topic "/lwr_test/joint_controllers/passive_ds_command_orient"
    ros::Publisher _pubDesiredTwist=n.advertise<geometry_msgs::Twist>("/lwr_test/joint_controllers/passive_ds_command_vel", 10); 							// Publish desired twist to topic "/lwr/joint_controllers/passive_ds_command_vel"

    // Messages declaration
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
	geometry_msgs::Twist _msgDesiredTwist;


    startTime=ros::Time::now().toSec();

    // set the loop rate
    ros::Rate loop_rate(sRate);

    double checkTime=ros::Time::now().toSec();

    double currentTime=0.0;

    std::vector<double> currentVel(3,0);


    int count = 0;
    while (ros::ok())
    {
        
        

        

        // if the key 't' is pressed, save the data and clear the data for the next trial

        // if(getch_()=='t'){
        //     saveRecordings();

        // }
    	// std::cout<<"--------------------------------------------------------------------\n";
    	
    	currentTime=ros::Time::now().toSec();

    	handRelPos.push_back(handRP(handPosition,shoulderPosition));

    	// std::cout<<"size of handRePos: " << (int)handRelPos.size()<<"\n";

    	// std::cout<< "time passed: " << currentTime - checkTime<< "\n";

    	if(currentTime-checkTime>=lookTW){
    		currentVel=handRV(handRelPos);
    		std::cout<<"current vel: " << currentVel[0] << ", " << currentVel[1] << ", " << currentVel[2] << "\n"; 
    		
    		// update checking time
    		checkTime=ros::Time::now().toSec();

    		_msgDesiredTwist.linear.x  = currentVel[0];
			_msgDesiredTwist.linear.y  = currentVel[1];
			_msgDesiredTwist.linear.z  = currentVel[2];
			_msgDesiredTwist.angular.x = _omegad[0];
			_msgDesiredTwist.angular.y = _omegad[1];
			_msgDesiredTwist.angular.z = _omegad[2];

			_pubDesiredTwist.publish(_msgDesiredTwist);

			// Publish desired orientation
			// _msgDesiredOrientation.w = _qd(0);
			// _msgDesiredOrientation.x = _qd(1);
			// _msgDesiredOrientation.y = _qd(2);
			// _msgDesiredOrientation.z = _qd(3);

			// _pubDesiredOrientation.publish(_msgDesiredOrientation);

    		// remove the oldest relative position
    		handRelPos.clear();

    	}


    	// std::cout<<"--------------------------------------------------------------------\n";

    	// std::cout<<"hand postion: (" << handPosition[0] << ", " << handPosition[1] << ", " << handPosition[2] <<")\n";
    	// std::cout<<"hand orientation: (" << handOrientation[0] << ", " << handOrientation[1] << ", " << handOrientation[2] << ", " << handOrientation[3] <<")\n";


     //    std::cout<<"elbow postion: (" << elbowPosition[0] << ", " << elbowPosition[1] << ", " << elbowPosition[2] <<")\n";
    	// std::cout<<"elbow orientation: (" << elbowOrientation[0] << ", " << elbowOrientation[1] << ", " << elbowOrientation[2] << ", " << elbowOrientation[3] <<")\n";

    	// std::cout<<"shoulder postion: (" << shoulderPosition[0] << ", " << shoulderPosition[1] << ", " << shoulderPosition[2] <<")\n";
    	// std::cout<<"shoulder orientation: (" << shoulderOrientation[0] << ", " << shoulderOrientation[1] << ", " << shoulderOrientation[2] << ", " << shoulderOrientation[3] <<")\n";




        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}