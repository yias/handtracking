#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "handtracker/spper.h"
#include "mathlib_eigen_conversions.h"
#include "MotionGenerators/CDDynamics.h"


#include <vector>
#include <string>
#include <stdio.h>

#include <algorithm>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <unistd.h>
#include <termios.h>

#include <math.h>  
#include <Eigen/Eigen>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <iostream>



struct stat st = {0};

enum hand{
    left= 0,
    right=1,
};

int sRate=100;                                                   // set the sample rate (Hz)
int trialCounter=0;                                              // counter for the trials
double startTime;



/*-- Variables related to the mocap system --*/

double gain=2;													// velocity gain
double a=0.2;													// smoothing factor for the velocity
double a2=0.1;													// smoothing factor for the position

double lookTW=0.1;	                                            // the timewindow to look back for the average velocity (in seconds)
double velThreshold=0.018;                                      // velocity threshold for destinguish the motion or no=motion of the hand
int mocapRate=250;                                              // the sample rate of the motion capture system

unsigned int handCounter=0;                                     // counter for hand-related messages from the mocap system

bool _firsthandPoseReceived=false;
double handStamp=0;
std::vector<double> handPosition(3,0);                          // vector for the position of the hand (in)
std::vector<double> handOrientation(4,0);                       // vector for the orientation of the hand (in quaternions)
std::vector<double> handVelocity(3,0);                          // vector for the velocity of the hand (in m/s)

std::vector<double> handPrevPosition(3,0);                      // vector for the previous position of the hand (in)
std::vector<double> handPrevOrientation(4,0);                   // vector for the previous orientation of the hand (in quaternions)

CDDynamics *hand_pos_filter, *hand_real_vel_filter;

Eigen::VectorXd prev_hand_position(3);
Eigen::VectorXd prev_hand_real_velocity(3);

Eigen::VectorXd hand_velocity_filtered(3);

unsigned int elbowCounter=0;                                    // counter for elbow-related messages from the mocap system
std::vector<double> elbowPosition(3,0);                         // vector for the position of the elbow (in)
std::vector<double> elbowOrientation(4,0);                      // vector for the orientation of the elbow (in quaternions)
std::vector<double> elbowVelocity(3,0);                         // vector for the velocity of the elbow (in m/s)

unsigned int shoulderCounter=0;                                 // counter for shoulder-related messages from the mocap system

bool _firstshoulderPoseReceived=false;
double shoulderStamp=0;
std::vector<double> shoulderPosition(3,0);						// vector for the position of the shoulder
std::vector<double> shoulderOrientation(4,0);					// vector for the orientation of the shoulder (in quaternions)
std::vector<double> shoulderVelocity(3,0);						// vector for the velocity of the shoulder (in m/s)

std::vector<double> shoulderPrevPosition(3,0);					// vector for the previous position of the shoulder
std::vector<double> shoulderPrevOrientation(4,0);				// vector for the previous orientation of the shoulder (in quaternions)

bool _firstdistance=false;
double current_shdistance=0;									// current distance of the hand from the shoulder
double previous_shdistance=0;									// previous distance of the hand from the shoulder
float handDirection=0;

double velUpperBound=0.32;

float speedPer=0;





bool _firstRealPoseReceived=false;
geometry_msgs::Pose _msgRealPose;

Eigen::Vector4f _qd; // Desired end effector quaternion (4x1)



std::vector<double> mocapTime;                                   	// timestamp for the mocap system

std::vector< std::vector<double> > handRelPos;
std::vector< double > _omegad(3,0); 								// Desired angular velocity [rad/s] (3x1)

std::vector< double > velpreviousValue(3,0);


/*-- functions for the system --*/

// void saveRecordings();                                           // a function to save the data after each trial
// int getch_();                                                    // a function to catch a key press asynchronously



/*-- Callback functions --*/

void robotListener(const geometry_msgs::Pose::ConstPtr& msg){

	_msgRealPose = *msg;

	if(!_firstRealPoseReceived)
	{
		_firstRealPoseReceived = true;
		_qd << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
		
	}

}


void handListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

	Eigen::VectorXd human_hand_position_world(3);
    Eigen::VectorXd human_hand_real_velocity_world(3);

    MathLib::Vector human_hand_position_world_filtered_mathlib;
    MathLib::Vector human_hand_real_velocity_world_mathlib;
    MathLib::Vector human_hand_real_velocity_root_filtered_mathlib;
    MathLib::Vector human_hand_real_acceleration_root_mathlib;

   



	if(!_firsthandPoseReceived)
	{
			_firsthandPoseReceived=true;
		   	handPosition[0]=mocapmsg.pose.position.x;
		    handPosition[1]=mocapmsg.pose.position.y;
		    handPosition[2]=mocapmsg.pose.position.z;

		    human_hand_position_world(0)=mocapmsg.pose.position.x;
		    human_hand_position_world(1)=mocapmsg.pose.position.y;
		    human_hand_position_world(2)=mocapmsg.pose.position.z;


	        hand_pos_filter->SetTarget(E2M_v(human_hand_position_world));
	        hand_pos_filter->Update();
	        hand_pos_filter->GetState(human_hand_position_world_filtered_mathlib, human_hand_real_velocity_world_mathlib);
	        human_hand_real_velocity_world = M2E_v(human_hand_real_velocity_world_mathlib);

	        hand_real_vel_filter->SetTarget(E2M_v(human_hand_real_velocity_world));
	        hand_real_vel_filter->Update();
	        hand_real_vel_filter->GetState(human_hand_real_velocity_root_filtered_mathlib, human_hand_real_acceleration_root_mathlib);
	        hand_velocity_filtered = M2E_v(human_hand_real_velocity_root_filtered_mathlib);


		    handOrientation[0]=mocapmsg.pose.orientation.x;
		    handOrientation[1]=mocapmsg.pose.orientation.y;
		    handOrientation[2]=mocapmsg.pose.orientation.z;
		    handOrientation[3]=mocapmsg.pose.orientation.w;

		    handPrevPosition[0]=mocapmsg.pose.position.x;
		    handPrevPosition[1]=mocapmsg.pose.position.y;
		    handPrevPosition[2]=mocapmsg.pose.position.z;

		    handPrevOrientation[0]=mocapmsg.pose.orientation.x;
		    handPrevOrientation[1]=mocapmsg.pose.orientation.y;
		    handPrevOrientation[2]=mocapmsg.pose.orientation.z;
		    handPrevOrientation[3]=mocapmsg.pose.orientation.w;

		    prev_hand_position(0)=mocapmsg.pose.position.x;
			prev_hand_position(1)=mocapmsg.pose.position.y;		    
			prev_hand_position(2)=mocapmsg.pose.position.z;

			

		    handStamp=mocapmsg.header.seq;
		
	}else{
		if(mocapmsg.header.seq!=handStamp){


			human_hand_position_world(0)=mocapmsg.pose.position.x;
		    human_hand_position_world(1)=mocapmsg.pose.position.y;
		    human_hand_position_world(2)=mocapmsg.pose.position.z;

		    
	        hand_pos_filter->SetTarget(E2M_v(human_hand_position_world));
	        hand_pos_filter->Update();
	        hand_pos_filter->GetState(human_hand_position_world_filtered_mathlib, human_hand_real_velocity_world_mathlib);
	        human_hand_real_velocity_world = M2E_v(human_hand_real_velocity_world_mathlib);

	        hand_real_vel_filter->SetTarget(E2M_v(human_hand_real_velocity_world));
	        hand_real_vel_filter->Update();
	        hand_real_vel_filter->GetState(human_hand_real_velocity_root_filtered_mathlib, human_hand_real_acceleration_root_mathlib);
	        hand_velocity_filtered = M2E_v(human_hand_real_velocity_root_filtered_mathlib);

	        

			handPosition[0]=(1-a2)*handPrevPosition[0]+a2*mocapmsg.pose.position.x;
		    handPosition[1]=(1-a2)*handPrevPosition[1]+a2*mocapmsg.pose.position.y;
		    handPosition[2]=(1-a2)*handPrevPosition[2]+a2*mocapmsg.pose.position.z;

		    handOrientation[0]=(1-a2)*handPrevOrientation[0]+a2*mocapmsg.pose.orientation.x;
		    handOrientation[1]=(1-a2)*handPrevOrientation[1]+a2*mocapmsg.pose.orientation.y;
		    handOrientation[2]=(1-a2)*handPrevOrientation[2]+a2*mocapmsg.pose.orientation.z;
		    handOrientation[3]=(1-a2)*handPrevOrientation[3]+a2*mocapmsg.pose.orientation.w;

		    handPrevPosition[0]=handPosition[0];
		    handPrevPosition[1]=handPosition[1];
		    handPrevPosition[2]=handPosition[2];

		    handPrevOrientation[0]=handOrientation[0];
		    handPrevOrientation[1]=handOrientation[1];
		    handPrevOrientation[2]=handOrientation[2];
		    handPrevOrientation[3]=handOrientation[3];

		   

		    prev_hand_position(0)=human_hand_position_world_filtered_mathlib(0);
			prev_hand_position(1)=human_hand_position_world_filtered_mathlib(1);		    
			prev_hand_position(2)=human_hand_position_world_filtered_mathlib(2);

			

		    handStamp=mocapmsg.header.seq;

		}
	}

    


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



	if(!_firstshoulderPoseReceived)
		{
			_firstshoulderPoseReceived=true;
		   	shoulderPosition[0]=mocapmsg.pose.position.x;
		    shoulderPosition[1]=mocapmsg.pose.position.y;
		    shoulderPosition[2]=mocapmsg.pose.position.z;

		    shoulderOrientation[0]=mocapmsg.pose.orientation.x;
		    shoulderOrientation[1]=mocapmsg.pose.orientation.y;
		    shoulderOrientation[2]=mocapmsg.pose.orientation.z;
		    shoulderOrientation[3]=mocapmsg.pose.orientation.w;

		    shoulderPrevPosition[0]=mocapmsg.pose.position.x;
		    shoulderPrevPosition[1]=mocapmsg.pose.position.y;
		    shoulderPrevPosition[2]=mocapmsg.pose.position.z;

		    shoulderPrevOrientation[0]=mocapmsg.pose.orientation.x;
		    shoulderPrevOrientation[1]=mocapmsg.pose.orientation.y;
		    shoulderPrevOrientation[2]=mocapmsg.pose.orientation.z;
		    shoulderPrevOrientation[3]=mocapmsg.pose.orientation.w;
		    shoulderStamp=mocapmsg.header.seq;
		
	}else{
		if(mocapmsg.header.seq!=handStamp){
			shoulderPosition[0]=(1-a2)*shoulderPrevPosition[0]+a2*mocapmsg.pose.position.x;
		    shoulderPosition[1]=(1-a2)*shoulderPrevPosition[1]+a2*mocapmsg.pose.position.y;
		    shoulderPosition[2]=(1-a2)*shoulderPrevPosition[2]+a2*mocapmsg.pose.position.z;

		    shoulderOrientation[0]=(1-a2)*shoulderPrevOrientation[0]+a2*mocapmsg.pose.orientation.x;
		    shoulderOrientation[1]=(1-a2)*shoulderPrevOrientation[1]+a2*mocapmsg.pose.orientation.y;
		    shoulderOrientation[2]=(1-a2)*shoulderPrevOrientation[2]+a2*mocapmsg.pose.orientation.z;
		    shoulderOrientation[3]=(1-a2)*shoulderPrevOrientation[3]+a2*mocapmsg.pose.orientation.w;

		    shoulderPrevPosition[0]=shoulderPosition[0];
		    shoulderPrevPosition[1]=shoulderPosition[1];
		    shoulderPrevPosition[2]=shoulderPosition[2];

		    shoulderPrevOrientation[0]=shoulderOrientation[0];
		    shoulderPrevOrientation[1]=shoulderOrientation[1];
		    shoulderPrevOrientation[2]=shoulderOrientation[2];
		    shoulderPrevOrientation[3]=shoulderOrientation[3];

		    shoulderStamp=mocapmsg.header.seq;

		}
	}


 

    // mocapTime.push_back((ros::Time::now().toSec())-startTime);

 

    shoulderCounter++;
    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}



std::vector<double> handRV(std::vector< std::vector<double> > handrpos){

	/*-- a function to calculate the velocity of thand with respect to the shoulder --*/

	std::vector<double> hrv(3,0);				// the relative velocity of the hand

	// std::cout<<"size of handrpos: " << handrpos.size() <<"\n";

	current_shdistance=sqrt(handrpos[0][0]*handrpos[0][0]+handrpos[0][1]*handrpos[0][1]+handrpos[0][2]*handrpos[0][2]);

	for(int i=1;i<(int)handrpos.size();i++){

		hrv[0]=hrv[0]+((handrpos[i][0]-handrpos[i-1][0])*sRate);
		hrv[1]=hrv[1]+((handrpos[i][1]-handrpos[i-1][1])*sRate);
		hrv[2]=hrv[2]+((handrpos[i][2]-handrpos[i-1][2])*sRate);

		current_shdistance=current_shdistance+sqrt(handrpos[i][0]*handrpos[i][0]+handrpos[i][1]*handrpos[i][1]+handrpos[i][2]*handrpos[i][2]);

	}

	hrv[0]=hrv[0]/(int)handrpos.size();
	hrv[1]=hrv[1]/(int)handrpos.size();
	hrv[2]=hrv[2]/(int)handrpos.size();

	current_shdistance=current_shdistance/(int)handrpos.size();

	if(!(_firstdistance)){
		_firstdistance=true;
		previous_shdistance=current_shdistance;
		handDirection=0;
	}else{
		if(current_shdistance>previous_shdistance){
			handDirection=1;
		}else{
			if(current_shdistance<previous_shdistance){
				handDirection=-1;
			}else{
				handDirection=0;
			}
		}
	}

	previous_shdistance=current_shdistance;
	
	return hrv;
}

std::vector<double> handRP(std::vector<double> handPos, std::vector<double> shoulderPos){


/*-- a function to calculate the position of the hand with respect to the shoulder --*/

	std::vector<double> hrp(3,0);				// the relative velocity of the hand

	hrp[0]=handPos[0]-shoulderPos[0];
	hrp[1]=handPos[1]-shoulderPos[1];
	hrp[2]=handPos[2]-shoulderPos[2];


	return hrp;
}


std::vector<double> compDesiredVel(std::vector<double> curVel){

	std::vector<double> desVel(3,0);				// the desired velocity of the hand

	double speed=std::sqrt(curVel[0]*curVel[0]+curVel[1]*curVel[1]+curVel[2]*curVel[2]);

	if(!(speed<=velThreshold)){

		if(speed>1){
			desVel[0]=velpreviousValue[0];
			desVel[1]=velpreviousValue[1];
			desVel[2]=velpreviousValue[2];

		}else{
			desVel[0]=gain*((1-a)*velpreviousValue[0]+a*(velUpperBound*curVel[0]/speed))/2;
			desVel[1]=gain*((1-a)*velpreviousValue[1]+a*(velUpperBound*curVel[1]/speed))/2;
			desVel[2]=gain*((1-a)*velpreviousValue[2]+a*(velUpperBound*curVel[2]/speed))/2;

			// speedPer=(std::sqrt(desVel[0]*desVel[0]+desVel[1]*desVel[1]+desVel[2]*desVel[2]))/(2*velUpperBound);

			velpreviousValue[0]=desVel[0];
			velpreviousValue[1]=desVel[1];
			velpreviousValue[2]=desVel[2];
		}

	}

	//speedPer=(std::sqrt(desVel[0]*desVel[0]+desVel[1]*desVel[1]+desVel[2]*desVel[2]))/(2*velUpperBound);
	//speedPer=(std::sqrt(desVel[0]*desVel[0]+desVel[1]*desVel[1]+desVel[2]*desVel[2]))/(2*velUpperBound);
	speedPer=hand_velocity_filtered.norm()/(velUpperBound);

	return desVel;

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
	
    ros::Publisher _pubDesiredOrientation=n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);  				// Publish desired orientation to the topic "/lwr_test/joint_controllers/passive_ds_command_orient"
    ros::Publisher _pubDesiredTwist=n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 10); 							// Publish desired twist to topic "/lwr/joint_controllers/passive_ds_command_vel"

    ros::Publisher _pubVelTester=n.advertise<geometry_msgs::Twist>("/velocity/tester", 10); 

	ros::Publisher _pubSpeedPer=n.advertise<handtracker::spper>("/lwr/speedPercentage", 10);  				// Publish the percentage of speed with respect to the maximum velocity"

    // Messages declaration
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
	geometry_msgs::Twist _msgDesiredTwist;
	handtracker::spper speedMsg;

	geometry_msgs::Twist _msgVelTester;


    startTime=ros::Time::now().toSec();

    // set the loop rate
    ros::Rate loop_rate(sRate);

    double checkTime=ros::Time::now().toSec();

    double currentTime=0.0;

    std::vector<double> currentVel(3,0);

	std::vector<double> desiredVel(3,0);    



	// set initial position and velocity to zero
	prev_hand_position(0)=0.0;
	prev_hand_position(1)=0.0; 
	prev_hand_position(2)=0.0;

	

	prev_hand_real_velocity(0)=0.0;
	prev_hand_real_velocity(1)=0.0;
	prev_hand_real_velocity(2)=0.0;

	// filter parameters

	double wn_filter_position, wn_filter_velocity, wn_filter_c, dt, dim,sample_time;
    wn_filter_position = 1.0;
    wn_filter_velocity = 30.0;
	wn_filter_c = 25.0;
	dt = (1.0/sRate);
    dim = 3;
	sample_time = dt;

	// initialize filters
	hand_pos_filter = new CDDynamics(dim, sample_time, wn_filter_position);

	hand_real_vel_filter = new CDDynamics(dim, sample_time, wn_filter_velocity);

	hand_pos_filter->SetStateTarget(E2M_v(prev_hand_position), E2M_v(prev_hand_position));

	hand_real_vel_filter->SetStateTarget(E2M_v(prev_hand_real_velocity), E2M_v(prev_hand_real_velocity));

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
    		std::cout<<"current vel: " << currentVel[0] << ", " << currentVel[1] << ", " << currentVel[2] << " sp: " << std::sqrt(currentVel[0]*currentVel[0]+currentVel[1]*currentVel[1]+currentVel[2]*currentVel[2]) << "\n"; 
    		

    		desiredVel=compDesiredVel(currentVel);

    		speedMsg.sPer=speedPer;
    		speedMsg.dir=handDirection;

    		// update checking time
    		checkTime=ros::Time::now().toSec();



    		_msgDesiredTwist.linear.x  = desiredVel[0];
			_msgDesiredTwist.linear.y  = desiredVel[1];
			_msgDesiredTwist.linear.z  = desiredVel[2];
			_msgDesiredTwist.angular.x = _omegad[0];
			_msgDesiredTwist.angular.y = _omegad[1];
			_msgDesiredTwist.angular.z = _omegad[2];

			_pubDesiredTwist.publish(_msgDesiredTwist);

			// Publish desired orientation
			_msgDesiredOrientation.w = _qd(0);
			_msgDesiredOrientation.x = _qd(1);
			_msgDesiredOrientation.y = _qd(2);
			_msgDesiredOrientation.z = _qd(3);

			_pubDesiredOrientation.publish(_msgDesiredOrientation);

			_pubSpeedPer.publish(speedMsg);

			_msgVelTester.linear.x = hand_velocity_filtered(0);
    		_msgVelTester.linear.y = hand_velocity_filtered(1);
	    	_msgVelTester.linear.z = hand_velocity_filtered(2);
			_msgVelTester.angular.x = _omegad[0];
			_msgVelTester.angular.y = _omegad[1];
			_msgVelTester.angular.z = _omegad[2];

			_pubVelTester.publish(_msgVelTester);

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