#include "ros/ros.h"

#include <geometry_msgs/PoseStamped.h>
#include "sensor_msgs/JointState.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "handtracker/spper.h"

#include "MotionGenerators/CDDynamics.h"
#include "mathlib_eigen_conversions.h"

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
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <iostream>
#include "Utils.h"



struct stat st = {0};

enum hand{
    left= 0,
    right=1,
};

int sRate=100;                                                   // set the sample rate (Hz)
int trialCounter=0;                                              // counter for the trials
double startTime;



/*-- Variables related to the mocap system --*/

double vel_gain=1.2;											// velocity gain

double lookTW=0.06;	                                            // the timewindow to look back for the average velocity (in seconds)
double velThreshold=0.05;                                      // velocity threshold for destinguish the motion or no=motion of the hand

double robot_velUpperBound=0.45;
double robot_velLowerBound=0.1;									// 

double hand_velUpperBound=0.8;
double hand_velLowerBound=0.0;

double alpha,beta;


//double velUpperBound=0.4;

float speedPer=0;
float ee_speed=0;

bool initOK=false;

int mocapRate=120;                                              // the sample rate of the motion capture system


double a=0.9;													// smoothing factor for the velocity
double a2=0.1;													// smoothing factor for the position



// ----------------- variables for the robot base -----------------

bool _firstKukaBasePoseReceived=false;
double robotBaseStamp=0;

Eigen::VectorXd robot_base_position(3);							// the position of the robot's base as received from the mocap system

Eigen::VectorXd robot_base_position_filtered(3);				// the filtered position of the robot's base

// helper variables for the filtering

MathLib::Vector robot_base_position_filtered_mathlib;
MathLib::Vector robot_base_velocity_filted_mathlib;

CDDynamics *robot_base_pos_filter;								// the filter for the robot-base position


// ----------------- variables for the object -----------------

bool _firstObjectPoseReceived=false;
double objectPoseStamp=0;
float _objectZOffset=0.15f;
float _objectYOffset=0.08f;

Eigen::VectorXd object_position(3);								// the position of the object as received from the mocap system

Eigen::VectorXd object_orientation(4);							// the orientation of the object as received from the mocap system

Eigen::VectorXd object_position_filtered(3);						// the filtered position of object

Eigen::VectorXd object_orientation_filtered(4);					// the filtered orientation of the object


// helper variables for the filtering

MathLib::Vector object_position_filtered_mathlib;
MathLib::Vector object_velocity_filted_mathlib;

MathLib::Vector object_orientation_filtered_mathlib;
MathLib::Vector object_angular_vel_filted_mathlib;

CDDynamics *object_position_filter, *object_orientation_filter;	// filters for the position and orientation of the object


// ----------------- variables for the hand, elbow and shoulder -----------------

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

Eigen::VectorXd curr_hand_position_filtered(3);

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

CDDynamics *shoulder_pos_filter;								// shoulder position filter

Eigen::VectorXd prev_shoulder_position(3);

Eigen::VectorXd curr_shoulder_position_filtered(3);

Eigen::VectorXd curr_shoulder_velocity_filtered(3);

Eigen::VectorXd curr_hand_rev_position_filtered(3);

Eigen::VectorXd cuur_hand_rev_velocity_filtered(3);




bool _firstdistance=false;
double current_shdistance=0;									// current distance of the hand from the shoulder
double previous_shdistance=0;									// previous distance of the hand from the shoulder
float handDirection=0;



bool _firstHandRP=false;



bool pub_true=false;

bool _firstRealPoseReceived=false;
geometry_msgs::Pose _msgRealPose;


float _distance2Target;


std::vector<double> mocapTime;                                   	// timestamp for the mocap system

std::vector< std::vector<double> > handRelPos;
Eigen::Vector3f _omegad; 								// Desired angular velocity [rad/s] (3x1)

std::vector< double > velpreviousValue(3,0);



// ------- motion variables --------------------- //
Eigen::VectorXd _motionDirection; 			// Direction of motion (3x1)
Eigen::VectorXd _obj2RobotFrame; 			// Direction of motion (3x1)
Eigen::Vector4f _qd; // Desired end effector quaternion (4x1)
Eigen::VectorXd _eePosition(3);
Eigen::VectorXd initialPosition(3);
bool motionOn=false;						// a variable to set the motion on (in case the hand valocity is larger than a threshold)
float TARGET_TOLERANCE = 0.05f; 				// Tolerance radius for reaching a target [m]
float _toolOffset=0.14f;



Eigen::Matrix3f _wRb;
Eigen::Vector4f _q;
Eigen::Vector4f _q0;


/*-- functions for the system --*/

// void saveRecordings();                                           // a function to save the data after each trial
// int getch_();                                                    // a function to catch a key press asynchronously



/*-- Callback functions --*/

void robotListener(const geometry_msgs::Pose::ConstPtr& msg){

	_msgRealPose = *msg;

	_eePosition << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
  	_q <<_msgRealPose.orientation.w,_msgRealPose.orientation.x,_msgRealPose.orientation.y,_msgRealPose.orientation.z;
	_wRb = Utils::quaternionToRotationMatrix(_q);
	// Eigen::Vector4d temp;
	// temp=_wRb.cast

	if(!_firstRealPoseReceived)
	{
		_firstRealPoseReceived = true;
		 _qd = _q;
		_eePosition = _eePosition+_toolOffset*_wRb.col(2).cast <double> ();
		_q0 = _q;
		// std::cerr << _qd.transpose() << std::endl;
		
	}
}

void computeDesiredOrientation()
{

	// Compute rotation error between current orientation and plane orientation using Rodrigues' law
	Eigen::Vector3f k;
	Eigen::Vector3f ref;
	Eigen::Vector3d temp;
	temp = _obj2RobotFrame-_eePosition;
	temp(2) = 0.0f;
	temp.normalize();
	ref = temp.cast<float>();
	k = (_wRb.col(2)).cross(ref);
	float c = (_wRb.col(2)).transpose()*ref;  
	float s = k.norm();
	k /= s;

	Eigen::Matrix3f K;
	K << Utils::getSkewSymmetricMatrix(k);

	Eigen::Matrix3f Re;
	if(fabs(s)< FLT_EPSILON)
	{
		Re = Eigen::Matrix3f::Identity();
	}
	else
	{
		Re = Eigen::Matrix3f::Identity()+s*K+(1-c)*K*K;
	}

	// Convert rotation error into axis angle representation
	Eigen::Vector3f omega;
	float angle;
	Eigen::Vector4f qtemp = Utils::rotationMatrixToQuaternion(Re);
	Utils::quaternionToAxisAngle(qtemp,omega,angle);

	// Compute final quaternion on plane
	Eigen::Vector4f qf = Utils::quaternionProduct(qtemp,_q);
	_qd = qf;

	// Perform quaternion slerp interpolation to progressively orient the end effector while approaching the plane
	_qd = Utils::slerpQuaternion(_q0,qf,1.0f-std::tanh(3.0f*_distance2Target));
	std::cerr << _wRb.col(2).transpose() << " " << ref.transpose() <<  std::endl;
	std::cerr << _distance2Target << std::endl;
	std::cerr << angle << std::endl;

	// Compute needed angular velocity to perform the desired quaternion
	Eigen::Vector4f qcurI, wq;
	qcurI(0) = _q(0);
	qcurI.segment(1,3) = -_q.segment(1,3);
	wq = 5.0f*Utils::quaternionProduct(qcurI,_qd-_q);
	Eigen::Vector3f omegaTemp = _wRb*wq.segment(1,3);
	_omegad = omegaTemp; 

}


void robotBaseListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for acquiring the position of the base of the robot arm --*/
    

	if(!_firstKukaBasePoseReceived)
	{
			
			_firstKukaBasePoseReceived=true;

			// extract position from the message
		   	robot_base_position(0)=mocapmsg.pose.position.x;
		    robot_base_position(1)=mocapmsg.pose.position.y;
		    robot_base_position(2)=mocapmsg.pose.position.z;

			// std::cout<<"Initial robot base pose received\n";
			ROS_INFO("Initial robot base pose received\n");
			
			// update the stamp of the mocap system
		    robotBaseStamp=mocapmsg.header.seq;
		
	}else{
		if(mocapmsg.header.seq!=robotBaseStamp){

			// extract position from the message
			robot_base_position(0)=mocapmsg.pose.position.x;
		    robot_base_position(1)=mocapmsg.pose.position.y;
		    robot_base_position(2)=mocapmsg.pose.position.z;

			if (initOK){
				// filtering the robot base position:

				// set the new position of the robot base
				robot_base_pos_filter->SetTarget(E2M_v(robot_base_position));

				// update the filter
				robot_base_pos_filter->Update();

				// get filtered positiono of the robot's base
				robot_base_pos_filter->GetState(robot_base_position_filtered_mathlib, robot_base_velocity_filted_mathlib);
				
				// convert the filtered position from mathlib to Eigen
				robot_base_position_filtered=M2E_v(robot_base_position_filtered_mathlib);

				// update the stamp of the mocap system
			    robotBaseStamp=mocapmsg.header.seq;
	
			}
			
		}
	}


    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}

void objectListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for acquiring the position of the base of the robot arm --*/
    

	if(!_firstObjectPoseReceived)
	{
			
			_firstObjectPoseReceived=true;

			// extract position from the message
		   	object_position(0)=mocapmsg.pose.position.x;
		    object_position(1)=mocapmsg.pose.position.y-_objectYOffset;
		    object_position(2)=mocapmsg.pose.position.z-_objectZOffset;

		    // extract orientation from the message
		    object_orientation << mocapmsg.pose.orientation.w, mocapmsg.pose.orientation.x, mocapmsg.pose.orientation.y, mocapmsg.pose.orientation.z;

			// std::cout<<"Initial object pose received\n";
			ROS_INFO("Initial object pose received\n");

			// update the stamp of the mocap system
		    objectPoseStamp=mocapmsg.header.seq;
		
	}else{
		if(mocapmsg.header.seq!=objectPoseStamp){

			// extract position from the message
			object_position(0)=mocapmsg.pose.position.x;
		    object_position(1)=mocapmsg.pose.position.y-_objectYOffset;
		    object_position(2)=mocapmsg.pose.position.z-_objectZOffset;

		    // extract orientation from the message
		    object_orientation << mocapmsg.pose.orientation.w, mocapmsg.pose.orientation.x, mocapmsg.pose.orientation.y, mocapmsg.pose.orientation.z;

		    if (initOK){
				// filtering the object's position:

				// set the new position of the object
				object_position_filter->SetTarget(E2M_v(object_position));

				// update the filter
				object_position_filter->Update();

				// get filtered position of the objcet
				object_position_filter->GetState(object_position_filtered_mathlib, object_velocity_filted_mathlib);
				
				// convert the filtered position from mathlib to Eigen
				object_position_filtered=M2E_v(object_position_filtered_mathlib);


				// filtering the object's orientation:

				// set the new orientation of the object in the filter
				object_orientation_filter->SetTarget(E2M_v(object_orientation));

				// update the filter
				object_orientation_filter->Update();

				// get filtered orientation of the objcet
				object_position_filter->GetState(object_orientation_filtered_mathlib, object_angular_vel_filted_mathlib);

				// convert the filtered position from mathlib to Eigen
				object_orientation_filtered=M2E_v(object_orientation_filtered_mathlib);

				// update the stamp of the mocap system
			    objectPoseStamp=mocapmsg.header.seq;
		    }

			

		}
	}


    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
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

		    //std::cout<<"hand pose: " << handPosition[0] << ", " << handPosition[1] << ", " << handPosition[2] << "\n"; 

			std::cout<<"Initial hand pose received\n";

			

		    handStamp=mocapmsg.header.seq;
		
	}else{
		if(mocapmsg.header.seq!=handStamp){

			

			human_hand_position_world(0)=mocapmsg.pose.position.x;
		    human_hand_position_world(1)=mocapmsg.pose.position.y;
		    human_hand_position_world(2)=mocapmsg.pose.position.z;

		    handPosition[0]=mocapmsg.pose.position.x;
		    handPosition[1]=mocapmsg.pose.position.y;
		    handPosition[2]=mocapmsg.pose.position.z;

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

    Eigen::VectorXd shoulder_position_world(3);
    Eigen::VectorXd shoulder_real_velocity(3);

    MathLib::Vector shoulder_position_filtered_mathlib;
    MathLib::Vector shoulder_real_velocity_mathlib;
    MathLib::Vector shoulder_velocity_filtered_mathlib;
    MathLib::Vector shoulder_acceleration_mathlib;


    


	if(!_firstshoulderPoseReceived)
		{
			
			_firstshoulderPoseReceived=true;

		   	shoulderPosition[0]=mocapmsg.pose.position.x;
		    shoulderPosition[1]=mocapmsg.pose.position.y;
		    shoulderPosition[2]=mocapmsg.pose.position.z;

		    // std::cout<<"shoulder pose: " << shoulderPosition[0] << ", " << shoulderPosition[1] << ", " << shoulderPosition[2] << "\n";

			std::cout<<"Initial shoulder pose received\n";

		    shoulderStamp=mocapmsg.header.seq;
		
	}else{
		
		if(mocapmsg.header.seq!=shoulderStamp){


		    shoulderPosition[0]=mocapmsg.pose.position.x;
		    shoulderPosition[1]=mocapmsg.pose.position.y;
		    shoulderPosition[2]=mocapmsg.pose.position.z;

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

		    prev_shoulder_position(0)=mocapmsg.pose.position.x;
			prev_shoulder_position(1)=mocapmsg.pose.position.y;		    
			prev_shoulder_position(2)=mocapmsg.pose.position.z;

		    shoulderStamp=mocapmsg.header.seq;

		}
	}

    shoulderCounter++;
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

	std:cout<<"hrv: " << hrv[0] << ", " << hrv[1] << ", " << hrv[2] <<"\n";

	if(!(_firstdistance)){
		_firstdistance=true;
		// previous_shdistance=current_shdistance;
		handDirection=0;
	}else{

		if(std::fabs(hrv[0])<0.05){
			// handDirection=0;
		}else{
			if(hrv[0]>0){
				std::cout<<"testokkkkkkkkkkkkkkkkkkkkk111111111111111111111111111111111111111111111111111111111\n";
			handDirection=1;
			}else{
				handDirection=-1;
				std::cout<<"testokkkkkkkkkkkkkkkkkkkkk22222222222222222222222222222222222222222222222222222222222222\n";
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

	//std::cout<<"current real hand position: " << hrp[0] << ", " << hrp[1] << ", " << hrp[2] <<" \n";

	if(!_firstHandRP){
		
		_firstHandRP=true;
		return hrp;
	}

	
	if (initOK){
		Eigen::VectorXd human_hand_position_world(3);
	    Eigen::VectorXd human_hand_real_velocity_world(3);

	    MathLib::Vector human_hand_position_world_filtered_mathlib;
	    MathLib::Vector human_hand_real_velocity_world_mathlib;
	    MathLib::Vector human_hand_real_velocity_root_filtered_mathlib;
	    MathLib::Vector human_hand_real_acceleration_root_mathlib;


		human_hand_position_world(0)=hrp[0];
		human_hand_position_world(1)=hrp[1];
		human_hand_position_world(2)=hrp[2];

		//std::cout<<"m current real hand position: " << human_hand_position_world(0) << ", " << human_hand_position_world(1) << ", " << human_hand_position_world(2) <<" \n";
			    
		// filtering the hand position
		hand_pos_filter->SetTarget(E2M_v(human_hand_position_world));


		
		hand_pos_filter->Update();
		hand_pos_filter->GetState(human_hand_position_world_filtered_mathlib, human_hand_real_velocity_world_mathlib);
		cuur_hand_rev_velocity_filtered = M2E_v(human_hand_real_velocity_world_mathlib);
		curr_hand_rev_position_filtered=M2E_v(human_hand_position_world_filtered_mathlib);


		hrp[0]=curr_hand_rev_position_filtered(0);
		hrp[1]=curr_hand_rev_position_filtered(1);
		hrp[2]=curr_hand_rev_position_filtered(2);
	}

	

	//std::cout<<"current filtered hand position: " << hrp[0] << ", " << hrp[1] << ", " << hrp[2] <<" \n";

	return hrp;
}


std::vector<double> compDesiredVel(std::vector<double> curVel){



	std::vector<double> desVel(3,0);				// the desired velocity of the hand

	_obj2RobotFrame=object_position_filtered-robot_base_position_filtered;

	_motionDirection=_obj2RobotFrame-_eePosition;
	if (handDirection<0.0){
		_motionDirection=initialPosition-_eePosition;
	}
	_motionDirection.normalize();



	if (!motionOn){

		// Go in pause state
		desVel[0]=0.0f;
		desVel[1]=0.0f;
		desVel[2]=0.0f;;


	}else{
		desVel[0]=ee_speed*_motionDirection(0);
		desVel[1]=ee_speed*_motionDirection(1);
		desVel[2]=ee_speed*_motionDirection(2);

		// Compute distance to target
		_distance2Target = (_obj2RobotFrame-_eePosition).norm();
		if(_distance2Target < TARGET_TOLERANCE){

			// Go in pause state
			desVel[0]=0.0f;
			desVel[1]=0.0f;
			desVel[2]=0.0f;;

		}
	}
	
	

	return desVel;

}


double Hand2RobotSpeed(double handspeed){

	double y1=((robot_velUpperBound+robot_velLowerBound)/2)+((robot_velUpperBound-robot_velLowerBound)/2)*std::tanh((handspeed-((hand_velUpperBound+hand_velLowerBound)/2))*(6/(hand_velUpperBound-hand_velLowerBound)));

	double y2=alpha*+beta;

	double y=(1-std::tanh(handspeed/((hand_velUpperBound+hand_velLowerBound)/2)))*y2+std::tanh(handspeed/((hand_velUpperBound+hand_velLowerBound)/2))*y1;

	return y;

}



std::vector<double> compHandRelVel(){

	std::vector<double> curVel(3,0);
	std::vector<double> desVel(3,0);				// the desired velocity of the hand

	//ee_speed=0;
	double speed=0;


    MathLib::Vector hand_rel_velocity_mathlib;
    MathLib::Vector hand_rel_acceleration_mathlib;


	// filtering the velocity
	hand_real_vel_filter->SetTarget(E2M_v(cuur_hand_rev_velocity_filtered));
	hand_real_vel_filter->Update();
	hand_real_vel_filter->GetState(hand_rel_velocity_mathlib, hand_rel_acceleration_mathlib);
	hand_velocity_filtered = M2E_v(hand_rel_velocity_mathlib);

	curVel[0]=hand_rel_velocity_mathlib(0);
	curVel[1]=hand_rel_velocity_mathlib(1);
	curVel[2]=hand_rel_velocity_mathlib(2);


	speed=std::sqrt(curVel[0]*curVel[0]+curVel[1]*curVel[1]+curVel[2]*curVel[2]);

	// map to the robot speed

	ee_speed=Hand2RobotSpeed(speed);
	

	if(speed>=velThreshold){

		motionOn=true;
		desVel[0]=vel_gain*ee_speed*curVel[0]/speed;
		desVel[1]=vel_gain*ee_speed*curVel[1]/speed;
		desVel[2]=vel_gain*ee_speed*curVel[2]/speed;



	}else{
		desVel[0]=0.0;
		desVel[1]=0.0;
		desVel[2]=0.0;
	}

	
	std::cout<< "dif" << ee_speed-std::sqrt(desVel[0]*desVel[0]+desVel[1]*desVel[1]+desVel[2]*desVel[2]) << "\n";

	speedPer=hand_velocity_filtered.norm()/(hand_velUpperBound-0.2);

	// if(!(speed<=velThreshold)){

	// 	if(speed>1){
	// 		desVel[0]=velpreviousValue[0];
	// 		desVel[1]=velpreviousValue[1];
	// 		desVel[2]=velpreviousValue[2];

	// 	}else{
	// 		desVel[0]=gain*((1-a)*velpreviousValue[0]+a*(velUpperBound*curVel[0]/speed))/2;
	// 		desVel[1]=gain*((1-a)*velpreviousValue[1]+a*(velUpperBound*curVel[1]/speed))/2;
	// 		desVel[2]=gain*((1-a)*velpreviousValue[2]+a*(velUpperBound*curVel[2]/speed))/2;

	// 		// speedPer=(std::sqrt(desVel[0]*desVel[0]+desVel[1]*desVel[1]+desVel[2]*desVel[2]))/(2*velUpperBound);

	// 		velpreviousValue[0]=desVel[0];
	// 		velpreviousValue[1]=desVel[1];
	// 		velpreviousValue[2]=desVel[2];
	// 	}

	// }

	// //speedPer=(std::sqrt(desVel[0]*desVel[0]+desVel[1]*desVel[1]+desVel[2]*desVel[2]))/(2*velUpperBound);
	// //speedPer=(std::sqrt(desVel[0]*desVel[0]+desVel[1]*desVel[1]+desVel[2]*desVel[2]))/(2*velUpperBound);
	// speedPer=hand_velocity_filtered.norm()/(velUpperBound);

	// return desVel;

	return curVel;
	

}


void computeLinearParameters_speed(){

	Eigen::Matrix2d A;
	Eigen::Vector2d B;
	Eigen::Vector2d Sol;

	A << (hand_velUpperBound+hand_velLowerBound)/2, 1, 0.1, 1;

	//std::cout<<"A=\n "<< A <<"\n";

	B << (robot_velUpperBound+robot_velLowerBound)/2, 0.1;
	
	//std::cout<<"B=\n "<< B <<"\n";

	Sol=A.colPivHouseholderQr().solve(B);

	alpha=Sol(0);
	beta=Sol(1);

	//std::cout<<"alpha= " << alpha << ", beta= " << beta <<"\n";

}




int main(int argc, char **argv)
{


    // initialize the node
    ros::init(argc, argv, "handtracker");

    ros::NodeHandle n;


    // set the publishers for the allegro hand

    //ros::Publisher allegorRight_pub = n.advertise<sensor_msgs::JointState>("rhand/velocity", 100);



    // set the subscribers to listen the classification outcome from the windows machine and the position of the hand

    ros::Subscriber handSub=n.subscribe("/hand/pose", 10, handListener);

    ros::Subscriber elbowSub=n.subscribe("elbow/pose", 10, elbowListener);

    ros::Subscriber shoulderSub=n.subscribe("/shoulder/pose", 10, shoulderListener);

    ros::Subscriber objectSub=n.subscribe("/object/pose", 10, objectListener);

    ros::Subscriber robotBaseSub=n.subscribe("/robotBase/pose", 10, robotBaseListener);

    ros::Subscriber robotSub=n.subscribe("lwr/ee_pose", 10, robotListener);


	// ros::Subscriber _subRealPose;						// Subscribe to robot current pose
    // ros::Subscriber _subRealTwist;          				// Subscribe to robot current pose
	
    ros::Publisher _pubDesiredOrientation=n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);  				// Publish desired orientation to the topic "/lwr_test/joint_controllers/passive_ds_command_orient"
    ros::Publisher _pubDesiredTwist=n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 10); 							// Publish desired twist to topic "/lwr/joint_controllers/passive_ds_command_vel"

    ros::Publisher _pubVelTester=n.advertise<geometry_msgs::Twist>("/handtracker/tester/velocity", 10); 

    ros::Publisher _pubSpeedTester=n.advertise<handtracker::spper>("/handtracker/tester/handspeed", 10); 


    ros::Publisher _pubPosTester=n.advertise<geometry_msgs::Twist>("/handtracker/tester/position", 10); 

	ros::Publisher _pubSpeedPer=n.advertise<handtracker::spper>("/handtracker/speedPercentage", 10);  				// Publish the percentage of speed with respect to the maximum velocity"

    // Messages declaration
    geometry_msgs::Pose _msgRealPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
	geometry_msgs::Twist _msgDesiredTwist;
	handtracker::spper speedMsg;

	geometry_msgs::Twist _msgVelTester;
	handtracker::spper _msgEeSpeedTester;
	geometry_msgs::Twist _msgPosTester;


    startTime=ros::Time::now().toSec();

    // set the loop rate
    ros::Rate loop_rate(sRate);

    double checkTime=ros::Time::now().toSec();

    double currentTime=0.0;

    std::vector<double> currentVel(3,0);

	std::vector<double> desiredVel(3,0);    

	std::vector<double> currentVel2(3,0);

	computeLinearParameters_speed();

	_omegad.setConstant(0.0f);

	// set initial position and velocity to zero
	// prev_hand_position(0)=0.0;
	// prev_hand_position(1)=0.0; 
	// prev_hand_position(2)=0.0;	

	// prev_hand_real_velocity(0)=0.0;
	// prev_hand_real_velocity(1)=0.0;
	// prev_hand_real_velocity(2)=0.0;

	// spinonce for finding the initial position and velocity ot the hand

	
	while(!_firstshoulderPoseReceived){

		while(!_firsthandPoseReceived){
			ros::spinOnce();
			loop_rate.sleep();	
		}
		ros::spinOnce();
		loop_rate.sleep();
	}


	// while(!_firsthandPoseReceived){
		
	// 	ros::spinOnce();
	// 	loop_rate.sleep();
	// }
	
	std::vector <double> initPos=handRP(handPosition,shoulderPosition);
	

	// std::vector <double> initVel=compHandRelVel();

	

	prev_hand_position(0)=initPos[0];
	prev_hand_position(1)=initPos[1];
	prev_hand_position(2)=initPos[2];

	std::cout<<"prev_hand_position: " << prev_hand_position << "\n";

	prev_hand_real_velocity(0)=0.0;
	prev_hand_real_velocity(1)=0.0;
	prev_hand_real_velocity(2)=0.0;

	// filter parameters

	double wn_filter_position, wn_filter_velocity, wn_filter_c, dt, dim,sample_time;
    wn_filter_position = 10.0;
    wn_filter_velocity = 200.0;//30
	wn_filter_c = 25.0;
	dt = (1.0/mocapRate);
    dim = 3;
	sample_time = dt;

	// initialize filters
	hand_pos_filter = new CDDynamics(dim, sample_time, wn_filter_position);

	hand_real_vel_filter = new CDDynamics(dim, sample_time, wn_filter_velocity);

	shoulder_pos_filter= new  CDDynamics(dim, sample_time, wn_filter_position);

	hand_pos_filter->SetStateTarget(E2M_v(prev_hand_position), E2M_v(prev_hand_position));

	shoulder_pos_filter->SetStateTarget(E2M_v(prev_hand_position), E2M_v(prev_hand_position));

	hand_real_vel_filter->SetStateTarget(E2M_v(prev_hand_real_velocity), E2M_v(prev_hand_real_velocity));

	// robot-base's position filter
	robot_base_pos_filter= new  CDDynamics(dim, sample_time, wn_filter_position);

	robot_base_pos_filter->SetStateTarget(E2M_v(robot_base_position), E2M_v(prev_hand_position));

	// object's position filter
	object_position_filter= new  CDDynamics(dim, sample_time, wn_filter_position);

	object_position_filter->SetStateTarget(E2M_v(object_position), E2M_v(object_position));

	// object's orientation filter
	object_orientation_filter= new  CDDynamics(4, sample_time, wn_filter_position);

	object_orientation_filter->SetStateTarget(E2M_v(object_orientation), E2M_v(object_orientation));

	initialPosition<< -0.47225f, 0.0f, 0.2847f;


	initOK=true;



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


    	_msgPosTester.linear.x = curr_hand_rev_position_filtered(0);
    	_msgPosTester.linear.y = curr_hand_rev_position_filtered(1);
    	_msgPosTester.linear.z = curr_hand_rev_position_filtered(2);
    	_msgPosTester.angular.x = _omegad[0];
		_msgPosTester.angular.y = _omegad[1];
		_msgPosTester.angular.z = _omegad[2];

    	_pubPosTester.publish(_msgPosTester);
    	
    	// std::cout<<"size of handRePos: " << (int)handRelPos.size()<<"\n";

    	// std::cout<< "time passed: " << currentTime - checkTime<< "\n";

    	//currentVel2=compHandRelVel();

		// _msgVelTester.linear.x = currentVel2[0];
  		// _msgVelTester.linear.y = currentVel2[1];
	 	// _msgVelTester.linear.z = currentVel2[2];
	    // _msgVelTester.linear.x = hand_velocity_filtered(0);
    	// _msgVelTester.linear.y = hand_velocity_filtered(1);
	    // _msgVelTester.linear.z = hand_velocity_filtered(2);
		// _msgVelTester.angular.x = _omegad[0];
		// _msgVelTester.angular.y = _omegad[1];
		// _msgVelTester.angular.z = _omegad[2];

		// _pubVelTester.publish(_msgVelTester);


    	 if(currentTime-checkTime>=lookTW){
    		currentVel=handRV(handRelPos);

    		// remove the oldest relative position
    		handRelPos.clear();

    		// update checking time
    		checkTime=ros::Time::now().toSec();
    	}

    		

    		currentVel2=compHandRelVel();
    		//std::cout<<"current vel: " << currentVel[0] << ", " << currentVel[1] << ", " << currentVel[2] << " sp: " << std::sqrt(currentVel[0]*currentVel[0]+currentVel[1]*currentVel[1]+currentVel[2]*currentVel[2]) << "\n"; 
    		
    		


    		desiredVel=compDesiredVel(currentVel);

    		computeDesiredOrientation();

    		speedMsg.sPer=speedPer;
    		speedMsg.dir=handDirection;

    		_msgEeSpeedTester.sPer=ee_speed;
    		_msgEeSpeedTester.dir=handDirection;





   //  		_msgDesiredTwist.linear.x  = -currentVel2[0];
			// _msgDesiredTwist.linear.y  = currentVel2[1];
			// _msgDesiredTwist.linear.z  = currentVel2[2];
			_msgDesiredTwist.angular.x = _omegad[0];
			_msgDesiredTwist.angular.y = _omegad[1];
			_msgDesiredTwist.angular.z = _omegad[2];

			_msgDesiredTwist.linear.x  = desiredVel[0];
			_msgDesiredTwist.linear.y  = desiredVel[1];
			_msgDesiredTwist.linear.z  = desiredVel[2];
			// _msgDesiredTwist.angular.x = _omegad[0];
			// _msgDesiredTwist.angular.y = _omegad[1];
			// _msgDesiredTwist.angular.z = _omegad[2];

			_pubDesiredTwist.publish(_msgDesiredTwist);

			// Publish desired orientation
			_msgDesiredOrientation.w = _qd(0);
			_msgDesiredOrientation.x = _qd(1);
			_msgDesiredOrientation.y = _qd(2);
			_msgDesiredOrientation.z = _qd(3);

			_pubDesiredOrientation.publish(_msgDesiredOrientation);

			_pubSpeedPer.publish(speedMsg);
			_pubSpeedTester.publish(_msgEeSpeedTester);


			// _msgVelTester.linear.x = hand_velocity_filtered(0);
   //  		_msgVelTester.linear.y = hand_velocity_filtered(1);
	  //   	_msgVelTester.linear.z = hand_velocity_filtered(2);
			// currentVel2=compHandRelVel();
			_msgVelTester.linear.x = currentVel2[0];
    		_msgVelTester.linear.y = currentVel2[1];
	    	_msgVelTester.linear.z = currentVel2[2];
			_msgVelTester.angular.x = _omegad[0];
			_msgVelTester.angular.y = _omegad[1];
			_msgVelTester.angular.z = _omegad[2];

			_pubVelTester.publish(_msgVelTester);
	    	

    		
    		

    	// }


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


	_msgDesiredTwist.linear.x  = 0.0f;
	_msgDesiredTwist.linear.y  = 0.0f;
	_msgDesiredTwist.linear.z  =0.0f;
	_msgDesiredTwist.angular.x = 0.0f;
	_msgDesiredTwist.angular.y = 0.0f;
	_msgDesiredTwist.angular.z = 0.0f;

	_pubDesiredTwist.publish(_msgDesiredTwist);

	// Publish desired orientation
	_msgDesiredOrientation.w = _qd(0);
	_msgDesiredOrientation.x = _qd(1);
	_msgDesiredOrientation.y = _qd(2);
	_msgDesiredOrientation.z = _qd(3);

	_pubDesiredOrientation.publish(_msgDesiredOrientation);

	ros::spinOnce();

	loop_rate.sleep();

    return 0;
}
