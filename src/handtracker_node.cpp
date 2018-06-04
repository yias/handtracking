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
#include <fstream>
#include "Utils.h"



struct stat st = {0};

enum hand{
    left= 0,
    right=1,
};

int sRate=100;                                                   // set the sample rate (Hz)
int trialCounter=0;                                              // counter for the trials
double startTime;

std::ofstream _outputFile;    									// stream object to store the log data
std::string fileName;											// name of the log-file
bool saveData=true;


/*-- Variables related to the mocap system --*/

double vel_gain=1.2;											// velocity gain

double lookTW=0.06;	                                            // the timewindow to look back for the average velocity (in seconds)
double velThreshold=0.05;                                      // velocity threshold for destinguish the motion or no=motion of the hand

double robot_velUpperBound=0.45;
double robot_velLowerBound=0.05;									// 

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

Eigen::VectorXd object_position_filtered(3);					// the filtered position of object

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

Eigen::Vector3d handPosition;                      			    // vector for the position of the hand (in)
Eigen::Vector4d handOrientation;                       	// vector for the orientation of the hand (in quaternions)
// std::vector<double> handVelocity(3,0);                          // vector for the velocity of the hand (in m/s)

// std::vector<double> handPrevPosition(3,0);                      // vector for the previous position of the hand (in)
// std::vector<double> handPrevOrientation(4,0);                   // vector for the previous orientation of the hand (in quaternions)

CDDynamics *hand_pos_filter, *hand_real_vel_filter;

// Eigen::VectorXd hand_revPosition(3);

Eigen::VectorXd prev_hand_position(3);

Eigen::VectorXd curr_hand_rev_position_filtered(3);

Eigen::VectorXd cuur_hand_rev_velocity_filtered(3);

Eigen::VectorXd prev_hand_real_velocity(3);

Eigen::VectorXd hand_velocity_filtered(3);

MathLib::Vector hand_position_filtered_mathlib;				// helper variable for the fintering
MathLib::Vector hand_real_velocity_mathlib;					// helper variable for the fintering
MathLib::Vector hand_rel_velocity_mathlib;					// helper variable for the fintering
MathLib::Vector hand_rel_acceleration_mathlib;				// helper variable for the fintering


bool _firstHandRP=false;

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

unsigned int elbowCounter=0;                                    // counter for elbow-related messages from the mocap system
bool _firstelbowPoseReceived=false;
double elbowStamp=0;

Eigen::Vector3d elbowPosition;                         			// vector for the position of the elbow as receieved from the mocap system(in m)
Eigen::Vector4d elbowOrientation;                      			// vector for the orientation of the elbow as receieved from the mocap system (in quaternions)
std::vector<double> elbowVelocity(3,0);                         // vector for the velocity of the elbow (in m/s)

CDDynamics *elbow_pos_filter, *elbow_real_vel_filter;			// position and velocity filters for the elbow

Eigen::VectorXd elbow_revPosition(3);							// the elbow position with respect to the shoulder

Eigen::VectorXd elbow_revPosition_filtered(3);					// the filtered elbow position with respect to the shoulder

Eigen::VectorXd elbow_revPvelocity(3);							// the elbow velocity with respect to the shoulder

Eigen::VectorXd elbow_revPvelocity_filtered(3);					// the filtered elbow velocity with respect to the shoulder

MathLib::Vector elbow_position_filtered_mathlib;				// helper variable for the fintering
MathLib::Vector elbow_real_velocity_mathlib;					// helper variable for the fintering

bool _firstElbowRP=false;										// a boolian variable for checking if the first relative position has reveiced (used for the purpose of fintering)

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

unsigned int shoulderCounter=0;                                 // counter for shoulder-related messages from the mocap system
bool _firstshoulderPoseReceived=false;
double shoulderStamp=0;

Eigen::Vector3d shoulderPosition;							// vector for the position of the shoulder
Eigen::Vector4d shoulderOrientation;						// vector for the orientation of the shoulder (in quaternions)
// std::vector<double> shoulderVelocity(3,0);						// vector for the velocity of the shoulder (in m/s)

// std::vector<double> shoulderPrevPosition(3,0);					// vector for the previous position of the shoulder
std::vector<double> shoulderPrevOrientation(4,0);				// vector for the previous orientation of the shoulder (in quaternions)

// CDDynamics *shoulder_pos_filter;								// shoulder position filter

// Eigen::VectorXd prev_shoulder_position(3);




bool _firstdistance=false;
double current_shdistance=0;									// current distance of the hand from the shoulder
double previous_shdistance=0;									// previous distance of the hand from the shoulder
float handDirection=0;



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
	// std::cerr << _wRb.col(2).transpose() << " " << ref.transpose() <<  std::endl;
	// std::cerr << _distance2Target << std::endl;
	// std::cerr << angle << std::endl;

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
		   	handPosition(0)=mocapmsg.pose.position.x;
		    handPosition(1)=mocapmsg.pose.position.y;
		    handPosition(2)=mocapmsg.pose.position.z;

		    //std::cout<<"hand pose: " << handPosition[0] << ", " << handPosition[1] << ", " << handPosition[2] << "\n"; 

		    ROS_INFO("Initial hand pose received\n");			

		    handStamp=mocapmsg.header.seq;
		
	}else{
		if(mocapmsg.header.seq!=handStamp){

			

			handPosition(0)=mocapmsg.pose.position.x;
		    handPosition(1)=mocapmsg.pose.position.y;
		    handPosition(2)=mocapmsg.pose.position.z;

		    handOrientation(0)=mocapmsg.pose.orientation.x;
		    handOrientation(1)=mocapmsg.pose.orientation.x;
		    handOrientation(2)=mocapmsg.pose.orientation.x;
		    handOrientation(3)=mocapmsg.pose.orientation.x;
		    
		    handStamp=mocapmsg.header.seq;

		}
	}


    handCounter++;
}



void elbowListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

    	if(!_firstelbowPoseReceived)
		{
			
			_firstelbowPoseReceived=true;

		    elbowPosition(0)=mocapmsg.pose.position.x;
		    elbowPosition(1)=mocapmsg.pose.position.y;
		    elbowPosition(2)=mocapmsg.pose.position.z;

		    ROS_INFO("Initial elbow pose received\n");	

		    elbowStamp=mocapmsg.header.seq;
		}else{
			if(mocapmsg.header.seq!=elbowStamp){

				elbowPosition(0)=mocapmsg.pose.position.x;
			    elbowPosition(1)=mocapmsg.pose.position.y;
			    elbowPosition(2)=mocapmsg.pose.position.z;

				elbowOrientation(0)=mocapmsg.pose.orientation.x;
			    elbowOrientation(1)=mocapmsg.pose.orientation.y;
			    elbowOrientation(2)=mocapmsg.pose.orientation.z;
			    elbowOrientation(3)=mocapmsg.pose.orientation.w;

			    shoulderStamp=mocapmsg.header.seq;
			}
		}

    
    elbowCounter++;
}

void shoulderListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/


	if(!_firstshoulderPoseReceived)
		{
			
			_firstshoulderPoseReceived=true;

		   	shoulderPosition(0)=mocapmsg.pose.position.x;
		    shoulderPosition(1)=mocapmsg.pose.position.y;
		    shoulderPosition(2)=mocapmsg.pose.position.z;

		    ROS_INFO("Initial shoulder pose received\n");	

		    shoulderStamp=mocapmsg.header.seq;
		
	}else{
		
		if(mocapmsg.header.seq!=shoulderStamp){


		    shoulderPosition(0)=mocapmsg.pose.position.x;
		    shoulderPosition(1)=mocapmsg.pose.position.y;
		    shoulderPosition(2)=mocapmsg.pose.position.z;

		    shoulderOrientation(0)=mocapmsg.pose.orientation.x;
		    shoulderOrientation(1)=mocapmsg.pose.orientation.y;
		    shoulderOrientation(2)=mocapmsg.pose.orientation.z;
		    shoulderOrientation(3)=mocapmsg.pose.orientation.w;

		    shoulderStamp=mocapmsg.header.seq;

		}
	}

    shoulderCounter++;
}



int compHandDirection(std::vector< std::vector<double> > handrpos){

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

	// std:cout<<"hrv: " << hrv[0] << ", " << hrv[1] << ", " << hrv[2] <<"\n";

	if(!(_firstdistance)){
		_firstdistance=true;
		// previous_shdistance=current_shdistance;
		handDirection=0;
	}else{

		if(std::fabs(hrv[0])<0.05){
			// handDirection=0;
		}else{
			if(hrv[0]>0){
			handDirection=1;
			}else{
				handDirection=-1;
			}
		}
	}

	previous_shdistance=current_shdistance;
	
	return handDirection;
}

Eigen::Vector3d handRP(Eigen::Vector3d handPos, Eigen::Vector3d shoulderPos){


/*-- a function to calculate the position of the hand with respect to the shoulder --*/


	Eigen::Vector3d hrp;				// the relative velocity of the hand

	hrp(0)=handPos(0)-shoulderPos(0);
	hrp(1)=handPos(1)-shoulderPos(1);
	hrp(2)=handPos(2)-shoulderPos(2);

	//std::cout<<"current real hand position: " << hrp[0] << ", " << hrp[1] << ", " << hrp[2] <<" \n";

	if(!_firstHandRP){
		
		_firstHandRP=true;
		ROS_INFO("First relative position computed\n");
		return hrp;
	}

	
	if (initOK){
		    
		// filtering the hand position
		hand_pos_filter->SetTarget(E2M_v(hrp));	
		hand_pos_filter->Update();
		hand_pos_filter->GetState(hand_position_filtered_mathlib, hand_real_velocity_mathlib);
		cuur_hand_rev_velocity_filtered = M2E_v(hand_real_velocity_mathlib);
		curr_hand_rev_position_filtered=M2E_v(hand_position_filtered_mathlib);

	}

	
	return hrp;
}


Eigen::Vector3d elbowRP(){


/*-- a function to calculate the position of the elbow with respect to the shoulder --*/

	Eigen::Vector3d Erp;							// the relative position of the elbow

	Erp(0)=elbowPosition(0)-shoulderPosition[0];
	Erp(1)=elbowPosition(1)-shoulderPosition[1];
	Erp(2)=elbowPosition(2)-shoulderPosition[2];
	

	if(!_firstElbowRP){
		
		_firstElbowRP=true;
		return Erp;
	}

	
	if (initOK){

			    
		//	filtering the elbow position
		elbow_pos_filter->SetTarget(E2M_v(Erp));
		elbow_pos_filter->Update();
		elbow_pos_filter->GetState(elbow_position_filtered_mathlib, elbow_real_velocity_mathlib);
		elbow_revPosition_filtered=M2E_v(elbow_position_filtered_mathlib);
		elbow_revPvelocity=M2E_v(elbow_real_velocity_mathlib);

	}


	return Erp;
}


std::vector<double> compDesiredVel(){



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
		// desVel[0]=vel_gain*ee_speed*curVel[0]/speed;
		// desVel[1]=vel_gain*ee_speed*curVel[1]/speed;
		// desVel[2]=vel_gain*ee_speed*curVel[2]/speed;



	}else{
		// desVel[0]=0.0;
		// desVel[1]=0.0;
		// desVel[2]=0.0;
	}


	speedPer=hand_velocity_filtered.norm()/(hand_velUpperBound-0.2);

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

int saveData2File(std::ofstream& optF,double ctTime,Eigen::VectorXd _eePos,Eigen::Vector4f _oriPose,float eeS, std::vector<double> dVel,float handDir,Eigen::Vector3d _handRP,Eigen::VectorXd handRP_filtered, Eigen::VectorXd _handRV, Eigen::VectorXd handRV_filtered,Eigen::Vector3d eRP,Eigen::Vector3d eRP_filtered,Eigen::Vector3d elbow_revVel, Eigen::Vector3d rawHandPosition, Eigen::Vector4d rawHandOrientation,Eigen::Vector3d rawElbowPosition, Eigen::Vector4d rawElbowOrientation,Eigen::Vector3d rawShoulderPosition, Eigen::Vector4d rawShoulderOrientation){

	if (optF.is_open()){
		optF << ctTime << " ";
		optF << _eePos(0) << " " << _eePos(1) << " " << _eePos(2) << " ";
		optF << _oriPose(0) << " " << _oriPose(1) << " " << _oriPose(2) << " " << _oriPose(3) << " ";
		optF <<  eeS << " ";
		optF << dVel[0] << " " << dVel[1] << " " << dVel[2] << " ";
		optF << handDir << " ";
		optF << _handRP(0) << " " << _handRP(1) << " " << _handRP(2) <<  " ";
		optF << handRP_filtered(0) << " " << handRP_filtered(1) << " " << handRP_filtered(2) << " ";
		optF << _handRV(0) << " " << _handRV(1) << " " << _handRV(2) <<  " ";
		optF << handRV_filtered(0) << " " << handRV_filtered(1) << " " << handRV_filtered(2) << " ";
		optF << eRP(0) << " " << eRP(1) << " " << eRP(2) << " ";
		optF << eRP_filtered(0) << " " << eRP_filtered(1) << " " << eRP_filtered(2) << " ";
		optF << elbow_revVel(0) << " " << elbow_revVel(1) << " " << elbow_revVel(2) << " ";
		optF << rawHandPosition(0) << " " << rawHandPosition(1) << " " << rawHandPosition(2) << " ";
		optF << rawHandOrientation(0) << " " << rawHandOrientation(1) << " " << rawHandOrientation(2) << " ";
		optF << rawElbowPosition(0) << " " << rawElbowPosition(1) << " " << rawElbowPosition(2) << " ";
		optF << rawElbowOrientation(0) << " " << rawElbowOrientation(1) << " " << rawElbowOrientation(2) << " ";
		optF << rawShoulderPosition(0) << " " << rawShoulderPosition(1) << " " << rawShoulderPosition(2) << " ";
		optF << rawShoulderOrientation(0) << " " << rawShoulderOrientation(1) << " " << rawShoulderOrientation(2);
		optF << std::endl;	
		return 1;
	}else{
		return 0;
	}
		
}




int main(int argc, char **argv)
{


    // initialize the node
    ros::init(argc, argv, "handtracker");

    ros::NodeHandle n;


    // set the publishers for the allegro hand

   
    // set the subscribers to listen the classification outcome from the windows machine and the position of the hand

    ros::Subscriber handSub=n.subscribe("/hand/pose", 10, handListener);

    ros::Subscriber elbowSub=n.subscribe("elbow/pose", 10, elbowListener);

    ros::Subscriber shoulderSub=n.subscribe("/shoulder/pose", 10, shoulderListener);

    ros::Subscriber objectSub=n.subscribe("/object/pose", 10, objectListener);

    ros::Subscriber robotBaseSub=n.subscribe("/robotBase/pose", 10, robotBaseListener);

    ros::Subscriber robotSub=n.subscribe("lwr/ee_pose", 10, robotListener);


	
    ros::Publisher _pubDesiredOrientation=n.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 1);  				// Publish desired orientation to the topic "/lwr_test/joint_controllers/passive_ds_command_orient"
    
    ros::Publisher _pubDesiredTwist=n.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 10); 							// Publish desired twist to topic "/lwr/joint_controllers/passive_ds_command_vel"

    ros::Publisher _pubVelTester=n.advertise<geometry_msgs::Twist>("/handtracker/tester/velocity", 10); 

    ros::Publisher _pubSpeedTester=n.advertise<handtracker::spper>("/handtracker/tester/handspeed", 10); 


    ros::Publisher _pubPosTester=n.advertise<geometry_msgs::Twist>("/handtracker/tester/position", 10); 

	ros::Publisher _pubSpeedPer=n.advertise<handtracker::spper>("/handtracker/speedPercentage", 10);  				// Publish the percentage of speed with respect to the maximum velocity"

    // messages declaration
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

	std::vector<double> currHandVel(3,0);

	computeLinearParameters_speed();

	_omegad.setConstant(0.0f);


	while(!_firstshoulderPoseReceived){
		while(!_firsthandPoseReceived){
			ros::spinOnce();
			loop_rate.sleep();	
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	
	Eigen::Vector3d initHandPos=handRP(handPosition,shoulderPosition);


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

	// hand filters

	hand_pos_filter = new CDDynamics(dim, sample_time, wn_filter_position);

	hand_real_vel_filter = new CDDynamics(dim, sample_time, wn_filter_velocity);

	hand_pos_filter->SetStateTarget(E2M_v(initHandPos), E2M_v(initHandPos));

	hand_real_vel_filter->SetStateTarget(E2M_v(prev_hand_real_velocity), E2M_v(prev_hand_real_velocity));


	// elbow filters

	Eigen::Vector3d initElbowPos;
	initElbowPos=elbowRP();

	elbow_pos_filter = new CDDynamics(dim, sample_time, wn_filter_position);

	elbow_pos_filter->SetStateTarget(E2M_v(initElbowPos), E2M_v(initElbowPos));


	// robot-base's position filter
	robot_base_pos_filter= new  CDDynamics(dim, sample_time, wn_filter_position);

	robot_base_pos_filter->SetStateTarget(E2M_v(robot_base_position), E2M_v(robot_base_position));

	// object's position filter
	object_position_filter= new  CDDynamics(dim, sample_time, wn_filter_position);

	object_position_filter->SetStateTarget(E2M_v(object_position), E2M_v(object_position));

	// object's orientation filter
	object_orientation_filter= new  CDDynamics(4, sample_time, wn_filter_position);

	object_orientation_filter->SetStateTarget(E2M_v(object_orientation), E2M_v(object_orientation));






	initialPosition<< -0.47225f, 0.0f, 0.2847f;

	fileName="Logfile_HT";

	// _outputFile = std::ofstream(fileName + ".txt");

	_outputFile.open(("src/handtracker/" + fileName + ".txt").c_str());

	Eigen::Vector3d tmpHandRP;

	std::vector<double> tmpHandRP_vector(3,0);

	Eigen::Vector3d tmpElbowRP;

	int hD=0;

	initOK=true;

	ROS_INFO("Initialization complete\n");

    int count = 0;

   
    while (ros::ok())
    {
        
	
    	currentTime=ros::Time::now().toSec();

    	tmpHandRP=handRP(handPosition,shoulderPosition);
    	tmpElbowRP=elbowRP();

    	tmpHandRP_vector[0]=tmpHandRP(0);
    	tmpHandRP_vector[1]=tmpHandRP(1);
    	tmpHandRP_vector[2]=tmpHandRP(2);

    	handRelPos.push_back(tmpHandRP_vector);


    	_msgPosTester.linear.x = curr_hand_rev_position_filtered(0);
    	_msgPosTester.linear.y = curr_hand_rev_position_filtered(1);
    	_msgPosTester.linear.z = curr_hand_rev_position_filtered(2);
    	_msgPosTester.angular.x = _omegad[0];
		_msgPosTester.angular.y = _omegad[1];
		_msgPosTester.angular.z = _omegad[2];

    	_pubPosTester.publish(_msgPosTester);


    	 if(currentTime-checkTime>=lookTW){
    		hD=compHandDirection(handRelPos);

    		// remove the oldest relative position
    		handRelPos.clear();

    		// update checking time
    		checkTime=ros::Time::now().toSec();
    	}

    		

    	currHandVel=compHandRelVel();    		
    		


    	desiredVel=compDesiredVel();

    	computeDesiredOrientation();

    	speedMsg.sPer=speedPer;
    	speedMsg.dir=handDirection;

    	_msgEeSpeedTester.sPer=ee_speed;
    	_msgEeSpeedTester.dir=handDirection;


		_msgDesiredTwist.angular.x = _omegad[0];
		_msgDesiredTwist.angular.y = _omegad[1];
		_msgDesiredTwist.angular.z = _omegad[2];

		_msgDesiredTwist.linear.x  = desiredVel[0];
		_msgDesiredTwist.linear.y  = desiredVel[1];
		_msgDesiredTwist.linear.z  = desiredVel[2];

		_pubDesiredTwist.publish(_msgDesiredTwist);

		// Publish desired orientation
		_msgDesiredOrientation.w = _qd(0);
		_msgDesiredOrientation.x = _qd(1);
		_msgDesiredOrientation.y = _qd(2);
		_msgDesiredOrientation.z = _qd(3);

		_pubDesiredOrientation.publish(_msgDesiredOrientation);

		_pubSpeedPer.publish(speedMsg);
		_pubSpeedTester.publish(_msgEeSpeedTester);

		_msgVelTester.linear.x = currHandVel[0];
    	_msgVelTester.linear.y = currHandVel[1];
	    _msgVelTester.linear.z = currHandVel[2];
		_msgVelTester.angular.x = _omegad[0];
		_msgVelTester.angular.y = _omegad[1];
		_msgVelTester.angular.z = _omegad[2];

		_pubVelTester.publish(_msgVelTester);
	    	
   		

    	if(saveData){
    		if(saveData2File(_outputFile,currentTime,_eePosition,_q,ee_speed,desiredVel,handDirection,tmpHandRP,curr_hand_rev_position_filtered,cuur_hand_rev_velocity_filtered,hand_velocity_filtered,tmpElbowRP,elbow_revPosition_filtered,elbow_revPvelocity,handPosition,handOrientation,elbowPosition,elbowOrientation,shoulderPosition,shoulderOrientation)<1){
    			std::cout<<"The data are not stored properly\n";
    		}
    	}

    		




        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    	
    
    }

    _outputFile.close();

	ROS_INFO("The data are saved to %s \n",("src/handtracker/" + fileName + ".txt").c_str());


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
