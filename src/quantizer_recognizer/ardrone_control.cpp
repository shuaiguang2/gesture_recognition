#include "ardrone_control.h"

#define LEFTRIGHT 	0
#define RIGHTLEFT	1
#define DOWNUP		2
#define UPDOWN		3
#define LEFTTOP		4
#define RIGHTTOP	5
#define CIRCLE		6



ArdroneControl::ArdroneControl(){}

void ArdroneControl::init(ros::NodeHandle n){

    //initialize the node	
    pub_empty_takeoff = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_empty_land = n.advertise<std_msgs::Empty>("/ardrone/land", 1) ;
    pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   
}

void ArdroneControl::sendCommand(int cmd){
    switch(cmd){
	case LEFTRIGHT:
		flyRight(); break;
     	case RIGHTLEFT:
		flyLeft(); break;
	case DOWNUP:
		takeOff(); break;
	case UPDOWN:
		land(); break;
 	case LEFTTOP:
		forward(); break;
	case RIGHTTOP:
		backward(); break;
	case CIRCLE:
 		circle(); break;
    }//switch
}

void ArdroneControl::takeOff(){
    double time =(double)ros::Time::now().toSec();	
    ROS_INFO("Take off command, and will last for 5 sec");

    while ((double)ros::Time::now().toSec()< time+5.0){
        pub_empty_takeoff.publish(m_empty_msg); //launches the drone
        setVelocity(0, 0, 0, 0, 0, 0);
	pub_twist.publish(m_twist_msg); //drone is flat
	//ROS_INFO("Taking off");
    }//takeoff before t+5

}

void ArdroneControl::land(){
    double time =(double)ros::Time::now().toSec();	
    ROS_INFO("Land command, and will last for 5 sec");

    while ((double)ros::Time::now().toSec()< time+5.0){
        setVelocity(0, 0, 0, 0, 0, 0);
	pub_twist.publish(m_twist_msg); //drone is flat
	pub_empty_land.publish(m_empty_msg); //lands the drone
	//ROS_INFO("Landing");
    }//land before t+5

    exit(0);
}

void ArdroneControl::flyLeft(){
    double time =(double)ros::Time::now().toSec();	
    ROS_INFO("Fly to left side command, and will last for 1 sec");

    while ((double)ros::Time::now().toSec()< time+1.0){
        setVelocity(0, 0.05, 0, 0, 0, 0);
	pub_twist.publish(m_twist_msg); 
	//ROS_INFO("Flying to left side");
    }//fly to left before t+1

}

void ArdroneControl::flyRight(){
    double time =(double)ros::Time::now().toSec();	
    ROS_INFO("Fly to right side command, and will last for 1 sec");

    while ((double)ros::Time::now().toSec()< time+1.0){
        setVelocity(0, -0.05, 0, 0, 0, 0);
	pub_twist.publish(m_twist_msg); 
	//ROS_INFO("Flying to right side");
    }//fly to right before t+1
}

void ArdroneControl::forward(){
    double time =(double)ros::Time::now().toSec();	
    ROS_INFO("Forward command, and will last for 1 sec");

    while ((double)ros::Time::now().toSec()< time+1.0){
        setVelocity(0.05, 0, 0, 0, 0, 0);
	pub_twist.publish(m_twist_msg); 
	//ROS_INFO("Flying forward");
    }//Fly forward before t+1
}

void ArdroneControl::backward(){
    double time =(double)ros::Time::now().toSec();	
    ROS_INFO("Backward command, and will last for 1 sec");

    while ((double)ros::Time::now().toSec()< time+1.0){
        setVelocity(-0.05, 0, 0, 0, 0, 0);
	pub_twist.publish(m_twist_msg); 
	//ROS_INFO("Flying backward");
    }//Fly backward before t+1
}

void ArdroneControl::circle(){
    double time =(double)ros::Time::now().toSec();	
    ROS_INFO("Circle command, and will last for 1 sec");

    while ((double)ros::Time::now().toSec()< time+1.0){
        setVelocity(0, 0, 0, 0, 0, 0.2);
	pub_twist.publish(m_twist_msg); 
	//ROS_INFO("Circle");
    }//Fly forward before t+1
}

void ArdroneControl::setVelocity(float vx, float vy, float vz, float ax, float ay, float az){
   m_twist_msg.linear.x = vx;
   m_twist_msg.linear.y = vy;
   m_twist_msg.linear.z = vz;
   m_twist_msg.angular.x = ax;
   m_twist_msg.angular.y = ay;
   m_twist_msg.angular.z = az;    
}
