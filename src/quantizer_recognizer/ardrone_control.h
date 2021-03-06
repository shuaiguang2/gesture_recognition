/*Get the gesture number and send 
  the message to ardrone to do the 
  corresponding movements.
*/

#ifndef _ARDRONE_CONTROL_H
#define _ARDRONE_CONTROL_H

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

class ArdroneControl{
public:
    ArdroneControl();
    void init(ros::NodeHandle n);
    void sendCommand(int cmd);

private:
    geometry_msgs::Twist m_twist_msg;
    std_msgs::Empty m_empty_msg;
    ros::Publisher pub_empty_takeoff, pub_empty_land, pub_twist;

    void takeOff();
    void land();
    void flyLeft();
    void flyRight();
    void forward();
    void backward();
    void circle();
    void setVelocity(float vx, float vy, float vz, float ax, float ay, float az);

};

#endif
