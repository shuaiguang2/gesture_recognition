#ifndef _QUANTIZER_H
#define _QUANTIZER_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <math.h>
#include <ros/ros.h>

class Quantizer{
public:
    geometry_msgs::Point currentHandPosition, lastHandPosition;
    Quantizer();
    void updateHandPosition(const geometry_msgs::PolygonStamped::ConstPtr& msg);
    int quantize();

private:
    double dx, dy;
   

};


#endif
