#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include "quantizer.h"
#include "recognizer.h"

Quantizer quant;
Recognizer recog;
void Callback(const geometry_msgs::PolygonStamped::ConstPtr& msg){  
    quant.updateHandPosition(msg);
    recog.AddObservation(quant.quantize());
    recog.GestureRecognize();
}

int main(int argc, char** argv){
    ros::init(argc, argv, "quantizer_recognizer_node");
    ros::NodeHandle n;
    
    recog.Init(n);
    ros::Subscriber handPosition = n.subscribe("/tracking", 1, Callback);
    ros::spin();

    return 0;
}
