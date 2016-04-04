#include "quantizer.h"

Quantizer::Quantizer(){
    //initialize the member variables
    lastHandPosition.x = -1;
    lastHandPosition.y = -1;
    currentHandPosition.x = -1;
    currentHandPosition.y = -1;
}

void Quantizer::updateHandPosition(const geometry_msgs::PolygonStamped::ConstPtr& msg){
    lastHandPosition = currentHandPosition;
    currentHandPosition.x = (msg->polygon.points[0].x + msg->polygon.points[1].x) /2;
    currentHandPosition.y = (msg->polygon.points[0].y + msg->polygon.points[1].y) /2;
    dx = currentHandPosition.x - lastHandPosition.x;
    dy = -(currentHandPosition.y - lastHandPosition.y);

}

int Quantizer::quantize(){
    if( -1 == lastHandPosition.x || -1 == currentHandPosition.x || 0 == dx && 0 == dy)
	return -1;
  
    else if (0 == dx && dy > 0){
	//ROS_INFO("5");
    	return 5;
    }
    else if(0 == dx && dy < 0){
	//ROS_INFO("13");
	return 13;
	}
    else if(0 == dy && dx > 0){
	//ROS_INFO("1");
	return 1;
	}
    else if(0 == dy && dx < 0){
  	//ROS_INFO("9");
	return 9;
	}
    
    else{
        double m_angle = atan2(dy, dx) * 180/M_PI;
	if(m_angle >= 0 && m_angle < 22.5){
	    //ROS_INFO("1");
	    return 1;	
     	}
	else if(m_angle >= 22.5 && m_angle < 45){
	    //ROS_INFO("2");
	    return 2;
	}
	else if(m_angle >= 45 && m_angle < 67.5){
	    //ROS_INFO("3");
	    return 3;
	}
   	else if(m_angle >= 67.5 && m_angle < 90){
	    //ROS_INFO("4");
	    return 4;
	}
	else if(m_angle >= 90 && m_angle < 112.5){
	    //ROS_INFO("5");
	    return 5;
	}
	else if(m_angle >= 112.5 && m_angle < 135){
	    //ROS_INFO("6");
	    return 6;
	}
	else if(m_angle >= 135 && m_angle < 157.5){
	    //ROS_INFO("7");
	    return 7;
	}
	else if(m_angle >= 157.5 && m_angle < 180){
	    //ROS_INFO("8");
	    return 8;
	}
	else if(m_angle >= -180 && m_angle < -157.5){
	    //ROS_INFO("9");
	    return 9;
	}
   	else if(m_angle >= -157.5 && m_angle < -135){
	    //ROS_INFO("10");
	    return 10;
	}
	else if(m_angle >= -135 && m_angle < -112.5){
	    //ROS_INFO("11");
	    return 11;
	}
	else if(m_angle >= -112.5 && m_angle < -90){
	    //ROS_INFO("12");
	    return 12;
	}
	else if(m_angle >= -90 && m_angle < -67.5){
	    //ROS_INFO("13");
	    return 13;
	}
	else if(m_angle >= -67.5 && m_angle < -45){
	    //ROS_INFO("14");
	    return 14;
	}
	else if(m_angle >= -45 && m_angle < -22.5){
	    //ROS_INFO("15");
	    return 15;
	}
	else if(m_angle >= -22.5 && m_angle < 0){
	    //ROS_INFO("16");
	    return 16;
 	}

    } //elif

}







