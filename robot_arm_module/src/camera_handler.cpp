#include "camera_handler.h"
#include "ros/ros.h"
#include <find_object_2d/ObjectsStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <QTransform>
#include <QColor>
#include "std_msgs/Float32MultiArray.h"

#include <iostream>
#include <string>
#include <unordered_map>



camera_handler::camera_handler(ros::NodeHandle nh, ros::NodeHandle nh_priv) 
:_nh(nh),_nh_priv(nh_priv)
{
    object_sub = nh.subscribe("objects", 1000, & camera_handler::objectsDetectedCallback, this);
}

camera_handler::~camera_handler() 
{
}

void camera_handler::objectsDetectedCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	printf("---\n");
	const std::vector<float> & data = msg->data;
	if(data.size())
	{
		for(unsigned int i=0; i<data.size(); i+=12)
		{
			// get data
			int id = (int)data[i];
			float objectWidth = data[i+1];
			float objectHeight = data[i+2];

			// Find corners Qt
			QTransform qtHomography(data[i+3], data[i+4], data[i+5],
									data[i+6], data[i+7], data[i+8],
									data[i+9], data[i+10], data[i+11]);

			QPointF qtTopLeft = qtHomography.map(QPointF(0,0));
			QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0));
			QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
			QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));

			printf("Object %d detected, Qt corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
					id,
					qtTopLeft.x(), qtTopLeft.y(),
					qtTopRight.x(), qtTopRight.y(),
					qtBottomLeft.x(), qtBottomLeft.y(),
					qtBottomRight.x(), qtBottomRight.y());

            // save required data

		}
	}
	else
	{
		printf("No objects detected.\n");
	}
}