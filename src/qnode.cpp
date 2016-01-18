/*
 * qnode.cpp
 * Description: QT based Gui panel for POW system
 *
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of an industry based research project on Assistive Robotic Devices.
 *
 * Author: Bojan Andonovski
 * Date: 25/01/2013
 *
 * Rev History:
 *       0.0 - Bojan Andonovski
 */


#include "pow_gui/qnode.hpp"

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <spawn.h>

//#include <hoist_kinect/Obstacle.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <stdlib.h> 
#include <string>
#include <sstream>
#include <cstdlib>
#include <spawn.h>
#define AUTO_SPIN

#include <QDebug>

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace enc = sensor_msgs::image_encodings;
namespace pow_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode( int argc, char** argv )
 : init_argc( argc )
 , init_argv( argv )
 , m_visionCallbackCount( 0 )
 , m_scoreCallbackCount( 0 )
 , m_parameterCallbackCount( 0 )
 , m_graphCallbackCount( 0 )
 , m_taskCallbackCount( 0 )
 , m_fileCallbackCount( 0 )
{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown( ); // explicitly needed since we use ros::start();
      ros::waitForShutdown( );
    }
	wait( );
}

bool QNode::initCommon( )
{
	//ros::start( ); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh( "~" );
	// Add your ros communications here.
	/*std::string poseTopic = nh.resolveName( "/slam_out_pose" );
	ROS_INFO( "subscribing to %s", poseTopic.c_str( ));
	m_pose_subscriber = nh.subscribe(poseTopic, 10, &QNode::poseCallback, this);
	//m_pose_subscriber = nh.subscribe<hoist_kinect::ObstacleDisplay>(obstacleTopic, 1000, &QNode::obstacleCallback, this);

	std::string imuTopic = nh.resolveName("/raw_imu_throttle");
	ROS_INFO( "subscribing to %s", scanTopic.c_str( ));
	m_imu_subscriber = nh.subscribe(scanTopic, 10, &QNode::imuCallback, this);*/

// Load bag file
//std::string global_name, relative_name, default_param;
//if (nh.getParam("/global_name", global_name))
///

	std::string scanTopic = nh.resolveName("/scan_throttle");
	std::string scoreTopic = nh.resolveName("/score");
	std::string parameterTopic = nh.resolveName("/stats");
	std::string imageTopic = nh.resolveName("/image_color_throttle");
	std::string graphTopic = nh.resolveName("/to_graph");
	std::string taskTopic = nh.resolveName("/to_task");
	std::string fileTopic = nh.resolveName("/to_file");
	
	m_task_subscriber = nh.subscribe<std_msgs::String>(taskTopic, 10, &QNode::taskCallback, this );
	m_file_subscriber = nh.subscribe<std_msgs::String>(fileTopic, 10, &QNode::fileCallback, this );
 m_image_subscriber = nh.subscribe<sensor_msgs::Image>(imageTopic, 10, &QNode::visionCallback, this);
	ROS_INFO( "subscribing to %s", scoreTopic.c_str( ));
	//ROS_INFO( "subscribing to %s", taskTopic.c_str( ));
 m_score_subscriber = nh.subscribe<std_msgs::Int32>(scoreTopic, 10, &QNode::scoreCallback, this );
 m_parameter_subscriber = nh.subscribe<std_msgs::Float64MultiArray>(parameterTopic, 10, &QNode::parameterCallback, this );
 gsub = nh.subscribe<std_msgs::Float64MultiArray>(graphTopic, 10, &QNode::graphCallback, this );

 result = 0;
	start();
	return true;
}

bool QNode::init( )
{
	ros::init( init_argc, init_argv, "qdude" );
	//launch.param=task1;
	
	
	
	if ( ! ros::master::check( ) )
	{
		return false;
	}
	return initCommon( );
}

bool QNode::init( const std::string &master_url, const std::string &host_url )
{
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;

	ros::init( remappings, "qdude" );
	if ( ! ros::master::check( ) ) {
        std::cout << "Master url " << master_url << std::endl;
        std::cout << "Host url " << host_url << std::endl;
		return false;
	}
	return initCommon();
}

// Parameters output
void QNode::parameterCallback( const std_msgs::Float64MultiArray::ConstPtr& msg )
{
	std::cout << "parameterCallback " << ++m_parameterCallbackCount << std::endl;
	 parameters=msg->data;
	 Q_EMIT parameterReceived();
}

void QNode::taskCallback( const std_msgs::String::ConstPtr& msg )
{
	++m_taskCallbackCount;
	//std::cout << "parameterCallback " << ++m_parameterCallbackCount << std::endl;
	 task=msg->data;
	 Q_EMIT taskReceived();
}

void QNode::fileCallback( const std_msgs::String::ConstPtr& msg )
{
	++m_fileCallbackCount;
	 file=msg->data;
	 Q_EMIT fileReceived();
}

 void QNode::graphCallback( const std_msgs::Float64MultiArray::ConstPtr& msg )
{
//std::cout << "graphCallback: " << ++m_graphCallbackCount << std::endl;
++m_graphCallbackCount;
          
  linv_g<< QPointF( (float)msg->data[0], (float)msg->data[1] );
    yaw_g<< QPointF( (float)msg->data[0], abs (57.3*(float)msg->data[2]) );
      trav_g<< QPointF( (float)msg->data[0], (float)msg->data[3] );
        wall_g<< QPointF( (float)msg->data[0], (float)msg->data[4] );
          Q_EMIT graphReceived();
            
}

void QNode::scoreCallback( const std_msgs::Int32::ConstPtr& msg )
{
	++m_scoreCallbackCount;
	//std::cout << "scoreCallback " << m_scoreCallbackCount << std::endl;
	 result=msg->data;
	 Q_EMIT scoreReceived();
}
void QNode::visionCallback( const sensor_msgs::ImageConstPtr& msg )
{
	++m_visionCallbackCount;
		 //std::cout << "visionCallback " << m_visionCallbackCount << std::endl;
	    cv_bridge::CvImagePtr cv_ptr;
	     try
	     {
	             cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	             //cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);

	             cv::Mat mat = cv_ptr->image;
	             {
	         		    QMutexLocker locker(&m_mutex);
	                    QMutexLocker locker2(&m_obstacleMutex);
	                    cv::circle(mat, m_obstacle, 10, CV_RGB(255,0,0), 5);

						// 8-bits unsigned, NO. OF CHANNELS=1
						if(mat.type()==CV_8UC1)
						{
							// Set the color table (used to translate colour indexes to qRgb values)
							QVector<QRgb> colorTable;
							for (int i=0; i<256; i++)
								colorTable.push_back(qRgb(i,i,i));
							// Copy input Mat
							const uchar *qImageBuffer = (const uchar*)mat.data;
							// Create QImage with same dimensions as input Mat
							m_image = QImage(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8);
							m_image.setColorTable(colorTable);
						}
						// 8-bits unsigned, NO. OF CHANNELS=3
						if(mat.type()==CV_8UC3)
						{
							// Copy input Mat
							const uchar *qImageBuffer = (const uchar*)mat.data;
							// Create QImage with same dimensions as input Mat
							m_image = QImage(qImageBuffer, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).rgbSwapped();
						}
						else
						{
							//ROS_INFO("ERROR: Mat could not be converted to QImage.");
							m_image = QImage();
						}
	             }
	          	 Q_EMIT imageReceived(); // used to signal the gui for a shutdown (useful to roslaunch)
	             //Q_EMIT imageReceived();
	     }
	     catch (cv_bridge::Exception& e)
	     {
	             ROS_ERROR("cv_bridge exception: %s", e.what());
	             return;
	     }

}
/*void QNode::scanCallback( const sensor_msgs::LaserScan::ConstPtr& msg )
{
	++m_scanCallbackCount;
	 std::cout << "scanCallback " << m_scanCallbackCount << std::endl;

	 std::vector<float> laser;
	 laser = msg->ranges;

	 int size_laser = laser.size();
	    for (int i=0;i<size_laser;i++){
	        if (laser[i] < 0.01){
	            laser[i] = 99999;
	        }
	        if (laser[i] > 45){
	            laser[i] = 99999;
	        }
	    }

	    min_range = 2;
	    int index_min;
	    for (int i=0;i<size_laser;i++){
	        if (laser[i] < min_range){
	            min_range = laser[i];
	            index_min = i;
		   //ROS_INFO("Minimum Range = %f", min_range);
	        }
	    }

	    for (int j=0;j<size_laser;j++){
	        if (laser[j] > min_range + 0.5){
	            laser[j] = 0;
	        }
	    }

	    laser_scan = *msg;
	    laser_scan.ranges.clear();
	    laser_scan.ranges = laser;



	 /////////////////////////////////////////


		 //std::vector<float> laser; 	// ROS scancallback
		 laser = msg->ranges; 		    // ROS scancallback

		 Q_EMIT scanReceived(); // used to signal the gui for a shutdown (useful to roslaunch)
		 //Q_EMIT imageReceived();

 }
*/
void QNode::makeSystemCall( )
{
   //pid_t pid;

	//char *argv[] = { (char*)"/bin/sh", (char*)"matlab_bridge", (char*)"matlab_run.sh", (char *)0 };
	//char *argv1[] = { (char*)"roslaunch", (char*)"pow_analyzer", (char*)"pow_analyze_hector_assess.launch", (char *)0 };
	//char *argv2[] = { (char*)"", (char*)"matlab_bridge", (char*)"test_fit_data.m", (char *)0 };
	//char *argv3[] = { (char*)"", (char*)"matlab_bridge", (char*)"predict_score2.m", (char *)0 };
	//int status1;
	///test
	//process.start("/bin/sh", QStringList() << "/home/bojan/svn/POW/ROS/Fuerte/utils/matlab_run.sh");
	//status1 = posix_spawn( &pid, "/home/bojan/svn/POW/ROS/Fuerte/utils/matlab_run.sh", NULL, NULL, argv, environ );
	//status2 = posix_spawn( &pid, "/home/bojan/matlab_bridge/test_fit_data.m", NULL, NULL, argv2, environ );
	//status3 = posix_spawn( &pid, "/home/bojan/matlab_bridge/predict_score2.m", NULL, NULL, argv3, environ );

}


int QNode::requestMotorControl()
{
	return 0;
}

void QNode::run( )
{

	ros::Rate loop_rate( 50 );
	int count = 0;
	//while ( ros::ok( ) && result==0 )
	while ( ros::ok( ) )
	{
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
//#endif

	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg)
{
	m_logging_model.insertRows( m_logging_model.rowCount(), 1 );
	std::stringstream logging_model_msg;
	switch ( level )
	{
		case(Debug) :
		{
				ROS_DEBUG_STREAM( msg );
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) :
		{
				ROS_INFO_STREAM( msg );
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) :
		{
				ROS_WARN_STREAM( msg );
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) :
		{
				ROS_ERROR_STREAM( msg );
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) :
		{
				ROS_FATAL_STREAM( msg );
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row( QString( logging_model_msg.str( ).c_str( )));
	m_logging_model.setData( m_logging_model.index( m_logging_model.rowCount() - 1 ), new_row);
	Q_EMIT loggingUpdated( ); // used to readjust the scrollbar

}

}  // namespace qpanel
