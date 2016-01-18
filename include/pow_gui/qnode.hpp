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

#ifndef __CAS_QPANEL_QNODE_HPP_
#define __CAS_QPANEL_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QMutex>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pow_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc,  char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &m_logging_model; }
	void log( const LogLevel &level, const std::string &msg);
    int  requestMotorControl();
    inline int getresult() { return result; }
    QImage& image() { return m_image; }
    int get_result() { return result; }
    inline std::vector<double> get_parameters() { return parameters; }
    inline std::string get_task() { return task; }
    inline std::string get_file() { return file; }
    
        QPolygonF linv_g,yaw_g,trav_g,wall_g;
   

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void scanReceived();
    void taskReceived();
    void fileReceived();
    void scoreReceived();
    void imageReceived();
    void parameterReceived();
    void graphReceived();
	// void imageReceived(cv::Mat& mat);


private:
    bool initCommon();
    //void scanCallback( const sensor_msgs::LaserScanConstPtr& msg );
    void scoreCallback( const std_msgs::Int32ConstPtr& msg );
    void makeSystemCall();
    void visionCallback( const sensor_msgs::ImageConstPtr& msg );
    void parameterCallback( const std_msgs::Float64MultiArrayConstPtr& msg );
        void graphCallback( const std_msgs::Float64MultiArrayConstPtr& msg );
        void taskCallback( const std_msgs::StringConstPtr& msg );
        void fileCallback( const std_msgs::StringConstPtr& msg );
public:
    QMutex m_mutex;
    QMutex m_obstacleMutex;

private:
	int init_argc;
	char** init_argv;

	ros::Publisher m_chatter_publisher;
	ros::Subscriber m_msg_subscriber;
	//ros::Subscriber m_scan_subscriber;
	ros::Subscriber m_score_subscriber;
	ros::Subscriber m_parameter_subscriber;
	ros::Subscriber m_image_subscriber;
	ros::Subscriber m_task_subscriber;
	ros::Subscriber m_file_subscriber;
	ros::Subscriber gsub;
	ros::ServiceClient m_motor_client;

    QStringListModel m_logging_model;
    //int m_scanCallbackCount;
    int m_scoreCallbackCount;
    float m_parameterCallbackCount;
    float m_taskCallbackCount;
    float m_fileCallbackCount;
    float m_graphCallbackCount;
    int m_visionCallbackCount;
    float min_range;
    std::vector<double> parameters;
    std::string task;
    std::string file;
    int result;
    sensor_msgs::LaserScan laser_scan;
    QImage m_image;
    cv::Point2d m_obstacle;

};

}  // namespace qpanel

#endif /* __CAS_QPANEL_QNODE_HPP_ */
