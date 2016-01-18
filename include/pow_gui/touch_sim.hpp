/*
 * touch_sim.cpp
 * Description: QT based Gui panel for Hoist system
 *
 * Copyright (c) 2013, Centre for Intelligent Mechatronics Systems, University of Technology, Sydney, Australia.
 * All rights reserved.
 *
 * This software was developed as a part of an industry based research project on Assistive Robotic Devices.
 *
 * Author: Liyang Liu
 * Date: 25/06/2013
 *
 * Rev History:
 *       0.0 - Liyang Liu
 */


#ifndef __CAS_QPANEL_TOUCH_SIM_HPP_
#define __CAS_QPANEL_TOUCH_SIM_HPP_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <QThread>
#include <QMutex>

namespace pow_gui
{
class TouchSimulator : public QThread
{
    Q_OBJECT
public:
    TouchSimulator( int32_t speed, int32_t turn );
	virtual ~TouchSimulator( );
	// bool init();
	void stop();

protected:
	void run();

public:
    QMutex m_mutex;
    volatile bool 					m_inputChanged;

private:
    void stringCallback( const std_msgs::StringConstPtr& abc );

    volatile bool 					m_stopped;

    ros::NodeHandle 				m_nh;
    ros::Publisher 					m_pub;
    ros::CallbackQueue 				m_callBackQueue;
    std_msgs::String					m_strMsg;
};

}

#endif /* __CAS_QPANEL_TOUCH_SIM_HPP_ */
