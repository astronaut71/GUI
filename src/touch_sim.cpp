/*
 * touch_sim.cpp
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

#include "pow_gui/touch_sim.hpp"

using namespace pow_gui;

TouchSimulator::TouchSimulator(int32_t speed, int32_t turn)
: m_inputChanged( false )
, m_stopped( false )
{

	m_nh.setCallbackQueue( &m_callBackQueue );
	m_pub = m_nh.advertise<std_msgs::String>( "some_topic", 1000 );


	this->start();
}

TouchSimulator::~TouchSimulator()
{
	stop();
	this->wait();
	m_pub.shutdown();
}

void TouchSimulator::stop()
{
	m_stopped = true;
}

void TouchSimulator::run()
{
	ros::Rate loop_rate( 10 );
	int count = 0;
	while( !m_stopped )
	{

		if (m_inputChanged)
		{
			QMutexLocker locker( &m_mutex );
			m_strMsg.data = "hello";
			m_inputChanged = false;
		}
		m_pub.publish( m_strMsg );
		m_callBackQueue.callOne( );

		std::cout << "Sim " << count++ << " msg = " << m_strMsg.data << std::endl;

		loop_rate.sleep( );
	}
	std::cout << "Stopped" << std::endl;
}

void TouchSimulator::stringCallback( const std_msgs::StringConstPtr& abc )
{
	std::cout << "Callback get called " << std::endl;
}


