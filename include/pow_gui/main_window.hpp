/*
 * main_window.cpp
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

#ifndef __CAS_QPANEL_MAIN_WINDOW_H
#define __CAS_QPANEL_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <boost/scoped_ptr.hpp>
#include <QtGui/QMainWindow>
#include <qwidget.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "touch_sim.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/
class Plot;
class Knob;
class WheelBox;

namespace pow_gui
{

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow( int argc, char** argv, QWidget *parent = 0 );
	~MainWindow( );

	void ReadSettings( ); // Load up qt program settings at startup
	void WriteSettings( ); // Save qt program settings when closing
	void start();
	void closeEvent( QCloseEvent *event ); // Overloaded function
	void showNoMasterMessage( );

	double amplitude() const;
	double frequency() const;
	double signalInterval() const;
	//double curve() const;

Q_SIGNALS:
	void amplitudeChanged( double );
	void frequencyChanged( double );
	void signalIntervalChanged( double );

//public slots:
public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName( ))
	*******************************************/
	void on_actionAbout_triggered( );
	void on_button_connect_clicked( bool check );
	void on_button_controlMotor_clicked( bool check );
	void on_checkbox_useEnvironment_stateChanged( int state );
	void on_doubleSpinBox_speed_valueChanged( double val );
	void on_doubleSpinBox_turn_valueChanged( double val );
	void on_button_stop_clicked( bool check );
	void onImageReceived( );
	void onScoreReceived( );
	void onParameterReceived( );
	void onGraphReceived( );
	void onTaskReceived( );
	void onFileReceived( );
	// void onImageReceived( cv::Mat& mat );

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView( ); // no idea why this can't connect automatically

//public Q_SLOTS:
   //void onObstacleReceived( );
   //void onImageReceived( );
  inline QPolygonF get_linv_g() { return qnode.linv_g; }
  inline QPolygonF get_yaw_g() { return qnode.yaw_g; }
  inline QPolygonF get_trav_g() { return qnode.trav_g; }
  inline QPolygonF get_wall_g() { return qnode.wall_g; }

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	std::stringstream ss1, ss4, ss3,ss2;
	boost::scoped_ptr<TouchSimulator> m_touchSim;

 //d_amplitudeKnob = new Knob( "Amplitude", 0.0, 200.0, osc );

	    Knob *d_frequencyKnob;
	    Knob *d_amplitudeKnob;
	    WheelBox *d_timerWheel;
	    WheelBox *d_intervalWheel;

					QTabWidget* tabWidget;
					//QFileInfo *file;


	       Plot *d_plot;
        QwtPlot* p_plot;
        QwtPlot* p_plot1;
        QwtPlot* p_plot2;
        QwtPlotCurve *curve;
        QwtPlotCurve *curve1;
        QwtPlotCurve *curve2;
};

}  // namespace hoist_qpanel

#endif // __CAS_QPANEL_MAIN_WINDOW_H
