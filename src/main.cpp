/*
 * main.cpp
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
 *       0.0 -Bojan Andonovski
 */


#include <QtGui>
#include <QApplication>
//#include "pow_gui/samplingthread.hpp"
#include "pow_gui/main_window.hpp"

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_legend.h>
/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

	//Roslaunch
	QProcess process,process1;
	process.start("roslaunch", QStringList() << "../pow_analyzer/launch/pow_analyze_hector_assess.launch");
        ROS_INFO("Attempting to launch:");
	//process1.start("/bin/sh", QStringList() << "/home/bojan/svn/POW/ROS/Fuerte/utils/matlab_run.sh");
	//GUI
    QApplication app(argc, argv);
    pow_gui::MainWindow w(argc, argv);

	   w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    
    int result = app.exec();
   
    std::cout << "q application finished" << std::endl;

	return result;
}
