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



#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "pow_gui/main_window.hpp"

#include <unistd.h>
#include <spawn.h>
#include <sys/wait.h>

#include "pow_gui/tabdialog.h"
//#include "pow_gui/plot.hpp"
//#include "pow_gui/knob.hpp"
//#include "pow_gui/wheelbox.hpp"
#include <qwt_scale_engine.h>
#include <qlabel.h>
#include <qlayout.h>


//#include "pow_gui/samplingthread.hpp"
//#include "pow_gui/main_window.hpp"

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_legend.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace Qt;

Q_DECLARE_METATYPE(cv::Mat)

namespace pow_gui
{

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/


MainWindow::MainWindow( int argc, char** argv, QWidget *parent )
 : QMainWindow( parent )
 , qnode( argc,argv )
{
	ui.setupUi( this ); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    QObject::connect( ui.actionAbout_Qt, SIGNAL( triggered( bool )), qApp, SLOT( aboutQt( ))); // qApp is a global variable for the application

    ReadSettings( );
	setWindowIcon( QIcon( ":/images/icon.png" ));
	ui.tab_manager->setCurrentIndex( 0 ); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect( &qnode, SIGNAL( rosShutdown( )), this, SLOT( close( )));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel( qnode.loggingModel( ));
    QObject::connect( &qnode, SIGNAL( loggingUpdated( )), this, SLOT( updateLoggingView( )));

    /*********************
     ** Image display
     */
    QObject::connect( &qnode, SIGNAL( imageReceived( )), this, SLOT( onImageReceived( )));
    QObject::connect( &qnode, SIGNAL( graphReceived( )), this, SLOT( onGraphReceived( )));
    QObject::connect( &qnode, SIGNAL( taskReceived( )), this, SLOT( onTaskReceived( )));
    QObject::connect( &qnode, SIGNAL( fileReceived( )), this, SLOT( onFileReceived( )));
    QObject::connect( &qnode, SIGNAL( scoreReceived( )), this, SLOT( onScoreReceived( )));
    QObject::connect( &qnode, SIGNAL( parameterReceived( )), this, SLOT( onParameterReceived( )));
    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked( ))
    {
        on_button_connect_clicked( true );
    }
    ui.camera->setAttribute( Qt::WA_NoMousePropagation );
    ui.camera->setAttribute( Qt::WA_OpaquePaintEvent );
    ui.results->setAttribute( Qt::WA_NoMousePropagation );
    ui.results->setAttribute( Qt::WA_OpaquePaintEvent );
    ui.task_running->setAttribute( Qt::WA_NoMousePropagation );
    ui.task_running->setAttribute( Qt::WA_OpaquePaintEvent );
    ui.file_playing->setAttribute( Qt::WA_NoMousePropagation );
    ui.file_playing->setAttribute( Qt::WA_OpaquePaintEvent );
    ui.parameters->setAttribute( Qt::WA_NoMousePropagation );
    ui.parameters->setAttribute( Qt::WA_OpaquePaintEvent );
    ui.plotgraph->setAttribute( Qt::WA_NoMousePropagation );
    ui.plotgraph->setAttribute( Qt::WA_OpaquePaintEvent );
    //ui.camera->setAutoFillBackground(true);
    //ui.camera->setAttribute(Qt::WA_PaintOnScreen);
    
    
    
      /******************************************
					** PLOT OSCILLOSCOPE
			*******************************************/
    /*
    				QWidget * osc = ui.oscilloscope;
        const double intervalLength = 10.0; // seconds
        d_plot = new Plot( osc );
        d_plot->setIntervalLength( intervalLength );

        d_amplitudeKnob = new Knob( "Amplitude", 0.0, 200.0, osc );
        d_amplitudeKnob->setValue( 160.0 );

        d_frequencyKnob = new Knob( "Frequency [Hz]", 0.1, 20.0, osc );
        d_frequencyKnob->setValue( /to_graph17.8 );

        d_intervalWheel = new WheelBox( "Displayed [s]", 1.0, 100.0, 1.0, osc );
        d_intervalWheel->setValue( intervalLength );

        d_timerWheel = new WheelBox( "Sample Interval [ms]", 0.0, 20.0, 0.1, osc );
        d_timerWheel->setValue( 10.0 );
        
        QVBoxLayout* vLayout1 = new QVBoxLayout();
        vLayout1->addWidget( d_intervalWheel );
        vLayout1->addWidget( d_timerWheel );
        vLayout1->addStretch( 10 );
        vLayout1->addWidget( d_amplitudeKnob );
        vLayout1->addWidget( d_frequencyKnob );

        QHBoxLayout *layout = new QHBoxLayout( osc );
        layout->addWidget( d_plot, 10 );
        layout->addLayout( vLayout1 );

        connect( d_amplitudeKnob, SIGNAL( valueChanged( double ) ),
            SIGNAL( amplitudeChanged( double ) QwtLegend *) );
        connect( d_frequencyKnob, SIGNAL( valueChanged( double ) ),
            SIGNAL( frequencyChanged( double ) ) );
        connect( d_timerWheel, SIGNAL( valueChanged( double ) ),
            SIGNAL( signalIntervalChanged( double ) ) );

        connect( d_intervalWheel, SIGNAL( valueChanged( double ) ),
            d_plot, SLOT( setIntervalLength( double ) ) );
											*/
											//Tab Dialog
											//const QString &fileName
											//QFileInfo fileInfo(file);
											//tabWidget = new QTabWidget;
											//tabWidget->addTab(new GeneralTab(file), tr("General"));
     						
     						//QLabel *fileNameLabel = new QLabel(tr("File Name:"));
     						//QLineEdit *fileNameEdit = new QLineEdit(fileInfo.fileName());

											//QVBoxLayout* tabLayout = new QVBoxLayout(ui.tab);
    							//tabLayout->addWidget(fileNameLabel);
     						//tabLayout->addWidget(fileNameEdit);
												////Layout
						  

												//Set Plot Lin Vel 
												//QWidget * vel = ui.plotgraph;
        				p_plot = new QwtPlot(ui.plotgraph);
        			 //	p_plot = new QwtPlot(vel);
            p_plot->setTitle( "Lin. Vel." );
            p_plot->setCanvasBackground( Qt::white );
            // Axis
            p_plot->setAxisTitle( QwtPlot::xBottom, "Time(sec)" );
            p_plot->setAxisTitle( QwtPlot::yLeft, "Linear Velocity (m/sec)" );
            p_plot->setAxisScale( QwtPlot::yLeft, 0.0, 1.0);
            p_plot->setAxisScale( QwtPlot::xBottom, 0.0, 10);
            //p_plot->insertLegend( new QwtLegend );
 
           // p_plot->insertLegend(* new QwtLegend(QwtPlot::LegendPosition));



            //samplingThread.start();
            QwtPlotGrid *grid = new QwtPlotGrid();
            grid->attach( p_plot );

            curve = new QwtPlotCurve();
            curve->setTitle( "Linear velocity" );
            // Set curve styles
             // void QwtPlotCurve::setPen(const QPen & pen)
            // void QwtPlotCurve::setPen(const QPen&) 

           curve->setPen(* new QPen(Qt::blue)),   

           //curve->setPen(Qt::blue,4);
            curve->setRenderHint( QwtPlotItem::RenderAntialiased, true );

            QwtSymbol *symbol = new QwtSymbol( QwtSymbol::Ellipse,
            QBrush( Qt::yellow), QPen( Qt::red, 2 ), QSize( 10, 10 ) );
            curve->setSymbol(symbol);
										
            // Assign values to the curve
  										//QPolygonF linv_g();
  										//std::cout << "linv_g: " <<std::endl;
  										/*
  										qDebug() << qnode.linv_g;										  
            curve->setSamples(qnode.linv_g);//yaw_g,trav_g,wall_g;
            curve->attach( p_plot );
            p_plot->resize( 600, 400 );
            p_plot->replot();
            p_plot->show();*/
            
            //QWidget * vel1 = ui.plotgraph;
            p_plot1 = new QwtPlot(ui.plotgraph);
            p_plot1->setTitle( "Ang. Vel." );
            p_plot1->setCanvasBackground( Qt::white );
            // Axis
            p_plot1->setAxisTitle( QwtPlot::xBottom, "Time(sec)" );
            p_plot1->setAxisTitle( QwtPlot::yLeft, "Angular Velocity (deg/sec)" );
            p_plot1->setAxisScale( QwtPlot::yLeft, 0.0, 75.0);
            p_plot1->setAxisScale( QwtPlot::xBottom, 0.0, 10);
            //p_plot1->insertLegend( new QwtLegend() );

					
            //samplingThread.start();
            QwtPlotGrid *grid1 = new QwtPlotGrid();
            grid1->attach( p_plot1 );

            curve1 = new QwtPlotCurve("2");
            curve1->setTitle( "Angular velocity" );
            // Set curve styles
            curve1->setPen(* new QPen(Qt::blue)),
            //curve1->setPen( Qt::blue,4),
            curve1->setRenderHint( QwtPlotItem::RenderAntialiased, true );

            QwtSymbol *symbol1 = new QwtSymbol( QwtSymbol::Ellipse,
            QBrush( Qt::yellow), QPen( Qt::red, 2 ), QSize( 8, 8 ) );
            curve1->setSymbol( symbol1);
            
            // Min Wall Distance
            p_plot2 = new QwtPlot(ui.plotgraph);
            p_plot2->setTitle( "Travelled Distance" );
            p_plot2->setCanvasBackground( Qt::white );
            // Axis
            p_plot2->setAxisTitle( QwtPlot::xBottom, "Time(sec)" );
            p_plot2->setAxisTitle( QwtPlot::yLeft, "Distance (m)" );
            p_plot2->setAxisScale( QwtPlot::yLeft, 0.0, 10.0);
            p_plot2->setAxisScale( QwtPlot::xBottom, 0.0, 10);
            //p_plot2->insertLegend( new QwtLegend() );

            //samplingThread.start();
            QwtPlotGrid *grid2 = new QwtPlotGrid();
            grid2->attach( p_plot2 );

            curve2 = new QwtPlotCurve("3");
            curve2->setTitle( "Distance" );
            // Set curve styles
            //curve2->setPen( QPen),
            curve2->setPen(* new QPen(Qt::blue)),
	    //curve2->setPen( Qt::blue,4),
            curve2->setRenderHint( QwtPlotItem::RenderAntialiased, true );

            const QwtSymbol *symbol2 = new QwtSymbol( QwtSymbol::Ellipse, 
		QBrush( Qt::yellow), QPen( Qt::red, 2 ), QSize( 8, 8 ) );
            
            curve2->setSymbol( symbol2);            
            
            ////Layout
						      QVBoxLayout* vLayout_vel = new QVBoxLayout(ui.plotgraph);
						      vLayout_vel->addWidget( p_plot );
						      vLayout_vel->addStretch( 0.1 );
						      vLayout_vel->addWidget( p_plot1);
						      vLayout_vel->addStretch( 0.1 );
						      vLayout_vel->addWidget( p_plot2);
						      //vLayout_vel->addStretch( 0.1 );
				   				
								    
}

MainWindow::~MainWindow() {}

/***************************************************************************** ** Implementation [Slots] *****************************************************************************/


void MainWindow::showNoMasterMessage( )
{
	QMessageBox msgBox;
	msgBox.setText( "Couldn't find the ros master." );
	msgBox.exec( );
    close( );
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked( bool check )
{
	if ( ui.checkbox_useEnvironment->isChecked( ))
	{
	if ( !qnode.init( ))
		{
		showNoMasterMessage( );
		} else {
		ui.button_connect->setEnabled( false );
		}
	}
else
	{
		if ( ! qnode.init( ui.line_edit_master->text( ).toStdString( ),
				   ui.line_edit_host->text( ).toStdString( )))
		{
			showNoMasterMessage( );
	}
		else
	{
			ui.button_connect->setEnabled( false );
	 	        ui.line_edit_master->setReadOnly( true );
			ui.line_edit_host->setReadOnly( true );
			ui.line_edit_topic->setReadOnly( true );
		}
	}
	std::cout << "Connect clicked, go to start d_plot " << std::endl;
	//d_plot->start();
}
/*
double MainWindow::frequency() const
{
    return d_frequencyKnob->value();
}

double MainWindow::amplitude() const
{
    return d_amplitudeKnob->value();
}

double MainWindow::signalInterval() const
{
    return d_timerWheel->value();
}
*/
void MainWindow::on_button_controlMotor_clicked( bool check )
{
if ( check )
	{
		std::cout << "Motor started" << std::endl;
		m_touchSim.reset( new TouchSimulator(
				ui.doubleSpinBox_speed->value( ), ui.doubleSpinBox_turn->value( )));
	}
	else
	{
		std::cout << "Motor stopping..." << std::endl;
		m_touchSim.reset( );
	}
	
     if ( check)
     {
    	 std::cout << "Classification Started" << std::endl;
    
    	 ss1.str(std::string()); ss2.str(std::string()); ss3.str(std::string());std::vector<double> p_ = qnode.get_parameters();
    	 ss4.str(std::string());
    	 ss1 <<    "Your result is: "   << qnode.get_result();
    	 ss3 <<    "You are running task: " << qnode.get_task();
    	 ss4 <<    "You are running file: " << qnode.get_file();
    	// std::string color[1]={"red"};
    	 ss2 << "Task specific parameters are: " << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << "1) Stand. Dev.of Angular Vel. ="  <<p_[0] << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << "2) Stand. Dev.of Linear Vel. = "  <<p_[1] << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << "3) Av. Linear Vel. = "   <<p_[2] << "[m/sec]"<<std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << "4) Time = "   <<p_[3] <<"[sec]"<< std::endl;
    	 //ss2 << "4) Total Dist. = "   <<p_[3] <<"[m]"<<std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << "5) Min. Wall(Obstacle) Dist. = "   <<p_[4] <<"[m]"<<std::endl;
    	 //ss2 << "5) Turn Dist. = "   <<p_[4] <<"[m]"<<std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 //ss2 << "6) Time = "   <<p_[5] <<"[sec]"<< std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 //ss2 << "7) Idle Time = "   <<p_[6] <<"[sec]"<< std::endl;
    	 ss2 << " " << std::endl;
    	 ss2 << " " << std::endl;
    	 //ss2 << "8) Min. Wall(Obstacle) Dist. = "   <<p_[7] <<"[m]"<<std::endl;
    	 
    	 
    	 ui.results->setText(QString::fromStdString(ss1.str()));
    	 ui.task_running->setText(QString::fromStdString(ss3.str()));
    	 ui.file_playing->setText(QString::fromStdString(ss4.str()));
    	 ui.parameters->setText(QString::fromStdString(ss2.str()));
    	 //ui.plotgraph->plot.show();
    	 
     }
}

void MainWindow::on_doubleSpinBox_speed_valueChanged( double val )
{
	 if(ui.doubleSpinBox_speed->hasFocus( ))
	 {
		 std::cout << "SpinBox Has focus" << std::endl;
		 std::cout << val << std::endl;
		 if (m_touchSim.get( ))
		 {
			 /*
			 std_msgs::String& str_data = m_touchSim->m_inputMotorData;
			 {
				 QMutexLocker locker( &m_touchSim->m_mutex );
				 motor_data.speed = val;
			 }
			 */
			 m_touchSim->m_inputChanged = true;
		 }
	 }
}

void MainWindow::on_doubleSpinBox_turn_valueChanged( double val )
{
	 if( ui.doubleSpinBox_turn->hasFocus( ))
	 {
		 std::cout << "SpinBox Has focus" << std::endl;
		 std::cout << val << std::endl;
		 if (m_touchSim.get( ))
		 {
			 /*
			 hoist_core::MotorData& motor_data = m_touchSim->m_inputMotorData;
			 {
				 QMutexLocker locker( &m_touchSim->m_mutex );
				 motor_data.turn = val;
			 }
			 */
			 m_touchSim->m_inputChanged = true;
		 }

	 }
}

void MainWindow::on_checkbox_useEnvironment_stateChanged( int state )
{
	bool enabled;
	if ( state == 0 )
	{
		enabled = true;
	}
	else
	{
		enabled = false;
	}
	ui.line_edit_master->setEnabled( enabled );
	ui.line_edit_host->setEnabled( enabled );
	//ui.line_edit_topic->setEnabled( enabled );
}

void MainWindow::onImageReceived( )
// void MainWindow::onImageReceived(cv::Mat& mat)
{

	//std::cout << "Main window: on image received" << std::endl;
	{
		QMutexLocker locker( &qnode.m_mutex );
		ui.camera->setPixmap( QPixmap::fromImage( qnode.image( )));
		update();
		//ui.plotgraph->setPixmap( QPixmap::fromImage( qnode.image( )));
	}
}


void MainWindow::onScoreReceived( )
{
	// std::cout << "Main window: on image received" << std::endl;
	{
		QMutexLocker locker( &qnode.m_mutex );
		//ui.camera->setPixmap( QPixmap::fromImage( qnode.image( )));
		std::cout << qnode.getresult() << std::endl;
		//ui.camera->setText("This is the score of your run");
		//ui.camera->data;
		update();
	}
}

void MainWindow::onGraphReceived( )
{
 {
		QMutexLocker locker( &qnode.m_mutex );
		update();
	   
	}
}

void MainWindow::onTaskReceived( )
{
 {
		QMutexLocker locker( &qnode.m_mutex );
		std::cout << qnode.get_task() << std::endl;
                update();
				
	}
}

void MainWindow::onFileReceived( )
{
 {
		QMutexLocker locker( &qnode.m_mutex );
		std::cout << qnode.get_file() << std::endl;
				
	}
}


void MainWindow::onParameterReceived( )
{
	// std::cout << "Main window: on image received" << std::endl;
	{
		QMutexLocker locker( &qnode.m_mutex );
		//ui.camera->setPixmap( QPixmap::fromImage( qnode.image( )));
		std::vector<double> p_ = qnode.get_parameters();
		std::cout << p_[0]<<" "<<p_[1]<<" "<<p_[2]<<" "<<p_[3]<<" "<<p_[4] <<" "<<p_[5]<<" "<<p_[6]<<" "<<p_[7]<< std::endl;
		//ui.camera->setText("This is the score of your run");
		//ui.camera->data;
		
		//std::cout << "linv_g: " <<std::endl;
  										//Lin Vel
            qDebug() << qnode.linv_g;								  
            curve->setSamples(qnode.linv_g);//yaw_g,trav_g,wall_g;
            curve->attach( p_plot );
            p_plot->resize( 600, 400 );
            p_plot->replot();
            p_plot->show();
            
            //Ang Vel
            qDebug() << qnode.yaw_g;								  
            curve1->setSamples(qnode.yaw_g);//yaw_g,trav_g,wall_g;
            curve1->attach( p_plot1 );
            p_plot1->resize( 600, 400 );
            p_plot1->replot();
            p_plot1->show();
            
            
            //Min Obst Distance
            qDebug() << qnode.trav_g;								  
            curve2->setSamples(qnode.trav_g);//yaw_g,trav_g,wall_g;
            curve2->attach( p_plot2 );
            p_plot2->resize( 600, 400 );
            p_plot2->replot();
            p_plot2->show();
           
		
	}
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView( )
{
        ui.view_logging->scrollToBottom( );
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered( )
{
    QMessageBox::about( this, tr("About ..."),
    		tr( "<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>" ));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings( )
{
    QSettings settings( "Qt-Ros Package", "qdude" );
    restoreGeometry( settings.value( "geometry" ).toByteArray( ));
    restoreState( settings.value( "windowState" ).toByteArray( ));

    QString master_url = settings.value( "master_url", QString( "http://localhost:11311/" )).toString( );
    QString host_url = settings.value( "host_url", QString( "http://localhost" )).toString( );
    //QString topic_name = settings.value( "topic_name", QString( "/chatter" )).toString( );
    ui.line_edit_master->setText( master_url );
    ui.line_edit_host->setText( host_url );
    //ui.line_edit_topic->setText( topic_name );

    bool remember = settings.value( "remember_settings", false ).toBool( );
    ui.checkbox_remember_settings->setChecked( remember );
    bool checked = settings.value( "use_environment_variables", false ).toBool( );
    ui.checkbox_useEnvironment->setChecked( checked );

    if ( checked )
    {
    	ui.line_edit_master->setEnabled( false );
    	ui.line_edit_host->setEnabled( false );
    	//ui.line_edit_topic->setEnabled( false );
    }
}

void MainWindow::WriteSettings( )
{
    QSettings settings( "Qt-Ros Package", "qdude" );
    settings.setValue( "master_url", ui.line_edit_master->text( ));
    settings.setValue( "host_url", ui.line_edit_host->text( ));
    //settings.setValue( "topic_name", ui.line_edit_topic->text( ));
    settings.setValue( "use_environment_variables", QVariant( ui.checkbox_useEnvironment->isChecked( )));
    settings.setValue( "geometry", saveGeometry( ));
    settings.setValue( "windowState", saveState( ));
    settings.setValue( "remember_settings", QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent( QCloseEvent *event )
{
	WriteSettings( );
	QMainWindow::closeEvent( event );
}

void MainWindow::on_button_stop_clicked( bool check )
{
	std::cout << "Stop All being pressed" << std::endl;

	pid_t pid;
	char *argv[] = { (char*)"rosnode", (char*)"kill", (char*)"-a", (char *)0 };
	int status;
	// status = posix_spawn( &pid, "rostopic list", NULL, NULL, argv, environ );
	status = posix_spawn( &pid, "/opt/ros/fuerte/bin/rosnode", NULL, NULL, argv, environ );

	if (status == 0)
	{
		std::cout << "Shutting down ..." << std::endl;
		/*
		  if ( waitpid( pid, &status, 0 ) != -1 )
		  {
		      printf( "Child exited with status %i\n", status );
		  }
		  else
		  {
		      perror( "waitpid" );
		  }
		*/
	}
	else
	{
		std::cerr << "posix_spawn: " <<  strerror( status ) << std::endl;
	}
}

}  // namespace qpanel

