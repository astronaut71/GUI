1. Dependencies:

     1) Requirements for Qt: qt_ros (http://wiki.ros.org/qt_ros).  
        Installation:  sudo apt-get install ros-fuerte-qt-ros

     2) Dependency with standard qwt 6.0.0 


2. Install ROS-Matlab Bridge from https://code.google.com/p/mplab-ros-pkg/wiki/java_matlab_bridge. Place classifier_turn.m matlab  and classifier_forward.m files (for 180º turn and 10m forward tasks respectively) into the ROS-Matlab Bridge installed folder. 

3. Install Matlab version 2012/2011 ( Versions supported for Ubuntu 10 and 12.04 LTS)
 .The Neural Network toolbox must be installed. 

4. Install libsvm libraryfrom https://www.csie.ntu.edu.tw/~cjlin/libsvm/ .The tested version was 3.21. Older version can be find  in  https://www.csie.ntu.edu.tw/~cjlin/libsvm/oldfiles/. 


Installation and linked with Matlab

- On Ubuntu machine, just to make sure you have gcc in your machine. If not, you need to install it using the command below:
sudo apt-get install build-essential g++

- Download LIBSVM  version 3.21 from  https://www.csie.ntu.edu.tw/~cjlin/libsvm/

- Extract it 
tar -xvf libsvm-3.21.tar.gz
- Change the current directory to libsvm directory 
cd libsvm-3.21
- Build svm-train, svm-predict and svm-scale 
make
-In matlab sub folder check :it is assume 
your MATLAB is installed in '/usr/local/matlab'. If not, please change in make file the 
MATLABDIR ?=  to the local directory where MatLab is installed.

-Change the current directory to matlab libsvm sub folder  
cd matlab
- Build
make
-In Matlab  Set Path of the libsvm-3.2.1 matlab subfolder

- In Matlab Command Window type: svmtrain

It should appears following
Usage: model = svmtrain(training_label_vector, training_instance_matrix, 'libsvm_options');
libsvm_options:  ….
Now is ready to run the GUI

4. Running the GUI

    1) Make sure running the roscore

    2) According to step 2 depending of which task is performed, in one tab of the terminal where the ROS-Matlab Bridge folder is type: matlab -nodesktop -nosplash -r "run  classifier_forward.m; quit;"
 or 

matlab -nodesktop -nosplash -r "run  classifier_turn.m; quit;"
    3)  In the other tab of the terminal where the pow_gui folder is type:  rosrun pow_gui pow_gui

    4)  After Matlab finished the classification in GUI press the Get Results tab to obtain the results of the performed task. The “File” tab displays the file was running. The “Task” tab displays the task was performed. The “Parameters” tab displays the  task specific parameters. The “Results” tab displays the performing task score between 1-4. The “Graph” tab displays the graphically change of specific parameters such as angular , linear velocity and travelled distance over the time.  The last “Camera” task is video display of the run. 

5) To change the running file and task need to change in pow_analyser launch folder the pow_analyze_hector_assess.launch file
