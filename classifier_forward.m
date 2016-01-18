%matlab callback made with
%http://undocumentedmatlab.com/blog/matlab-callbacks-for-java-events/

clear all; clc;

% Setup classpath 
jmb_init();

MASTER_URI='http://localhost:11311';
NODE_NAME='classifier';
node=jmb_init_node(NODE_NAME, MASTER_URI);

% Create a Subscriber
sub=edu.ucsd.SubscriberAdapter(node,'/stats','std_msgs/Float64MultiArray');
pub = node.newPublisher('/score','std_msgs/Int32'); 

timeout=5; run_ok = 1; %ctr = 0; ctr_limit = 10/timeout;

%logger=node.getLog()
while run_ok == 1

	%Blocking call, return upon timeout or receiving a msg
	msg=sub.takeMessage(timeout);

	if isempty(msg)
		%logger.warn('timeout');
        %ctr = ctr + 1;
        %if ctr > ctr_limit
            %run_ok = 0;
        %end
    else
        %ctr = 0;
        run_ok = 0;
        if exist('datapredict.mat') > 0
            data = [ cell2mat(struct2cell(load('datapredict.mat'))); msg.data' ];
        else
            data = msg.data';
        end
		% save can also be called using "function" format
        save('datapredict.mat', 'data');    
	end
end

%load('/home/bojan/Desktop/SVM/POWSVM/Data/guitest.mat');
load ('forw100runs.mat');
%load ('/home/bojan/Desktop/SVM/POWSVM/Data/turn180DataLabel.mat');
load datapredict.mat;
%libsvm_options = '-s 4 -b 0 -t 2 -d 3 -r 0 -c 1 -n 0.2 -p 0.1 -m 100 -e 0.0000001 -h 1 -b 0 -wi 1 -q' 
%libsvm_options = '-s 1 -t 2 -d 3 -r 0 -c 1 -n 0.1 -p 0.1 -m 100 -e 0.00001 -h 1 -b 0 -wi 1 -q';
 libsvm_options = '-s 1 -t 2 -d 5 -r 0 -c 1 -n 0.009 -p 0.1 -m 100 -e 0.0000001 -h 1 -b 0 -wi 1 -q';
%feature_training= forw10mAll4Class1(:,1:end-1); class_training =forw10mAll4Class1(:,end);
feature_training= forw100runs(:,1:end-1); class_training =forw100runs(:,end);
%feature_training= turn180DataLabel(:,1:end-1); class_training =turn180DataLabel(:,end);

%guitest
class_testing = [1];


% SVM Training
       disp('training');
       [feature_training,ps] = mapminmax(feature_training',0,1);
       feature_training = feature_training';
       feature_testing = mapminmax('apply',msg.data,ps)';
       model = svmtrain(class_training,feature_training,libsvm_options);
%      save SVM_Trained_Model model;

% SVM Prediction 
disp('testing and prediction');
[predicted_label, accuracy, prob_estimates]=svmpredict(class_testing,sparse(feature_testing),model);
%Score = svmpredict(class_testing,sparse(feature_testing),model); Score
predicted_label
 %[Score, ~, p] = svmpredict(cls_tst,sparse(ftr_tst),model, ' -b 1');
 
    msg=org.ros.message.std_msgs.Int32();
	msg.data = round(predicted_label);
    pub.publish(msg); 
 
 node.shutdown();
