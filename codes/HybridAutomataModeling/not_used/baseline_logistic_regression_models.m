%% Baseline Logistic Regression Models


%% 1) Gap acceptance
% Load the gap features .mat file and run the file
% 'runGapAcceptance_SVMModel.m'
% Note: Not necessary to normalize data for logistic regression. The beta
% coefficients indicate what is the effect of a change in one unit of the
% associated variable has on the output.

Y = double(SVMTrainData_bootStrapped.CrossDecision) + 1; % must contain positive integer number for category variables
X = table2array(SVMTrainData_bootStrapped(:,[6,7,8,11,12,13,14,15]));
% fit multinomial logistic regression
[B, dev, stats] = mnrfit(X, Y);
% test set and predictions
X_test =  table2array(SVMTestData(:,[6,7,8,11,12,13,14,15]));
Y_predictedProb = mnrval(B,X_test);
[~,Y_predicted] = max(Y_predictedProb,[],2);
Y_actual = double(SVMTestData.CrossDecision) + 1;    
% classfier performance
[Performance,Actual,Predicted] = classifierPerformance(Y_actual,Y_predicted,1.1); % the categorical outputs are 1 and 2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 2) Cross Intent
% Load the cross intent features .mat file and run the file
% 'runCrossIntent_SVMModel.m'
% Note: Not necessary to normalize data for logistic regression. The beta
% coefficients indicate what is the effect of a change in one unit of the
% associated variable has on the output.

Y = double(SVMTrainData_BootSrapped_WOE.cross_intent) + 1; % must contain positive integer number for category variables
% X = table2array(SVMTrainData_BootSrapped_WOE(:,[6,7,8,11,12,13,14,15]));
X = table2array(SVMTrainData_BootSrapped_WOE(:,[1,3,5,10,12,14,17,18,19]));
% fit multinomial logistic regression
[B, dev, stats] = mnrfit(X, Y);
% test set and predictions
% X_test =  table2array(SVMTestData(:,[6,7,8,11,12,13,14,15]));
X_test =  table2array(SVMTestData(:,[1,3,5,10,12,14,17,18,19]));
Y_predictedProb = mnrval(B,X_test);
[~,Y_predicted] = max(Y_predictedProb,[],2);
Y_actual = double(SVMTestData.cross_intent) + 1;    
% classfier performance
[Performance,Actual,Predicted] = classifierPerformance(Y_actual,Y_predicted,1.1); % the categorical outputs are 1 and 2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%