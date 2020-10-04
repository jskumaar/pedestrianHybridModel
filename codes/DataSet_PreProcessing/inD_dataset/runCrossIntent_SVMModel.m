%% This code is used to split train and testing set and run the SVM model from cross intent.

%% Step 1: For the SVM model trained on VR dataset, test the entire Cross data
% Use the model without Gaze

% % a) VR dataset trained model
% load('SVM_ExtremeOutlier_NoHighDeceleration_6Features_noCross_noGaze.mat');
% SVMModel = SVM_ExtremeOutlier_NoHighDeceleration_6Features_noCross_noGaze.ClassificationSVM;
% 
% % b) inD dataset trained model (subset of rejected Crosss)
% % load('CrossAcceptance_inD_6Features_noGaze_noTimeCross_QuadraticSVM.mat');
% % SVMModel = CrossAcceptance_inD_6Features_noGaze_noTimeCross_QuadraticSVM.ClassificationSVM;
% 


CrossFeatures_SVM_withOutEgo = CrossIntentData_withOutEgoCar;

%% Step 3: Split the data into test and train sets
TrainData_percent = 80;

N_tracks = size(CrossFeatures_SVM_withOutEgo,1);
All_tracks = [1:N_tracks]';

Train_tracks = randperm(N_tracks, int32(TrainData_percent/100*N_tracks))';
Test_tracks = All_tracks;
Test_tracks(Train_tracks) = [];

SVMTrainData = CrossFeatures_SVM_withOutEgo(Train_tracks,:);
SVMTestData = CrossFeatures_SVM_withOutEgo(Test_tracks,:);


% indices
Cross = find(CrossFeatures_SVM_withOutEgo.cross_intent==1);
NotCross = find(CrossFeatures_SVM_withOutEgo.cross_intent==0);

Cross_train = find(SVMTrainData.cross_intent==1);
NotCross_train = find(SVMTrainData.cross_intent==0);

Cross_test = find(SVMTestData.cross_intent==1);
NotCross_test = find(SVMTestData.cross_intent==0);

% SVM data
SVMTrainData_Cross_WOE = SVMTrainData(Cross_train,:);
SVMTrainData_NotCross_WOE = SVMTrainData(NotCross_train,:);

SVMTestData_NotCross_WOE = SVMTestData(NotCross_test,:);
SVMTestData_Cross_WOE = SVMTestData(Cross_test,:);

SVMTrainData_BootSrapped_WOE = SVMTrainData;

% boot strapping x times provided optimal results when tuned manually
for ii=1:1
   SVMTrainData_BootSrapped_WOE = [SVMTrainData_BootSrapped_WOE; SVMTrainData_NotCross_WOE];
end


%% Step 4: Test the existing VR SVM model
% load('trainedModelFG.mat');
% SVMModel = trainedModelFG.ClassificationSVM;

% SVMModel = CrossIntent_inD_7Features_onlyMean_GaussianSVM.ClassificationSVM;
% SVMModel = CrossIntent_inD_6Features_onlyMean_noVehAcc_GaussianSVM.ClassificationSVM;
% SVMModel = CrossIntent_inD_6Features_onlyMean_noVehAcc_GaussianSVM_1s.ClassificationSVM;
% SVMModel = CrossIntent_inD_6Features_onlyMean_noVehAcc_GaussianSVM_2s.ClassificationSVM;
% SVMModel = CrossIntent_inD_6Features_onlyMean_noVehAcc_GaussianSVM_3s.ClassificationSVM;
% SVMModel = CrossIntent_inD_6Features_onlyMean_noVehAcc_GaussianSVM_4s.ClassificationSVM;

SVMModel =CrossIntent_NoCar_inD_3Features_BS1_noDuration_FGaussianSVM_v2.ClassificationSVM;


predicted_decision = [];
actual_decision = [];
for test_ind = 1:size(SVMTestData,1)
    
    % feature order for VR dataset model
    % features = [SVMTestData.F_cumWait(test_ind), SVMTestData.F_pedSpeed(test_ind), SVMTestData.F_pedDistToCurb(test_ind),...
    %            SVMTestData.F_pedDistToCW(test_ind), SVMTestData.F_pedDistToVeh(test_ind),  SVMTestData.F_vehVel(test_ind), ];
    
%     mean_veh_speed          = SVMTestData.mean_veh_speed(test_ind);
% %     std_veh_speed           = SVMTestData.std_veh_speed(test_ind);
%     mean_veh_acc            = SVMTestData.mean_veh_acc(test_ind);
% %     std_veh_acc             = SVMTestData.std_veh_acc(test_ind);
%     mean_veh_ped_dist       = SVMTestData.mean_veh_ped_dist(test_ind);
%     std_veh_ped_dist        = SVMTestData.std_veh_ped_dist(test_ind);
    mean_ped_speed          = SVMTestData.mean_ped_speed(test_ind);
%     std_ped_speed           = SVMTestData.std_ped_speed(test_ind);
    mean_DTCurb             = SVMTestData.mean_DTCurb(test_ind);
%     std_DTCurb              = SVMTestData.std_DTCurb(test_ind);
    mean_DTCW               = SVMTestData.mean_DTCW(test_ind);
%     std_DTCW                = SVMTestData.std_DTCW(test_ind);
%     duration_ego_vehicle    = SVMTestData.duration_ego_vehicle(test_ind);
%     gaze_ratio              = SVMTestData.gaze_ratio(test_ind);
%     isNearLane              = SVMTestData.isNearLane(test_ind);
%     isSamedirection              = SVMTestData.isSamedirection(test_ind);
    
%     SVMFeatures = table(mean_veh_speed, std_veh_speed, mean_veh_acc, std_veh_acc,...
%                         mean_veh_ped_dist, std_veh_ped_dist, mean_ped_speed, std_ped_speed,...
%                         mean_DTCurb, std_DTCurb, mean_DTCW, gaze_ratio);
             
%    SVMFeatures = table(mean_veh_speed, mean_veh_acc, mean_veh_ped_dist, mean_ped_speed,...
%                        mean_DTCurb, mean_DTCW, gaze_ratio, isNearLane, isSamedirection);
% %        
%     
     SVMFeatures = table(mean_ped_speed, mean_DTCurb, mean_DTCW);
%       
    [predicted_decision(test_ind)] = predict(SVMModel, SVMFeatures);  
    actual_decision(test_ind) = SVMTestData.cross_intent(test_ind);
end

[Performance,Actual,Predicted] = classifierPerformance(actual_decision,predicted_decision,0.5);


ind_correct_crossing_intent = find(predicted_decision==0 & actual_decision==0);

cw_dist = SVMTrainData_BootSrapped_WOE.mean_DTCW(ind_correct_crossing_intent);
%%


boxplot(cw_dist)
cw_dist_outlier_rem = cw_dist;
cw_dist_outlier_rem(cw_dist<0) = [];


mean(cw_dist_outlier_rem)



%% Step 5: Train SVM models on the real-world datasets
% Train with multiple combinations of features






%% Step 6: Test the newly trained SVM models








%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
