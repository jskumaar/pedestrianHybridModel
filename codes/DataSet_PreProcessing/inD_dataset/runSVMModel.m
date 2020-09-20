%% This code is used to split train and testing set and run the SVM model.

%% run some code from GapDataDescriptives_and_plot.m to get the indices
% 'ind_all_legal_gaps'


%% Step 1: For the SVM model trained on VR dataset, test the entire Gap data
%% 1a)Use existing models 

% i) VR dataset trained model
load('SVM_ExtremeOutlier_NoHighDeceleration_6Features_noGap_noGaze.mat');
SVMModel = SVM_ExtremeOutlier_NoHighDeceleration_6Features_noGap_noGaze.ClassificationSVM;

% ii) inD dataset trained model (subset of rejected gaps)
load('GapAcceptance_inD_7Features_FGaussianSVM_BootStrappedOnce.mat');
SVMModel = GapAcceptance_inD_7Features_FGaussianSVM_BootStrappedOnce.ClassificationSVM;

%% 1b) load saved gap data to develop SVM data and train the model

% load('GapData_12Scenes.mat');
% load('GapData_12Scenes_jaywalking_v3.mat');


%% Split the data into test and train sets
GapFeatures_SVM_NoOutliers = GapFeatures(ind_all_legal_gaps,:);

TrainData_percent = 80;

N_tracks = size(GapFeatures_SVM_NoOutliers,1);
All_tracks = [1:N_tracks]';
Train_tracks = randperm(N_tracks, int32(TrainData_percent/100*N_tracks))';
Test_tracks = All_tracks;
Test_tracks(Train_tracks) = [];

% subset of gaps
AcceptedGaps = find(GapFeatures_SVM_NoOutliers.CrossDecision==1);
RejectedGaps = find(GapFeatures_SVM_NoOutliers.CrossDecision==0);

% SVM data
SVMTrainData = GapFeatures_SVM_NoOutliers(Train_tracks,:);
SVMTestData = GapFeatures_SVM_NoOutliers(Test_tracks,:);

% For bootstrapping
AcceptedGaps_test = find(SVMTestData.CrossDecision==1);
RejectedGaps_test = find(SVMTestData.CrossDecision==0);

AcceptedGaps_train = find(SVMTrainData.CrossDecision==1);
RejectedGaps_train = find(SVMTrainData.CrossDecision==0);

SVMTrainData_Accepted = SVMTrainData(AcceptedGaps_train,:);

% RejectedGaps_train_subset = RejectedGaps_train(randperm(length(RejectedGaps_train), 600));
% SVMTrainData_subset = GapFeatures_SVM_legal([Gap_ind_range(Train_tracks(AcceptedGaps_train)); Gap_ind_range(Train_tracks(RejectedGaps_train_subset))],:);
% AcceptedGaps_train_ss = find(SVMTrainData_subset.CrossDecision==1);
% RejectedGaps_train_ss = find(SVMTrainData_subset.CrossDecision==0);

%% Boot-strapping training data to balance the classes
SVMTrainData_bootStrapped = SVMTrainData;


for ii=1:1
   % boot strapping 7 times provided optimal results when tuned manually
   SVMTrainData_bootStrapped = [SVMTrainData_bootStrapped; SVMTrainData_Accepted];
end


%% Step 4: Train SVM models on the real-world datasets
% Train with multiple combinations of features








%% Step 5: Test the existing VR SVM model
% load('GapAcceptance_inD_6Features_noGaze_noTimeGap_FGaussianSVM_2300RejGaps_BootStrapped.mat');
% %SVMModel = GapAcceptance_inD_6Features_noGaze_noTimeGap_QuadraticSVM.ClassificationSVM;
% SVMModel = GapAcceptance_inD_6Features_noGaze_noTimeGap_GaussianSVM.ClassificationSVM;


load('GapAcceptance_inD_9Features_FGaussianSVM_BootStrappedTwice.mat');
SVMModel = GapAcceptance_inD_9Features_FGaussianSVM_BootStrappedTwice.ClassificationSVM;


for test_ind = 1:size(Test_tracks,1)
    
    F_cumWait =   SVMTestData.F_cumWait(test_ind);
    F_pedSpeed = SVMTestData.F_pedSpeed(test_ind);
    F_pedDistToCurb = SVMTestData.F_pedDistToCurb(test_ind);
    F_pedDistToCW = SVMTestData.F_pedDistToCW(test_ind);
    F_pedDistToVeh = SVMTestData.F_pedDistToVeh(test_ind);
    F_vehVel = SVMTestData.F_vehVel(test_ind);
    F_gazeRatio = SVMTestData.F_gazeRatio(test_ind);
    F_isSameDirection = SVMTestData.F_isSameDirection(test_ind);
    F_isEgoNearLane = SVMTestData.F_isEgoNearLane(test_ind);
    
    SVMFeatures = table(F_cumWait, F_pedDistToCW, F_pedDistToCurb,...
                           F_pedDistToVeh, F_pedSpeed, F_vehVel, F_gazeRatio,...
                           F_isSameDirection, F_isEgoNearLane);
            
%     % feature order for inD model
%     features = [SVMTestData.F_pedSpeed(test_ind), SVMTestData.F_pedDistToCW(test_ind), SVMTestData.F_cumWait(test_ind), ...
%                 SVMTestData.F_pedDistToVeh(test_ind),  SVMTestData.F_vehVel(test_ind), SVMTestData.F_pedDistToCurb(test_ind)];
%               
    
    [predicted_decision(test_ind)] = predict(SVMModel, SVMFeatures);  
    actual_decision(test_ind) = SVMTestData.CrossDecision(test_ind);
end

[Performance2,Actual,Predicted] = classifierPerformance(actual_decision,predicted_decision,0.5);



%% Step 6: Test the newly trained SVM models








%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
