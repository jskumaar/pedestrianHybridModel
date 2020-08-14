%% This code is used to split train and testing set and run the SVM model.

%% Step 1: For the SVM model trained on VR dataset, test the entire Gap data
% Use the model without Gaze

% a) VR dataset trained model
load('SVM_ExtremeOutlier_NoHighDeceleration_6Features_noGap_noGaze.mat');
SVMModel = SVM_ExtremeOutlier_NoHighDeceleration_6Features_noGap_noGaze.ClassificationSVM;



% b) inD dataset trained model (subset of rejected gaps)
% load('GapAcceptance_inD_6Features_noGaze_noTimeGap_QuadraticSVM.mat');
% SVMModel = GapAcceptance_inD_6Features_noGaze_noTimeGap_QuadraticSVM.ClassificationSVM;





load('GapData_12Scenes_jaywalking.mat');

% combine the boolean outputs of cross and jaywalk into a single variale if
% needed
GapFeatures.Decision = GapFeatures.CrossDecision + 2*GapFeatures.JaywalkDecision;

%% Step 2: Process Gap data
scaling_factor = 12;
orthopxToMeter = 0.0081;


GapFeatures_SVM = GapFeatures;

GapFeatures_SVM.F_pedDistToCW = double(abs(GapFeatures_SVM.F_pedDistToCW))*(scaling_factor*orthopxToMeter);
GapFeatures_SVM.F_pedDistToVeh = double(abs(GapFeatures_SVM.F_pedDistToVeh))*(scaling_factor*orthopxToMeter);
GapFeatures_SVM.F_pedDistToCurb = double(abs(GapFeatures_SVM.F_pedDistToCurb))*(scaling_factor*orthopxToMeter);

VelGap = GapFeatures_SVM.F_pedDistToVeh./GapFeatures_SVM.F_vehVel;
Gap_ind_range = find(VelGap > 0 & VelGap <= 12.90);

%% Step 3: Split the data into test and train sets
TrainData_percent = 80;

N_tracks = size(Gap_ind_range,1);
All_tracks = [1:N_tracks]';
Train_tracks = randperm(N_tracks, int32(TrainData_percent/100*N_tracks))';
Test_tracks = All_tracks;
Test_tracks(Train_tracks) = [];

SVMTrainData = GapFeatures_SVM(Gap_ind_range(Train_tracks),:);
SVMTestData = GapFeatures_SVM(Gap_ind_range(Test_tracks),:);

% subset of gaps
AcceptedGaps = find(GapFeatures_SVM.CrossDecision==1);
RejectedGaps = find(GapFeatures_SVM.CrossDecision==0);

AcceptedGaps_test = find(SVMTestData.CrossDecision==1);
RejectedGaps_test = find(SVMTestData.CrossDecision==0);

AcceptedGaps_train = find(SVMTrainData.CrossDecision==1);
RejectedGaps_train = find(SVMTrainData.CrossDecision==0);

RejectedGaps_train_subset = RejectedGaps_train(randperm(length(RejectedGaps_train), 600));


SVMTrainData_Accepted = SVMTrainData(AcceptedGaps_train,:);


SVMTrainData_subset = GapFeatures_SVM([Gap_ind_range(Train_tracks(AcceptedGaps_train)); Gap_ind_range(Train_tracks(RejectedGaps_train_subset))],:);


AcceptedGaps_train_ss = find(SVMTrainData_subset.CrossDecision==1);
RejectedGaps_train_ss = find(SVMTrainData_subset.CrossDecision==0);


%% Step 4: Test the existing VR SVM model
load('GapAcceptance_inD_6Features_noGaze_noTimeGap_FGaussianSVM_2300RejGaps_BootStrapped.mat');
%SVMModel = GapAcceptance_inD_6Features_noGaze_noTimeGap_QuadraticSVM.ClassificationSVM;
SVMModel = GapAcceptance_inD_6Features_noGaze_noTimeGap_GaussianSVM.ClassificationSVM;

for test_ind = 1:size(Test_tracks,1)
    
    % feature order for VR dataset model
    % features = [SVMTestData.F_cumWait(test_ind), SVMTestData.F_pedSpeed(test_ind), SVMTestData.F_pedDistToCurb(test_ind),...
    %            SVMTestData.F_pedDistToCW(test_ind), SVMTestData.F_pedDistToVeh(test_ind),  SVMTestData.F_vehVel(test_ind), ];
    
    F_cumWait =   SVMTestData.F_cumWait(test_ind);
    F_pedSpeed = SVMTestData.F_pedSpeed(test_ind);
    F_pedDistToCurb = SVMTestData.F_pedDistToCurb(test_ind);
    F_pedDistToCW = SVMTestData.F_pedDistToCW(test_ind);
    F_pedDistToVeh = SVMTestData.F_pedDistToVeh(test_ind);
    F_vehVel = SVMTestData.F_vehVel(test_ind);
    
    SVMFeatures = table(F_cumWait, F_pedDistToCW, F_pedDistToCurb,...
                           F_pedDistToVeh, F_pedSpeed, F_vehVel);
            
%     % feature order for inD model
%     features = [SVMTestData.F_pedSpeed(test_ind), SVMTestData.F_pedDistToCW(test_ind), SVMTestData.F_cumWait(test_ind), ...
%                 SVMTestData.F_pedDistToVeh(test_ind),  SVMTestData.F_vehVel(test_ind), SVMTestData.F_pedDistToCurb(test_ind)];
%               
    
    [predicted_decision(test_ind)] = predict(SVMModel, SVMFeatures);  
    actual_decision(test_ind) = SVMTestData.CrossDecision(test_ind);
end

[Performance4,Actual,Predicted] = classifierPerformance(actual_decision,predicted_decision,0.5);

%%

for ii=1:5
    % boot strapping 7 times provided optimal results when tuned manually
   SVMTrainData = [SVMTrainData; SVMTrainData_Accepted];
    
    
end




%% Boot-strapping jaywalk data to balance the three classes
for ii=1:7
    % boot strapping 7 times provided optimal results when tuned manually
   SVMTrainData_subset = [SVMTrainData_subset; JWData];
    
    
end






%% Step 5: Train SVM models on the real-world datasets
% Train with multiple combinations of features






%% Step 6: Test the newly trained SVM models








%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
