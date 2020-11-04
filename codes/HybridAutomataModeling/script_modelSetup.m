%% This script sets up the model based on the configuration

% 1) Load the SVM models
% if strcmp(predictionModel,"MultipleHybridPedestrian")
%     % load the gap acceptance model
%     GapAcceptanceStruct = load(gapModelFile, gapModel);
%     GapAcceptanceModel = GapAcceptanceStruct.ClassificationSVM;
%     Prob_GapAcceptanceModel = fitSVMPosterior(GapAcceptanceModel);
%     % load the crossing intent models - with and without an Ego Car
%     CrossIntentModelStruct = load(crossIntentModelFile, crossIntentModel);
%     CrossIntentModelCar = CrossIntentModelStruct.ClassificationSVM;
%     Prob_CrossIntentModelCar = fitSVMPosterior(CrossIntentModelCar);
%     CrossIntentNoCarModelStruct = load(crossIntentNoCarModelFile, crossIntentNoCarModel);
%     CrossIntentNoCarModel = CrossIntentNoCarModelStruct.ClassificationSVM;
%     Prob_CrossIntentModelNoCar = fitSVMPosterior(CrossIntentNoCarModel);   
% elseif strcmp(predictionModel,"BaselineHybrid")
%     % load the gap acceptance model
%     GapAcceptanceStruct = load(gapModelFile, gapModel);
%     GapAcceptanceModel = GapAcceptanceStruct.ClassificationSVM;
%     Prob_GapAcceptanceModel = fitSVMPosterior(GapAcceptanceModel);   
% end
%%%%%%%%%%%%%%%%%%%

% 2) initialize output variables
if  strcmp(predictionModel,"MultipleHybridPedestrian")
    predictedPedTraj_MHP = cell(12, 211, 613); %maximum sizes of scenes, no. of moving cars, and tracks in scene respectively; pre-allocated for speed
elseif  strcmp(predictionModel,"BaselineHybrid")
    predictedPedTraj_HBase = cell(12, 211, 613);
else
    predictedPedTraj_CV = cell(12, 211, 613);
end
%%%%%%%%%%%%%%%%%%%%

% 3) initialize prediction variables
GapFeaturesAllScenes = struct('recordingId',[],'pedcarTrackId',[],'pedTrackTimeStep',[],'egoCarTrack',[],'pedCloseCw',[],'F_pedSpeed',[],'F_pedDistToCW',[],...
                              'F_cumWait',[],'F_pedDistToVeh',[],'F_vehVel',[],'F_pedDistToCurb',[],'F_vehAcc',[],'F_isEgoNearLane',[],...
                              'F_isSameDirection',[],'predDecision',[],'timeStepInHorizon',[]);
CrossFeaturesAllScenes = struct('recordingId',[],'pedcarTrackId',[],'pedTrackTimeStep',[],'timeStepInHorizon',[],'mean_ped_speed',[],...
                                'mean_DTCurb',[],'mean_DTCW',[],'mean_veh_speed',[],'mean_veh_acc',[],'mean_veh_ped_dist',[],...
                                'gaze_ratio',[],'isSameDirection',[], 'isNearLane',[], 'duration_ego_vehicle',[],'closestCW',[],'predDecision',[]);
predPedIndex = 0;
GapFeatureId = 1;
CrossFeatureId = 1;
flag.pred = false; % this is to run the close CW function ('hybridState') w/o hybrid state update
%%%%%%%%%%%%%%%%%%%