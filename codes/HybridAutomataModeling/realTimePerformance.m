%% This script checks the real-time performance of the H-Ped model

% time = 926.972 s

% load the models


% compile the actual gap results
N_gaps = length(GapFeaturesAllScenes);

for gapId = 1:N_gaps
    sceneId = GapFeaturesAllScenes(gapId).recordingId;
    trackId = GapFeaturesAllScenes(gapId).pedTrackId; % actual track id starts from 0
    timeStep = GapFeaturesAllScenes(gapId).pedTrackTimeStep;
    veh_id_gap = GapFeaturesAllScenes(gapId).egoCarTrack;
    pedData = formattedTracksData{sceneId}{trackId};
    
    timeStep_2 = pedData.trackLifetime(1) + timeStep*5-5;
    % check when all the pedestrian crossed
    crossTimeSteps = find(pedData.HybridState=="Crossing");
    if isempty(crossTimeSteps)
        GapDecisionActual(gapId,1) = false;
    else
        crossTimeSteps_2 = [pedData.trackLifetime(1) + crossTimeSteps*5-5]; 
        crossTimeSteps(crossTimeSteps<timeStep)=[];
        if isempty(crossTimeSteps)
            GapDecisionActual(gapId,1) = false;
        else
            veh_id = pedData.closeCar_ind(crossTimeSteps(1));
            if veh_id==veh_id_gap
                 GapDecisionActual(gapId,1) = true;
            else
                 GapDecisionActual(gapId,1) = false;
            end
            
        end
        
    end
    
    
    % predict gap decision
    F_cumWait = GapFeaturesAllScenes(gapId).F_cumWait;
    F_pedDistToCW = GapFeaturesAllScenes(gapId).F_pedDistToCW; 
    F_pedDistToCurb = GapFeaturesAllScenes(gapId).F_pedDistToCurb;
    F_pedDistToVeh = GapFeaturesAllScenes(gapId).F_pedDistToVeh; 
    F_pedSpeed = GapFeaturesAllScenes(gapId).F_pedSpeed; 
    F_vehVel = GapFeaturesAllScenes(gapId).F_vehVel;
    F_gazeRatio = GapFeaturesAllScenes(gapId).F_gazeRatio; 
    F_isSameDirection = GapFeaturesAllScenes(gapId).F_isSameDirection; 
    F_isEgoNearLane = GapFeaturesAllScenes(gapId).F_isEgoNearLane; 

    GapSVMFeatures = table(F_cumWait, F_pedDistToCW, F_pedDistToCurb, ...
                           F_pedDistToVeh, F_pedSpeed, F_vehVel, F_gazeRatio,...
                           F_isSameDirection, F_isEgoNearLane);
    [pred_gap_class, prob_GA_outputs] = predict(Prob_GapAcceptanceModel, GapSVMFeatures);
    probGapAccept = prob_GA_outputs(2);
%     GapSVMFeatures
%     probGapAccept
%         
    GapDecisionPredictedProb(gapId,1) = probGapAccept;
    if GapDecisionPredictedProb(gapId,1)>0.5
        GapDecisionPredicted(gapId,1) = true;
    else
        GapDecisionPredicted(gapId,1) = false;
    end

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

N_cross = length(CrossFeaturesAllScenes);


for crossId = 1:N_cross
    sceneId = CrossFeaturesAllScenes(crossId).recordingId;
    trackId = CrossFeaturesAllScenes(crossId).pedTrackId; % actual track id starts from 0
    timeStep = CrossFeaturesAllScenes(crossId).pedTrackTimeStep;
    pedData = formattedTracksData{sceneId}{trackId};
    cw_checkIntent = [];
    
    timeStep_2 = pedData.trackLifetime(1) + timeStep*5-5;
    % check when all the pedestrian crossed
    crossTimeSteps = find(pedData.HybridState=="Crossing");
    if isempty(crossTimeSteps)
        CrossIntentActual(crossId,1) = false;
    else
        crossTimeSteps_2 = pedData.trackLifetime(1) + crossTimeSteps*5 - 5; 
        crossTimeSteps(crossTimeSteps<timeStep)=[];
        if isempty(crossTimeSteps)
            CrossIntentActual(crossId,1) = false;
        else
            CrossIntentActual(crossId,1) = true;
        end       
    end
    
    % check if cross intent was in the presence of an ego-car
    car_id = pedData.closeCar_ind(timeStep);
    
    if (car_id ~=0 && car_id ~=inf)
        % predict gap decision
       mean_veh_speed = CrossFeaturesAllScenes(crossId).mean_veh_speed;
       mean_veh_acc = CrossFeaturesAllScenes(crossId).mean_veh_acc;
       mean_DTCurb = CrossFeaturesAllScenes(crossId).mean_DTCurb;
       mean_veh_ped_dist = CrossFeaturesAllScenes(crossId).mean_veh_ped_dist;
       mean_ped_speed = CrossFeaturesAllScenes(crossId).mean_ped_speed;
       gaze_ratio = CrossFeaturesAllScenes(crossId).gaze_ratio;
       mean_DTCW = CrossFeaturesAllScenes(crossId).mean_DTCW;
       isSamedirection = CrossFeaturesAllScenes(crossId).isSameDirection;
       isNearLane = CrossFeaturesAllScenes(crossId).isNearLane;
       duration_ego_vehicle = CrossFeaturesAllScenes(crossId).duration_ego_vehicle;

       CrossSVMFeatures = table(mean_veh_speed, mean_veh_acc, mean_DTCurb , mean_veh_ped_dist, duration_ego_vehicle,...
                                        mean_ped_speed, gaze_ratio, mean_DTCW, isSamedirection, isNearLane);
       [~, prob_CI_outputs] = predict(Prob_CrossIntentModelCar, CrossSVMFeatures);
       probCrossingIntent = prob_CI_outputs(2);
       CrossDecisionPredicted(gapId,1) = 1;
    else
       mean_DTCurb = CrossFeaturesAllScenes(crossId).mean_DTCurb;
       mean_ped_speed = CrossFeaturesAllScenes(crossId).mean_ped_speed;
       mean_DTCW = CrossFeaturesAllScenes(crossId).mean_DTCW;

       CrossSVMFeatures = table(mean_DTCurb , mean_ped_speed, mean_DTCW);
       [~, prob_CI_outputs] = predict(Prob_CrossIntentModelNoCar, CrossSVMFeatures);
       probCrossingIntent = prob_CI_outputs(2);
       CrossDecisionPredicted(gapId,1) = 0;
    end   
    
    CrossDecisionPredicted(gapId,2) = probGapAccept;
    if GapDecisionPredictedProb(gapId,1)>0.5
        CrossDecisionPredicted(gapId,3) = 1;
    else
        CrossDecisionPredicted(gapId,3) = 0;
    end

end


egoInd = [];
for crossId = 1:N_cross

if ~isempty(CrossFeaturesAllScenes(crossId).mean_veh_speed)
    egoInd = [egoInd; crossId];
end

end

NoegoInd = [1:N_cross];
NoegoInd(egoInd) = [];

RT_data_Cross_intent_with_Ego_ped_speed = [];
for ii=1:length(egoInd)
    crossId = egoInd(ii);
    RT_data_Cross_intent_with_Ego_ped_speed = [RT_data_Cross_intent_with_Ego_ped_speed; CrossFeaturesAllScenes(crossId).mean_veh_speed]; 
end


RT_data_Gap_ped_speed = [];
RT_data_Gap_Wait = [];
RT_data_Gap_distCW = [];
RT_data_Gap_distVeh = [];
RT_data_Gap_distCurb = [];
RT_data_Gap_gaze = [];
for gapId=1:N_gaps
    RT_data_Gap_ped_speed = [RT_data_Gap_ped_speed; GapFeaturesAllScenes(gapId).F_pedSpeed]; 
    RT_data_Gap_Wait = [RT_data_Gap_Wait; GapFeaturesAllScenes(gapId).F_cumWait];
    RT_data_Gap_distCW = [RT_data_Gap_distCW; GapFeaturesAllScenes(gapId).F_pedDistToCW]; 
    RT_data_Gap_distVeh = [RT_data_Gap_distVeh; GapFeaturesAllScenes(gapId).F_pedDistToVeh]; 
    RT_data_Gap_distCurb = [RT_data_Gap_distCurb; GapFeaturesAllScenes(gapId).F_pedDistToCurb]; 
    RT_data_Gap_gaze = [RT_data_Gap_gaze; GapFeaturesAllScenes(gapId).F_gazeRatio];  
end


figure()
h2 = histogram(GapFeatures_SVM_legal.F_gazeRatio,'Normalization','probability','BinWidth',0.1);
% h2 = histogram(RT_data_Gap_gaze,'Normalization','probability','BinWidth',0.1);
xlabel('Wait')
ylabel('Probability')
title('GapFeatures_SVM_legal_gaze')

