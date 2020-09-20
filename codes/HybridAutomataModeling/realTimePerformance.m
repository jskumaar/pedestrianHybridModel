%% This script checks the real-time performance of the H-Ped model

% time = 926.972 s

% load the models


% compile the actual gap results
N_gaps = length(GapFeaturesAllScenes);

for gapId = 1:N_gaps
    sceneId = GapFeaturesAllScenes{gapId}.recording - 17;
    trackId = GapFeaturesAllScenes{gapId}.pedTrack + 1; % actual track id starts from 0
    timeStep = GapFeaturesAllScenes{gapId}.pedTrackTimeStep;
    veh_id_gap = GapFeaturesAllScenes{gapId}.egoCarTrack;
    pedData = formattedTracksData{sceneId}{trackId};
    GapFeatures = GapFeaturesAllScenes{gapId};
    
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
    F_cumWait = GapFeatures.F_cumWait;
    F_pedDistToCW = GapFeatures.F_pedDistToCW; 
    F_pedDistToCurb = GapFeatures.F_pedDistToCurb;
    F_pedDistToVeh = GapFeatures.F_pedDistToVeh; 
    F_pedSpeed = GapFeatures.F_pedSpeed; 
    F_vehVel = GapFeatures.F_vehVel;
    F_gazeRatio = GapFeatures.F_gazeRatio; 
    F_isSameDirection = GapFeatures.F_isSameDirection; 
    F_isEgoNearLane = GapFeatures.F_isEgoNearLane; 

    GapSVMFeatures = table(F_cumWait, F_pedDistToCW, F_pedDistToCurb, ...
                           F_pedDistToVeh, F_pedSpeed, F_vehVel, F_gazeRatio,...
                           F_isSameDirection, F_isEgoNearLane);
    [pred_gap_class, prob_GA_outputs] = predict(Prob_GapAcceptanceModel, GapSVMFeatures);
    probGapAccept = prob_GA_outputs(2);
    GapSVMFeatures
    probGapAccept
        
    GapDecisionPredictedProb(gapId,1) = probGapAccept;
    if GapDecisionPredictedProb(gapId,1)>0.5
        GapDecisionPredicted(gapId,1) = true;
    else
        GapDecisionPredicted(gapId,1) = false;
    end

end


