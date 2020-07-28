%% This script performs the prediction of the pedestrian


function pedPredictions = updateStates(currentPedData, currentCarData, currentPedMetaData, ped_track_time_step, pred_horizon, SampFreq, cw, annotatedImage_enhanced)

    load('crossDelayExpDist.mat')
    


    predPedData = currentPedData;  % full track
    predTSActiveCarData = currentCarData;  % this is just the instantaneous data though, not the full track.
    
    for time_step = 1:pred_horizon
        pred_time_step = ped_track_time_step + time_step;
   
        %Step 6a: Prepare the data to send to find the ego car           
%        currentTSPedData = predPedData(pred_time_step-1, :);
        currentTSActiveCarData = predTSActiveCarData(time_step);
        if pred_time_step > SampFreq            
            currentTSPedData = predPedData(pred_time_step-SampFreq-1: pred_time_step, :);
        else
            currentTSPedData = predPedData;
        end
        
        %Step 6b: Find the ego-car and save the car track index
        currentTSPedEgoData = egoCarFunc(currentTSPedData, currentTSActiveCarData, cw, annotatedImage_enhanced);
        predPedData.closeCar_ind(pred_time_step-1) = currentTSPedEgoData.closeCar_ind;

        % Step 7b: if there is an ego car, find if gap starts and the
        % gap features
        if currentTSPedEgoData.closeCar_ind~=inf 
            flag.EgoCar = true;
            ego_activeCar_ind = find(currentTSActiveCarData.trackId == currentTSPedEgoData.closeCar_ind);
            currentTSEgoCarData = currentTSActiveCarData(ego_activeCar_ind, :);

            if pred_time_step > 1
            % Step 7: Check if a gap has started and gather the features of the gap acceptance model
              [GapFeatures, currentPedMetaData, flag] = gapCheck(currentTSPedData, currentTSEgoCarData, currentTSPedEgoData, currentPedMetaData, delta_T, pred_time_step-1, flag);
            end  %end of if loop for gap checking
        end   %end of if loop for the presence of an ego-vehicle

        % Step 8: Predict the crossing state transition of the pedestrians
        if flag.GapStart
            F_cumWait = GapFeatures.F_cumWait;
            F_pedDistToCW = GapFeatures.F_pedDistToCW;
            F_pedDistToCurb = GapFeatures.F_pedDistToCurb;
            F_pedDistToVeh = GapFeatures.F_pedDistToVeh;
            F_pedSpeed = GapFeatures.F_pedSpeed;
            F_vehVel = GapFeatures.F_vehVel;

            SVMFeatures = table(F_cumWait, F_pedDistToCW, F_pedDistToCurb, ...
                           F_pedDistToVeh, F_pedSpeed, F_vehVel);
            flag.GapAccept = predict(DiscreteModel, SVMFeatures);
        end
               
        % Step 8a: Discrete State update
        if flag.GapAccept
           % sample a time from the exponential distribution of crossDelay
           GapFeatures.AcceptGapTime = pred_time_step;
           time_start_cross = int32(exprnd(crossDelayExpDist.mu)/dt);
           
            
        end
        
        
        % Step 7a: Check if the pedestrian has crossed during the most
        % recent gap.
            if ( strcmp(predPedData.HybridState{pred_time_step-1},'Crossing') && ~strcmp(predPedData.HybridState{pred_time_step-2},'Crossing') )
                if GapFeatures.egoCarTrack == predPedData.closeCar_ind(pred_time_step-1)                  
                    GapFeatures.ActualCrossDecision = true;
                    GapFeatures.ActualCrossStart = pred_time_step-1;
                    GapFeatures.CrossCW = predPedData.cw_ped(pred_time_step-1);
                end
            end

    end

end