%% This script checks if a gap has started and calculates the gap features

function [GapFeatures, currentPedMetaData, flag] = gapCheck(currentTSPedData, currentTSEgoCarData, currentTSPedEgoData, currentPedMetaData, AdjustedSampFreq, flag)
    
    % 1) for easier downstream processing
    ped_vel = [currentTSPedData.xVelocity, currentTSPedData.yVelocity];
    
    % 2) is this a gap? A new vehicle becomes closest vehicle when the
    % pedestrian is approaching the crosswalk or waiting
    if ( currentTSPedData.closeCar_ind(end) ~= currentTSPedData.closeCar_ind(end-1) && ...
         currentTSPedData.closeCar_ind(end) ~= inf && ( strcmp(currentTSPedData.HybridState{end},'Approach') || strcmp(currentTSPedData.HybridState{end},'Wait') ) ) 
        % 2a) is this a new gap? i.e. a new car and not a previous car gap?  
        if isempty(intersect(currentTSPedData.closeCar_ind(end), currentPedMetaData.ego_veh_gap_hist))
                flag.GapStart = true;
                currentPedMetaData.ego_veh_gap_hist = [currentPedMetaData.ego_veh_gap_hist; currentTSPedData.closeCar_ind(end)];

                % 3)Features for Gap Model
                % calculate the features for the hybrid system model
                % i) speed of the pedestrian in the last one sec
                F_pedSpeed = mean(sqrt(ped_vel(:,1).^2 + ped_vel(:,2).^2));
                % ii) lateral distance to road - already calculated
                % iii) longitudinal distance to vehicle - already calculated
                % iv) longitudinal distance to crosswalk - already calculated
                % v) cumulative waiting time
                if currentPedMetaData.waitStartFlag
                    if (strcmp(currentTSPedData.HybridState{end}, 'Wait'))
                        F_cumWait = (ped_track_time_step - currentTSPedData.wait_start_time_step(end))*AdjustedSampFreq;
                    else
                        F_cumWait = 0;
                    end
                elseif ( strcmp(currentTSPedData.HybridState{end}, 'Wait') && strcmp(currentTSPedData.HybridState{end-1}, 'Approach') )
                    currentPedMetaData.waitStartFlag = true;
                    currentTSPedData.wait_start_time_step = [currentTSPedData.wait_start_time_step, scene_time_step];
                end
                % vi) vehicle speed
                if ( currentTSPedData.closeCar_ind(end)~=inf)
                    F_vehVel = currentTSEgoCarData.lonVelocity;
                    F_vehAcc = currentTSEgoCarData.lonAcceleration;
                else
                    F_vehVel = inf;
                    F_vehAcc = inf;
                end
                
                % vii) Need to include Gaze later!!

                % compile gap features
                gap_ind = 1;  %vector sent as output of the function for every gap
                GapFeatures.recording(gap_ind,1) = currentPedData.recordingId(end);
                GapFeatures.frame(gap_ind,1) = ped_track_time_step;
                GapFeatures.pedTrack(gap_ind,1) = currentPedData.trackID(end);
                GapFeatures.egoCarTrack(gap_ind,1) = currentPedData.closestCar_ind(end);
                GapFeatures.ped_close_cw(gap_ind,1) = currentPedData.cw_ped(end);

                GapFeatures.F_pedSpeed(gap_ind,1) = F_pedSpeed;
                GapFeatures.F_pedDistToCW(gap_ind,1) = currentTSPedEgoData.long_disp_ped_cw_pixels;
                GapFeatures.F_cumWait(gap_ind,1) = F_cumWait;
                GapFeatures.F_pedDistToVeh(gap_ind,1) = currentTSPedEgoData.long_disp_car_pixels;
                GapFeatures.F_vehVel(gap_ind,1) = F_vehVel;
                GapFeatures.F_pedDistToCurb(gap_ind,1) = currentTSPedEgoData.lat_disp_cw_pixels;
                GapFeatures.F_vehAcc(gap_ind,1) = F_vehAcc;
        end
    end
     
end