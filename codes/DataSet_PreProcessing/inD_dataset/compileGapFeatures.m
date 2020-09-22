%% This script checks if a gap has started and calculates the gap features

function [GapFeatures, egoVehGapHist, flag] = compileGapFeatures(PedData, currentTSPedEgoData, currentTSEgoCarData, currentPedMetaData, pedTrackTimeStep, Params, flag, trackletNo)

% parameters
GapFeatures = [];
decZone = Params.decZone;
AdjustedSampFreq = Params.AdjustedSampFreq;

N_ts = size(PedData,2);
N = min(N_ts, AdjustedSampFreq);

% for easier downstream processing
pedVel = [PedData.xVelocity, PedData.yVelocity];
% pedPosPixels = [currentTSPedData.xCenterPix(ped_track_time_step-1), currentTSPedData.yCenterPix(ped_track_time_step-1)];

% 2) is this a gap? 
% A vehicle has just passed the pedestrian when the pedestrian is within
% the decision zone and is either approaching or waiting
GapCond_1 = PedData.closeCar_ind(end) ~= PedData.closeCar_ind(end-1) && ...
            abs(PedData.longDispPedCw(end)) < decZone && PedData.closeCar_ind(end) ~= inf && ...
            ( strcmp(PedData.HybridState{end},'Approach') || strcmp(PedData.HybridState{end},'Wait') );
% There is an ego-vehicle and pedestrian has just entered the decision zone
GapCond_2 = abs(PedData.longDispPedCw(end-1)) >= decZone && ...
            abs(PedData.longDispPedCw(end)) < decZone && ...
            PedData.closeCar_ind(end) ~= inf && ( strcmp(PedData.HybridState{end},'Approach') || strcmp(PedData.HybridState{end},'Wait') );

if ( GapCond_1 || GapCond_2 ) 
    % 2a) is this a new gap? i.e. a new car and not a previous car gap?  
    if isempty(intersect(PedData.closeCar_ind(end), currentPedMetaData.egoVehGapHist)) % this is an additional check; actually not necessary
            flag.GapStart(trackletNo) = true;
            egoVehGapHist = [egoVehGapHist, PedData.closeCar_ind(end)];

            % 3)Features for Gap Model
            % calculate the features for the hybrid system model
            % i) speed of the pedestrian in the last one sec
            F_pedSpeed = mean(sqrt(pedVel(end-N+1:end,1).^2 + pedVel(end-N+1:end,2).^2));
            % ii) lateral distance to road - already calculated
            % iii) longitudinal distance to vehicle - already calculated
            % iv) longitudinal distance to crosswalk - already calculated
            % v) cumulative waiting time
            F_cumWait = PedData.waitTimeSteps(end);
            % vi) current vehicle speed
            if ( PedData.closeCar_ind(end)~=inf)
                F_vehVel = currentTSEgoCarData.lonVelocity(end);
                F_vehAcc = currentTSEgoCarData.lonAcceleration(end);
            else
                F_vehVel = inf;
                F_vehAcc = inf;
            end

            % vii) Gaze!
            F_gazeRatio = sum(PedData.isLooking(end-N+1:end))/N;
            % viii) Ego-car Lane to pedestrian
            F_isEgoNearLane = (mean(PedData.isNearLane(end-N+1:end)) > 0.5);
            % ix) Pedestrian direction
            F_isSameDirection = (mean(PedData.isPedSameDirection(end-N+1:end)) > 0.5);
            
            % compile gap features
            gap_ind = 1;  %vector sent as output of the function for every gap
            GapFeatures.recording(gap_ind,1) = PedData.recordingId(end);
            GapFeatures.pedTrackTimeStep(gap_ind,1) = pedTrackTimeStep;
            GapFeatures.pedTrack(gap_ind,1) = PedData.trackId(end);
            GapFeatures.egoCarTrack(gap_ind,1) = currentTSPedEgoData.closeCar_ind(end);
            GapFeatures.pedCloseCw(gap_ind,1) = currentTSPedEgoData.cw_ped(end);

            GapFeatures.F_pedSpeed(gap_ind,1) = F_pedSpeed;
            GapFeatures.F_pedDistToCW(gap_ind,1) = currentTSPedEgoData.longDispPedCw(end);
            GapFeatures.F_cumWait(gap_ind,1) = F_cumWait;
            GapFeatures.F_pedDistToVeh(gap_ind,1) = currentTSPedEgoData.long_disp_ped_car(end);
            GapFeatures.F_vehVel(gap_ind,1) = F_vehVel;
            GapFeatures.F_pedDistToCurb(gap_ind,1) = currentTSPedEgoData.latDispPedCw(end);
            GapFeatures.F_vehAcc(gap_ind,1) = F_vehAcc;
            GapFeatures.F_gazeRatio(gap_ind,1) = F_gazeRatio;
            
            GapFeatures.F_isEgoNearLane = F_isEgoNearLane;
            GapFeatures.F_isSameDirection = F_isSameDirection;
    else
         flag.GapStart(trackletNo) = false; %reset gap start
    end
else
    flag.GapStart(trackletNo) = false;
end

end