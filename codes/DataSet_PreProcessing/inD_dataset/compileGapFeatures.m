%% This script checks if a gap has started and calculates the gap features

function [GapFeatures, egoVehGapHist, flag] = compileGapFeatures(PedData, currentTSEgoCarData, carTrackCurrentTimeStep, egoVehGapHist, pedTrackTimeStep, Params, flag, trackletNo, predTimeStep)

%% 1)setup
% parameters
GapFeatures = [];
decZone = Params.decZone;
AdjustedSampFreq = Params.AdjustedSampFreq;
SampFreq = Params.SampFreq;
% variables for faster downstream processing
pedVel = [PedData.xVelocity, PedData.yVelocity];
closeCar_ind = PedData.closeCar_ind;
longDispPedCw = PedData.longDispPedCw;
HybridState = PedData.HybridState;
waitTimeSteps = PedData.waitTimeSteps;
carTimeStep = carTrackCurrentTimeStep + predTimeStep - 1; 
% data sizes
N_ts = size(PedData.trackLifetime,1);
N = min(N_ts, AdjustedSampFreq);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 2) is this a gap? 
% A vehicle has just passed the pedestrian when the pedestrian is within
% the decision zone and is either approaching or waiting
if N>1
    GapCond_1 = closeCar_ind(end) ~= closeCar_ind(end-1) && ...
                abs(longDispPedCw(end)) < decZone && closeCar_ind(end) ~= inf && closeCar_ind(end) ~=0 && ...
                ( strcmp(HybridState{end},'Approach') || strcmp(HybridState{end},'Wait') );
    % There is an ego-vehicle and pedestrian has just entered the decision zone
    GapCond_2 = abs(longDispPedCw(end-1)) >= decZone && ...
                abs(longDispPedCw(end)) < decZone && ...
                closeCar_ind(end) ~= inf && ( strcmp(HybridState{end},'Approach') || strcmp(HybridState{end},'Wait') );
else
    GapCond_1 = false;
    GapCond_2 = false;
end
GapCond_3 = flag.startingFromWait(trackletNo);

if ( GapCond_1 || GapCond_2 || GapCond_3 ) 
    % 2a) is this a new gap? i.e. a new car and not a previous car gap?  
    if isempty(intersect(closeCar_ind(end), egoVehGapHist)) % this is an additional check; actually not necessary
            flag.GapStart(trackletNo) = true;
            egoVehGapHist = [egoVehGapHist, closeCar_ind(end)];

            % 3)Features for Gap Model
            % calculate the features for the hybrid system model
            % i) speed of the pedestrian in the last one sec
            F_pedSpeed = mean(sqrt(pedVel(end-N+1:end,1).^2 + pedVel(end-N+1:end,2).^2));
            % ii) lateral distance to road - already calculated
            % iii) longitudinal distance to vehicle - already calculated
            % iv) longitudinal distance to crosswalk - already calculated
            % v) cumulative waiting time
            F_cumWait = waitTimeSteps(end)/SampFreq; % in seconds
            % vi) current vehicle speed
            if ( closeCar_ind(end)~=inf)
                F_vehVel = currentTSEgoCarData.lonVelocity(carTimeStep);
                F_vehAcc = currentTSEgoCarData.lonAcceleration(carTimeStep);
            else
                F_vehVel = inf;
                F_vehAcc = inf;
            end
            % vii) Gaze (not used currently)
%             F_gazeRatio = sum(PedData.isLooking(end-N+1:end))/N;  % comment this line when gaze is calculated only from observed data (and does not include predicted data)
%             F_gazeRatio = sum(PedData.isLooking)/length(PedData.isLooking); % comment this line when gaze is updated within prediction horizon using predicted data
            % viii) Ego-car Lane to pedestrian
            F_isEgoNearLane = (mean(PedData.isNearLane(end-N+1:end)) > 0.5);
            % ix) Pedestrian direction
            F_isSameDirection = (mean(PedData.isPedSameDirection(end-N+1:end)) > 0.5);
            %%%%%%%%%%%%%%%%%%%%%%%%
            % compile gap features
            gap_ind = 1;  %vector sent as output of the function for every gap
            GapFeatures.pedTrackTimeStep(gap_ind,1) = pedTrackTimeStep;
            GapFeatures.egoCarTrack(gap_ind,1) = closeCar_ind(end);
            GapFeatures.pedCloseCw(gap_ind,1) = PedData.closestCW(end);
            % SVM gap features
            GapFeatures.F_pedSpeed(gap_ind,1) = F_pedSpeed;
            GapFeatures.F_pedDistToCW(gap_ind,1) = longDispPedCw(end);
            GapFeatures.F_cumWait(gap_ind,1) = F_cumWait;
            GapFeatures.F_pedDistToVeh(gap_ind,1) = PedData.long_disp_ped_car(end);
            GapFeatures.F_vehVel(gap_ind,1) = F_vehVel;
            GapFeatures.F_pedDistToCurb(gap_ind,1) = PedData.latDispPedCw(end);
            GapFeatures.F_vehAcc(gap_ind,1) = F_vehAcc;
%             GapFeatures.F_gazeRatio(gap_ind,1) = F_gazeRatio;
            GapFeatures.F_isEgoNearLane = F_isEgoNearLane;
            GapFeatures.F_isSameDirection = F_isSameDirection;
    else
            flag.GapStart(trackletNo) = false; %reset gap start
    end
else
    flag.GapStart(trackletNo) = false;
end

end