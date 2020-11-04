%% This function creates and initializes a new prediction tracklet
% Inputs: existing set of tracklets, current node no and (independent) probability of
% this tracklet
% Output: set of tracklets with additionally a new tracklet

function [data, kfData, probability, startNode, endNode, isActive, eventFlag, Goal, newTrackletId, flag, resetFlag] = func_newTracklet(data, kfData, probability, startNode, endNode, isActive, eventFlag, Goal, node_no, prob_tracklet, currentTrackletId, parentTrackletId, flag)
    % new tracklet id
    newTrackletId = currentTrackletId + 1;
    
    % create a new tracklet and initialize it       
    data{newTrackletId,1}.trackLifetime(1) = data{parentTrackletId,1}.trackLifetime(end);
    data{newTrackletId,1}.xCenter(1) = data{parentTrackletId,1}.xCenter(end);
    data{newTrackletId,1}.yCenter(1) = data{parentTrackletId,1}.yCenter(end);
    data{newTrackletId,1}.xVelocity(1) = data{parentTrackletId,1}.xVelocity(end);
    data{newTrackletId,1}.yVelocity(1) = data{parentTrackletId,1}.yVelocity(end);
    data{newTrackletId,1}.closestCW(1) = data{parentTrackletId,1}.closestCW(end);
    data{newTrackletId,1}.HybridState(1) = data{parentTrackletId,1}.HybridState(end);
    data{newTrackletId,1}.calcHeading(1) = data{parentTrackletId,1}.calcHeading(end);
    data{newTrackletId,1}.closeCar_ind(1) = data{parentTrackletId,1}.closeCar_ind(end);
    data{newTrackletId,1}.activeCar_ind(1) = data{parentTrackletId,1}.activeCar_ind(end);
%     data{newTrackletId,1}.isLooking(1) = data{parentTrackletId,1}.isLooking(end); 
    data{newTrackletId,1}.isPedSameDirection(1) = data{parentTrackletId,1}.isPedSameDirection(end);
    data{newTrackletId,1}.goalDisp(1) =  data{parentTrackletId,1}.goalDisp(end);
    data{newTrackletId,1}.waitTimeSteps(1) =  data{parentTrackletId,1}.waitTimeSteps(end);
    data{newTrackletId,1}.longDispPedCw(1) =  data{parentTrackletId,1}.longDispPedCw(end);
    data{newTrackletId,1}.latDispPedCw(1) =  data{parentTrackletId,1}.latDispPedCw(end);
    data{newTrackletId,1}.isNearLane(1) =  data{parentTrackletId,1}.isNearLane(end);
    data{newTrackletId,1}.long_disp_ped_car(1) =  data{parentTrackletId,1}.long_disp_ped_car(end);
    data{newTrackletId,1}.lonVelocity(1) =  data{parentTrackletId,1}.lonVelocity(end);
    data{newTrackletId,1}.Lane(1) =  data{parentTrackletId,1}.Lane(end);
    data{newTrackletId,1}.goalPositionPixels(1,:) =  data{parentTrackletId,1}.goalPositionPixels(end,:);
    data{newTrackletId,1}.swInd(1) =  data{parentTrackletId,1}.swInd(end);
    data{newTrackletId}.probGapAccept = 0;
    data{newTrackletId}.probCrossingIntent = 0;
      
    
    kfData{newTrackletId,1} = kfData{parentTrackletId}(end, :);
    probability(newTrackletId,1) = prob_tracklet; 
    startNode(newTrackletId,1) = node_no; 
    endNode(newTrackletId,1) = -1; 
    isActive(newTrackletId,1) = true; 
    eventFlag(newTrackletId,1) = false;
    Goal(newTrackletId,:) = 'NA';   

    % initialize all flags for the new tracklet
%     flag.newTracklet(newTrackletId) = true;
    flag.dataCompile(newTrackletId) = false;
    flag.hybridStatePred(newTrackletId) = false;
    flag.EgoCar(newTrackletId) = false;
    flag.GapStart(newTrackletId) = false;
    flag.sampleWaitTime(newTrackletId) = false;
    flag.startToCross(newTrackletId) = false;
    flag.finishedCrossing(newTrackletId) = false;
    flag.reachCrosswalk(newTrackletId) = false;    
    flag.checkIntent(newTrackletId) = false;
    flag.atCrosswalk(newTrackletId) = false;
    flag.startingFromWait(newTrackletId) = false;
    flag.predHorizonEnd(newTrackletId) = false;
    flag.checkIntentWOEgo(newTrackletId) = false;
    flag.reachGoal(newTrackletId) = false;
    flag.startedCrossing (newTrackletId) = false;
    flag.finishedCrossingDelayReached(newTrackletId) = false;
    
    resetFlag.approachReset(newTrackletId) = false;
    resetFlag.walkawayReset(newTrackletId) = false;
    resetFlag.check_goal(newTrackletId) = false;
    resetFlag.sample_goal(newTrackletId) = true;
    
    
end