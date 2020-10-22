%% This  function checks if the pedestrian has reached the current goal location and samples the next goal location

function [flag, goal, resetFlag] = check_sample_goal(trackletData, trackletNo, resetStates, Params, flag, resetFlag)

% params
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;

% ped states
pedPosPixels = [trackletData.xCenter(end), trackletData.yCenter(end)]/(scaleFactor*orthopxToMeter);
HybridState = trackletData.HybridState(end);
cwInd = trackletData.closestCW(end);
Lane = trackletData.Lane(end);

% initialize goal bounding box
goal_bounding_box = inf*ones(4,2);
goal = [inf, inf];

% check sidewalk number 
if cwInd==1 && strcmp(Lane,'Right')
    swInd = 1;
elseif cwInd==1
    swInd = 2;
elseif  cwInd==2 && strcmp(Lane,'Right')
    swInd = 3;
elseif cwInd==2
    swInd = 4;
elseif  cwInd==3 && strcmp(Lane,'Right')
    swInd = 5;
elseif cwInd==3
    swInd = 6;
elseif  cwInd==4 && strcmp(Lane,'Right')
    swInd=7;
elseif cwInd==4
    swInd=8;
end



% identify the bounding boxes of the various goal locations (when there is
% no reset. For e.g. the track is starting initially or the close crosswalk
% has changed)
if strcmp(HybridState,'Approach')
    goal_bounding_box = reshape(resetStates.approach.goal(swInd,3:end), [2,4])';
elseif (strcmp(HybridState,'Wait') || strcmp(HybridState,'Crossing') || strcmp(HybridState,'Jaywalking'))
    goal_bounding_box = reshape(resetStates.wait.goal(swInd,3:end), [2,4])';
elseif strcmp(HybridState,'Walk_away')
    goal_bounding_box = reshape(resetStates.walkaway.goal(swInd,3:end), [2,4])';
end


% update goal location if there is any reset flag
if resetFlag.approachReset(trackletNo)
    goal_bounding_box = reshape(resetStates.approachReset.goal(swInd,3:end), [2,4])';
    resetFlag.approachReset(trackletNo) = false;
elseif resetFlag.walkawayReset
    goal_bounding_box = reshape(resetStates.walkaway.goal(swInd,3:end), [2,4])';
    resetFlag.walkawayReset(trackletNo) = false;
end


%%%%%%%%%%%%%%%%%%%%%%%%
% check if goal bounding box has been reached. The output of this does not
% hold when samplign a new goal, so the flag is reset after sampling new goal in 'updatePedContStates.m'.
if resetFlag.check_goal(trackletNo)
    if goal_bounding_box(1,1)~=inf
%         for ii=1:4
%            if abs(pedPosPixels(1)) < abs(goal_bounding_box(ii,1)) &&  abs(pedPosPixels(2)) < abs(goal_bounding_box(ii,2)) 
%                insideBB(ii) = 1;
%            else
%                insideBB(ii) = 0;
%            end
%         end

%         if sum(insideBB)==0 || sum(insideBB)==4
%             flag.reachGoal(trackletNo) = true;
%         else
%             flag.reachGoal(trackletNo) = false;
%         end

        if ( pedPosPixels(1)<max(goal_bounding_box(:,1)) && pedPosPixels(1)>min(goal_bounding_box(:,1)) && ...
             pedPosPixels(2)<max(goal_bounding_box(:,2)) && pedPosPixels(2)>min(goal_bounding_box(:,2)) )
            flag.reachGoal(trackletNo) = true;
        else
            flag.reachGoal(trackletNo) = false;
        end

    else
        flag.reachGoal(trackletNo) = false;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% reset and sample a goal location
if resetFlag.sample_goal(trackletNo)
    goal_x = rand*[max(goal_bounding_box(:,1)) - min(goal_bounding_box(:,1))] + min(goal_bounding_box(:,1));
    goal_y = rand*[max(goal_bounding_box(:,2)) - min(goal_bounding_box(:,2))] + min(goal_bounding_box(:,2));
    goal = [goal_x, goal_y];
    resetFlag.sample_goal(trackletNo) = false;
end



end