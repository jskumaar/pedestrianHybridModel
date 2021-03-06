%% This  function checks if the pedestrian has reached the current goal location and samples the next goal location

function [flag, goal, resetFlag] = func_checkSampleGoal(trackletData, trackletNo, resetStates, Params, flag, resetFlag)

% params
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
% ped states
pedPos = [trackletData.xCenter(end), trackletData.yCenter(end)];
pedPosPixels = [trackletData.xCenter(end), trackletData.yCenter(end)]/(scaleFactor*orthopxToMeter);
goalPos = trackletData.goalPositionPixels(end,:)*(scaleFactor*orthopxToMeter);
HybridState = trackletData.HybridState(end);
cwInd = trackletData.closestCW(end);
swInd = trackletData.swInd(end);
Lane = trackletData.Lane(end);
pedHeading = atan2(trackletData.yVelocity(end), trackletData.xVelocity(end)) * 180/pi;
% initialize goal bounding box
goal_bounding_box = inf*ones(4,2);
goal = [inf, inf];

% check sidewalk number 
if swInd==1 && strcmp(Lane,'Right')
    swInd_side = 1;
elseif swInd==1
    swInd_side = 2;
elseif  swInd==2 && strcmp(Lane,'Right')
    swInd_side = 3;
elseif swInd==2
    swInd_side = 4;
elseif  swInd==3 && strcmp(Lane,'Right')
    swInd_side = 5;
elseif swInd==3
    swInd_side = 6;
elseif swInd==4 && strcmp(Lane,'Right')
    swInd_side=7;
elseif swInd==4
    swInd_side=8;
end

% check crosswalk side to approach
% check sidewalk number 
if cwInd==1 && strcmp(Lane,'Right')
    cwInd_side = 1;
elseif cwInd==1
    cwInd_side = 2;
elseif  cwInd==2 && strcmp(Lane,'Right')
    cwInd_side = 3;
elseif cwInd==2
    cwInd_side = 4;
elseif cwInd==3 && strcmp(Lane,'Right')
    cwInd_side = 5;
elseif cwInd==3
    cwInd_side = 6;
elseif cwInd==4 && strcmp(Lane,'Right')
    cwInd_side=7;
elseif cwInd==4
    cwInd_side=8;
end

% goal_bb_reset = true;
% while(goal_bb_reset)
% identify the bounding boxes of the various goal locations (when there is
% no reset. For e.g. the track is starting initially or the close crosswalk
% has changed)
if strcmp(HybridState,'Approach')
    goal_bounding_box = reshape(resetStates.approach.goal(cwInd_side,3:end), [2,4])';
elseif (strcmp(HybridState,'Wait') || strcmp(HybridState,'Crossing') || strcmp(HybridState,'Jaywalking'))
    goal_bounding_box = reshape(resetStates.wait.goal(swInd_side,3:end), [2,4])';
elseif strcmp(HybridState,'Walk_away')
    goal_bounding_box = reshape(resetStates.walkaway.goal(swInd_side,3:end), [2,4])';
end

% special cases
if ( strcmp(HybridState,'Walk_away') && swInd_side==1 && flag.reachGoal(trackletNo) && ~flag.reachCrosswalk(trackletNo)) 
    goal_bounding_box = reshape(resetStates.walkaway.erCw1Final(3:end), [2,4])';
end
if ( strcmp(HybridState,'Approach') && swInd_side==1 && flag.reachGoal(trackletNo) && ~flag.reachCrosswalk(trackletNo)) 
    goal_bounding_box = reshape(resetStates.approach.goal(1, 3:end), [2,4])';
end

%% debug
if (strcmp(HybridState,'Walk_away') && cwInd==1  ) || ( strcmp(HybridState,'Approach') && cwInd==1 )
    x=1;
end

% update goal location if there is any reset flag
if resetFlag.approachReset(trackletNo)
    goal_bounding_box = reshape(resetStates.approachReset.goal(swInd_side,3:end), [2,4])';
    resetFlag.approachReset(trackletNo) = false;
    goal_bb_reset = false;
elseif resetFlag.walkawayReset
    goal_bounding_box = reshape(resetStates.walkaway.goal(swInd_side,3:end), [2,4])';
    resetFlag.walkawayReset(trackletNo) = false;  
    goal_bb_reset = false;
end
%%%%%%%%%%%%%%%%%%%%%%%%
% check if goal bounding box has been reached. The output of this does not
% hold when samplign a new goal, so the flag is reset after sampling new goal in 'updatePedContStates.m'.
if resetFlag.check_goal(trackletNo)
    if goal_bounding_box(1,1)~=inf
        % circular bounds on the goal location
        dispGoal = vecnorm(goalPos-pedPos,2,2);
        if dispGoal < 2 && ~strcmp(HybridState,'Crossing') 
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