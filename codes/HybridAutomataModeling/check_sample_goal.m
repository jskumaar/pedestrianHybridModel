%% This  function checks if the pedestrian has reached the current goal location and samples the next goal location

function [flag, goal] = check_sample_goal(trackletData, trackletNo, resetStates, Params, flag, check_goal, sample_goal)

% params
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;

% ped states
pedPosPixels = [trackletData.xCenter(end), trackletData.yCenter(end)]/(scaleFactor*orthopxToMeter);
HybridState = trackletData.HybridState(end);
cwInd = trackletData.closestCW(end);
Lane = trackletData.Lane(end);

% initialize
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

if strcmp(HybridState,'Approach')
    goal_bounding_box = reshape(resetStates.approach.goal(swInd,3:end), [2,4])';
elseif strcmp(HybridState,'Wait')
    goal_bounding_box = reshape(resetStates.wait.goal(swInd,3:end), [2,4])';
elseif strcmp(HybridState,'Walkaway')
    goal_bounding_box = reshape(resetStates.walkaway.goal(swInd,3:end), [2,4])';
end
%%%%%%%%%%%%%%%%%%%%%%%%
% check if goal bounding box has been reached
if check_goal
    if goal_bounding_box(1,1)~=inf
        for ii=1:4
           if abs(pedPosPixels(1)) < abs(goal_bounding_box(ii,1)) &&  abs(pedPosPixels(2)) < abs(goal_bounding_box(ii,2)) 
               insideBB(ii) = 1;
           else
               insideBB(ii) = 0;
           end
        end

        if sum(insideBB)==0 || sum(insideBB)==4
            flag.reachGoal(trackletNo) = true;
        else
            flag.reachGoal(trackletNo) = false;
        end
    else
        flag.reachGoal(trackletNo) = false;
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% reset and sample a goal location
elseif sample_goal
    goal_x = rand*[max(goal_bounding_box(:,1)) - min(goal_bounding_box(:,1))] + min(goal_bounding_box(:,1));
    goal_y = rand*[max(goal_bounding_box(:,2)) - min(goal_bounding_box(:,2))] + min(goal_bounding_box(:,2));
    goal = [goal_x, goal_y];
end



end