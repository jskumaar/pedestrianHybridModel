%% walkaway reset and when approaching another crosswalk

function goal = walkawayReset(trackletData, reset, walkawayDirection)

% parameters
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
% states
HybridState = trackletData.HybridState;
cwInd = trackletData.closestCW(end);
Lane = trackletData.Lane(end);
pedPosPixels = [trackletData.xCenter(end), trackletData.xCenter(end)] * scaleFactor * orthopxToMeter;

if cwInd==1 && strcmp(Lane,'Right') 
        pedGoalPixels = reset.approach.goal(8,:);
        pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
        calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;












end



