%% nearLane calculator
% Note: While calcultaing the near lane in the formattedTracksData, the
% closest CW was used to identify the lane. However, later, for sampling
% goal, we use the combination of swInd and Lane and to maintain
% consistency, we use swInd here instead.


% if currentPedTrackData.swInd(end) == 1

if currentPedTrackData.swInd(end) == 1

distPedRightLane = norm(reset.carCW.goal(1,:) - pedPosPixels);
distPedLeftLane = norm(reset.carCW.goal(2,:) - pedPosPixels);

    if distPedRightLane < distPedLeftLane
        Lane = "Right";

        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Left'))
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    else
        Lane = "Left";

        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Right') )
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    end

elseif currentPedTrackData.swInd(end) == 2

    distPedRightLane = norm(reset.carCW.goal(3,:) - pedPosPixels);
    distPedLeftLane = norm(reset.carCW.goal(4,:) - pedPosPixels);

    if distPedRightLane < distPedLeftLane
        Lane = "Right";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right'))
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    else
        Lane = "Left";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Right') )
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    end



elseif currentPedTrackData.swInd(end) == 3

    distPedRightLane = norm(reset.carCW.goal(5,:) - pedPosPixels);
    distPedLeftLane = norm(reset.carCW.goal(6,:) - pedPosPixels);

    if distPedRightLane < distPedLeftLane
        Lane = "Right";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Right'))
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    else
        Lane = "Left";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Left') )
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    end


elseif currentPedTrackData.swInd(end) == 4
    
    distPedRightLane = norm(reset.carCW.goal(7,:) - pedPosPixels);
    distPedLeftLane = norm(reset.carCW.goal(8,:) - pedPosPixels);

    if distPedRightLane < distPedLeftLane
        Lane = "Right";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Right'))
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    else
        Lane = "Left";
        if currentPedTrackData.closeCar_ind(end) ~=0 && currentPedTrackData.closeCar_ind(end) ~=inf
            if (strcmp(egoCarData.car_lane(egoCarTimeStep), 'East_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'West_Right') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'North_Left') ||...
                strcmp(egoCarData.car_lane(egoCarTimeStep), 'South_Right') )
                currentPedTrackData.isNearLane(end) = true;
            else
                currentPedTrackData.isNearLane(end) = false;
            end
        end
    end


end

% update lane
currentPedTrackData.Lane(end) = Lane;