%% This script finds the index of the closest car (or the ego car)

function pedData = egoCarFunc(currentPedTrackData, activeCarTracksData, cw, annotatedImageEnhanced, Params )

%parameters
movingThreshold =  Params.movingThreshold; % m/s
headingThreshold =  Params.headingThreshold; %90 degrees
scaleDownFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;

% initialize variables
imgSize = size(annotatedImageEnhanced);
imgCenter = int32([imgSize(2)/2; -imgSize(1)/2]);
longDispCarPixels = inf;
latDispCwPixels = inf;
closeCar_ind = inf;
cwCar = inf;
longDispPedCwPixels = inf;
isLooking = false;
pedHeading = currentPedTrackData.calcHeading(end);
isPedSameDirection = false;

cwPed = currentPedTrackData.closestCW(end);
pedPosPixels = double([currentPedTrackData.xCenterPix(end), currentPedTrackData.yCenterPix(end)]);  
%rotation matrices for the different crosswalks
theta = cw.theta;

%% pedestrian states
if cwPed~=0
    cw_ped_rot = [cosd(theta(cwPed)), -sind(theta(cwPed)); sind(theta(cwPed)), cosd(theta(cwPed))];
    cw_PosPixels = double([cw.center_x(cwPed), cw.center_y(cwPed)]);
    % distance between pedestrian and the approaching crosswalk in the rotated
    % frame
    pedCwAngle = atan2(double([cw.center_y(cwPed) - pedPosPixels(2)]), double([cw.center_x(cwPed) - pedPosPixels(1)]))*180/pi;
    pedPosPixelsRot = (cw_ped_rot * (pedPosPixels'  - double(imgCenter)) + double(imgCenter));
    cwPosPixelsRot = (cw_ped_rot * (cw_PosPixels'  - double(imgCenter)) + double(imgCenter));
    dispPedCwPixels = pedPosPixelsRot - cwPosPixelsRot;

    % longitudinal displacement to cw center 
    if cwPed==1 || cwPed==2 
        if  ( abs(pedCwAngle - pedHeading) <= headingThreshold )
            longDispPedCwPixels = abs(dispPedCwPixels(1));
        else
            longDispPedCwPixels = -abs(dispPedCwPixels(1));
        end
    else
        if  ( abs(pedCwAngle - pedHeading) <= headingThreshold )
            longDispPedCwPixels = abs(dispPedCwPixels(2));
        else
            longDispPedCwPixels = -abs(dispPedCwPixels(2));
        end
    end
    % lateral displacement to cw center 
    if cwPed==1
        latDispCwPixels  = abs(dispPedCwPixels(2)) - cw.centerLatOffset(1);
    elseif cwPed==2
        latDispCwPixels  = abs(dispPedCwPixels(2)) - cw.centerLatOffset(2);
    elseif cwPed==3
        latDispCwPixels  = abs(dispPedCwPixels(1)) - cw.centerLatOffset(3);
    else
        latDispCwPixels  = abs(dispPedCwPixels(1)) - cw.centerLatOffset(4);
    end
end



%% for every car that is interacting with the pedestrian at this time step find states    
for carInd = 1:size(activeCarTracksData,1)
    cwCar = activeCarTracksData.closestCW(carInd);
    carPosPixels = double([activeCarTracksData.xCenterPix(carInd), activeCarTracksData.yCenterPix(carInd)]);    
    % initialize variables
    carHeading = activeCarTracksData.calcHeading(carInd);
    longDispPedCarPixelsTemp = inf;
    
    % 1) car heading: to reduce noise in the estimation of heading because of noisy position and velocity data
    y_vel = activeCarTracksData.yVelocity(carInd);
    if ( abs(y_vel) < movingThreshold )
        y_vel = 0;
    end
    x_vel = activeCarTracksData.xVelocity(carInd);
    if ( abs(x_vel) < movingThreshold )
        x_vel = 0;
    end
    % if the car is stopped, maintain the previous heading
    if x_vel~=0 || y_vel~=0 
        carHeading = atan2(y_vel, x_vel)*180/pi;
    end
       
    %% check if the pedestrian is looking at the car
     isLookingTemp = gazeCheck(carPosPixels, pedPosPixels, carHeading, pedHeading);
     isLooking = isLooking || isLookingTemp;
     
    %% check if the pedestrian is heading in the same direction as the ego-vehicle or the opposite direction
    if abs(pedHeading - carHeading) < 45    
        isPedSameDirection = true;
    else
        isPedSameDirection = false;
    end
        
    %% if the vehicle is close to crosswalk; % find the distance between the car and pedestrian based on the CW, the car is approaching
    if (cwCar~=0 && cwCar~=inf)
        cwCarRot = [cosd(theta(cwCar)), -sind(theta(cwCar)); sind(theta(cwCar)), cosd(theta(cwCar))];
        carPosPixelsRot = (cwCarRot * (carPosPixels'  - double(imgCenter)) + double(imgCenter));
    

        % % longitudinal displacement to ego-car; later cars that have passed the
        % pedestrian are discarded (considers both near lane and far lane
        % vehicles in both directions)
        if (cwPed==1 && (cwCar==1 || cwCar==2) )
            disp_ped_car_pixels = carPosPixelsRot - pedPosPixelsRot;
            if  ( strcmp(activeCarTracksData.car_lane{carInd},'East_Right') || strcmp(activeCarTracksData.car_lane{carInd},'West_Left') )
                longDispPedCarPixelsTemp = disp_ped_car_pixels(1);
            else
                longDispPedCarPixelsTemp = -disp_ped_car_pixels(1);
            end
        end

        if (cwPed==2 && (cwCar==1 || cwCar==2) )
            disp_ped_car_pixels = carPosPixelsRot - pedPosPixelsRot;
            if  ( strcmp(activeCarTracksData.car_lane{carInd},'West_Right') || strcmp(activeCarTracksData.car_lane{carInd},'East_Left') )
                longDispPedCarPixelsTemp = -disp_ped_car_pixels(1);
            else
                longDispPedCarPixelsTemp = disp_ped_car_pixels(1);
            end        
        end

        if (cwPed==3 && (cwCar==3 || cwCar==4) )
            disp_ped_car_pixels = carPosPixelsRot - pedPosPixelsRot;
            if  ( strcmp(activeCarTracksData.car_lane{carInd},'South_Right') || strcmp(activeCarTracksData.car_lane{carInd},'North_Left') )
                longDispPedCarPixelsTemp = -disp_ped_car_pixels(2);
            else
                longDispPedCarPixelsTemp = disp_ped_car_pixels(2);
            end
        end

        if (cwPed==4 && (cwCar==3 || cwCar==4) )  
            disp_ped_car_pixels = carPosPixelsRot - pedPosPixelsRot;
            if  ( strcmp(activeCarTracksData.car_lane{carInd},'North_Right') || strcmp(activeCarTracksData.car_lane{carInd},'South_Left') )
                longDispPedCarPixelsTemp = disp_ped_car_pixels(2);
            else
                longDispPedCarPixelsTemp = -disp_ped_car_pixels(2);
            end
        end

        % find the ego-vehicle:
        % Is the car approaching the pedestrian? Do not consider cars that have
        % crossed the pedestrian.
        % Is this car closer than the previous closest car?
        if ( (longDispPedCarPixelsTemp > 0) && (longDispPedCarPixelsTemp < longDispCarPixels) )
            longDispCarPixels = longDispPedCarPixelsTemp;
            closeCar_ind = carInd;           
        end   
    end % end of loop for car aproaching a crosswalk

end  % end of loop for all active cars interacting with the pedestrian at this time step

% save the variables
pedData.long_disp_ped_car = longDispCarPixels*(scaleDownFactor*orthopxToMeter);
pedData.latDispPedCw = latDispCwPixels*(scaleDownFactor*orthopxToMeter);
pedData.longDispPedCw = longDispPedCwPixels*(scaleDownFactor*orthopxToMeter);
pedData.closeCar_ind = closeCar_ind;
pedData.cw_car = cwCar;
pedData.cw_ped = cwPed;
pedData.isLooking = isLooking;
%pedData.calcPedHeading = pedHeading;
%pedData.calcCarHeading = carHeading;
pedData.isPedSameDirection = isPedSameDirection;

end  % end of the function





