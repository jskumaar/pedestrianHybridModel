%% This script finds the index of the closest car (or the ego car)

%% 1) setup
%parameters
movingThreshold =  Params.movingThreshold; % m/s
headingThreshold =  Params.cwHeadingThreshold; %90 degrees
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;

% initialize variables
imgSize = size(annotatedImageEnhanced);
imgCenter = int32([imgSize(2)/2; -imgSize(1)/2]);
%rotation angles for the different crosswalks
theta = cw.theta;

for sceneId = 1:12
    N_tracks = size(formattedTracksData{sceneId},1);
    car_moving_tracks = tracks{sceneId}.carMovingTracks;
    tracksMetaData_scene = tracksMetaData{sceneId};
    tracksMeta_cars = tracksMetaData_scene(car_moving_tracks,:);
    
    for trackId = 1:N_tracks
        if strcmp(tracksMetaData{sceneId}.class(trackId), 'pedestrian')
               pedData = formattedTracksData{sceneId}{trackId};
               N_ts = size(pedData.frame,1);
               prevCWDispSign = 0;
               prevCWLatDispSign = 0;
               
               for timeStep = 1:N_ts
                                      
                   isLookingTemp = false;
                   % pedtrack time
                   pedTrackTime = pedData.frame(timeStep);
                   % find the active cars in this time step
                   temp_car_activeId = find(tracksMeta_cars.initialFrame <= pedTrackTime & tracksMeta_cars.finalFrame >= pedTrackTime);
                   car_activeId = car_moving_tracks(temp_car_activeId);
                   
                   
                   for carLoopId = 1:length(car_activeId)
                       carIndex = car_activeId(carLoopId);
                           carTrackCurrentTimeStep(carLoopId) = (pedTrackTime - formattedTracksData{sceneId}{carIndex}.frame(1))/Params.reSampleRate + 1;                                             
                   end

                    % intialize
                    longDispCarPixels = inf;
                    latDispCwPixels = inf;
                    closeCar_ind = inf;
                    activeCar_ind = inf;
                    cwCar = inf;
                    longDispPedCwPixels = inf;
                    isLooking = false;
                    pedHeading = pedData.calcHeading(timeStep);
                    isPedSameDirection = false;
                    cwPed = pedData.closestCW(timeStep);
                    pedPos = [pedData.xCenter(timeStep), pedData.yCenter(timeStep)];  
                    pedPosPixels = pedPos/(orthopxToMeter*scaleFactor);  
                    pedVel = [pedData.xVelocity(timeStep), pedData.yVelocity(timeStep)];  
                    lonVelocity = pedData.lonVelocity(timeStep);
    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    %% 2) pedestrian states
                    if (cwPed~=0 && cwPed~=inf)
                        cw_ped_rot = [cosd(theta(cwPed)), -sind(theta(cwPed)); sind(theta(cwPed)), cosd(theta(cwPed))];
                        cw_PosPixels = double([cw.center_x(cwPed), cw.center_y(cwPed)]);
                        % distance between pedestrian and the approaching crosswalk in the rotated
                        % frame
                        pedCwAngle = atan2(double(cw.center_y(cwPed) - pedPosPixels(2)), double(cw.center_x(cwPed) - pedPosPixels(1)))*180/pi;
                        pedPosPixelsRot = (cw_ped_rot * (pedPosPixels'  - double(imgCenter)) + double(imgCenter));
                        cwPosPixelsRot  = (cw_ped_rot * (cw_PosPixels'  - double(imgCenter)) + double(imgCenter));
                        dispPedCwPixels = pedPosPixelsRot - cwPosPixelsRot;
                        %%%%%%%%%%%%%%%%%%%%%%%%
                        % find pedestrian longitudinal velocity
                        cwPedRot = [cosd(theta(cwPed)), -sind(theta(cwPed)); sind(theta(cwPed)), cosd(theta(cwPed))];
                        velRot = cwPedRot * pedVel';  % post-multiplication; this should not work (should do pre-multiplication), but for some reason matches with the longitudinal velocity provided by the dataset..
                        lonVelocity = velRot(1);
                        %%%%%%%%%%%%%%%%%%%%%%%%
                        
                       dispRot_cw1 = [cosd(cw.theta(1)), -sind(cw.theta(1)); sind(cw.theta(1)), cosd(cw.theta(1))] * ([cw.center_x(1),cw.center_y(1)] - pedPosPixels)';
                       dispRot_cw2 = [cosd(cw.theta(2)), -sind(cw.theta(2)); sind(cw.theta(2)), cosd(cw.theta(2))] * ([cw.center_x(2),cw.center_y(2)] - pedPosPixels)'; 
                       dispRot_cw3 = [cosd(cw.theta(3)), -sind(cw.theta(3)); sind(cw.theta(3)), cosd(cw.theta(3))] * ([cw.center_x(3),cw.center_y(3)] - pedPosPixels)';
                       dispRot_cw4 = [cosd(cw.theta(4)), -sind(cw.theta(4)); sind(cw.theta(4)), cosd(cw.theta(4))] * ([cw.center_x(4),cw.center_y(4)] - pedPosPixels)';

                       % shift to align the long. disp. along column 1
                       dispRot_cw3 = [dispRot_cw3(2); dispRot_cw3(1)];
                       dispRot_cw4 = [dispRot_cw4(2); dispRot_cw4(1)];
                       dispRot_cw = [dispRot_cw1'; dispRot_cw2'; dispRot_cw3'; dispRot_cw4'];

                        % longitudinal displacement to cw center 
%                         if cwPed==1 || cwPed==2 
%                             if  ( abs(pedCwAngle - pedHeading) <= headingThreshold )
%                                 longDispPedCwPixels = abs(dispPedCwPixels(1));
%                             else
%                                 longDispPedCwPixels = -abs(dispPedCwPixels(1));
%                             end
%                         else
%                             if  ( abs(pedCwAngle - pedHeading) <= headingThreshold )
%                                 longDispPedCwPixels = abs(dispPedCwPixels(2));
%                             else
%                                 longDispPedCwPixels = -abs(dispPedCwPixels(2));
%                             end
%                         end
%                         
                        

                        
                        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         % simpler method
%                         if  (sign(dispRot_cw(cwPed,1))== prevCWDispSign) || (timeStep==1 && strcmp(pedData.HybridState(timeStep), 'Walk_away'))
%                             longDispPedCwPixels = abs(dispRot_cw(cwPed,1));
%                         else
%                             longDispPedCwPixels = -abs(dispRot_cw(cwPed,1));
%                         end

                        %%%%%%%%%%%%%%%%%%%%%%%%%
%                         % lateral displacement to cw center 
%                         if cwPed==1
%                             latDispCwPixels  = abs(dispPedCwPixels(2)) - cw.centerLatOffset(1);
%                         elseif cwPed==2
%                             latDispCwPixels  = abs(dispPedCwPixels(2)) - cw.centerLatOffset(2);
%                         elseif cwPed==3
%                             latDispCwPixels  = abs(dispPedCwPixels(1)) - cw.centerLatOffset(3);
%                         else
%                             latDispCwPixels  = abs(dispPedCwPixels(1)) - cw.centerLatOffset(4);
%                         end
%                         
%                         %%%%%%%%%%%%%%%%%
%                         if (sign(dispRot_cw(cwPed,2))== prevCWLatDispSign)
%                             latDispCwPixels = abs(dispRot_cw(cwPed,2));
%                         else
%                             latDispCwPixels = -abs(dispRot_cw(cwPed,2));
%                         end


                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                    %% 3) for every car that is interacting with the pedestrian at this time step find states    
                    for carLoopInd = 1:size(car_activeId,1)
                        carTrackId = car_activeId(carLoopInd);
                        carData = formattedTracksData{sceneId}{carTrackId};
                        carTimeStep = carTrackCurrentTimeStep(carLoopId);
                        cwCar = carData.closestCW(carTimeStep);
                        carPosPixels = [carData.xCenter(carTimeStep), carData.yCenter(carTimeStep)]/(orthopxToMeter*scaleFactor);    
                        
%                         sceneId
%                         trackId
%                         timeStep
%                         carLoopId
                        
                        % initialize variables
                        carHeading = carData.calcHeading(carTimeStep);
                        longDispPedCarPixelsTemp = inf;
                        %%%%%%%%%%%%%%%%%%%%%%%%
                        % 1) car heading: to reduce noise in the estimation of heading because of noisy position and velocity data
                        y_vel = carData.yVelocity(carTimeStep);
                        if ( abs(y_vel) < movingThreshold )
                            y_vel = 0;
                        end
                        x_vel = carData.xVelocity(carTimeStep);
                        if ( abs(x_vel) < movingThreshold )
                            x_vel = 0;
                        end
                        % if the car is stopped, maintain the previous heading
                        if x_vel~=0 || y_vel~=0 
                            carHeading = atan2(y_vel, x_vel)*180/pi;
                        end
                        %%%%%%%%%%%%%%%%%%%%%%%%  
                        % check if the pedestrian is looking at the car
                         isLookingTemp = gazeCheck(carPosPixels, pedPosPixels, carHeading, pedHeading);
                         isLooking = isLooking || isLookingTemp;
                        %%%%%%%%%%%%%%%%%%%%%%%%
                        % check if the pedestrian is heading in the same direction as the ego-vehicle or the opposite direction
                        if abs(pedHeading - carHeading) < 45    
                            isPedSameDirection = true;
                        else
                            isPedSameDirection = false;
                        end
                        %%%%%%%%%%%%%%%%%%%%%%%%   
                        % if the vehicle is close to crosswalk; % find the distance between the car and pedestrian based on the CW, the car is approaching
                        if (cwCar~=0 && cwCar~=inf)
                            cwCarRot = [cosd(theta(cwCar)), -sind(theta(cwCar)); sind(theta(cwCar)), cosd(theta(cwCar))];
                            carPosPixelsRot = (cwCarRot * (carPosPixels'  - double(imgCenter)) + double(imgCenter));

                            % % longitudinal displacement to ego-car; later cars that have passed the
                            % pedestrian are discarded (considers both near lane and far lane
                            % vehicles in both directions)
                            if (cwPed==1 && (cwCar==1 || cwCar==2) )
                                disp_ped_car_pixels = carPosPixelsRot - pedPosPixelsRot;
                                if  ( strcmp(carData.car_lane(carTimeStep,:),'East_Right') || strcmp(carData.car_lane(carTimeStep,:),'West_Left') )
                                    longDispPedCarPixelsTemp = disp_ped_car_pixels(1);
                                else
                                    longDispPedCarPixelsTemp = -disp_ped_car_pixels(1);
                                end
                            end

                            if (cwPed==2 && (cwCar==1 || cwCar==2) )
                                disp_ped_car_pixels = carPosPixelsRot - pedPosPixelsRot;
                                if  ( strcmp(carData.car_lane(carTimeStep,:),'West_Right') || strcmp(carData.car_lane(carTimeStep,:),'East_Left') )
                                    longDispPedCarPixelsTemp = -disp_ped_car_pixels(1);
                                else
                                    longDispPedCarPixelsTemp = disp_ped_car_pixels(1);
                                end        
                            end

                            if (cwPed==3 && (cwCar==3 || cwCar==4) )
                                disp_ped_car_pixels = carPosPixelsRot - pedPosPixelsRot;
                                if  ( strcmp(carData.car_lane(carTimeStep,:),'South_Right') || strcmp(carData.car_lane(carTimeStep,:),'North_Left') )
                                    longDispPedCarPixelsTemp = -disp_ped_car_pixels(2);
                                else
                                    longDispPedCarPixelsTemp = disp_ped_car_pixels(2);
                                end
                            end

                            if (cwPed==4 && (cwCar==3 || cwCar==4) )  
                                disp_ped_car_pixels = carPosPixelsRot - pedPosPixelsRot;
                                if  ( strcmp(carData.car_lane(carTimeStep,:),'North_Right') || strcmp(carData.car_lane(carTimeStep,:),'South_Left') )
                                    longDispPedCarPixelsTemp = disp_ped_car_pixels(2);
                                else
                                    longDispPedCarPixelsTemp = -disp_ped_car_pixels(2);
                                end
                            end
                            %%%%%%%%%%%%%%%%%%%%%%%%
                            % find the ego-vehicle:
                            % Is the car approaching the pedestrian? Do not consider cars that have
                            % crossed the pedestrian.
                            % Is this car closer than the previous closest car?
                            if ( (longDispPedCarPixelsTemp > 0) && (longDispPedCarPixelsTemp < longDispCarPixels) )
                                longDispCarPixels = longDispPedCarPixelsTemp;
                                closeCar_ind = carTrackId;   
                                activeCar_ind = carLoopInd;
                            end  

                        end % end of loop for car aproaching a crosswalk
                        
                    end  % end of loop for all active cars interacting with the pedestrian at this time step
                   
                    if closeCar_ind==inf
                        x=1;
                    end
                    
                    % update
                    prevCWDispSign = sign(dispRot_cw(cwPed,1));
                    prevCWLatDispSign = sign(dispRot_cw(cwPed,2));
               
                    
                    %%%%%%%%%%%%%%%%%%%%%%
                    %% save the variables
                    pedData.long_disp_ped_car(timeStep) = longDispCarPixels*(scaleFactor*orthopxToMeter);
%                     pedData.latDispPedCw(timeStep) = latDispCwPixels*(scaleFactor*orthopxToMeter);
%                     pedData.longDispPedCw(timeStep) = longDispPedCwPixels*(scaleFactor*orthopxToMeter);
                    pedData.closeCar_ind(timeStep) = closeCar_ind;
                    pedData.isPedSameDirection(timeStep) = isPedSameDirection;
                    pedData.calcLonVelocity(timeStep) = lonVelocity;
                    pedData.isLooking(timeStep) = isLooking;

               end
               formattedTracksData{sceneId}{trackId} = pedData;
        end
    end
end
