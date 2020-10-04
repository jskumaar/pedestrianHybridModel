%% This script compiles additional tracks data; should be later itegrated to the hybridState function

N_Scenes = 12;
imgSize = size(annotatedImageEnhanced);
imgCenter = int32([imgSize(2)/2; -imgSize(1)/2]);
scaleDownFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
movingThreshold =  Params.movingThreshold; % m/s
headingThreshold =  Params.headingThreshold; %90 degrees
theta = cw.theta;
reSampleRate = 5;

load('inD_trackDescriptives_removed_ped_tracks_v2.mat') 
tracks = tracksUpdated;


for sceneId = 1:N_Scenes
   allPedTracks = [tracks{sceneId}.pedCrossingTracks; tracks{sceneId}.pedNotCrossingTracks];
   N_pedTracks = length(allPedTracks);
   
   for trackNo = 1:N_pedTracks
       pedTrackId = allPedTracks(trackNo);
       
       pedData = formattedTracksData{sceneId}{pedTrackId};
       N_TimeSteps = size(pedData.trackLifetime,1);
             
       for pedTimeStep  = 1:N_TimeSteps
           
              pedPos = [pedData.xCenterPix(pedTimeStep), pedData.yCenterPix(pedTimeStep)];
              cwPed = pedData.closestCW(pedTimeStep);
              pedHeading = pedData.calcHeading(pedTimeStep);
              
              % initialize
              longDispCwPixels = inf;
              latDispCwPixels = inf;
              
%               %% Pedestrian wait time calculation
%               if strcmp(formattedTracksData{sceneId}{pedTrackId}.class(1), 'pedestrian')
%                     waitTimeSteps  = cumsum(formattedTracksData{sceneId}{pedTrackId}.ProbHybridState(:,2)) * reSampleRate;
%                     formattedTracksData{sceneId}{pedTrackId}.waitTimeSteps = waitTimeSteps;
%                     if sum(ismember(formattedTracksData{sceneId}{pedTrackId}.Properties.VariableNames,'wait_time_steps')) 
%                         formattedTracksData{sceneId}{pedTrackId}.wait_time_steps = []; 
%                     end
%               end
              
              %% Pedestrian distance calculation (earlier distance was calculated only when there was am ego-vehicle)
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              if cwPed~=0
                    cwPedRot = [cosd(theta(cwPed)), -sind(theta(cwPed)); sind(theta(cwPed)), cosd(theta(cwPed))];
                    cwPosPixels = double([cw.center_x(cwPed), cw.center_y(cwPed)]);
                    % distance between pedestrian and the approaching crosswalk in the rotated
                    % frame
                    pedCwAngle = atan2(double([cw.center_y(cwPed) - pedPos(2)]), double([cw.center_x(cwPed) - pedPos(1)]))*180/pi;
                    pedPosRot = cwPedRot * (pedPos'  - double(imgCenter)) + double(imgCenter);
                    cwPosPixelsRot = cwPedRot * (cwPosPixels'  - double(imgCenter)) + double(imgCenter);
                    dispPedCwPixels = pedPosRot - cwPosPixelsRot;

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
                    
                    pedData.latDispPedCw(pedTimeStep) = latDispCwPixels*(scaleDownFactor*orthopxToMeter);
                    pedData.longDispPedCw(pedTimeStep) = longDispPedCwPixels*(scaleDownFactor*orthopxToMeter);
                   
                    if longDispPedCwPixels < 10 % less than a meter
                        % plot
                        figure()
                        plot(cw.center_x, cw.center_y, 'b*', 'MarkerSize',10); hold on;
                        plot(pedPos(1), pedPos(2), 'r*', 'MarkerSize',10); hold on;
                        x=1;
                    end
                       
              
              
              
              
              
              end
              
              %% Near lane calculation
             
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               if sum(ismember(pedData.Properties.VariableNames,'closeCar_ind'))
                   
                  if pedData.closestCW(pedTimeStep) == 1
                      sceneId
                      trackNo
                      pedTimeStep
                      
                       distPedRightLane = norm(resetStates.carCW.goal(1,:) - pedPos);
                       distPedLeftLane = norm(resetStates.carCW.goal(2,:) - pedPos);

                       if distPedRightLane < distPedLeftLane
                            pedData.Lane(pedTimeStep, :) = "Right";

                            if pedData.closeCar_ind(pedTimeStep) ~=0 && pedData.closeCar_ind(pedTimeStep) ~=inf
                                carInd = formattedTracksData{sceneId}{pedTrackId}.closeCar_ind(pedTimeStep);
                                carData = formattedTracksData{sceneId}{carInd};
                                carTimeStep = find(carData.frame==pedData.frame(pedTimeStep));

                                if (strcmp(carData.car_lane(carTimeStep), 'East_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'West_Left') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'North_Left'))
                                    pedData.isNearLane(pedTimeStep) = true;
                                else
                                    pedData.isNearLane(pedTimeStep) = false;
                                end
                            end
                       else
                            pedData.Lane(pedTimeStep, :) = "Left";

                            if pedData.closeCar_ind(pedTimeStep) ~=0 && pedData.closeCar_ind(pedTimeStep) ~=inf
                                carInd = formattedTracksData{sceneId}{pedTrackId}.closeCar_ind(pedTimeStep);
                                carData = formattedTracksData{sceneId}{carInd};
                                carTimeStep = find(carData.frame==pedData.frame(pedTimeStep));
                                if (strcmp(carData.car_lane(carTimeStep), 'East_Left') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'West_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'North_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'South_Right') )
                                    pedData.isNearLane(pedTimeStep) = true;
                                else
                                    pedData.isNearLane(pedTimeStep) = false;
                                end
                            end
                       end

                   elseif pedData.closestCW(pedTimeStep) == 2
                       
                       sceneId
                      trackNo
                      pedTimeStep
                       
                       distPedRightLane = norm(resetStates.carCW.goal(3,:) - pedPos);
                       distPedLeftLane = norm(resetStates.carCW.goal(4,:) - pedPos);

                       if distPedRightLane < distPedLeftLane
                            pedData.Lane(pedTimeStep, :) = "Right";
                            if pedData.closeCar_ind(pedTimeStep) ~=0 && pedData.closeCar_ind(pedTimeStep) ~=inf
                                carInd = formattedTracksData{sceneId}{pedTrackId}.closeCar_ind(pedTimeStep);
                                carData = formattedTracksData{sceneId}{carInd};
                                carTimeStep = find(carData.frame==pedData.frame(pedTimeStep));
                                if (strcmp(carData.car_lane(carTimeStep), 'East_Left') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'West_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'North_Right'))
                                    pedData.isNearLane(pedTimeStep) = true;
                                else
                                    pedData.isNearLane(pedTimeStep) = false;
                                end
                            end
                       else
                            pedData.Lane(pedTimeStep, :) = "Left";
                            if pedData.closeCar_ind(pedTimeStep) ~=0 && pedData.closeCar_ind(pedTimeStep) ~=inf
                                carInd = formattedTracksData{sceneId}{pedTrackId}.closeCar_ind(pedTimeStep);
                                carData = formattedTracksData{sceneId}{carInd};
                                carTimeStep = find(carData.frame==pedData.frame(pedTimeStep));
                                if (strcmp(carData.car_lane(carTimeStep), 'East_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'West_Left') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'North_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'South_Right') )
                                    pedData.isNearLane(pedTimeStep) = true;
                                else
                                    pedData.isNearLane(pedTimeStep) = false;
                                end
                            end
                       end



                   elseif pedData.closestCW(pedTimeStep) == 3
                       
                       sceneId
                      trackNo
                      pedTimeStep
                      
                       distPedRightLane = norm(resetStates.carCW.goal(5,:) - pedPos);
                       distPedLeftLane = norm(resetStates.carCW.goal(6,:) - pedPos);

                       if distPedRightLane < distPedLeftLane
                            pedData.Lane(pedTimeStep, :) = "Right";
                            if pedData.closeCar_ind(pedTimeStep) ~=0 && pedData.closeCar_ind(pedTimeStep) ~=inf
                                carInd = formattedTracksData{sceneId}{pedTrackId}.closeCar_ind(pedTimeStep);
                                carData = formattedTracksData{sceneId}{carInd};
                                carTimeStep = find(carData.frame==pedData.frame(pedTimeStep));
                                if (strcmp(carData.car_lane(carTimeStep), 'East_Left') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'West_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'South_Right'))
                                    pedData.isNearLane(pedTimeStep) = true;
                                else
                                    pedData.isNearLane(pedTimeStep) = false;
                                end
                            end
                       else
                            pedData.Lane(pedTimeStep, :) = "Left";
                            if pedData.closeCar_ind(pedTimeStep) ~=0 && pedData.closeCar_ind(pedTimeStep) ~=inf
                                carInd = formattedTracksData{sceneId}{pedTrackId}.closeCar_ind(pedTimeStep);
                                carData = formattedTracksData{sceneId}{carInd};
                                carTimeStep = find(carData.frame==pedData.frame(pedTimeStep));
                                if (strcmp(carData.car_lane(carTimeStep), 'East_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'West_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'North_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'South_Left') )
                                    pedData.isNearLane(pedTimeStep) = true;
                                else
                                    pedData.isNearLane(pedTimeStep) = false;
                                end
                            end
                       end


                   elseif pedData.closestCW(pedTimeStep) == 4
                        
                       sceneId
                      trackNo
                      pedTimeStep
                      
                       distPedRightLane = norm(resetStates.carCW.goal(7,:) - pedPos);
                       distPedLeftLane = norm(resetStates.carCW.goal(8,:) - pedPos);

                       if distPedRightLane < distPedLeftLane
                            pedData.Lane(pedTimeStep, :) = "Right";
                            if pedData.closeCar_ind(pedTimeStep) ~=0 && pedData.closeCar_ind(pedTimeStep) ~=inf
                                carInd = formattedTracksData{sceneId}{pedTrackId}.closeCar_ind(pedTimeStep);
                                carData = formattedTracksData{sceneId}{carInd};
                                carTimeStep = find(carData.frame==pedData.frame(pedTimeStep));
                                if (strcmp(carData.car_lane(carTimeStep), 'East_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'West_Left') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'North_Right'))
                                    pedData.isNearLane(pedTimeStep) = true;
                                else
                                    pedData.isNearLane(pedTimeStep) = false;
                                end
                            end
                       else
                            pedData.Lane(pedTimeStep, :) = "Left";
                            if pedData.closeCar_ind(pedTimeStep) ~=0 && pedData.closeCar_ind(pedTimeStep) ~=inf
                                carInd = formattedTracksData{sceneId}{pedTrackId}.closeCar_ind(pedTimeStep);
                                carData = formattedTracksData{sceneId}{carInd};
                                carTimeStep = find(carData.frame==pedData.frame(pedTimeStep));
                                if (strcmp(carData.car_lane(carTimeStep), 'East_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'West_Right') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'North_Left') ||...
                                    strcmp(carData.car_lane(carTimeStep), 'South_Right') )
                                    pedData.isNearLane(pedTimeStep) = true;
                                else
                                    pedData.isNearLane(pedTimeStep) = false;
                                end
                            end
                       end


                  end
               end
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
           
       end
       if sum(ismember(pedData.Properties.VariableNames,'lat_disp_ped_cw'))
        pedData.lat_disp_ped_cw = [];
       end
       if sum(ismember(pedData.Properties.VariableNames,'long_disp_ped_cw'))
        pedData.long_disp_ped_cw = [];
       end
       formattedTracksData{sceneId}{pedTrackId} = pedData;

   end

end