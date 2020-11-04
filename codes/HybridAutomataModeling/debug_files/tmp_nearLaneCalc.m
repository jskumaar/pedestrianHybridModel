%% This script compiles additional tracks data; should be later itegrated to the hybridState function

N_Scenes = 12;
imgSize = size(annotatedImageEnhanced);
imgCenter = int32([imgSize(2)/2; -imgSize(1)/2]);
scaleDownFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
movingThreshold =  Params.movingThreshold; % m/s
headingThreshold =  Params.cwHeadingThreshold; %90 degrees
theta = cw.theta;
reSampleRate = 5;


for sceneId = 1:N_Scenes
   allPedTracks = [tracks{sceneId}.pedCrossingTracks; tracks{sceneId}.pedNotCrossingTracks];
   N_pedTracks = length(allPedTracks);
   
   for trackNo = 1:N_pedTracks
       pedTrackId = allPedTracks(trackNo);
       
       pedData = formattedTracksData{sceneId}{pedTrackId};
       pedData.isNearLane = false(size(pedData.frame));
       N_TimeSteps = size(pedData.trackLifetime,1);
             
       for pedTimeStep  = 1:N_TimeSteps
           
              %% Pedestrian wait time calculation
              if strcmp(formattedTracksData{sceneId}{pedTrackId}.class(1), 'pedestrian')
                    waitTimeSteps  = cumsum(formattedTracksData{sceneId}{pedTrackId}.ProbHybridState(:,2)) * reSampleRate;
                    formattedTracksData{sceneId}{pedTrackId}.waitTimeSteps = waitTimeSteps;
                    if isfield(formattedTracksData{sceneId}{pedTrackId},'wait_time_steps') 
                        formattedTracksData{sceneId}{pedTrackId}.wait_time_steps = []; 
                    end
              end
%              
%               
              %% Near lane calculation
             
              %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
               if (isfield(pedData,'closeCar_ind'))
                   
                  if pedData.closestCW(pedTimeStep) == 1
                      
                       distPedRightLane = norm(resetStates.carCW.goal(1,:) - pedPos);
                       distPedLeftLane = norm(resetStates.carCW.goal(2,:) - pedPos);

                       if distPedRightLane < distPedLeftLane
%                             pedData.Lane(pedTimeStep, :) = "Right";

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
%                             pedData.Lane(pedTimeStep, :) = "Left";

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
                       
                       
                       distPedRightLane = norm(resetStates.carCW.goal(3,:) - pedPos);
                       distPedLeftLane = norm(resetStates.carCW.goal(4,:) - pedPos);

                       if distPedRightLane < distPedLeftLane
%                             pedData.Lane(pedTimeStep, :) = "Right";
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
%                             pedData.Lane(pedTimeStep, :) = "Left";
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
                      
                      
                       distPedRightLane = norm(resetStates.carCW.goal(5,:) - pedPos);
                       distPedLeftLane = norm(resetStates.carCW.goal(6,:) - pedPos);

                       if distPedRightLane < distPedLeftLane
%                             pedData.Lane(pedTimeStep, :) = "Right";
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
%                             pedData.Lane(pedTimeStep, :) = "Left";
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
                      
                       distPedRightLane = norm(resetStates.carCW.goal(7,:) - pedPos);
                       distPedLeftLane = norm(resetStates.carCW.goal(8,:) - pedPos);

                       if distPedRightLane < distPedLeftLane
%                             pedData.Lane(pedTimeStep, :) = "Right";
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
%                             pedData.Lane(pedTimeStep, :) = "Left";
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
       if isfield(pedData,'lat_disp_ped_cw')
        pedData.lat_disp_ped_cw = [];
       end
       if isfield(pedData,'long_disp_ped_cw')
        pedData.long_disp_ped_cw = [];
       end
       formattedTracksData{sceneId}{pedTrackId} = pedData;

   end

end