%% plot 
% 
% % plot predictions
% N = size(predictionTrajectoryMatrix,1);
% prediction = reshape(predictionTrajectoryMatrix(:, 3:end)', [2,N*30])';
% figure()
% for ixx=1:N
%     plot(prediction((N-1)*30 + 1: N*30,1), prediction((N-1)*30 + 1: N*30,2), '*', 'MarkerSize', 8); hold on;
% %     plot(prediction(31:60,ii), prediction(31:60,2), 'r*', 'MarkerSize', 8); hold on;    
% end
%  
% 
% 
% plot tracks data
% xx = [1];
% xCenter  = tracksData.xCenter(xx);
% yCenter = tracksData.yCenter(xx);
% 
% xCenter  = tracksData.xCenter;
% yCenter = tracksData.yCenter;
% 
% 
% cw_x = cw.center_x*orthopxToMeter*scaleDownFactor;
% cw_y = cw.center_y*orthopxToMeter*scaleDownFactor;
% 
% figure()
% plot(xCenter, yCenter, '*', 'MarkerSize', 8);
% hold on;
% plot(cw_x, cw_y, 'r*','MarkerSize', 8);
% axis equal
% 
% 
% figure()
% plot(posPixels(1), posPixels(2), '*', 'MarkerSize', 8); hold on;
% plot(cw.center_x, cw.center_y, 'r*','MarkerSize', 8);
% axis equal


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sceneId = 1;
% pedTrackId = 27;
% pedTs = 4;
% carTrackId = 22;
% carTs = 38;
% 
% scaleDownFactor = Params.scaleFactor;
% orthopxToMeter = Params.orthopxToMeter;
% pedData =  formattedTracksData{sceneId}{pedTrackId};
% 
% %%
% xCenter  = pedData.xCenter(pedTs);
% yCenter = pedData.yCenter(pedTs);
% 
% % xCenter  = pedData.xCenter;
% % yCenter = pedData.yCenter;
% %%
% 
% cw_x = cw.center_x*orthopxToMeter*scaleDownFactor;
% cw_y = cw.center_y*orthopxToMeter*scaleDownFactor;
% 
% figure()
% plot(xCenter, yCenter, '*', 'MarkerSize', 8);
% hold on;
% plot(cw_x, cw_y, 'r*','MarkerSize', 8);hold on;
% axis equal
% 
% shift = 2;
% for ii=1:length(carTrackId)
%     carId = carTrackId(ii);
%     carData =   formattedTracksData{sceneId}{carId};
%     %%
%     xCenter  = carData.xCenter(carTs);
%     yCenter = carData.yCenter(carTs);
%     
% %     xCenter  = carData.xCenter;
% %     yCenter = carData.yCenter;
% %     %%
%     plot(xCenter, yCenter, '*', 'MarkerSize', 8);
%     text(xCenter(1), yCenter(1)-shift,strcat(num2str(carId),'-',num2str(carData.frame(1))));
%     hold on;
%     shift = shift+2;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %% plot tracklets
% % N_tracklets = size(trackletData,1);
scaleDownFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
sf = scaleDownFactor*orthopxToMeter;
cw_x = cw.center_x*sf;
cw_y = cw.center_y*sf;
shift = 0.5;

goals = reset.carCW.goal;
figure()
for id = 1:8
  
    % goal center
    goal_x =  [goals(id,1)]*sf;
    goal_y =  [goals(id,2)]*sf;
    plot(goal_x, goal_y, 'g*','MarkerSize', 8);hold on;
    text(goal_x(1), goal_y(1)-shift,strcat('AR-',num2str(id)));
   
    % plot crosswalk
    plot(cw_x, cw_y, 'r*','MarkerSize', 8);hold on;
    axis equal
    
%     % plot goal region
%     pgon = polyshape(goals(id,[3,7,5,9])*sf, goals(id,[4,8,6,10])*sf);
%     plot(pgon); hold on;
    pause(2)
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %% debug: plot predicted trajectories
% figure()
% for ii=1:size(predictionTrajectoryMatrix,1)
%     tempPredMatrix = reshape(predictionTrajectoryMatrix(ii,end-2*Params.predHorizon+1:end),[2, Params.predHorizon])';
%     plot(tempPredMatrix(:,1), tempPredMatrix(:,2), '*', 'MarkerSize', 10); hold on;
%     %     for jj=1:size(tempPredMatrix,1)
% %         plot(tempPredMatrix(jj,1), tempPredMatrix(jj,2), '*', 'MarkerSize', 10); hold on;
% %         pause(0.1)
% %     end
% end
%  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pedPosPixels = [750, -320];
% annotatedImageEnhanced(-int32(pedPosPixels(2)), int32(pedPosPixels(1))) = 255;
% imshow(annotatedImageEnhanced)

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [~,idx] = sortrows(MHP_good_ADE_performance_indices(:,6),'descend');
% MHP_good_ADE_sorted = MHP_good_ADE_performance_indices(idx,:);
% [~,idx2] = sortrows(MHP_good_FDE_performance_indices(:,6),'descend');
% MHP_good_FDE_sorted = MHP_good_FDE_performance_indices(idx2,:);
% 
% [~,idx] = sortrows(MHP_bad_ADE_performance_indices(:,6),'descend');
% MHP_bad_ADE_sorted = MHP_bad_ADE_performance_indices(idx,:);
% [~,idx2] = sortrows(MHP_bad_FDE_performance_indices(:,6),'descend');
% MHP_bad_FDE_sorted = MHP_bad_FDE_performance_indices(idx2,:);
% 
% % 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% sceneId = 1;
% car_index = 46;
% pedTrackId = 167;
% predictionTimeStep = 33;
% actualTimeStep = 75;
% 
% mostProbablePredictedTrajectory_MHP = [];
% mostProbablePredictedTrajectory_HBase = [];
% mostProbablePredictedTrajectory_CV = [];
% 
% pedPredictionsData_MHP = predictedPedTraj_MHP{sceneId}{car_index}{pedTrackId}.data; % the first data is a place holder, 'inf' value
% pedPredictionsData_HBase = predictedPedTraj_HBase{sceneId}{car_index}{pedTrackId}.data;
% pedPredictionsData_CV = predictedPedTraj_CV{sceneId}{car_index}{pedTrackId}.data;
% GTPedData = formattedTracksData{sceneId}{pedTrackId};
% 
% endTimeStep = min(actualTimeStep+30, length(GTPedData.xCenter));
% 
% 
% %% trajectories
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
% % MHP model: get the most probable trajectory
% N_futures_MHP = size(pedPredictionsData_MHP{predictionTimeStep},1);  
% [~, mostProbablePredictionId_MHP] = max(pedPredictionsData_MHP{predictionTimeStep}(:,2));  %probability of the tracks is in the second column
% 
% for predId = 1:N_futures_MHP
%     temp = pedPredictionsData_MHP{predictionTimeStep}(predId, :);
% 
%     if mod([length(temp)/2 - 1],1) == 0
%         temp = reshape(temp(3:end), [2, length(temp)/2 - 1])';
%         tempIsMHPUsed = true;
%     else
%         temp = reshape(temp(2:end), [2, floor(length(temp)/2) - 1])';
%         tempIsMHPUsed = false;
%     end
%     mostProbablePredictedTrajectory_MHP(predId,:,:) = temp([end-Params.predHorizon+1:end], :);
% end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Base Hybrid predictions
% N_futures_HBase = size(pedPredictionsData_HBase{predictionTimeStep},1);  
% [~, mostProbablePredictionId_HBase] = max(pedPredictionsData_HBase{predictionTimeStep}(:,2));  %probability of the tracks is in the second column
% 
% for predId = 1:N_futures_HBase
%     temp = pedPredictionsData_HBase{predictionTimeStep}(predId, :);
% 
%     if mod([length(temp)/2 - 1],1) == 0
%         temp = reshape(temp(3:end), [2, length(temp)/2 - 1])';
%         tempIsMHPUsed = true;
%     else
%         temp = reshape(temp(2:end), [2, floor(length(temp)/2) - 1])';
%         tempIsMHPUsed = false;
%     end
%     mostProbablePredictedTrajectory_HBase(predId,:,:) = temp([end-Params.predHorizon+1:end], :);
% end
% 
% % constant velocity predictions
% mostProbablePredictedTrajectory_CV = pedPredictionsData_CV{predictionTimeStep};
% mostProbablePredictedTrajectory_CV = reshape(mostProbablePredictedTrajectory_CV{1}(3:end), [2, Params.predHorizon])';
% 
% % ground truth
% GTTrajectory = [GTPedData.xCenter(actualTimeStep+1:endTimeStep), GTPedData.yCenter(actualTimeStep+1:endTimeStep)];
% N_PredTimeSteps = size(GTTrajectory,1);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% %% 
% figure()
% cw_x = cw.center_x*orthopxToMeter*scaleFactor;
% cw_y = cw.center_y*orthopxToMeter*scaleFactor;
% plot(cw_x, cw_y, 'ro','MarkerSize', 8);hold on;
% 
% plot(GTTrajectory(:,1), GTTrajectory(:,2), 'g*', 'MarkerSize',8); hold on;
% shift = 1;
% for ii = 1:N_futures_MHP
% %     predId = mostProbablePredictionId_MHP;
%     predId = ii;
%     if predId == mostProbablePredictionId_MHP
%         plot(mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,2), 'b*', 'MarkerSize',8); hold on;
%     else
%         plot(mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,2), 'm*', 'MarkerSize',8); hold on;
%     end
%         text(mostProbablePredictedTrajectory_MHP(predId,N_PredTimeSteps,1), mostProbablePredictedTrajectory_MHP(predId,N_PredTimeSteps,2)-shift,strcat(num2str(predId)));
% end
% for predId = 1:N_futures_HBase
%     plot(mostProbablePredictedTrajectory_HBase(predId,1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_HBase(predId,1:N_PredTimeSteps,2), 'k*', 'MarkerSize',8); hold on;
%     text(mostProbablePredictedTrajectory_HBase(predId,N_PredTimeSteps,1), mostProbablePredictedTrajectory_HBase(predId,N_PredTimeSteps,2)-shift,strcat(num2str(predId)));
% end
% 
% plot(mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,2), 'r*', 'MarkerSize',8); hold on;
% x=1;
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %% identify goal regions
% theta = cw.theta;
% % cwPed = 1;
% % swInd = 1;
% % cw_ped_rot = [cosd(theta(cwPed)), -sind(theta(cwPed)); sind(theta(cwPed)), cosd(theta(cwPed))];
% imgSize = size(annotatedImageEnhanced);
% imgCenter = int32([imgSize(2)/2; -imgSize(1)/2]);
% % sf = Params.orthopxToMeter*Params.scaleFactor;
% % pedPosPixels = resetStates.approach.goal(swInd,[1,2]);
% % pedPosPixelsRot = (cw_ped_rot * (pedPosPixels'  - double(imgCenter)) + double(imgCenter));
% % 
% 
% goal = resetStates.walkaway.goal(:,[1,2]);
% for jj=1:8
%     pedPosPixels_2 = [];
%     cwPed = ceil(jj/2);
%     cw_ped_rot = [cosd(theta(cwPed)), -sind(theta(cwPed)); sind(theta(cwPed)), cosd(theta(cwPed))];
%     pedPosPixels = goal(jj,:);
%     goal_rot = (cw_ped_rot * (pedPosPixels'  - double(imgCenter)) + double(imgCenter))';
%     
% 
%     if jj==1 || jj==4 
%         pedPosPixelsRot_2 = [goal_rot + [5, 0];
%                              goal_rot + [-5, 0];
%                              goal_rot + [5, 10];
%                              goal_rot + [-5, 10]];
%     elseif jj==5 || jj==8
%          pedPosPixelsRot_2 = [goal_rot + [0, 5];
%                               goal_rot + [0, -5];
%                               goal_rot + [10, 5];
%                               goal_rot + [10, -5]];
%     elseif jj==2 || jj==3
%          pedPosPixelsRot_2 = [goal_rot + [5, 0];
%                               goal_rot + [-5, 0];
%                               goal_rot + [5, -10];
%                               goal_rot + [-5, -10]];
%     elseif jj==6 || jj==7
%          pedPosPixelsRot_2 = [goal_rot + [0, 5];
%                               goal_rot + [0, -5];
%                               goal_rot + [-10, 5];
%                               goal_rot + [-10, -5]];
%         
%     end
%     % inverse rotations
%     for ii=1:4
%         pedPosPixels_2(:,ii) = double(imgCenter) + inv(cw_ped_rot)*(pedPosPixelsRot_2(ii,:)' - double(imgCenter));
%     end
%     pedPosPixels_2 = reshape(pedPosPixels_2, [1,8]);
%     pedPosPixels_bb(jj,:) = pedPosPixels_2;
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%