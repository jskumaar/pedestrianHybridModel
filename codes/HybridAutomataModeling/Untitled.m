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

%% plot tracklets
N_tracklets = size(trackletData,1);
scaleDownFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
sf = scaleDownFactor*orthopxToMeter;
cw_x = cw.center_x*sf;
cw_y = cw.center_y*sf;
shift = 0.5;
figure()
for id = 1:1
%    xCenter =  trackletData{id}.xCenter;
%    yCenter =  trackletData{id}.yCenter;
   
   goal_x =  [resetStates.approach.goal(:,1), resetStates.walkaway.goal(:,1)]*sf;
   goal_y =  [resetStates.approach.goal(:,2), resetStates.walkaway.goal(:,2)]*sf;
   
%     plot(xCenter, yCenter, '*', 'MarkerSize', 8); hold on;
    plot(cw_x, cw_y, 'r*','MarkerSize', 8);hold on;
    axis equal
%     text(xCenter(1), yCenter(1)-shift,strcat(num2str(id)));
    plot(goal_x, goal_y, 'g*','MarkerSize', 8);hold on;
    text(goal_x(1), goal_y(1)-shift,strcat('G-',num2str(id)));
    
    for jj=1:8
        pgon = polyshape(resetStates.approach.goal(jj,[3,7,5,9])*sf,resetStates.approach.goal(jj,[4,8,6,10])*sf);
        pgon2 = polyshape(resetStates.walkaway.goal(jj,[3,7,5,9])*sf,resetStates.walkaway.goal(jj,[4,8,6,10])*sf);
        plot(pgon); hold on;
        plot(pgon2); hold on;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% debug: plot predicted trajectories
figure()
for ii=1:size(predictionTrajectoryMatrix,1)
    tempPredMatrix = reshape(predictionTrajectoryMatrix(ii,end-2*Params.predHorizon+1:end),[2, Params.predHorizon])';
    plot(tempPredMatrix(:,1), tempPredMatrix(:,2), '*', 'MarkerSize', 10); hold on;
    %     for jj=1:size(tempPredMatrix,1)
%         plot(tempPredMatrix(jj,1), tempPredMatrix(jj,2), '*', 'MarkerSize', 10); hold on;
%         pause(0.1)
%     end
end
%  

