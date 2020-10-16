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
cw_x = cw.center_x*orthopxToMeter*scaleDownFactor;
cw_y = cw.center_y*orthopxToMeter*scaleDownFactor;
shift = 0.5;
figure()
for id = 1:N_tracklets
   xCenter =  trackletData{id}.xCenter;
   yCenter =  trackletData{id}.yCenter;
   
   goal_x =  trackletData{id}.goalPositionPixels(:,1)*orthopxToMeter*scaleDownFactor;
   goal_y =  trackletData{id}.goalPositionPixels(:,2)*orthopxToMeter*scaleDownFactor;
   
    plot(xCenter, yCenter, '*', 'MarkerSize', 8); hold on;
    plot(cw_x, cw_y, 'r*','MarkerSize', 8);hold on;
    axis equal
    text(xCenter(1), yCenter(1)-shift,strcat(num2str(id)));
    plot(goal_x, goal_y, 'g*','MarkerSize', 8);hold on;
    text(goal_x(1), goal_y(1)-shift,strcat('G-',num2str(id)));
end






