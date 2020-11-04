%% find the theta and plot for each crosswalk
theta = cw.theta;
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;

theta_cw_overall = [];
dist_cw_overall = [];
dist_closestCW_overall = [];
heading_overall = [];
theta_closestCW_overall = [];
diff_angle_ClosestCW_overall = [];

for sceneId = 1:12
    N_tracks = size(formattedTracksData{sceneId},1);
    for track_id = 1:N_tracks
           
           if strcmp(formattedTracksData{sceneId}{track_id}.class{1}, 'pedestrian') 
               pedData = formattedTracksData{sceneId}{track_id};
               N_ts = length(pedData.frame);
               % initialize
               dist_closestCW = [];
               disp_closestCW = [];
               theta_closestCW = [];
               heading_closestCW = [];
               
               % distance between each crosswalk
               disp_cw1 = ([pedData.xCenterPix, pedData.yCenterPix] - [cw.center_x(1), cw.center_y(1)]);
               disp_cw2 = ([pedData.xCenterPix, pedData.yCenterPix] - [cw.center_x(2), cw.center_y(2)]);
               disp_cw3 = ([pedData.xCenterPix, pedData.yCenterPix] - [cw.center_x(3), cw.center_y(3)]);
               disp_cw4 = ([pedData.xCenterPix, pedData.yCenterPix] - [cw.center_x(4), cw.center_y(4)]);
               
               xVelocity = pedData.xVelocity;
               xVelocity(xVelocity<0.1 & xVelocity>-0.1) = 0;
               yVelocity = pedData.yVelocity;
               yVelocity(yVelocity<0.1 & yVelocity>-0.1) = 0;
               heading = atan2(yVelocity, xVelocity) * 180/pi;
               
               % angle between crosswalk and the pedestrian
               theta_cw = [atan2(disp_cw1(:,2), disp_cw1(:,1)), atan2(disp_cw2(:,2), disp_cw2(:,1)),...
                           atan2(disp_cw3(:,2), disp_cw3(:,1)), atan2(disp_cw4(:,2), disp_cw4(:,1))] * 180/pi;
                
               % distance
               dist_cw = [vecnorm(disp_cw1,2,2), vecnorm(disp_cw2,2,2), vecnorm(disp_cw3,2,2), vecnorm(disp_cw4,2,2)] * (scaleFactor*orthopxToMeter);
                           
               for ii=1:N_ts
                  cwInd = pedData.closestCW(ii);
                  if cwInd~=0 && cwInd~=inf
                       disp_closestCW(ii,:) = ([cw.center_x(cwInd), cw.center_y(cwInd)]-[pedData.xCenterPix(ii), pedData.yCenterPix(ii)]);                      
                       dist_closestCW(ii,1) = norm(disp_closestCW(ii,:)) * (scaleFactor*orthopxToMeter);
                       
                       xVelocity_ts = pedData.xVelocity(ii);
                       xVelocity_ts(xVelocity_ts <0.1 & xVelocity_ts >-0.1) = 0;
                       yVelocity_ts  = pedData.yVelocity(ii);
                       yVelocity_ts (yVelocity_ts <0.1 & yVelocity_ts >-0.1) = 0;
                       theta_closestCW(ii,1) = atan2(disp_closestCW(ii,2) , disp_closestCW(ii,1) ) * 180/pi;
                       heading_closestCW(ii,1) = atan2(yVelocity_ts, xVelocity_ts)*180/pi;
                  end
               end
               
               %diff in heading
               diff_angle_ClosestCW = abs(heading_closestCW - theta_closestCW);
               
               % update data
               theta_cw_overall = [theta_cw_overall; theta];
               dist_cw_overall = [dist_cw_overall; dist_cw];
               dist_closestCW_overall = [dist_closestCW_overall;dist_closestCW];
               theta_closestCW_overall = [theta_closestCW_overall; theta_closestCW];
               heading_overall = [heading_overall; heading];
               diff_angle_ClosestCW_overall = [diff_angle_ClosestCW_overall; diff_angle_ClosestCW];
               
           end
    end
end



%% plot
figure()
scatter(dist_closestCW_overall, diff_angle_ClosestCW_overall, '*');


