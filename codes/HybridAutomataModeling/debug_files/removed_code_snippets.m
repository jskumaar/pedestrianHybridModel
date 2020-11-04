%% removed

% trackletData{trackletNo}.isLooking(end,1) = currentTSPedEgoData.isLooking;                   
trackletData{trackletNo}.isPedSameDirection(end,1) = currentTSPedEgoData.isPedSameDirection;
trackletData{trackletNo}.longDispPedCw(end,1) = currentTSPedEgoData.longDispPedCw;
trackletData{trackletNo}.latDispPedCw(end,1) = currentTSPedEgoData.latDispPedCw;
trackletData{trackletNo}.closeCar_ind(end,1) = currentTSPedEgoData.closeCar_ind; 
trackletData{trackletNo}.activeCar_ind(end,1) = currentTSPedEgoData.activeCar_ind; 
trackletData{trackletNo}.lonVelocity(end,1) = currentTSPedEgoData.lonVelocity; 
trackletData{trackletNo}.long_disp_ped_car(end,1) = currentTSPedEgoData.long_disp_ped_car; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% set goal location

%%%%%%%%%%%%%%%%%%%%%%%
% calculate angle between pedestrian and approaching crosswalk
pedCwAngle = [ atan2((cw.center_y(1) - pedPosPixels(2)), (cw.center_x(1) - pedPosPixels(1)))*180/pi;
               atan2((cw.center_y(2) - pedPosPixels(2)), (cw.center_x(2) - pedPosPixels(1)))*180/pi;
               atan2((cw.center_y(3) - pedPosPixels(2)), (cw.center_x(3) - pedPosPixels(1)))*180/pi;
               atan2((cw.center_y(4) - pedPosPixels(2)), (cw.center_x(4) - pedPosPixels(1)))*180/pi];             
% distance between pedestrian and crosswalk (in pixels)
dist_cw(1,1) = sqrt(double(cw.center_x(1) - pedPosPixels(1))^2 + double(cw.center_y(1) - pedPosPixels(2))^2);
dist_cw(2) = sqrt(double(cw.center_x(2) - pedPosPixels(1))^2 + double(cw.center_y(2) - pedPosPixels(2))^2); 
dist_cw(3) = sqrt(double(cw.center_x(3) - pedPosPixels(1))^2 + double(cw.center_y(3) - pedPosPixels(2))^2);
dist_cw(4) = sqrt(double(cw.center_x(4) - pedPosPixels(1))^2 + double(cw.center_y(4) - pedPosPixels(2))^2);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %% if it is a new tracklet (or 1st time step) or no previous reset state
% % if (size(HybridState,1)==1 || closeCar_ind(end)~=closeCar_ind(end-1))
% % if crosswalk 1
% if closestCW(end)==1
%     if (pedCwAngle(1) < 0 && dist_cw(1) > crosswalkThreshold)   % East right lane (Lane 1)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach')
%              if ( predTimeStep==1 || size(HybridState,1)==1 || size(HybridState,1)==2 || (size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) )  )
%                  pedGoalPixels = reset.approach.goal(1,:);
%                  pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
%                  calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%                  vel = norm( [xVelocity(end), predData.yVelocity(end)]);
%                  xVelocity(end) = vel*cosd(calcHeading(end));
%                  yVelocity(end) = vel*sind(calcHeading(end)); 
%                  resetState = true;
%              end
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait'))
%              pedGoalPixels = reset.approach.goal(2,:);
%              pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(1,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = true;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Crossing')  && size(HybridState,1)>1 &&  ~strcmp(HybridState(end-1),'Crossing') )
%              pedGoalPixels = reset.approach.goal(2,:);
%              pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = true;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') )
%              if strcmp(walkawayDirection, 'Approach')    
%                     pedGoalPixels = reset.approach.goal(8,:);
%                     pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalPixels = reset.walkaway.goal(1,:);
%                     pedGoalDispPixels = reset.walkaway.goal(1,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end), yVelocity(end)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%              resetState = true;
%          end
%     elseif   (pedCwAngle(1) > 0 && dist_cw(1) > crosswalkThreshold)       % East Left Lane (lane 2)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              if  ( predTimeStep==1 || size(HybridState,1)==1 || size(HybridState,1)==2 ||  ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) ) )
%                  pedGoalPixels = reset.approach.goal(2,:);
%                  pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
%                  calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%                  vel = norm( [xVelocity(end), yVelocity(end)]);
%                  xVelocity(end) = vel*cosd(calcHeading(end));
%                  yVelocity(end) = vel*sind(calcHeading(end)); 
%                  resetState = true;
%              end
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait')  )
%              pedGoalPixels = reset.wait.heading(2,:);
%              calcHeading(end) = reset.wait.heading(2,:);
%              pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%              resetState = true;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Crossing')  && size(HybridState,1)>1 &&  ~strcmp(HybridState(end-1),'Crossing')  )
%              pedGoalPixels = reset.approach.goal(1,:);
%              pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = true;
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway')  )
%              if strcmp(walkawayDirection, 'Approach')   
%                    pedGoalPixels = reset.approach.goal(5,:);
%                    pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
%                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                    pedGoalPixels = reset.walkaway.goal(2,:);
%                    pedGoalDispPixels = reset.walkaway.goal(2,:) - pedPosPixels;
%                    calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end), yVelocity(end)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%              resetState = true;
%          end
%     end
% 
% %%%%%%%%%%%%%%%%%%%%%%%   
% % if crosswalk 2
% elseif closestCW(end)==2
% 
%     if (pedCwAngle(2) > 0  && dist_cw(2) > crosswalkThreshold )% West right lane (Lane 3)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              if ( predTimeStep==1 || size(HybridState,1)==1 || size(HybridState,1)==2 || (size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) ) )
%                  pedGoalPixels = reset.approach.goal(3,:);
%                  pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
%                  calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%                  vel = norm( [xVelocity(end), yVelocity(end)]);
%                  xVelocity(end) = vel*cosd(calcHeading(end));
%                  yVelocity(end) = vel*sind(calcHeading(end)); 
%                  resetState = true;
%              end
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') )
%              pedGoalPixels = reset.approach.goal(4,:);
%              pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(3,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Crossing')  && size(HybridState,1)>1 &&  ~strcmp(HybridState(end-1),'Crossing')  )
%              pedGoalPixels = reset.approach.goal(4,:);
%              pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway')  )
%              if strcmp(walkawayDirection, 'Approach')   
%                     pedGoalPixels = reset.approach.goal(6,:);
%                     pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalPixels = reset.walkaway.goal(3,:);
%                     pedGoalDispPixels = reset.walkaway.goal(3,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end), yVelocity(end)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%          end
%     elseif (pedCwAngle(2) < 0  && dist_cw(2) > crosswalkThreshold )        % West Left Lane (Lane 4)
%          % Approach
%          %if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              if ( predTimeStep==1 || size(HybridState,1)==1 || size(HybridState,1)==2 || (size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) ) )
%                  pedGoalPixels = reset.approach.goal(4,:);
%                  pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
%                  calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%                  vel = norm( [xVelocity(end), yVelocity(end)]);
%                  xVelocity(end) = vel*cosd(calcHeading(end));
%                  yVelocity(end) = vel*sind(calcHeading(end)); 
%              end
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') )
%              pedGoalPixels = reset.approach.goal(3,:);
%              pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(4,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Crossing') && size(HybridState,1)>1 &&  ~strcmp(HybridState(end-1),'Crossing') )
%              pedGoalPixels = reset.approach.goal(3,:);
%              pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway')  )
%              if strcmp(walkawayDirection, 'Approach')         
%                     pedGoalPixels = reset.approach.goal(7,:);
%                     pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalPixels = reset.walkaway.goal(4,:);
%                     pedGoalDispPixels = reset.walkaway.goal(4,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end), yVelocity(end)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%          end
%     end
%     
% %%%%%%%%%%%%%%%%%%%%%%%
% % if crosswalk 3
% elseif closestCW(end)==3
%    if (abs(pedCwAngle(3)) > 90 && dist_cw(3) > crosswalkThreshold ) % South right lane (lane 5)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              if ( predTimeStep==1 || size(HybridState,1)==1 || size(HybridState,1)==2 || (size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) ) )
%                  pedGoalPixels = reset.approach.goal(5,:);
%                  pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
%                  calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%                  vel = norm( [xVelocity(end), yVelocity(end)]);
%                  xVelocity(end) = vel*cosd(calcHeading(end));
%                  yVelocity(end) = vel*sind(calcHeading(end)); 
%              end
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') )
%              pedGoalPixels = reset.approach.goal(6,:);
%              pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(5,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Crossing') && size(HybridState,1)>1 &&  ~strcmp(HybridState(end-1),'Crossing') )
%              pedGoalPixels = reset.approach.goal(6,:);
%              pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 1.5 m/s
%              vel = 0.5*rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') )
%              if strcmp(walkawayDirection, 'Approach') 
%                     pedGoalPixels = reset.approach.goal(2,:);
%                     pedGoalDispPixels = reset.approach.goal(2,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalPixels = reset.walkaway.goal(5,:);
%                     pedGoalDispPixels = reset.walkaway.goal(5,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end), yVelocity(end)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%          end
%    elseif (abs(pedCwAngle(3)) < 90 && dist_cw(3) > crosswalkThreshold )        % South Left Lane (Lane 6)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              if ( predTimeStep==1 || size(HybridState,1)==1 || size(HybridState,1)==2 || (size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) ))
%                  pedGoalPixels = reset.approach.goal(6,:);
%                  pedGoalDispPixels = reset.approach.goal(6,:) - pedPosPixels;
%                  calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%                  vel = norm( [xVelocity(end), yVelocity(end)]);
%                  xVelocity(end) = vel*cosd(calcHeading(end));
%                  yVelocity(end) = vel*sind(calcHeading(end)); 
%              end
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') )
%              pedGoalPixels = reset.approach.goal(5,:);
%              pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(6,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Crossing')  && size(HybridState,1)>1 &&  ~strcmp(HybridState(end-1),'Crossing') )
%              pedGoalPixels = reset.approach.goal(5,:);
%              pedGoalDispPixels = reset.approach.goal(5,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway')  )
%              if strcmp(walkawayDirection, 'Approach')    
%                     pedGoalPixels = reset.approach.goal(3,:);
%                     pedGoalDispPixels = reset.approach.goal(3,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalPixels = reset.walkaway.goal(6,:);
%                     pedGoalDispPixels = reset.walkaway.goal(6,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end), yVelocity(end)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%          end
%     end
%     
% %%%%%%%%%%%%%%%%%%%%%%%
% % if crosswalk 4
% elseif closestCW(end)==4
%    if (abs(pedCwAngle(4)) < 90 && dist_cw(4) > crosswalkThreshold )  % North right lane (Lane 7)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              if ( predTimeStep==1 || size(HybridState,1)==1 || size(HybridState,1)==2 || ( size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) ) )
%                  pedGoalPixels = reset.approach.goal(7,:);
%                  pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
%                  calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%                  vel = norm( [xVelocity(end), yVelocity(end)]);
%                  xVelocity(end) = vel*cosd(calcHeading(end));
%                  yVelocity(end) = vel*sind(calcHeading(end));
%              end
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') )
%              pedGoalPixels = reset.approach.goal(8,:);
%              pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(7,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Crossing')  && size(HybridState,1)>1 &&  ~strcmp(HybridState(end-1),'Crossing') )
%              pedGoalPixels = reset.approach.goal(8,:);
%              pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') )
%              if strcmp(walkawayDirection, 'Approach')  
%                     pedGoalPixels = reset.approach.goal(4,:);
%                     pedGoalDispPixels = reset.approach.goal(4,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalPixels = reset.walkaway.goal(7,:);
%                     pedGoalDispPixels = reset.walkaway.goal(7,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end), yVelocity(end)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%          end
%    elseif (abs(pedCwAngle(4)) > 90 && dist_cw(4) > crosswalkThreshold )         % North Left Lane (Lane 8)
%          % Approach
% %          if ( strcmp(HybridState(end), 'Approach') &&  ~strcmp(HybridState(end), 'Approach') )
%          if  strcmp(HybridState(end), 'Approach') 
%              if ( predTimeStep==1 || size(HybridState,1)==1 || size(HybridState,1)==2 || (size(HybridState,1)>1 && (closestCW(end)~=closestCW(end-1)) ))
%                  pedGoalPixels = reset.approach.goal(8,:);
%                  pedGoalDispPixels = reset.approach.goal(8,:) - pedPosPixels;
%                  calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%                  vel = norm( [xVelocity(end), yVelocity(end)]);
%                  xVelocity(end) = vel*cosd(calcHeading(end));
%                  yVelocity(end) = vel*sind(calcHeading(end)); 
%              end
%          % 1st Wait
%          elseif ( strcmp(HybridState(end), 'Wait') )
%              pedGoalPixels = reset.approach.goal(7,:);
%              pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
%              calcHeading(end) = reset.wait.heading(8,:);
%              xVelocity(end) = 0;
%              yVelocity(end) = 0;
%              waitTimeSteps(end) = 0;
%          % 1st Cross
%          elseif ( strcmp(HybridState(end), 'Crossing')  && size(HybridState,1)>1 &&  ~strcmp(HybridState(end-1),'Crossing')  )
%              pedGoalPixels = reset.approach.goal(7,:);
%              pedGoalDispPixels = reset.approach.goal(7,:) - pedPosPixels;
%              calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              %sample longitudinal velocity in the range of 1 - 2 m/s
%              vel = rand(1) + 1;
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end));
%          % 1st Walkaway
%          elseif ( strcmp(HybridState(end), 'Walkaway') )
%              if strcmp(walkawayDirection, 'Approach')    
%                     pedGoalPixels = reset.approach.goal(1,:);
%                     pedGoalDispPixels = reset.approach.goal(1,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              else
%                     pedGoalPixels = reset.walkaway.goal(8,:);
%                     pedGoalDispPixels = reset.walkaway.goal(8,:) - pedPosPixels;
%                     calcHeading(end) = atan2(pedGoalDispPixels(2), pedGoalDispPixels(1)) *180/pi;
%              end
%              vel = norm( [xVelocity(end), yVelocity(end)]);
%              xVelocity(end) = vel*cosd(calcHeading(end));
%              yVelocity(end) = vel*sind(calcHeading(end)); 
%          end
%    end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.sceneId = sceneId;
predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.carTrackId = carTrackId;
predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.pedcarTrackId = pedIndexWithinScene;  
predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.timeStep(1) = inf;
predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.data{1} = inf;
predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.kfData{1} = inf;
predictedPedTraj_MHP{sceneId}{track_index,1}{pedIndexWithinScene,1}.predictionModel(1) = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


                    
currentTSActiveCarData{carLoopId}.xCenter = formattedTracksData{sceneId}{carIndex}.xCenter(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.yCenter = formattedTracksData{sceneId}{carIndex}.yCenter(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.xVelocity = formattedTracksData{sceneId}{carIndex}.xVelocity(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.yVelocity = formattedTracksData{sceneId}{carIndex}.yVelocity(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.xAcceleration = formattedTracksData{sceneId}{carIndex}.xAcceleration(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.yAcceleration = formattedTracksData{sceneId}{carIndex}.yAcceleration(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.lonVelocity = formattedTracksData{sceneId}{carIndex}.lonVelocity(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.lonAcceleration = formattedTracksData{sceneId}{carIndex}.lonAcceleration(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.latAcceleration = formattedTracksData{sceneId}{carIndex}.latAcceleration(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.closestCW = formattedTracksData{sceneId}{carIndex}.closestCW(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.calcHeading = formattedTracksData{sceneId}{carIndex}.calcHeading(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.car_lane = formattedTracksData{sceneId}{carIndex}.car_lane(carTrackTimeStep);
currentTSActiveCarData{carLoopId}.carTrackId = formattedTracksData{sceneId}{carIndex}.trackId(carTrackTimeStep);
% define new variables
currentTSActiveCarData{carLoopId}.turn = false;
currentTSActiveCarData{carLoopId}.changeLane = false;
currentTSActiveCarData{carLoopId}.reachGoal = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%