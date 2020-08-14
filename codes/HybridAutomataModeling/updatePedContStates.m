function pred_data = updatePedContStates(pred_data, ped_cw_angle, reset, walkawayDirection, del_t)

%
pedPos = [pred_data.xCenterPix(end-1), pred_data.yCenterPix(end-1)];

% set default update values for constant velocity model
pred_data.calcHeading(end) = pred_data.calcHeading(end-1);
pred_data.xVelocity(end) = pred_data.xVelocity(end-1);
pred_data.yVelocity(end) = pred_data.xVelocity(end-1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% reset continuous states if discrete state changed for first time
if size(pred_data,1) > 1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
% if crosswalk 1
if pred_data.closestCW(end-1)==1
    if ped_cw_angle(1) < 0  % West right lane (Lane 1)
         % 1st Approach
         if ( strcmp(pred_data.HybridState(end), 'Approach') &&  ~strcmp(pred_data.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(1,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             pred_data.xVelocity(end) = pred_data.xVelocity(end-1);
             pred_data.yVelocity(end) = pred_data.xVelocity(end-1);                  
         % 1st Wait
         elseif ( strcmp(pred_data.HybridState(end), 'Wait') &&  strcmp(pred_data.HybridState(end-1), 'Approach') )
             pred_data.calcHeading(end) = reset.wait.heading(1,:);
             pred_data.xVelocity(end) = 0;
             pred_data.yVelocity(end) = 0;                  
         % 1st Cross
         elseif ( strcmp(pred_data.HybridState(end), 'Cross') &&  (strcmp(pred_data.HybridState(end-1), 'Approach') || strcmp(pred_data.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(2,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));                 
         % 1st Walkaway
         elseif ( strcmp(pred_data.HybridState(end), 'Walkaway') &&  (strcmp(pred_data.HybridState(end-1), 'Cross') || strcmp(pred_data.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    pred_data.calcHeading(end) = reset.approach.heading(5,:);
             else
                    pred_data.calcHeading(end) = reset.walkaway.heading(2,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         end
    else        % West Left Lane (lane 2)
         % 1st Approach
         if ( strcmp(pred_data.HybridState(end), 'Approach') &&  ~strcmp(pred_data.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(2,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             pred_data.xVelocity(end) = pred_data.xVelocity(end-1);
             pred_data.yVelocity(end) = pred_data.xVelocity(end-1);                  
         % 1st Wait
         elseif ( strcmp(pred_data.HybridState(end), 'Wait') &&  strcmp(pred_data.HybridState(end-1), 'Approach') )
             pred_data.calcHeading(end) = reset.wait.heading(2,:);
             pred_data.xVelocity(end) = 0;
             pred_data.yVelocity(end) = 0;
         % 1st Cross
         elseif ( strcmp(pred_data.HybridState(end), 'Cross') &&  (strcmp(pred_data.HybridState(end-1), 'Approach') || strcmp(pred_data.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(1,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(pred_data.HybridState(end), 'Walkaway') &&  (strcmp(pred_data.HybridState(end-1), 'Cross') || strcmp(pred_data.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    pred_data.calcHeading(end) = reset.approach.heading(8,:);
             else
                    pred_data.calcHeading(end) = reset.walkaway.heading(1,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% if crosswalk 2
if pred_data.closestCW(end-1)==2

    if ped_cw_angle(2) > 0  % East right lane (Lane 3)
         % 1st Approach
         if ( strcmp(pred_data.HybridState(end), 'Approach') &&  ~strcmp(pred_data.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(4,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             pred_data.xVelocity(end) = pred_data.xVelocity(end-1);
             pred_data.yVelocity(end) = pred_data.xVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(pred_data.HybridState(end), 'Wait') &&  strcmp(pred_data.HybridState(end-1), 'Approach') )
             pred_data.calcHeading(end) = reset.wait.heading(3,:);
             pred_data.xVelocity(end) = 0;
             pred_data.yVelocity(end) = 0;
         % 1st Cross
         elseif ( strcmp(pred_data.HybridState(end), 'Cross') &&  (strcmp(pred_data.HybridState(end-1), 'Approach') || strcmp(pred_data.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(4,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(pred_data.HybridState(end), 'Walkaway') &&  (strcmp(pred_data.HybridState(end-1), 'Cross') || strcmp(pred_data.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    pred_data.calcHeading(end) = reset.approach.heading(7,:);
             else
                    pred_data.calcHeading(end) = reset.walkaway.heading(4,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         end
    else        % East Left Lane (Lane 4)
         % 1st Approach
         if ( strcmp(pred_data.HybridState(end), 'Approach') &&  ~strcmp(pred_data.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(4,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             pred_data.xVelocity(end) = pred_data.xVelocity(end-1);
             pred_data.yVelocity(end) = pred_data.xVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(pred_data.HybridState(end), 'Wait') &&  strcmp(pred_data.HybridState(end-1), 'Approach') )
             pred_data.calcHeading(end) = reset.wait.heading(4,:);
             pred_data.xVelocity(end) = 0;
             pred_data.yVelocity(end) = 0;
         % 1st Cross
         elseif ( strcmp(pred_data.HybridState(end), 'Cross') &&  (strcmp(pred_data.HybridState(end-1), 'Approach') || strcmp(pred_data.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(3,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(pred_data.HybridState(end), 'Walkaway') &&  (strcmp(pred_data.HybridState(end-1), 'Cross') || strcmp(pred_data.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    pred_data.calcHeading(end) = reset.approach.heading(6,:);
             else
                    pred_data.calcHeading(end) = reset.walkaway.heading(3,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         end
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 3
if pred_data.closestCW(end-1)==3
   if abs(ped_cw_angle(3)) > 90  % South right lane (lane 5)
         % 1st Approach
         if ( strcmp(pred_data.HybridState(end), 'Approach') &&  ~strcmp(pred_data.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(5,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             pred_data.xVelocity(end) = pred_data.xVelocity(end-1);
             pred_data.yVelocity(end) = pred_data.xVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(pred_data.HybridState(end), 'Wait') &&  strcmp(pred_data.HybridState(end-1), 'Approach') )
             pred_data.calcHeading(end) = reset.wait.heading(5,:);
             pred_data.xVelocity(end) = 0;
             pred_data.yVelocity(end) = 0;
         % 1st Cross
         elseif ( strcmp(pred_data.HybridState(end), 'Cross') &&  (strcmp(pred_data.HybridState(end-1), 'Approach') || strcmp(pred_data.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(6,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(pred_data.HybridState(end), 'Walkaway') &&  (strcmp(pred_data.HybridState(end-1), 'Cross') || strcmp(pred_data.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    pred_data.calcHeading(end) = reset.approach.heading(3,:);
             else
                    pred_data.calcHeading(end) = reset.walkaway.heading(6,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         end
    else        % South Left Lane (Lane 6)
         % 1st Approach
         if ( strcmp(pred_data.HybridState(end), 'Approach') &&  ~strcmp(pred_data.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(6,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             pred_data.xVelocity(end) = pred_data.xVelocity(end-1);
             pred_data.yVelocity(end) = pred_data.xVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(pred_data.HybridState(end), 'Wait') &&  strcmp(pred_data.HybridState(end-1), 'Approach') )
             pred_data.calcHeading(end) = reset.wait.heading(6,:);
             pred_data.xVelocity(end) = 0;
             pred_data.yVelocity(end) = 0;
         % 1st Cross
         elseif ( strcmp(pred_data.HybridState(end), 'Cross') &&  (strcmp(pred_data.HybridState(end-1), 'Approach') || strcmp(pred_data.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(5,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(pred_data.HybridState(end), 'Walkaway') &&  (strcmp(pred_data.HybridState(end-1), 'Cross') || strcmp(pred_data.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    pred_data.calcHeading(end) = reset.approach.heading(2,:);
             else
                    pred_data.calcHeading(end) = reset.walkaway.heading(5,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         end
    end
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if crosswalk 4
if pred_data.closestCW(end-1)==4
   if abs(ped_cw_angle(4)) < 90  % North right lane (Lane 7)
         % 1st Approach
         if ( strcmp(pred_data.HybridState(end), 'Approach') &&  ~strcmp(pred_data.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(7,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             pred_data.xVelocity(end) = pred_data.xVelocity(end-1);
             pred_data.yVelocity(end) = pred_data.xVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(pred_data.HybridState(end), 'Wait') &&  strcmp(pred_data.HybridState(end-1), 'Approach') )
             pred_data.calcHeading(end) = reset.wait.heading(7,:);
             pred_data.xVelocity(end) = 0;
             pred_data.yVelocity(end) = 0;
         % 1st Cross
         elseif ( strcmp(pred_data.HybridState(end), 'Cross') &&  (strcmp(pred_data.HybridState(end-1), 'Approach') || strcmp(pred_data.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(8,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(pred_data.HybridState(end), 'Walkaway') &&  (strcmp(pred_data.HybridState(end-1), 'Cross') || strcmp(pred_data.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    pred_data.calcHeading(end) = reset.approach.heading(1,:);
             else
                    pred_data.calcHeading(end) = reset.walkaway.heading(8,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         end
    else        % North Left Lane (Lane 8)
         % 1st Approach
         if ( strcmp(pred_data.HybridState(end), 'Approach') &&  ~strcmp(pred_data.HybridState(end-1), 'Approach') )
             pedGoalDisp = reset.approach.goal(8,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             pred_data.xVelocity(end) = pred_data.xVelocity(end-1);
             pred_data.yVelocity(end) = pred_data.xVelocity(end-1);
         % 1st Wait
         elseif ( strcmp(pred_data.HybridState(end), 'Wait') &&  strcmp(pred_data.HybridState(end-1), 'Approach') )
             pred_data.calcHeading(end) = reset.wait.heading(8,:);
             pred_data.xVelocity(end) = 0;
             pred_data.yVelocity(end) = 0;
         % 1st Cross
         elseif ( strcmp(pred_data.HybridState(end), 'Cross') &&  (strcmp(pred_data.HybridState(end-1), 'Approach') || strcmp(pred_data.HybridState(end-1), 'Wait') ) )
             pedGoalDisp = reset.approach.goal(7,:) - pedPos;
             pred_data.calcHeading(end) = atan2(pedGoalDisp(2), pedGoalDisp(1));
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         % 1st Walkaway
         elseif ( strcmp(pred_data.HybridState(end), 'Walkaway') &&  (strcmp(pred_data.HybridState(end-1), 'Cross') || strcmp(pred_data.HybridState(end-1), 'Jaywalking') ) )
             if strcmp(walkawayDirection, 'Approach')         
                    pred_data.calcHeading(end) = reset.approach.heading(4,:);
             else
                    pred_data.calcHeading(end) = reset.walkaway.heading(7,:);
             end
             %sample longitudinal velocity in the range of 1 - 2 m/s
             vel = rand(1) + 1;
             pred_data.xVelocity(end) = vel*cosd(pred_data.calcHeading(end));
             pred_data.yVelocity(end) = vel*sind(pred_data.calcHeading(end));
         end
   end
    
end


end  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% update position (currently no noise is directly included)
pred_data.xCenterPix(end) = pred_data.xCenterPix(end-1) + del_t*pred_data.xVelocity(end-1);
pred_data.yCenterPix(end) = pred_data.yCenterPix(end-1) + del_t*pred_data.yVelocity(end-1);


end