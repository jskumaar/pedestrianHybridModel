%% This function plots the predicted and ground truth trajectories

function vid = hybridModelVideo(groundTruthTraj, PredictedTraj_Hybrid, PredictedTraj_CV, vehPosition_leftLane, vehPosition_rightLane)

% groundTruthTraj = crossing45PedGT_CV(:,[1,2]);
% PredictedTraj_Hybrid = crossing45PedPred_Hybrid;
% PredictedTraj_CV = crossing45PedPred_CV;
% vehPosition_leftLane = crossing45Veh;
% vehPosition_rightLane = crossing45Veh_rightLane;

for ii=1:50
    groundTruthTraj = [groundTruthTraj;groundTruthTraj(end,:)+groundTruthTraj(end,:)-groundTruthTraj(end-1,:)];
end

% predicted pedestrian trajectory-CV
% indices to remove
ind = [[1:4:200],[2:4:200]]';
ind_predictions = sort(ind);

predPedTraj_CV = PredictedTraj_CV(:,ind_predictions);
predPedTraj_Hybrid = PredictedTraj_Hybrid(:,ind_predictions);

% ground truth trajectory for 
for ii=1:length(PredictedTraj_CV)
    groundTruthTraj_actual(ii,:) = reshape(groundTruthTraj(ii:ii+49,:)',[100,1]);
end



%addpath('G:\My Drive\Research\Journals and Conferences\Pics\Gallery')
addpath('G:\My Drive\Research\av-ped_simulations\Scripts\1. main_scripts')

addpath('G:\My Drive\Research\av-ped_simulations\Data')
p = genpath('G:\My Drive\Research\av-ped_simulations\Scripts');
addpath(p)

simulation_parameters;
        
       
%% Step 1: Initialize the animation

figure('units','normalized','outerposition',[0 0 1 1])
%Start Recording
name = 'Trial_45';
vid = VideoWriter(strcat(name,'.avi'));
vid.FrameRate = 10;
open(vid);
makeVideo = 1;

marker_size = 8;
font_size = 14;
line_width = 2;

plot_last_AV_index = 1;
plot_last_Ped_index = 1;
scale = 1;

% car size
width = 5;
height = 2;



%% Step 2: Plot the layout and the fixed elements in the simulation

EnvParams.road_end = -20;
EnvParams.road_start = 40;

layout = rectangle('Position',[EnvParams.road_end,-2*EnvParams.road_width,EnvParams.road_start-EnvParams.road_end,4*EnvParams.road_width]);hold on;   % Plot the boundaries
%axis([EnvParams.road_end EnvParams.road_start/2  -2*EnvParams.road_width 2*EnvParams.road_width]) 
axis equal;

lane_1 = rectangle('Position',[EnvParams.road_end,-EnvParams.road_width,EnvParams.road_start-EnvParams.road_end,EnvParams.road_width]);hold on;   % Plot the lanes
lane_2 = rectangle('Position',[EnvParams.road_end,0,EnvParams.road_start-EnvParams.road_end,EnvParams.road_width]);hold on;   

road_color = [190/255 190/255  190/255 ];
set(lane_1, 'FaceColor',  road_color)
set(lane_2, 'FaceColor',  road_color)

crosswalk = rectangle('Position',[EnvParams.crosswalk_location(1)-EnvParams.crosswalk_width/2,-EnvParams.road_width,EnvParams.crosswalk_width,2*EnvParams.road_width]);hold on;    % Plot the crosswalk
set(crosswalk, 'FaceColor',  [120/255  120/255  120/255 ])

xticks([])
yticks([])
set(gca,'Visible','off')    %removes the axes lines; just shows the roads and the sidewalks

dispPredictionHorizon = text(20,10,'Prediction horizon = 5 s','FontSize',font_size,'Interpreter', 'none');


Ped_current_position_color  = [0, 0.7, 0.3];        %Green
Ped_CV_pred_color           = [0.8, 0.1, 0.1];      %Red
Ped_Hybrid_pred_color       = [0, 0, 1];            %Blue

%% Step 3a: Plot the AVs in the near lane within the range for the first time step

for agent_num = 1 : size(vehPosition_leftLane,2) %for each AV

    AV_car_color_left_lane(agent_num,:) = rand(1,3);
    AV_car_color_right_lane(agent_num,:) = rand(1,3);

    %% a)plot AV rectangle left lane
   
        % left lane AV position update
        if (vehPosition_leftLane(1,agent_num)>EnvParams.road_start | vehPosition_leftLane(1,agent_num)<EnvParams.road_end | vehPosition_leftLane(1,agent_num)==0)
            AV_position_left_lane(agent_num,1) = rectangle('Position', [40, 0.2, 0.001, 0.001], 'EdgeColor', road_color, 'FaceColor', road_color);
        else
            xLeft = vehPosition_leftLane(1,agent_num)-width/2;
            yBottom = -1.75-height/2;
            AV_position_left_lane(agent_num,1) = rectangle('Position', [xLeft, yBottom, width, height],'EdgeColor',  AV_car_color_left_lane(agent_num,:), 'FaceColor',  AV_car_color_left_lane(agent_num,:));
        end
        
        % right lane AV position update
        if (vehPosition_rightLane(1,agent_num)>EnvParams.road_start | vehPosition_rightLane(1,agent_num)<EnvParams.road_end | vehPosition_rightLane(1,agent_num)==0)
            AV_position_right_lane(agent_num,1) = rectangle('Position', [40, 0.2, 0.001, 0.001], 'EdgeColor', road_color, 'FaceColor', road_color);
        else
            xLeft = vehPosition_rightLane(1,agent_num)-width/2;
            yBottom = 1.75-height/2;
            AV_position_right_lane(agent_num,1) = rectangle('Position', [xLeft, yBottom, width, height], 'EdgeColor',  AV_car_color_right_lane(agent_num,:), 'FaceColor',  AV_car_color_right_lane(agent_num,:));
        end
    
    
end

    %% c)plot current pedestrian position
    Plot_Ped_Current_Position = plot(groundTruthTraj_actual(1,1),groundTruthTraj_actual(1,2),...
                                        'o','MarkerSize',marker_size,'color',Ped_current_position_color );

    
    %% d)plot ground truth pedestrian trajectory - constant velocity model
    Plot_Ped_GT = plot(groundTruthTraj_actual(1,[1:2:end]),groundTruthTraj_actual(1,[2:2:end]),...
                                        'k--','LineWidth',line_width,'Color',Ped_current_position_color);

                                    
        
    %% e)plot predicted pedestrian trajectory - constant velocity model
    Plot_Ped_Pred_CV = plot(predPedTraj_CV(1,[1:2:end]),predPedTraj_CV(1,[2:2:end]),...
                                        'k--','LineWidth',line_width,'Color',Ped_CV_pred_color);

                                    
   %% f)plot predicted pedestrian trajectory - hybrid model
    Plot_Ped_Pred_Hybrid = plot(predPedTraj_Hybrid(1,[1:2:end]),predPedTraj_Hybrid(1,[2:2:end]),...
                                        'k--','LineWidth',line_width,'Color',Ped_Hybrid_pred_color);

      
    if makeVideo==1
        fr=getframe(gcf);
        writeVideo(vid,fr);
    end  
                                    
for ts = 2 : size(vehPosition_leftLane,1) %for every time step
    
    for agent_num = 1 : size(vehPosition_leftLane,2) %for each AV
        % left lane AV position update
        if (vehPosition_leftLane(ts,agent_num)>EnvParams.road_start | vehPosition_leftLane(ts,agent_num)<EnvParams.road_end | vehPosition_leftLane(ts,agent_num)==0)
            set(AV_position_left_lane(agent_num,1),'Position', [40, 0.2, 0.001, 0.001], 'EdgeColor', road_color, 'FaceColor', road_color);
        else
            xLeft = vehPosition_leftLane(ts,agent_num)-width/2;
            yBottom = -1.75-height/2;
            set(AV_position_left_lane(agent_num,1),'Position', [xLeft, yBottom, width, height],'EdgeColor',  AV_car_color_left_lane(agent_num,:), 'FaceColor',  AV_car_color_left_lane(agent_num,:));
        end
        
        % right lane AV position update
        if (vehPosition_rightLane(ts,agent_num)>EnvParams.road_start | vehPosition_rightLane(ts,agent_num)<EnvParams.road_end | vehPosition_rightLane(ts,agent_num)==0)
            set(AV_position_right_lane(agent_num,1),'Position', [40, 0.2, 0.001, 0.001], 'EdgeColor', road_color, 'FaceColor', road_color);
        else
            xLeft = vehPosition_rightLane(ts,agent_num)-width/2;
            yBottom = 1.75-height/2;
            set(AV_position_right_lane(agent_num,1),'Position', [xLeft, yBottom, width, height], 'EdgeColor',  AV_car_color_right_lane(agent_num,:), 'FaceColor',  AV_car_color_right_lane(agent_num,:));
        end

    
    
    end
    
    % pedestrian plots
    set(Plot_Ped_Current_Position,'XData',[groundTruthTraj_actual(ts,1)],'YData',[groundTruthTraj_actual(ts,2)]);
    set(Plot_Ped_GT,'XData',[groundTruthTraj_actual(ts,[1:2:end])],'YData',[groundTruthTraj_actual(ts,[2:2:end])]);
    set(Plot_Ped_Pred_CV,'XData',[predPedTraj_CV(ts,[1:2:end])],'YData',[predPedTraj_CV(ts,[2:2:end])]);
    set(Plot_Ped_Pred_Hybrid,'XData',[predPedTraj_Hybrid(ts,[1:2:end])],'YData',[predPedTraj_Hybrid(ts,[2:2:end])]);

                                   
    %% Step 7: save video

    if makeVideo==1
        fr=getframe(gcf);
        writeVideo(vid,fr);
    end   
        
end       


close(vid)

        
end