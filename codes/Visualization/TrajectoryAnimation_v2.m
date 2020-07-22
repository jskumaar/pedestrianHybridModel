clear all
close all

load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat')
load('Performance_Hybrid_KF_100_SVM_StartGap.mat')

ind = 12;       % the crossing number to plot
PedData = DataPredict{ind}.PedestrianPosition;
VehData = [DataPredict{ind}.VehiclePosition, DataPredict{ind}.NextVehiclePosition, DataPredict{ind}.AdjacentVehiclePosition];

Indices.VehicleSameLane = find( diff(DataPredict{ind}.VehiclePosition) > 0) + 1;
Indices.NextVehicleSameLane = find( diff(DataPredict{ind}.NextVehiclePosition) > 0);
Indices.VehicleAdjacentLane = find( diff(DataPredict{ind}.AdjacentVehiclePosition) > 0);
N_VehicleSameLane = length(Indices.VehicleSameLane);
N_NextVehicleSameLane = length(Indices.NextVehicleSameLane);
N_VehicleAdjacentLane = length(Indices.VehicleAdjacentLane);

N = length(PedData);

lane_ID2 = find(DataPredict{ind}.VehicleLaneID==2);
lane_ID1 = find(DataPredict{ind}.VehicleLaneID==1);

% Vehicle_1 =  [DataPredict{ind}.VehiclePosition(lane_ID2);DataPredict{ind}.AdjacentVehiclePosition(lane_ID1)];
% Vehicle_2 =  [DataPredict{ind}.VehiclePosition(lane_ID1);DataPredict{ind}.AdjacentVehiclePosition(lane_ID2)];
% Vehicle_3 =  [DataPredict{ind}.NextVehiclePosition(lane_ID2);DataPredict{ind}.NextVehiclePosition(lane_ID1)-7];
% Vehicle_4 =  [DataPredict{ind}.NextVehiclePosition(lane_ID1);DataPredict{ind}.NextVehiclePosition(lane_ID2)+7];

%% Initialize the Animation
figure()
%Start Recording
vid=VideoWriter(strcat('Trajectory_',num2str(5),'.avi'));
vid.FrameRate=10;
open(vid);
makeVideo=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Plot the layout
rectangle('Position',[-12,-7,25,14]);hold on
axis([-13 16 -8 8]) %focus only on pedestrian path
rectangle('Position',[-1,-3.5,2,7]);hold on
plot([-12,13],[3.5,3.5],'k');hold on;
plot([-12,13],[-3.5,-3.5],'k');hold on;
plot([-12,13],[0,0],'k');hold on;
xticks([])
yticks([])
set(gca,'Visible','off')    %removes the axes lines

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Insert the simulation environment image (for later)







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% initial positions of agents (pedestrian, vehicles)
markSize=8;
% data to plot
actualTraj = PedData;       %full actual trajectory; will always be visible

ind2 = sort([5:4:241,6:4:242]);
predTraj = Performance{1,1}.Predicted_x(:,ind2);    %predicted trajectory, for every time step will incrementally show the predicted positions
predTrajVar = Performance{1,1}.Predicted_x_cov(:,ind2); % variance of the positions
predQ = Performance{1,1}.Predicted_q(:,2:61);   %predicted discrete state


% plot for every time step
for ii=1:N
    
    dispActualTime = text(4,6,strcat('Actual TimeStep = ',num2str(ii)),'FontSize',10);
     
    % actual pedestrian trajectory
    actualTrajPlot = plot(actualTraj(1:200,1),actualTraj(1:200,2),'.b','markersize',markSize);
       
    
    for jj=2:60
        
        % actual vehicle trajectory
        if 
        actualVehPlot = plot(actualTraj(1:200,1),actualTraj(1:200,2),'.b','markersize',markSize);
   
          
        
        % pedestrian trajectory
        predTrajPlot = plot(predTraj(ii,[1:2:jj*2-1]),predTraj(ii,[2:2:jj*2]),'.r','markersize',markSize);
        predTrajCovPlot = ellipse(predTraj(ii,[jj*2-1]),predTraj(ii,[jj*2]),predTrajVar(ii,[jj*2-1]),predTrajVar(ii,[jj*2]));
        dispTime = text(4,5,strcat('Predict TimeStep = ',num2str(ii+jj)),'FontSize',10);
        dispQ = text(predTraj(ii,[jj*2-1]),predTraj(ii,[jj*2])+1,strcat('Action: ',num2str(predQ(ii,jj))),'FontSize',10);
    end
end

















for ii=2:size(UserPos,1)-predHor-1

%Actual pedestrian trajectory
    set(actualPos,'XData',UserPos(ii,4),'YData',UserPos(ii,5),'markersize',markSize);
    set(actualTraj,'XData',UserPos(ii:ii+predHor,4),'YData',UserPos(ii:ii+predHor,5));
    set(predTraj,'XData',predictedTraj(ii,2*(1:predHor+1)-1),'YData',predictedTraj(ii,2*(1:predHor+1)));            
    set(dispTime,'Position',[12,6,0],'String',num2str(UserPos(ii,3)));
    set(gazeTraj,'XData',[UserPos(ii,4),GazePointGraphDisplay(ii,1)],'YData',[UserPos(ii,5),GazePointGraphDisplay(ii,2)]);            
    set(dispGaze,'Position',[9,4,0],'String',GazeGraphDisplay(ii,1));

    for mm=-1:1
        try
            CarPos = VehPosZohLaneA(ii,indPedLaneA(ii)+mm);
        catch
            CarPos = [];
        end
        if (CarPos>-12&CarPos<15&~isnan(CarPos)&~isempty(CarPos)&CarPos~=0) 
            set(VehicleLaneA(mm+2),'Position',[CarPos-1.5,-1,2.5,1.5]);
        else
            set(VehicleLaneA(mm+2),'Position',[-10,-8,0.1,0.1]);
        end
        try
            CarPos = VehPosZohLaneB(ii,indPedLaneB(ii)+mm);
        catch
            CarPos = [];
        end
        if (CarPos>-12&CarPos<15&~isnan(CarPos)&~isempty(CarPos)&CarPos~=0)  
            set(VehicleLaneB(mm+2),'Position',[CarPos-1.5,1,2.5,1.5]);
        else
            set(VehicleLaneA(mm+2),'Position',[-10,-8,0.1,0.1]);
        end
    end

    drawnow;
    hold on;           
    fr=getframe(gcf);
    writeVideo(vid,fr);
end
close(vid)
PlotTime=toc;