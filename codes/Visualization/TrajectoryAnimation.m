load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat')
load('Performance_Hybrid_KF_100_SVM_StartGap.mat')

PedData = DataPredict{ind}.PedestrianPosition;
lane_ID2 = find(DataPredict{ind}.VehicleLaneID==2);
lane_ID1 = find(DataPredict{ind}.VehicleLaneID==1);
ind = 12;

Vehicle_1 =  [DataPredict{ind}.VehiclePosition(lane_ID2);DataPredict{ind}.AdjacentVehiclePosition(lane_ID1)];
Vehicle_2 =  [DataPredict{ind}.VehiclePosition(lane_ID1);DataPredict{ind}.AdjacentVehiclePosition(lane_ID2)];
Vehicle_3 =  [DataPredict{ind}.NextVehiclePosition(lane_ID2);DataPredict{ind}.NextVehiclePosition(lane_ID1)-7];
Vehicle_4 =  [DataPredict{ind}.NextVehiclePosition(lane_ID1);DataPredict{ind}.NextVehiclePosition(lane_ID2)+7];

%% Animation
figure()
%Start Recording
vid=VideoWriter(strcat('Trajectory_',num2str(5),'.avi'));
vid.FrameRate=10;
open(vid);
makeVideo=1;

tic
%Scenario
rectangle('Position',[-12,-7,25,14]);hold on
axis([-13 16 -8 8]) %focus only on pedestrian path
rectangle('Position',[-1,-3.5,2,7]);hold on
plot([-12,13],[3.5,3.5],'k');hold on;
plot([-12,13],[-3.5,-3.5],'k');hold on;
plot([-12,13],[0,0],'k');hold on;
xticks([])
yticks([])
set(gca,'Visible','off')


% initial positions of agents (pedestrian, vehicles)
markSize=8;
actualTraj = plot(PedData(:,1),PedData(:,2)); hold on;
predTraj = plot(predictedTraj(1,2*(1:predHor+1)-1),predictedTraj(1,2*(1:predHor+1)),'*r'); hold on;
dispTime = text(12,6,num2str(PedZohData(1,3)),'FontSize',14);
actualPos = plot(PedZohData(1,4),PedZohData(1,5),'bsquare','markersize',markSize);
gazeTraj = plot([PedZohData(1,4),GazePointGraphDisplay(1,1)],[PedZohData(1,5),GazePointGraphDisplay(1,2)],'--g');
dispGaze = text(9,4,GazeGraphDisplay(1,1),'FontSize',14);

for mm=-1:1      %Note Lane A and Lane B variable names and data are interchanged here
    try
        CarPos = VehPosZohLaneA(ii,VehIndLaneA(ii)+mm);  
    catch
        CarPos = [];
    end          
    if (CarPos>-12&CarPos<15&~isnan(CarPos)&~isempty(CarPos)&CarPos~=0)   %To ensure it is within plot range
        VehicleLaneA(mm+2) = rectangle('Position',[CarPos-1.5,-1,3,1.5]);hold on
    else
        VehicleLaneA(mm+2) = rectangle('Position',[-10,-8,0.1,0.1]);hold on;
    end
    try
        CarPos = VehPosZohLaneB(1,VehIndLaneB(1)+mm);        %Note Lane A and Lane B variable names and data are interchanged here               
    catch
        CarPos = [];
    end
    if (CarPos>-12&CarPos<15&~isnan(CarPos)&~isempty(CarPos)&CarPos~=0)
        VehicleLaneB(mm+2) = rectangle('Position',[CarPos-1.5,1,3,1.5]);hold on                
     else
        VehicleLaneB(mm+2) = rectangle('Position',[-10,-8,0.1,0.1]);hold on
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