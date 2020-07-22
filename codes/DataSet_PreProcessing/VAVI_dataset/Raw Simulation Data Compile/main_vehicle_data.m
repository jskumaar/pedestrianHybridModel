% clear all
% close all
% 
VehPosMatrix = [];
VehVelMatrix = [];
VehAccMatrix = [];

for kk=1:30
    for jj=1:3

        %Read vehicle data  
        tic
        %[EventsData,~] = xlsread('EventsData.xlsx',1);
        [VehPosZoh,~] = xlsread('VehiclePositionZoh.xlsx',6*(kk-1)+jj);
        [VehVelZoh,~] = xlsread('VehicleVelocityZoh.xlsx',6*(kk-1)+jj);
        [VehAccZoh,~] = xlsread('VehicleAccelerationZoh.xlsx',6*(kk-1)+jj);
        [PedZohData,~] = xlsread('PedestrianDataZoh.xlsx',6*(kk-1)+jj);
        toc

        %% Read vehicle  position data
        VehPosZoh = VehPosZoh(2:end,:); %remove the header row

        %replace '0' in vehicle position data with 'NaN'; there should be a
        %much more elegant way to do this!
        indNaN = VehPosZoh(:,4:end)==0;
        temp = VehPosZoh(:,4:end);
        temp(indNaN)=NaN;        
        VehPosZoh(:,4:end) = temp;

        %split into Lane A and Lane B; in Excel Lane B data comes first
        indLaneChange = find(VehPosZoh(:,3)==0,1,'last');   %Lane A starts again with time=0
        VehPosZohLaneB = VehPosZoh(1:indLaneChange-1,:);
        VehPosZohLaneA = VehPosZoh(indLaneChange:end,:);

        %remove the first and last indices of each column of position
        %data; the Zoh filter added data at the ends due to convolution;
        %this will affect the DTC calculation
        for ii=5:size(VehPosZohLaneA,2)
            StartInd = find(~isnan(VehPosZohLaneA(:,ii)),1,'first');
            EndInd = find(~isnan(VehPosZohLaneA(:,ii)),1,'last');
            StartInd2 = find(~isnan(VehPosZohLaneB(:,ii)),1,'first');
            EndInd2 = find(~isnan(VehPosZohLaneB(:,ii)),1,'last');

            VehPosZohLaneA([StartInd,EndInd],ii) = NaN;
            VehPosZohLaneB([StartInd2,EndInd2],ii) = NaN;           
        end
        clear StartInd EndInd StartInd2 EndInd2

        %sometimes vehicle data between lanes and with pedestrian data have
        %different lengths
        [VehPosZohLaneA,VehPosZohLaneB] = VectorLength(VehPosZohLaneA,VehPosZohLaneB,PedZohData);

        %NaN the vehicle data which have crossed the pedestrian
        for ii=1:size(VehPosZohLaneA,1)
                VehPosZohLaneA(ii,VehPosZohLaneA(ii,:)<=PedZohData(ii,4))=NaN;
                VehPosZohLaneB(ii,VehPosZohLaneB(ii,:)<=PedZohData(ii,4))=NaN;
        end

        %% Vehicle velocity and acceleration
        VehVelZohLaneA = PedZohData(:,1:3);
        VehVelZohLaneB = PedZohData(:,1:3);
        if size(VehVelZoh,1)<2
            for mm=1:size(VehPosZoh,2)-4
                 XZohDiff = diff(VehPosZohLaneA(:,mm+4));
                 VelA =  (XZohDiff(1:end-1) +  XZohDiff(2:end))/0.2;
                 VehVelZohLaneA = [VehVelZohLaneA,[0;VelA;0]];
                 
                 XZohDiff = diff(VehPosZohLaneB(:,mm+4));
                 VelB =  (XZohDiff(1:end-1) +  XZohDiff(2:end))/0.2;
                 VehVelZohLaneB = [VehVelZohLaneB,[0;VelB;0]];          
               
            end            
            
        else
            VehVelZohLaneB = VehVelZoh(2:indLaneChange-1,:);     %Excel data in Lane and Lane B interchanged; soo changing variables names here
            VehVelZohLaneA = VehVelZoh(indLaneChange:end,:);    % remove top header row
        end

        VehAccZohLaneB = VehAccZoh(2:indLaneChange-1,:);     %Excel data in Lane and Lane B interchanged; soo changing variables names here
        VehAccZohLaneA = VehAccZoh(indLaneChange:end,:);        

        [VehVelZohLaneA,VehVelZohLaneB] = VectorLength(VehVelZohLaneA,VehVelZohLaneB,PedZohData);
        [VehAccZohLaneA,VehAccZohLaneB] = VectorLength(VehAccZohLaneA,VehAccZohLaneB,PedZohData);

        
%%  DTC calculation    
        
        % find the closest car in both the lanes for each time step     
        [~,VehIndLaneA] = min(VehPosZohLaneA(:,4:end)-PedZohData(:,4),[],2);
        [~,VehIndLaneB] = min(VehPosZohLaneB(:,4:end)-PedZohData(:,4),[],2);
        VehIndLaneA = VehIndLaneA+3;    %index starts from 4 in previous step
        VehIndLaneB = VehIndLaneB+3;
        
        for ii=1:size(PedZohData,1)
            VehCloseLaneA(ii,1) = VehPosZohLaneA(ii,VehIndLaneA(ii));                
            VehNextLaneA(ii,1) = VehPosZohLaneA(ii,VehIndLaneA(ii)+1);
            VehCloseLaneB(ii,1) = VehPosZohLaneB(ii,VehIndLaneB(ii));
            VehNextLaneB(ii,1) = VehPosZohLaneB(ii,VehIndLaneB(ii)+1);
            VehSpeedLaneA(ii,1) = VehVelZohLaneA(ii,VehIndLaneA(ii));
            VehSpeedLaneB(ii,1) = VehVelZohLaneB(ii,VehIndLaneB(ii));
            VehAccLaneA(ii,1) = VehAccZohLaneA(ii,VehIndLaneA(ii));
            VehAccLaneB(ii,1) = VehAccZohLaneB(ii,VehIndLaneB(ii));
        end
        
        % find the indices for which pedestrain is close to either lane A or lane B     
        indPedLaneA = find(PedZohData(:,5)<=0);
        indPedLaneB = find(PedZohData(:,5)>0);
        
        %Distance to Collision
        temp = [[PedZohData(indPedLaneA,1:3),2*ones(size(indPedLaneA,1),1)];...
        [PedZohData(indPedLaneB,1:3),ones(size(indPedLaneB,1),1)]];
    
        VehPos = [VehCloseLaneA(indPedLaneA);VehCloseLaneB(indPedLaneB)];
        VehPos = [temp(:,3),VehPos];
        VehPos = sortrows(VehPos,1);
        
        DTC = VehPos(:,2) - PedZohData([indPedLaneA;indPedLaneB],4);
        DTC = [temp,DTC];       
        DTC = sortrows(DTC,[1,2,3]); 
        
        VehSpeed = [VehSpeedLaneA(indPedLaneA);VehSpeedLaneB(indPedLaneB)];
        VehSpeed = [temp(:,3),VehSpeed];
        VehSpeed = sortrows(VehSpeed,1);
        
        VehAcc = [VehAccLaneA(indPedLaneA);VehAccLaneB(indPedLaneB)];
        VehAcc = [temp(:,3),VehAcc];
        VehAcc = sortrows(VehAcc,1);
        
        VehNext = [VehNextLaneA(indPedLaneA);VehNextLaneB(indPedLaneB)];
        VehNext = [temp(:,3),VehNext];
        VehNext = sortrows(VehNext,1);
        
        VehAdj = [VehCloseLaneB(indPedLaneA);VehCloseLaneA(indPedLaneB)];
        VehAdj = [temp(:,3),VehAdj];
        VehAdj = sortrows(VehAdj,1);

%% Events indices        
%          PlotEvents = EventsData(36*(kk-1)+6*(jj-1)+1:36*(kk-1)+6*(jj),3:8);
%          PlotEvents = reshape(PlotEvents,[size(PlotEvents,1)*size(PlotEvents,2),1]);
%          PlotEvents = [PlotEvents,reshape(repmat([1:6],6,1),[36,1])];
% 
%          waitStart = int32(PlotEvents(1:6,1)*10)+1;
%          waitEnd = int32(PlotEvents(7:12,1)*10)+1;
%          CrossStart = waitEnd;
%          CrossEnd = int32(PlotEvents(19:24,1)*10)+1;
%          TaskStart = CrossEnd;
%          TaskEnd = int32(PlotEvents(31:36,1)*10)+1;
%  
%         for ii=1:6
%             VehVelMatrix = [VehVelMatrix,VehSpeed([CrossStart(ii)-50:CrossStart(ii)+10,CrossEnd(ii)-11:CrossEnd(ii)+20])];
%             VehAccMatrix = [VehAccMatrix,VehAcc([CrossStart(ii)-50:CrossStart(ii)+10,CrossEnd(ii)-11:CrossEnd(ii)+20])];       
%             VehPosMatrix = [VehPosMatrix,VehPos([CrossStart(ii)-50:CrossStart(ii)+10,CrossEnd(ii)-11:CrossEnd(ii)+20])]; 
%         end
        
        
        %% Write to excel
        excelwrite = [DTC,VehPos(:,2),VehSpeed(:,2),VehAcc(:,2),VehNext(:,2),VehAdj(:,2)];
        excelHeader = {'SubjectID','ScenarioID','Time','Lane ID','DTC','Vehicle x-Position','Vehicle x-Speed','Vehicle x-Acceleration','Next vehcle position','Vehicle Adjacent Lane'};
        
        xlswrite('VehicleDTCSpeedData_unfiltPed.xlsx',excelwrite,6*(kk-1)+jj,'A2');   
        xlswrite('VehicleDTCSpeedData_unfiltPed.xlsx',excelHeader,6*(kk-1)+jj,'A1');
        
        
        x=1;
    end
end       

% save('VehicleSpeed.mat','VehPosMatrix','VehVelMatrix','VehAccMatrix')