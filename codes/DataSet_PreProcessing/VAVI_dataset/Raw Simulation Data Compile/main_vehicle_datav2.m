clear all
close all

%% updated:04-24-2019
%This code --calculates the vehicle and interaction parameters at every time step such as
% 1) Distance to collision (x-distance between pedestrian and closest vehicle)
% 2) Closest lane closest Vehicle X-position, X-speed and x-acceleration
% 3) Adjacent lane closest vehicle X-position, X-speed and x-acceleration
% 4) Closest lane next closest Vehicle X-position, X-speed and x-acceleration
% 5) Adjacent lane next closest vehicle X-position, X-speed and x-acceleration

%% Important
% This version uses filtered pedestrian and vehicle data unlike the
% previous version; However, there coould be minor mislaignments in the
% vehicle and pedestrian data maybe due to different filter window
% lengths? There seemed to be no way around it! Vehicle data could not have
% high filter windows as it averages the position a lot when slowing down. 






%% filter parameters
windowSize = 5;
differenceOperationOrder = 2;  % order of difference operator
roundDen = 100;             % in case position needs to be rounded; the significant digits of position is higher than the noise.


for kk=1:30   % for every subject
    for jj=1:3  % for every unsignalized scenario

        %Read vehicle data  
        tic
        [VehPosRaw,~] = xlsread('VehiclePositionZoh.xlsx',6*(kk-1)+jj);     %Same as vehicle raw position data
        [PedZohData,~] = xlsread('PedestrianDiscreteDataW5.xlsx',6*(kk-1)+jj);     %Same as pedestrian raw position data
        toc
        
        %% Read vehicle  position data
        VehPosRaw = VehPosRaw(2:end,:); %remove the header row which has numbers
        % round off to 1 cm significant digits
        VehPosRaw = ceil(VehPosRaw*roundDen)/roundDen;
        
        %split into Lane A and Lane B; in Excel Lane B data comes first
        indLaneChange = find(VehPosRaw(:,3)==0,1,'last');   %Lane A starts again with time=0
        VehPosRawLaneB = VehPosRaw(1:indLaneChange-1,:);
        VehPosRawLaneA = VehPosRaw(indLaneChange:end,:);
        
%         remove the first and last indices of each column of position
%         data; the Zoh filter added data at the ends due to convolution;
%         this will affect the DTC calculation
        for ii=5:size(VehPosRawLaneA,2)
            StartInd = find(VehPosRawLaneA(:,ii)~=0,1,'first');
            EndInd = find(VehPosRawLaneA(:,ii)~=0,1,'last');
            StartInd2 = find(VehPosRawLaneB(:,ii)~=0,1,'first');
            EndInd2 = find(VehPosRawLaneB(:,ii)~=0,1,'last');

            VehPosRawLaneA([StartInd,EndInd],ii) = 0;
            VehPosRawLaneB([StartInd2,EndInd2],ii) = 0;           
        end
        clear StartInd EndInd StartInd2 EndInd2
        
        
        %sometimes vehicle data between lanes and pedestrian data have
        %different lengths
        [VehPosRawLaneA,VehPosRawLaneB] = VectorLength(VehPosRawLaneA,VehPosRawLaneB,PedZohData);
 
        %% vehicle filtered position, filtered velocity and filtered acceleration
        [VehPosZohLaneA,VehVelZohLaneA,VehAccZohLaneA] =  vehicleSpeedAcc(VehPosRawLaneA,windowSize,differenceOperationOrder);
        [VehPosZohLaneB,VehVelZohLaneB,VehAccZohLaneB] =  vehicleSpeedAcc(VehPosRawLaneB,windowSize,differenceOperationOrder);

        % NaN the vehicle data which have crossed the pedestrian or are
        % zero - easier for DTC calculation
        for ii=1:size(VehPosZohLaneA,1)
                VehPosZohLaneA(ii,VehPosZohLaneA(ii,:)<=PedZohData(ii,4))=NaN;
                VehPosZohLaneA(ii,VehPosZohLaneA(ii,:)==0)=NaN;
                              
                VehPosZohLaneB(ii,VehPosZohLaneB(ii,:)<=PedZohData(ii,4))=NaN;
                VehPosZohLaneB(ii,VehPosZohLaneB(ii,:)==0)=NaN;
        end
      
%%  DTC calculation       
        % find the closest car to pedestrian in both the lanes for each time step     
        [~,VehIndLaneA] = min(VehPosZohLaneA(:,4:end)-PedZohData(:,4),[],2);
        [~,VehIndLaneB] = min(VehPosZohLaneB(:,4:end)-PedZohData(:,4),[],2);
        VehIndLaneA = VehIndLaneA+3;    %index starts from 4 in previous step
        VehIndLaneB = VehIndLaneB+3;
        
        % copy the position, velocity and acceleration parameters of the
        % closet vehicle in each lane to new variables
        for ii=1:size(PedZohData,1)
            VehCloseLaneA(ii,1) = VehPosZohLaneA(ii,VehIndLaneA(ii));
            try
                VehNextLaneA(ii,1) = VehPosZohLaneA(ii,VehIndLaneA(ii)+1);
                VehNextSpeedLaneA(ii,1) = VehVelZohLaneA(ii,VehIndLaneA(ii)+1);
                VehNextAccLaneA(ii,1) = VehAccZohLaneA(ii,VehIndLaneA(ii)+1);
            catch
                VehNextLaneA(ii,1) = VehPosZohLaneA(ii,VehIndLaneA(ii));
                VehNextSpeedLaneA(ii,1) = VehVelZohLaneA(ii,VehIndLaneA(ii));
                VehNextAccLaneA(ii,1) = VehAccZohLaneA(ii,VehIndLaneA(ii));
            end
            VehCloseLaneB(ii,1) = VehPosZohLaneB(ii,VehIndLaneB(ii));
            try
                VehNextLaneB(ii,1) = VehPosZohLaneB(ii,VehIndLaneB(ii)+1);
                VehNextSpeedLaneB(ii,1) = VehVelZohLaneB(ii,VehIndLaneB(ii)+1);
                VehNextAccLaneB(ii,1) = VehAccZohLaneB(ii,VehIndLaneB(ii)+1);
            catch
                VehNextLaneB(ii,1) = VehPosZohLaneB(ii,VehIndLaneB(ii));
                VehNextSpeedLaneB(ii,1) = VehVelZohLaneB(ii,VehIndLaneB(ii));
                VehNextAccLaneB(ii,1) = VehAccZohLaneB(ii,VehIndLaneB(ii));
            end
            VehSpeedLaneA(ii,1) = VehVelZohLaneA(ii,VehIndLaneA(ii));
            VehSpeedLaneB(ii,1) = VehVelZohLaneB(ii,VehIndLaneB(ii));
            VehAccLaneA(ii,1) = VehAccZohLaneA(ii,VehIndLaneA(ii));
            VehAccLaneB(ii,1) = VehAccZohLaneB(ii,VehIndLaneB(ii));
        end
        
        % find the indices for which pedestrain is close to either lane A or lane B     
        indPedLaneA = find(PedZohData(:,5)<=0);
        indPedLaneB = find(PedZohData(:,5)>0);
        
        %% Vehicle and interaction parameter calculations
        % subject ID, scenario ID, Time, Lane ID (1 - Lane close to spawn region, 2 - other lane)
        temp = [[PedZohData(indPedLaneA,1:3),2*ones(size(indPedLaneA,1),1)];...
        [PedZohData(indPedLaneB,1:3),ones(size(indPedLaneB,1),1)]];
        
        % close vehicle position at every time step
        VehPos = [VehCloseLaneA(indPedLaneA);VehCloseLaneB(indPedLaneB)];
        VehPos = [temp(:,1:4),VehPos];
        VehPos = sortrows(VehPos,3);    % sort by time
        
        DTC = VehPos(:,5) - PedZohData(:,4);
        DTC = [VehPos(:,1:4),DTC];       
        DTC = sortrows(DTC,3); 
        
        VehSpeed = [VehSpeedLaneA(indPedLaneA);VehSpeedLaneB(indPedLaneB)];
        VehSpeed = [temp(:,3),VehSpeed];
        VehSpeed = sortrows(VehSpeed,1);
        
        VehAcc = [VehAccLaneA(indPedLaneA);VehAccLaneB(indPedLaneB)];
        VehAcc = [temp(:,3),VehAcc];
        VehAcc = sortrows(VehAcc,1);
        
        VehNextPos = [VehNextLaneA(indPedLaneA);VehNextLaneB(indPedLaneB)];
        VehNextPos = [temp(:,3),VehNextPos];
        VehNextPos = sortrows(VehNextPos,1);
        
        VehNextSpeed = [VehNextSpeedLaneA(indPedLaneA);VehNextSpeedLaneB(indPedLaneB)];
        VehNextSpeed = [temp(:,3),VehNextSpeed];
        VehNextSpeed = sortrows(VehNextSpeed,1);
        
        VehNextAcc = [VehNextAccLaneA(indPedLaneA);VehNextAccLaneB(indPedLaneB)];
        VehNextAcc = [temp(:,3),VehNextAcc];
        VehNextAcc = sortrows(VehNextAcc,1);
        
        VehAdj = [VehCloseLaneB(indPedLaneA);VehCloseLaneA(indPedLaneB)];
        VehAdj = [temp(:,3),VehAdj];
        VehAdj = sortrows(VehAdj,1);   
        
        VehSpeedAdj  = [VehSpeedLaneB(indPedLaneA);VehSpeedLaneA(indPedLaneB)];
        VehSpeedAdj  = [temp(:,3),VehSpeedAdj ];
        VehSpeedAdj  = sortrows(VehSpeedAdj,1);
        
        VehAccAdj  = [VehAccLaneB(indPedLaneA);VehAccLaneA(indPedLaneB)];
        VehAccAdj  = [temp(:,3),VehAccAdj];
        VehAccAdj  = sortrows(VehAccAdj,1);
        
        
        VehNextAdj = [VehNextLaneB(indPedLaneA);VehNextLaneA(indPedLaneB)];
        VehNextAdj = [temp(:,3),VehNextAdj];
        VehNextAdj = sortrows(VehNextAdj,1);   
        
        VehNextSpeedAdj  = [VehNextSpeedLaneB(indPedLaneA);VehNextSpeedLaneA(indPedLaneB)];
        VehNextSpeedAdj  = [temp(:,3),VehNextSpeedAdj ];
        VehNextSpeedAdj  = sortrows(VehNextSpeedAdj,1);
        
        VehNextAccAdj  = [VehAccLaneB(indPedLaneA);VehAccLaneA(indPedLaneB)];
        VehNextAccAdj  = [temp(:,3),VehNextAccAdj];
        VehNextAccAdj  = sortrows(VehNextAccAdj,1);
        
%         figure()
%         plot(DTC(:,5),'.','MarkerSize',10)
        
        
        %% Write to excel
        excelwrite = [DTC,VehPos(:,5),VehSpeed(:,2),VehAcc(:,2),VehNextPos(:,2),VehNextSpeed(:,2),VehNextAcc(:,2),...
                    VehAdj(:,2),VehSpeedAdj(:,2),VehAccAdj(:,2),VehNextAdj(:,2),VehNextSpeedAdj(:,2),VehNextAccAdj(:,2)];
        excelHeader = {'SubjectID','ScenarioID','Time','Lane ID','DTC','Same Lane Vehicle x-Position','Same Lane Vehicle x-Speed','Same Lane Vehicle x-Acceleration',...
                    'Same Lane Next vehcle position','Same Lane Next vehcle Speed','Same Lane Next vehcle Acceleration','Adjacent Lane Vehicle x-Position','Adjacent Lane Vehicle x-Speed',...
                    'Adjacent Lane Vehicle x-Acceleration','Adjacent Lane Next vehcle position','Adjacent Lane Next vehcle Speed','Adjacent Lane Next vehcle Acceleration'};
        
        xlswrite('VehicleDTCSpeedData_allFourVehicles.xlsx',excelwrite,6*(kk-1)+jj,'A2');   
        xlswrite('VehicleDTCSpeedData_allFourVehicles.xlsx',excelHeader,6*(kk-1)+jj,'A1');
        x=1;
        
    end
end       