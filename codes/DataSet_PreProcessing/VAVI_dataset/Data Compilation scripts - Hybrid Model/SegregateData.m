%% Segregate Data
clear all 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Inputs

M = 21;             %No. of features releavant features

WindowMethodMat = [1,2,3,4,1,2,3,4];
DataMethodMat = [1,1,1,1,2,2,2,2];
WSMat = [10,32,10,32,10,32,10,32]; TSMat = [1,4,1,4,1,4,1,4];

filename{1} = 'Rolling_Window_0.1s.mat';           %Window-1, Data-1
filename{2} = 'Rolling_Window_0.4s.mat';           %Window-2, Data-1
filename{3} = 'Rolling_Window_Gap_0.1s.mat';       %Window-3, Data-1
filename{4} = 'Rolling_Window_Gap_0.4s.mat';       %Window-4, Data-1
filename{5} = 'Rolling_Window_Mean_0.1s.mat';      %Window-1, Data-2
filename{6} = 'Rolling_Window_Mean_0.4s.mat';      %Window-1, Data-2
filename{7} = 'Rolling_Window_Gap_Mean_0.1s.mat';  %Window-3, Data-2
filename{8} = 'Rolling_Window_Gap_Mean_0.4s.mat';  %Window-3, Data-2

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[ActivityData,~] = xlsread('DiscreteStateEventIndicesW5.xlsx');
[GapData,~] = xlsread('VehicleGapTimes_onlyforGapTimes.xlsx');
GapStart = GapData(:,4);
GapEnd = GapData(:,8);

for nn=2:8
    WS = WSMat(nn);
    TS = TSMat(nn);
    WindowMethod = WindowMethodMat(nn);
    DataMethod = DataMethodMat(nn);
    excelname = filename{nn};
    excelData=[];

for kk=1:30
    for jj=1:3
        Data = HybridReadData(kk,jj);   % read  and compiles data for the classifier model
           GapCount = find(GapData(:,1)==kk&GapData(:,2)==jj,1,'first');
           N = size(Data,1);
           
        % intialize the InputData for Model        
        if (WindowMethod==1)
            OverallIndex = [1:N];
        elseif (WindowMethod==1)
            OverallIndex = [1:TS:N];
        elseif (WindowMethod==3)
            OverallIndex = [GapStart(GapCount):N];
        else
            OverallIndex = [GapStart(GapCount):TS:N];
        end
        
        if DataMethod==1
             InputData = zeros(length(OverallIndex),M*(WS/TS)+4);
        else
             InputData = zeros(length(OverallIndex),M+4);      
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% For each time step
        ii=1;

        while ii<=length(OverallIndex)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %% Window indices   
            switch WindowMethod
                case 1  %rolling
                    if OverallIndex(ii)<WS
                        indices.window = [1:OverallIndex(ii)];
                    else
                        indices.window = [OverallIndex(ii)-WS+1:OverallIndex(ii)];
                    end
                case 2
                    if OverallIndex(ii)<WS
                        indices.window = [1:TS:OverallIndex(ii)];
                    else
                        indices.window = [OverallIndex(ii)-WS+1:TS:OverallIndex(ii)];
                    end
                case 3
                    if (OverallIndex(ii)>GapEnd(GapCount))
                        GapCount=GapCount+1;
                    end
                    if (OverallIndex(ii)-GapStart(GapCount))<WS
                        indices.window = [GapStart(GapCount):OverallIndex(ii)];
                    else
                        indices.window = [OverallIndex(ii)-WS+1:OverallIndex(ii)];
                    end
                case 4
                    if (OverallIndex(ii)>GapEnd(GapCount))
                        GapCount=GapCount+1;
                    end
                    if (OverallIndex(ii)-GapStart(GapCount))<WS
                        indices.window = [GapStart(GapCount):4:OverallIndex(ii)];
                    else
                        indices.window = [OverallIndex(ii)-WS+1:4:OverallIndex(ii)];
                    end
            end
            S = vartype('numeric');
            FullData = Data{indices.window,S};
            L = size(FullData,1);
            N2 = L*M+4;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            switch DataMethod
                case 1  %True Data
                    if OverallIndex(ii)==1
                        InputData(ii,1:N2) = reshape(FullData',[1,N2])';   
                    else
                        toWrite = [FullData(1,1:M+4),reshape(FullData(2:L,5:end)',[1,N2-M-4])];
                        InputData(ii,1:N2) = toWrite;
                    end
                case 2  %Summary statistics (Mean)
                      InputData(ii,:) = nanmean(FullData,1); 
            end  
            %update ii
            ii = ii+1;
        end

       x=1;
       excelData = [excelData;InputData];


    end
end

excelheader = {'SubjectID','ScenarioID','CrossingID','Time','PedestrianPosition','PedestrianCartesianVelocity','PedestrianAbsoluteVelocity',...
            'PedestrianDiscreteState','PedestrianGazeAtVehicle','PedestrianDistancetoCW','PedestrianDistancetoCurb','PedestrianHeading',...
            'PedestrianCumulativeWaitTime','VehicleLaneID','VehiclePosition','VehicleSpeed','VehicleAcceleration','NextVehiclePosition',...
            'AdjacentVehiclePosition','AdjacentVehicleSpeed','AdjacentVehicleAcceleration','PedestrianVehicleDistance','VehicleTimeGaptoPedestrian'};

% xlswrite(excelname,excelData,1,'A2');
% xlswrite(excelname,excelheader,1,'A1');

save(excelname,'excelData');

end
