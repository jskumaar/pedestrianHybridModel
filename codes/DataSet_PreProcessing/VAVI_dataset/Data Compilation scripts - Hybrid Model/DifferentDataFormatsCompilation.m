%% Different Data Formats

WindowMethod = 1;
filename1 = 'RollingWindow_0.1s_';
% filename1 = 'RollingWindow_0.4s_';
% filename1 = 'GapRollingWindow_0.1s_';
% filename1 = 'GapRollingWindow_0.4s_';
DataMethod = 1;
filename2 = 'TimeSeriesData.xlsx';
% filename2 = 'SummaryData.xlsx';
filename = strcat(filename1,filename2);

% for kk=1:30
%     for jj=1:3
%         
%         Data = HybridReadData(kk,jj);
%         N = length(Data.SubjectID);
if DataMethod==1       
    InputData = zeros(N,
 


        for ii=1:N
           
            %Window indices   
            switch WindowMethod
                case 1
                    if ii<N
                        indices.window = [1:ii];
                    else
                        indices.window = [ii-WS+1:ii];
                    end
                case 2
                    if ii<N
                        indices.window = [1:4:ii];
                    else
                        indices.window = [ii-WS+1:4:ii];
                    end
                case 3
                    if (ii==GapEnd(GapCount))
                        GapCount=GapCount+1;
                    end
                    indices.window = [GapStart(GapCount):ii];
                case 4
                    if (ii==GapEnd(GapCount))
                        GapCount=GapCount+1;
                    end
                    indices.window = [GapStart(GapCount):WS:ii];
            end


            S = vartype('numeric');
            FullData = Data{indices.window,S};
            
            
            
            

            if DataMethod==2
                InputData(ii,:) = nanmean(FullData,1);
                window = ones(length(indices.window),1)/length(indices.window);
                GazeAtVehicleRatio = conv(FullData(:,11),window,'same');
            else
                InputData(ii,:) = reshape(FullData,[size(FullData,1)*size(FullData,2),1]);
            end
        
        end
        
        excelHeader = {'SubjectID','ScenarioID','CrossingID','Time','PedestrianPosition','PedestrianCartesianVelocity','PedestrianAbsoluteVelocity',...
            'PedestrianDiscreteState','PedestrianGazeAtVehicle','PedestrianDistancetoCW','PedestrianDistancetoCurb','PedestrianHeading',...
            'PedestrianCumulativeWaitTime','VehicleLaneID','VehilclePosition','VehicleSpeed','VehicleAcceleration','NextVehiclePosition',...
            'AdjacentVehiclePosition','AdjacentVehicleSpeed','AdjacentVehicleAcceleration','PedestrianVehicleDistance','VehicleTimeGaptoPedestrian'};

         xlswrite(filename,excelHeader,6*(kk-1)+jj,'A1');
         xlswrite(filename,InputData,6*(kk-1)+jj,'A2');
        
%     end
% end
      

