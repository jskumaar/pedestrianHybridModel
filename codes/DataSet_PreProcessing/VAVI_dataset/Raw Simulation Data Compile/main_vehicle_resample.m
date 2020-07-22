% The simulation log files that stores vehicle data haave varying sampling
% times; This script resamples the data to have a constant sampling time of
% 0.1 s




clear all
close all

for kk = 1:30 %for every subject
    for jj = 1:6 %for every scenario

        % load compiled simulation data
        filename = char(strcat('Subject_',num2str(kk),'_','Cond_',num2str(jj)));
        load(filename)
        
        
        if jj<4
            userData = unsigData.userData;
            crossData = unsigData.crossData;
            waitData = unsigData.waitData;
            jaywalkData = unsigData.jaywalkData;
            vehicleData = unsigData.vehicleData;
            taskData = unsigData.taskData;
        else
            userData = sigData.userData;
            crossData = sigData.crossData;
            waitData = sigData.waitData;
            jaywalkData = sigData.jaywalkData;
            vehicleData = sigData.vehicleData;
            taskData = sigData.taskData;
        end

 %% split data lanewise       
     vehicleDataLaneA = vehicleData.veh_pos(vehicleData.veh_pos(:,3)<0,:);
     vehicleDataLaneB = vehicleData.veh_pos(vehicleData.veh_pos(:,3)>0,:);
   
%%   for lane A
     temp.uniqueVehicleIDsLaneA = unique(vehicleDataLaneA(:,1));
     temp.uniqueTimeStepsLaneA = unique(vehicleDataLaneA(:,4));
     
     matALength = size(temp.uniqueTimeStepsLaneA,1);
     matAWidth = size(temp.uniqueVehicleIDsLaneA,1);
     
     vehicleDataHorSortedLaneA = zeros(matALength,matAWidth+4);    %additional columns for Subject ID, Scenario ID, Lane ID, Time
     
     for ll = 1:matALength
         for mm = 1:matAWidth
             try
             vehicleDataHorSortedLaneA(ll,mm+4) = vehicleDataLaneA((vehicleDataLaneA(:,1)==temp.uniqueVehicleIDsLaneA(mm) & vehicleDataLaneA(:,4)==temp.uniqueTimeStepsLaneA(ll)),2);  
             catch
             vehicleDataHorSortedLaneA(ll,mm+4) = 0;
             end
         end
     end
         
     
     % resample data based on linear interpolation (ignore values at the
     % edge of the time period for the particular vehicle; additional
     % values added due to interpolation
     resampleTimeA = [0:0.1:round(temp.uniqueTimeStepsLaneA(end),1)]';
     vehicleDataResampledLaneA = [];
     vehicleDataResampledLaneA(:,5:matAWidth+4) = interp1(temp.uniqueTimeStepsLaneA,vehicleDataHorSortedLaneA(:,5:matAWidth+4),[0:0.1:round(temp.uniqueTimeStepsLaneA(end),1)],'pchip');
     
     % filter data
     windowSize = 3; 
     b = (1/windowSize)*ones(1,windowSize);
     a = 1;
     vehicleDataResampledLaneAFiltered = filter(b,a,vehicleDataResampledLaneA);
     
     
%%   for lane B
     temp.uniqueVehicleIDsLaneB = unique(vehicleDataLaneB(:,1));
     temp.uniqueTimeStepsLaneB = unique(vehicleDataLaneB(:,4));
     vehicleDataHorSortedLaneB = zeros(size(temp.uniqueVehicleIDsLaneB,1)+4,size(temp.uniqueTimeStepsLaneB,1))';
        
     matBLength = size(temp.uniqueTimeStepsLaneB,1);
     matBWidth = size(temp.uniqueVehicleIDsLaneB,1);
     
    for ll = 1:matBLength
         for mm = 1:matBWidth
             try
             vehicleDataHorSortedLaneB(ll,mm+4) = vehicleDataLaneB((vehicleDataLaneB(:,1)==temp.uniqueVehicleIDsLaneB(mm) & vehicleDataLaneB(:,4)==temp.uniqueTimeStepsLaneB(ll)),2);  
             catch
             vehicleDataHorSortedLaneB(ll,mm+4) = 0;
             end
             
         end
    end
     
     resampleTimeB = [0:0.1:round(temp.uniqueTimeStepsLaneB(end),1)]';
     vehicleDataResampledLaneB = [];
     vehicleDataResampledLaneB(:,5:matBWidth+4) = interp1(temp.uniqueTimeStepsLaneB,vehicleDataHorSortedLaneB(:,5:matBWidth+4),[0:0.1:round(temp.uniqueTimeStepsLaneB(end),1)],'pchip');
     
     vehicleDataResampledLaneBFiltered = filter(b,a,vehicleDataResampledLaneB);
     
        %% add subject and scenario details
        vehicleDataHorSortedLaneA(:,1:3) = repmat([kk,jj,1],matALength,1);
        vehicleDataHorSortedLaneA(:,4) = temp.uniqueTimeStepsLaneA;
        
        vehicleDataHorSortedLaneB(:,1:3) = repmat([kk,jj,2],matALength,1);
        vehicleDataHorSortedLaneB(:,4) = temp.uniqueTimeStepsLaneB;
        
        vehicleDataResampledLaneA(:,1:3) = repmat([kk,jj,1],size(resampleTimeA,1),1);
        vehicleDataResampledLaneA(:,4) = resampleTimeA;
        
        vehicleDataResampledLaneB(:,1:3) = repmat([kk,jj,2],size(resampleTimeB,1),1);
        vehicleDataResampledLaneB(:,4) = resampleTimeB;
        
        %% write to excel
        excelHeader = {'SubjectID','ScenarioID','LaneID','Time'};
        dataCell = num2cell(1:max(matAWidth,matBWidth));
        for ll=1:max(matAWidth,matBWidth)
            excelHeader = [excelHeader, num2str(ll)];
        end
        
        xlswrite('VehicleDataRaw.xlsx',[vehicleDataHorSortedLaneA;vehicleDataHorSortedLaneB],6*(kk-1)+jj,'A2');
        xlswrite('VehicleDataRaw.xlsx',excelHeader,6*(kk-1)+jj,'A1');        
        xlswrite('VehicleDataResampled.xlsx',[vehicleDataResampledLaneA;vehicleDataResampledLaneB],6*(kk-1)+jj,'A2');
        xlswrite('VehicleDataResampled.xlsx',excelHeader,6*(kk-1)+jj,'A1');
        xlswrite('VehicleDataResampledFiltered.xlsx',[vehicleDataResampledLaneAFiltered;vehicleDataResampledLaneBFiltered],6*(kk-1)+jj,'A2');
        xlswrite('VehicleDataResampledFiltered.xlsx',excelHeader,6*(kk-1)+jj,'A1');
        
        
        
    end
end