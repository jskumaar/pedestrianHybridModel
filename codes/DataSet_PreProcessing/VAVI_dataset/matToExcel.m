% This script copies the user position data and interval data from the compiled
% matlab files and writes to an excel
% it also applies a simple filter to the user position data which is saved
% in a separate excel file



clear all
close all


EventsData = [];

for kk = 1:30 %for every subject
    for jj = 1:3 %for every scenario
        
        PedestrianPositionFiltered = [];

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

        
        x=1;
        
        % compiled event parameters
        CrossStartTime = [crossData.crossStart_ind(1:6)*simParam.del_t]';       %Subject 9, Condition 2 has 4 crossings
        CrossEndTime = [crossData.crossEnd_ind(1:6)*simParam.del_t]';
        
        WaitStartTime =  [waitData.waitStart_ind(1:6)*simParam.del_t]';
        WaitEndTime =  [waitData.waitEnd_ind(1:6)*simParam.del_t]';
               
        TaskStartTime = CrossEndTime;
        TaskEndTime = [WaitStartTime(2:6);taskData.sim_end];
        
        
        % raw data
        PedestrianPosition = userData.pos;
        
        %filter
         windowSize = 3; 
         b = (1/windowSize)*ones(1,windowSize);
         a = 1;
         PedestrianPositionFiltered(:,4:5) = filter(b,a,PedestrianPosition(:,1:2)); %don't apply filter to time

        %% add subject and scenario details
        matLength = size(PedestrianPosition,1);    
        
        PedestrianPosition = [repmat([kk,jj],matLength,1),PedestrianPosition(:,3),PedestrianPosition(:,1:2)];
        
        PedestrianPositionFiltered(:,1:3) = [repmat([kk,jj],matLength,1),PedestrianPosition(:,3)];
        
        EventsData = [EventsData;[repmat([kk,jj],6,1),WaitStartTime,WaitEndTime,CrossStartTime,...
            CrossEndTime,TaskStartTime,TaskEndTime]];
        
        %% write to excel
        excelHeader = {'SubjectID','ScenarioID','Time','x-position','z-position'};

        
        xlswrite('PedestrianDataRaw.xlsx',PedestrianPosition,6*(kk-1)+jj,'A2');
        xlswrite('PedestrianDataRaw.xlsx',excelHeader,6*(kk-1)+jj,'A1'); 
        
        
        xlswrite('PedestrianDataFiltered.xlsx',[PedestrianPosition(:,3),PedestrianPositionFiltered],6*(kk-1)+jj,'A2');
        xlswrite('PedestrianDataFiltered.xlsx',excelHeader,6*(kk-1)+jj,'A1'); 
        
        
    end
end


excelHeader2 = {'SubjectID','ScenarioID','Time','WaitStartTime','WaitEndTime','CrossStartTime',...
    'CrossEndTime','TaskStartTime','TaskEndTime'};

xlswrite('EventsData.xlsx',EventsData,1,'A2');
xlswrite('EventsData.xlsx',excelHeader2,1,'A1');