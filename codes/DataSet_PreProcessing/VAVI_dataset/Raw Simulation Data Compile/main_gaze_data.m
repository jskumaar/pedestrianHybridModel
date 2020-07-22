clear all
close all

%% Updated: 07/04/2019
% Gaze data taken from master gaze data file.
% This file removes the duplicate entries from the Gaze data file, due to
% minor scripting error in times of the task duration during gaze data
% compilation


%% read Gaze Data
%      [WaitGaze,WaitGazeTxt] = xlsread('Gaze_data_02_08_18.xlsx',2);
%      [CrossGaze,CrossGazeTxt] = xlsread('Gaze_data_02_08_18.xlsx',3);
%      [TaskGaze,TaskGazeTxt] = xlsread('Gaze_data_02_08_18.xlsx',4);
% 
%      GazeData = [[WaitGaze,ones(size(WaitGaze,1),1)];[CrossGaze,2*ones(size(CrossGaze,1),1)];[TaskGaze,3*ones(size(TaskGaze,1),1)]];
%      GazeDataTxt = [WaitGazeTxt;CrossGazeTxt;TaskGazeTxt];
     
     [GazeData,GazeDataTxt]=xlsread('G:\My Drive\Research\TRI - VR Study I\Data Collected\Data Sheets\Compiled Data\GazeData.xlsx',2); %read the second sheet
     
     GazeData = GazeData(2:end,[1:7,11,14]);
     [GazeData,ind] = sortrows(GazeData,[1,2,4]);     %sort by time
               
     GazeDataTxt = GazeDataTxt(2:end,8:10);
     GazeDataTxt = GazeDataTxt(ind,:);

for kk = 1:30 %for every subject
    for jj = 1:6 %for every unsignalized scenario
        %% Read pedestrian data
         [PedZohData,~] = xlsread('G:\My Drive\Research\Pedestrian Modelling Project\Study I Data for Modeling\Raw Data\PedestrianDataRaw.xlsx',6*(kk-1)+jj);
                 
         %% Gaze data
          GazeStartInd = find((GazeData(:,1)==kk & GazeData(:,2)==jj),1,'first');
          GazeEndInd = find((GazeData(:,1)==kk & GazeData(:,2)==jj),1,'last');

          GazeScenario = GazeData(GazeStartInd:GazeEndInd,:);
          GazeScenarioTxt = GazeDataTxt(GazeStartInd:GazeEndInd,:);

          [~,indScenarioGaze,~] = unique(GazeScenario(:,4));

          GazeScenarioNoDuplicate = round(GazeScenario(indScenarioGaze,:),1);
          GazeScenarioTxtNoDuplicate = GazeScenarioTxt(indScenarioGaze,:);

          [~,indTimeCommon,ic] = intersect(round(PedZohData(:,3),1),GazeScenarioNoDuplicate(:,4));
          
          GazeNumericData = NaN*ones(size(PedZohData,1),size(GazeScenario,2)-1); %dont include the crossing ID data
          GazeTextData = cell(size(PedZohData,1),size(GazeScenarioTxt,2));
          
          GazeNumericData(:,1:3) = PedZohData(:,1:3);
          GazeNumericData(indTimeCommon,4:8) = GazeScenarioNoDuplicate(ic,5:9);
          
          GazeTextData(indTimeCommon,:) = GazeScenarioTxtNoDuplicate(ic,:);


%% write to excel
        excelHeader = {'SubjectID','ScenarioID','Time','gaze_x','gaze_y','gaze_z','car-gaze distance',...
                        'Activity Type','eye tag 0','eye tag 1','Object tag'};
        xlswrite('GazeNoDuplicate.xlsx',GazeNumericData,6*(kk-1)+jj,'A2');   
        xlswrite('GazeNoDuplicate.xlsx',GazeTextData,6*(kk-1)+jj,'I2');   
        xlswrite('GazeNoDuplicate.xlsx',excelHeader,6*(kk-1)+jj,'A1');
    end
end