clear all


% read data

% WaitToCrossModelData = xlsread('GapWiseCompiledDataV6_65535_removed.xlsx',3);
% N = length(WaitToCrossModelData);
% 
% indices.WaitGap = find(WaitToCrossModelData(:,13)==1);
% indices.CrossGap = find(WaitToCrossModelData(:,13)==2 | WaitToCrossModelData(:,13)==3);
% 
% N = length(indices.WaitGap);
% temp = randperm(N,N);
% indices.ReorderWaitGap=indices.WaitGap(temp);
% 
% M = length(indices.CrossGap);
% temp = randperm(M,M);
% indices.ReorderCrossGap=indices.CrossGap(temp);
% 
% indices.WaitTrain = indices.ReorderWaitGap(1:fix(N/10)*8);
% indices.CrossTrain = indices.ReorderCrossGap(1:fix(M/10)*8);
% 
% indices.WaitTest = indices.ReorderWaitGap(fix(N/10)*8+1:end);
% indices.CrossTest = indices.ReorderCrossGap(fix(M/10)*8+1:end);
% 
% 
% excelHeader = {'SubjectID','ScenarioID','Crossing ID','Crossing Decision','Gap Start Index',...
%                'Gap End Index','Gap Start (Wait Start)','Gap End (Cross Start)','TTC Gap','Cumulative Wait time',...
%                'Approach to Wait/Cross Gap Decision','Cross from wait Gap Decision',...
%                'GapDuration','ActualGap','ShortGapCounter','LongGapCounter','Cross Direction','Gaze Ratio Entire Duration','Pedestrian Speed','','','','','','Moving Window Gaze Ratio','','','','','',...
%                'Gaze Angle','','','','','','Pedestrian distance to curb','','','','','','Pedestrian distance to CW','','','','','',...
%                'Same Lane Vehicle distance to pedestrian','','','','','','Same Lane Vehicle Speed','','','','','','Same Lane Vehicle Acceleration','','','','','',...
%                'Adjacent Lane Vehicle distance to pedestrian','','','','','','Adjacent Lane Vehicle Speed','','','','','','Adjacent Lane Vehicle Acceleration','','','','','',...
%                'Same Lane Vehicle Distance Gap','','','','','','Same Lane Vehicle time to CW','','','','','','Same Lane Vehicle time to collision','','','','',''};
% 
% 
% xlswrite('WaitToCrossModelData_Train_65535_removed.xlsx',WaitToCrossModelData([indices.WaitTrain;indices.CrossTrain],:),1,'A2');
% xlswrite('WaitToCrossModelData_Train_65535_removed.xlsx',excelHeader,1,'A1');
% 
% xlswrite('WaitToCrossModelData_Test_65535_removed.xlsx',WaitToCrossModelData([indices.WaitTest;indices.CrossTest],:),1,'A2');
% xlswrite('WaitToCrossModelData_Test_65535_removed.xlsx',excelHeader,1,'A1');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Approach to Cross/Wait Model

% read data

ApproachToCrossModelData = xlsread('GapWiseCompiledDataV6_65535_removed.xlsx',2);
N = length(ApproachToCrossModelData);

indices.ApproachGap = find(ApproachToCrossModelData(:,12)==1);
indices.WaitGap = find(ApproachToCrossModelData(:,12)==2);
indices.CrossGap = find(ApproachToCrossModelData(:,12)==3);

P = length(indices.ApproachGap);
temp = randperm(P,P);
indices.ReorderApproachGap=indices.ApproachGap(temp);

N = length(indices.WaitGap);
temp = randperm(N,N);
indices.ReorderWaitGap=indices.WaitGap(temp);

M = length(indices.CrossGap);
temp = randperm(M,M);
indices.ReorderCrossGap=indices.CrossGap(temp);

indices.ApproachTrain = indices.ReorderApproachGap(1:fix(P/10)*8);
indices.WaitTrain = indices.ReorderWaitGap(1:fix(N/10)*8);
indices.CrossTrain = indices.ReorderCrossGap(1:fix(M/10)*8);

indices.ApproachTest = indices.ReorderApproachGap(fix(P/10)*8+1:end);
indices.WaitTest = indices.ReorderWaitGap(fix(N/10)*8+1:end);
indices.CrossTest = indices.ReorderCrossGap(fix(M/10)*8+1:end);


excelHeader = {'SubjectID','ScenarioID','Crossing ID','Crossing Decision','Gap Start Index',...
               'Gap End Index','Gap Start (Wait Start)','Gap End (Cross Start)','TTC Gap','Cumulative Wait time',...
               'Approach to Wait/Cross Gap Decision','Cross from wait Gap Decision',...
               'GapDuration','ActualGap','ShortGapCounter','LongGapCounter','Cross Direction','Gaze Ratio Entire Duration','Pedestrian Speed','','','','','','Moving Window Gaze Ratio','','','','','',...
               'Gaze Angle','','','','','','Pedestrian distance to curb','','','','','','Pedestrian distance to CW','','','','','',...
               'Same Lane Vehicle distance to pedestrian','','','','','','Same Lane Vehicle Speed','','','','','','Same Lane Vehicle Acceleration','','','','','',...
               'Adjacent Lane Vehicle distance to pedestrian','','','','','','Adjacent Lane Vehicle Speed','','','','','','Adjacent Lane Vehicle Acceleration','','','','','',...
               'Same Lane Vehicle Distance Gap','','','','','','Same Lane Vehicle time to CW','','','','','','Same Lane Vehicle time to collision','','','','',''};

xlswrite('ApproachToCrossModelData_Train_65535_removed.xlsx',ApproachToCrossModelData([indices.ApproachTrain;indices.WaitTrain;indices.CrossTrain],:),1,'A2');
xlswrite('ApproachToCrossModelData_Train_65535_removed.xlsx',excelHeader,1,'A1');

xlswrite('ApproachToCrossModelData_Test_65535_removed.xlsx',ApproachToCrossModelData([indices.ApproachTest;indices.WaitTest;indices.CrossTest],:),1,'A2');
xlswrite('ApproachToCrossModelData_Test_65535_removed.xlsx',excelHeader,1,'A1');

