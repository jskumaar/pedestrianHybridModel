clear all
close all

%% Crossing Logistic Regression Model
%3) Crossing vs Not crossing decision for gaps

GapData = xlsread('GapWiseCompiledData.xlsx');
GapData(isnan(GapData(:,1)),:)=[];
GapData(:,10) = GapData(:,10)/10;

temp = diff(GapData(:,3));
index = find(diff(GapData(:,3))~=0);
index = index + 1;
index = [0;index;index(end)];


%%
indices.Crossing = find(GapData(:,4)==1);
indices.NotCrossing = find(GapData(:,4)==0);
variable = categorical(GapData(:,4));

% predictors = [GapData(:,[1:3,8:9]),cumWaitTime];
% %predictorsNormalize = normalize(predictors);
% 
% [B,dev,stats] = mnrfit(predictors,variable);
% stats.p

temp = randperm(474,80);
temp2 = randperm(1514,300);

allIndices = sort([indices.Crossing;indices.NotCrossing]);
testIndex = sort([indices.Crossing(temp);indices.NotCrossing(temp2)]);
trainIndex = setdiff(allIndices,testIndex);


%predictors = [GapData(:,[8:9,11:15,17:18,20:22,29:46,48,53:57,59:63,65:69])];


predictors = [GapData(:,[7:8,10,11,13,19,31,37,43,49,55,61,67,73])];


%predictorsNormalize = normalize(predictors);

[B,dev,stats] = mnrfit(predictors(trainIndex,:),variable(trainIndex));
stats.p

yhat = mnrval(B,predictors(testIndex,:),stats);
yhatPredict = int32(yhat);

yhatPred(yhatPredict(:,1)==1) = 1;
yhatPred(yhatPredict(:,2)==1) = 2;
yhatPred = yhatPred';
actualVar = int32(variable(testIndex));

match = find(yhatPred==actualVar);
