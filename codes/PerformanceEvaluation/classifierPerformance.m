%% classifier performance

function [Performance,Actual,Predicted] = classifierPerformance(ActualOutput,PredictedOutput,Threshold)


Predicted.Output = PredictedOutput;
Actual.Output = ActualOutput;

Predicted.WaitInd = find(Predicted.Output<Threshold);
Predicted.CrossInd = find(Predicted.Output>=Threshold);

Actual.WaitInd = find(Actual.Output<Threshold);
Actual.CrossInd = find(Actual.Output>=Threshold);

ActualWaitPredWait = intersect(Actual.WaitInd,Predicted.WaitInd);
ActualWaitPredCross = intersect(Actual.WaitInd,Predicted.CrossInd);
ActualCrossPredWait = intersect(Actual.CrossInd,Predicted.WaitInd);
ActualCrossPredCross = intersect(Actual.CrossInd,Predicted.CrossInd);

ConfusionMatrix = [length(ActualWaitPredWait),length(ActualWaitPredCross);
                   length(ActualCrossPredWait),length(ActualCrossPredCross)];
               
%% Performance metrics

Performance.ConfusionMatrix = ConfusionMatrix;

Performance.Accuracy = sum(diag(Performance.ConfusionMatrix))/sum(sum(Performance.ConfusionMatrix));

Performance.Precision = ConfusionMatrix(2,2)/sum(ConfusionMatrix(1:2,2));

Performance.Recall = ConfusionMatrix(2,2)/sum(ConfusionMatrix(2,1:2));
        
Performance.F1Score = (2*Performance.Precision.*Performance.Recall)./(Performance.Precision + Performance.Recall);
        

