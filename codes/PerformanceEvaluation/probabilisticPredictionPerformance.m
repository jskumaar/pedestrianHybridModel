%% classifier performance

function [Performance] = probabilisticPredictionPerformance(ActualOutput,PredictedOutput)


Predicted.Output = PredictedOutput;
Actual.Output = ActualOutput;


Predicted.RejectInd = find(Predicted.Output==0);
Predicted.AcceptInd = find(Predicted.Output==1);

Actual.RejectInd = find(Actual.Output==0);
Actual.AcceptInd = find(Actual.Output==1);

ActualRejectPredReject = intersect(Actual.RejectInd,Predicted.RejectInd);
ActualRejectPredAccept = intersect(Actual.RejectInd,Predicted.AcceptInd);
ActualAcceptPredReject = intersect(Actual.AcceptInd,Predicted.RejectInd);
ActualAcceptPredAccept = intersect(Actual.AcceptInd,Predicted.AcceptInd);

ConfusionMatrix = [length(ActualRejectPredReject),length(ActualRejectPredAccept);
                   length(ActualAcceptPredReject),length(ActualAcceptPredAccept)];
               
%% Performance metrics

Performance.ConfusionMatrix = ConfusionMatrix;

Performance.Accuracy = sum(diag(Performance.ConfusionMatrix))/sum(sum(Performance.ConfusionMatrix));

Performance.Precision = ConfusionMatrix(2,2)/sum(ConfusionMatrix(1:2,2));

Performance.Recall = ConfusionMatrix(2,2)/sum(ConfusionMatrix(2,1:2));
        
Performance.F1Score = (2*Performance.Precision.*Performance.Recall)./(Performance.Precision + Performance.Recall);
        
end
