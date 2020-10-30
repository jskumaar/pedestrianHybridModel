figure()
cw_x = cw.center_x*orthopxToMeter*scaleFactor;
cw_y = cw.center_y*orthopxToMeter*scaleFactor;
plot(cw_x, cw_y, 'ro','MarkerSize', 8);hold on;

plot(GTTrajectory(:,1), GTTrajectory(:,2), 'g*', 'MarkerSize',8); hold on;
shift = 1;
for ii = 1:N_futures_MHP
            predId = mostProbablePredictionId_MHP;
    predId = ii;
    if predId == mostProbablePredictionId_MHP
        plot(mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,2), 'b*', 'MarkerSize',8); hold on;
    else
        plot(mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_MHP(predId,1:N_PredTimeSteps,2), 'm*', 'MarkerSize',8); hold on;
    end
        text(mostProbablePredictedTrajectory_MHP(predId,N_PredTimeSteps,1), mostProbablePredictedTrajectory_MHP(predId,N_PredTimeSteps,2)-shift,strcat(num2str(predId)));
end
for predId = 1:N_futures_HBase
    plot(mostProbablePredictedTrajectory_HBase(predId,1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_HBase(predId,1:N_PredTimeSteps,2), 'k*', 'MarkerSize',8); hold on;
    text(mostProbablePredictedTrajectory_HBase(predId,N_PredTimeSteps,1), mostProbablePredictedTrajectory_HBase(predId,N_PredTimeSteps,2)-shift,strcat(num2str(predId)));
end

plot(mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,1), mostProbablePredictedTrajectory_CV(1:N_PredTimeSteps,2), 'r*', 'MarkerSize',8); hold on;
x=1;