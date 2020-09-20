%% This function calculates the predicted probability of the predictions given a ground truth trajectory

function probGT = calculatePredictedProbability(GTTrajectory, pedPredictionsData, ped_KF_PredictionsData, Params)

% discretization of space
dx = 0.2; % 0.2 metres
dPixels = int32(dx/(Params.scaleFactor*Params.orthopxToMeter));
xLim = [Params.xMin, Params.xMax];
yLim = [Params.yMin, Params.yMax];

xRange = [1:Params.imgSize(2)];
yRange = [1:Params.imgSize(1)];

[X,Y] = meshgrid(xRange,yRange);
X_reshaped = reshape(X,[size(X,1)*size(X,2),1]);
Y_reshaped = reshape(Y,[size(Y,1)*size(Y,2),1]);

%GTTrajectory = reshape(GTTrajectory, [Params.predHorizon, 2]);

N_predTimeSteps = length(GTTrajectory);

% probability of the space
% initialize probability
for predHorTime = 1:Params.predHorizon
    probSpace{predHorTime,1} = zeros(Params.imgSize);
end


N_futures = 1:size(ped_KF_PredictionsData,1);

if length(ped_KF_PredictionsData)/8 > 1
    N_steps = length(ped_KF_PredictionsData) + 1;
else
    N_steps = length(ped_KF_PredictionsData{1})/8;
end


% probability
probGT = zeros(1 ,N_steps);


for futureNo = 1:N_futures
    if length(ped_KF_PredictionsData)/8 > 1
        KFstates = ped_KF_PredictionsData;
    else
        KFstates = reshape(ped_KF_PredictionsData{futureNo}, [8, N_steps])';
    end
    KFstates = KFstates(end-N_steps+2:end, :); % remove the first entry which is the current state
    
    if length(pedPredictionsData) < 10
        probTrajectory = pedPredictionsData{futureNo}(1);
    else
        probTrajectory = pedPredictionsData(futureNo,2);
    end
    if futureNo~=1
     x=1;
    end
    for predHorTime = 1:N_predTimeSteps
        mu = KFstates(predHorTime,[1,2])/(Params.scaleFactor*Params.orthopxToMeter);
        mu = [abs(mu(2)), mu(1)];
        sigma = diag(KFstates(predHorTime,[5:6]))/(Params.scaleFactor*Params.orthopxToMeter);
        %sigma
        pdf = mvnpdf([X_reshaped,Y_reshaped], mu, sigma);
        pdf_space = reshape(pdf, [Params.imgSize(1), Params.imgSize(2)]);
        % update distribution
        probSpace{predHorTime}  = probSpace{predHorTime}  + probTrajectory*pdf_space;

        % Ground truth probability
        GTindex = int32([abs(GTTrajectory(predHorTime,2)), GTTrajectory(predHorTime,1)] / (Params.scaleFactor*Params.orthopxToMeter));
        probGT(predHorTime) = probSpace{predHorTime}(GTindex(2), GTindex(1));

%         % plot distribution
%         figure()
%         surf(X,Y,probSpace{ii,predHorTime}); hold on;
%         x = 1;

    end
end






















end