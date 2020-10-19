%% This function calculates the predicted probability of the predictions given a ground truth trajectory

function [probGT, GTLikelihood, FRS_ratio] = probMetrics(GTTrajectory, pedPredictionsData, ped_KF_PredictionsData, Params, type)

% parameters
max_velocity = 2.5; % in m/s
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;

% discretization of space
dx = 2; % 2 pixels: ~0.2 metres (0.2 m X 0.2 m is assumed to be the area oocupied by a standing pedestrian)
prob_threshold = 0.01;
dPixels = int32(dx/(scaleFactor*orthopxToMeter));
xLim = [Params.xMin, Params.xMax];
yLim = [Params.yMin, Params.yMax];
% FRS parameters
head_steps = [0: 360/90: 360]; 
vel_steps = [0: max_velocity/20: max_velocity];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% size of the predicted trajectory
N_predTimeSteps = size(GTTrajectory,1);
% no. of plausible futures
if strcmp(type, 'Not_CV')
    N_futures = size(ped_KF_PredictionsData,1);

    if length(ped_KF_PredictionsData)/8 > 1
        N_steps = length(ped_KF_PredictionsData) + 1;
    else
        N_steps = length(ped_KF_PredictionsData{1})/8;
    end
else
    N_futures = 1;
    N_steps  =size(ped_KF_PredictionsData,1);
end

%%%%%%%%%%%%%%%%%%%%%%
% initialize probability
probSpace = zeros([N_predTimeSteps, Params.imgSize]);
predictionLikelihood  = zeros([N_predTimeSteps, Params.imgSize]);
% probability metrics
probGT = zeros(1 ,N_predTimeSteps);
GTLikelihood = zeros(1 ,N_predTimeSteps);
FRS_ratio = zeros(1, N_predTimeSteps);
%%%%%%%%%%%%%%%%%%%%

%% setup mvnpdf
% grid of the scenario
xRange = [1:dx:Params.imgSize(2)];
yRange = [1:dx:Params.imgSize(1)];
[mesh_X,mesh_Y] = meshgrid(xRange,yRange);
X_reshaped = reshape(mesh_X,[size(mesh_X,1)*size(mesh_X,2),1]);
Y_reshaped = reshape(mesh_Y,[size(mesh_Y,1)*size(mesh_Y,2),1]);
scene_mesh = [X_reshaped, Y_reshaped];


%% calculate probability of each grid space
% add probability of the predicted grids for each of the plausible futures
for futureNo = 1:N_futures
    if strcmp(type, 'Not_CV')
        if length(ped_KF_PredictionsData)/8 > 1
            KFstates = ped_KF_PredictionsData;
        else
            KFstates = reshape(ped_KF_PredictionsData{futureNo}, [8, N_steps])';
        end
        KFstates = KFstates(end-N_steps+2:end, :); % remove the first entry which is the current state
    else
        KFstates = ped_KF_PredictionsData;
    end
    %%%%%%%

    % why this?   
    if strcmp(type, 'CV')
        probTrajectory =1;
    else
        probTrajectory = pedPredictionsData(futureNo,2);
    end
    %%%%%%%%
    % for every prediction time step
    for predHorTime = 1:N_predTimeSteps
        % identify FRS, mvnpdf params for the prediction time step
        mu = KFstates(predHorTime,[1,2])/(scaleFactor*orthopxToMeter);       
        sigma = diag(KFstates(predHorTime,[5:6]))/(scaleFactor*orthopxToMeter);

        % calculate Forward reachable set, FRS
        delta_X_pos = vel_steps'*cosd(head_steps)/(scaleFactor*orthopxToMeter)*Params.delta_T;
        delta_X_pos = delta_X_pos(:);
        delta_Y_pos = vel_steps'*sind(head_steps)/(scaleFactor*orthopxToMeter)*Params.delta_T;
        delta_Y_pos = delta_Y_pos(:);
        FRS_pos = mu + [delta_X_pos, delta_Y_pos];        
        FRS_pos = floor(abs(FRS_pos)/2)*2 + 1;
        FRS_grid = unique(FRS_pos, 'rows'); 
        % reset mu to snap to the grid locations
        mu = floor(abs(mu)/2)*2 + 1;        
        % find row indices in meshgrid corresponding to FRS
        FRS_meshIndices = find(ismember(scene_mesh,FRS_grid,'rows'));
                
        % calculate mvnpdf       
        pdf = mvnpdf(FRS_grid, mu, sigma);
        pdf = min(pdf,1);
        % set the original grid space
        for FRS_id = 1:length(FRS_meshIndices)
            y_ind = [Y_reshaped(FRS_meshIndices(FRS_id)):Y_reshaped(FRS_meshIndices(FRS_id))+1];
            x_ind = [X_reshaped(FRS_meshIndices(FRS_id)):X_reshaped(FRS_meshIndices(FRS_id))+1];
            % update distribution
            probSpace(predHorTime, y_ind, x_ind) = probSpace(predHorTime, y_ind, x_ind) + probTrajectory*pdf(FRS_id);     
        end
        probSpace_2D = reshape(probSpace(predHorTime,:,:), Params.imgSize);  % for testing      
        %%%%%%%%%%%%%%%%%%%%
        %% Metrics
        % a) Prob. Metric 1: Ground truth probability (metric not used in paper)
        GTindex = floor([abs(GTTrajectory(predHorTime,2)), GTTrajectory(predHorTime,1)] / (scaleFactor*orthopxToMeter));
        probGT(predHorTime) = probSpace(predHorTime,GTindex(1), GTindex(2));
        
        % b) check probability threshold for likelihood of prediction
        predictionLikelihood(predHorTime,:,:) = probSpace(predHorTime,:,:)>prob_threshold;
        likelihoodSpace_2D = reshape(predictionLikelihood(predHorTime,:,:), Params.imgSize);  % for testing 
        % c) Metric 2: likelihood of ground truth
        GTLikelihood(predHorTime) = probGT(predHorTime)>prob_threshold;
        
        % d) Metric 3: ratio of predictionLikelihood to FRS
        FRS_ratio(predHorTime) = sum(sum(predictionLikelihood(predHorTime,:,:)==1))/ (length(FRS_meshIndices)*4 ); % a mesh grid occupies 4 pixels
        
        %%%%%%%%%%%%%%%%%%%%%%%
        
%         % plot distribution
%         figure()
%         surf(X,Y,probSpace{ii,predHorTime}); hold on;
%         x = 1;
    end
    
end



% c) Likelihood ratio



end