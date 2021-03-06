%% This function calculates the predicted probability of the predictions given a ground truth trajectory

function [probGT, GTLikelihood, FRS_ratio, probSpace_2D_overall] = func_probMetrics(GT_currentPos, GTTrajectory, pedPredictionsData, ped_KF_PredictionsData, Params, type)

%% setup
if iscell(pedPredictionsData)
    pedPredictionsData = double(cell2mat(pedPredictionsData));
end
% parameters
max_velocity = 2.5; % in m/s
scaleFactor = Params.scaleFactor;
orthopxToMeter = Params.orthopxToMeter;
isCVPred = true;
if strcmp(type, 'Not_CV')
    isCVPred = false;
end  
% FRS parameters
head_steps = (0: 360/90: 360); 
vel_steps = (0: max_velocity/20: max_velocity);
probSpace_2D_overall = zeros(Params.imgSize(1), Params.imgSize(2));
% discretization of space
dx = 2; % 2 pixels: ~0.2 metres (0.2 m X 0.2 m is assumed to be the area oocupied by a standing pedestrian)
N_dx = 1; % number of divisions of each axis of one discretized grid space
prob_threshold = 0.01;
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
%%%%%%%%%%%%%%%%%%%%%
% setup mvnpdf
% grid of the scenario
xRange = (1:dx/N_dx:Params.imgSize(2));
yRange = (1:dx/N_dx:Params.imgSize(1));
[mesh_X,mesh_Y] = meshgrid(xRange,yRange);
X_reshaped_fine = reshape(mesh_X,[size(mesh_X,1)*size(mesh_X,2),1]);
Y_reshaped_fine = reshape(mesh_Y,[size(mesh_Y,1)*size(mesh_Y,2),1]);
scene_mesh_fine = [X_reshaped_fine, Y_reshaped_fine];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% initialize FRS
FRS_grid_fine = GT_currentPos/(scaleFactor*orthopxToMeter); %average for 1st time step
% calculate the update on Forward reachable set, FRS (in pixels)
delta_X_pos = vel_steps'*cosd(head_steps)/(scaleFactor*orthopxToMeter)*Params.delta_T;
delta_X_pos = delta_X_pos(:);
delta_Y_pos = vel_steps'*sind(head_steps)/(scaleFactor*orthopxToMeter)*Params.delta_T;
delta_Y_pos = delta_Y_pos(:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% for every prediction time step calculate probability of each grid space
% add probability of the predicted grids for each of the plausible futures
for predHorTime = 1:N_predTimeSteps     
    % update the forward reachable set
    prev_FRS_pos = FRS_grid_fine;
    for ii=1:size(prev_FRS_pos,1)
        temp = prev_FRS_pos(ii,:) + [delta_X_pos, delta_Y_pos];
        FRS_pos_fine = [FRS_grid_fine; temp];
    end 
    % update FRS grids
    FRS_pos_fine = floor(abs(FRS_pos_fine)/dx)*dx + 1;
    FRS_grid_fine = unique(FRS_pos_fine, 'rows'); 
    % find row indices in meshgrid corresponding to FRS
    FRS_meshIndices_fine = find(ismember(scene_mesh_fine,FRS_grid_fine,'rows')); 
    FRS_grid_fine_pdf = FRS_grid_fine + dx/2; % to find pdf at the center of each FRS grid space
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % calculate prediction probability for every plausible future
    for futureNo = 1:N_futures
        % trajectory predictions
        if ~isCVPred && size(ped_KF_PredictionsData{futureNo},1)==1
            KFstates = reshape(ped_KF_PredictionsData{futureNo}, [8, N_steps])';
            KFstates = KFstates(end-N_steps+2:end, :); % remove the first entry which is the current state
        elseif ~isCVPred
            KFstates = ped_KF_PredictionsData{futureNo};
        else  
            KFstates = ped_KF_PredictionsData;
        end       
        % trajectory probability  
        probTrajectory = isCVPred + (1-isCVPred)*pedPredictionsData(futureNo,2);
        %%%%%%%%%%%%%%%%%%%%%%%%%          
        % identify mvnpdf params for the prediction time step for this
        % future
        mu = KFstates(predHorTime,[1,2])/(scaleFactor*orthopxToMeter);       
        sigma = diag(KFstates(predHorTime,5:6))/(scaleFactor*orthopxToMeter);

        % calculate mvnpdf for fine grid and rescale so that sum of
        % probability is 1.
        % reset mu to snap to the grid locations
%         mu = floor(abs(mu))+0.5;     
        mu = abs(mu);
        pdf = mvnpdf(FRS_grid_fine_pdf, mu, sigma);
%         sum(pdf) % debug
        ProbReScaleFactor = 1/min(1,sum(pdf));
        pdf_reScaled = pdf*ProbReScaleFactor;
        %%%%%%%%%%%%%%%%%%%%%%%%%
        % set the original grid space
        for FRS_id = 1:length(FRS_meshIndices_fine)     
            y_ind = [Y_reshaped_fine(FRS_meshIndices_fine(FRS_id)):Y_reshaped_fine(FRS_meshIndices_fine(FRS_id))+(dx-1)];
            x_ind = [X_reshaped_fine(FRS_meshIndices_fine(FRS_id)):X_reshaped_fine(FRS_meshIndices_fine(FRS_id))+(dx-1)];
%              y_ind = Y_reshaped_fine(FRS_meshIndices_fine(FRS_id));
%              x_ind = X_reshaped_fine(FRS_meshIndices_fine(FRS_id));
            % update distribution
            probSpace(predHorTime, y_ind, x_ind) = probSpace(predHorTime, y_ind, x_ind) + probTrajectory*pdf_reScaled(FRS_id);     
        end
        probSpace_2D = reshape(probSpace(predHorTime,:,:), Params.imgSize);  % for testing                 
    end % end of all plausible futures
%%%%%%%%%%%%%%%%%%%%

%% Metrics for each prediction time step
% a) Prob. Metric 1: Ground truth probability (metric not used in paper)
GTindex = floor([abs(GTTrajectory(predHorTime,2)), GTTrajectory(predHorTime,1)] / (scaleFactor*orthopxToMeter));
probGT(predHorTime) = probSpace(predHorTime,GTindex(1), GTindex(2));
% find the maximum probability of the ground truth assuming the GTindex is
% part of a 0.2 m X 0.2 m foot print of the pedestrian. GTindex can be any one
% of the four squares of the 2 X 2 (in pixels) foot print of the pedestrian.
ind = repmat([GTindex(1), GTindex(2)],[8,1]) + [0,1; 1,0; 1,1; 0,-1; -1,0; -1,-1; -1,1; 1,-1] ;
for xx=1:8
    tempProb(xx) = probSpace(predHorTime, ind(xx,1), ind(xx,2));
end
probGT(predHorTime) = max([probSpace(predHorTime,GTindex(1), GTindex(2)), tempProb]);
% b) check probability threshold for likelihood of prediction
predictionLikelihood(predHorTime,:,:) = probSpace(predHorTime,:,:)>prob_threshold;
%         likelihoodSpace_2D = reshape(predictionLikelihood(predHorTime,:,:), Params.imgSize);  % for testing 
% c) Metric 2: likelihood of ground truth
GTLikelihood(predHorTime) = probGT(predHorTime)>prob_threshold;
% d) Metric 3: ratio of predictionLikelihood to FRS
FRS_ratio(predHorTime) = sum(sum(predictionLikelihood(predHorTime,:,:)==1))/ (length(FRS_meshIndices_fine)*(dx*dx) ); % a mesh grid occupies dx^2 pixels        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end % end of prediction horizon

end