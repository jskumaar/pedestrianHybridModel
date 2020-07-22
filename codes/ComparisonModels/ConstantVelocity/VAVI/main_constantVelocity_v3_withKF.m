%% This is the main file to run the  hybrid automaton
clear all

%% vresion 3: changed the transition performance codes; maynot be a significant change;

%% inputs
tStep = 0.1;
p_thres = 0.50;  % probability threshold for discrete state change
% pred_Horizon = 10;

pred_Horizon_matrix = [10,15,20,25,30,35,40,45,50,55,60];
pred_Horizon_matrix = pred_Horizon_matrix+1;    %the first index in the prediction horizon is the previous time step's 


waitThreshold =52.2;     %extreme threshold, (3rd quartile + 3*interquartle range; all waiting crossing (N = 403))
% waitThreshold = 33.1;     %mild threshold, (3rd quartile + 1.5*interquartle range; all waiting crossing (N = 403))
% waitThreshold = 30;     %comparison threshold,
out=1;

% % Process and Measurement covariances
% Q = [0.01,  0,   0,    0;
%       0,  0.01,  0,    0;
%       0,   0,  0.001,  0;
%       0,   0,    0,  0.001];
  
load('CVProcess_Noise_matrix.mat');
Q = ProcessNoise.Overall;
  
R = 0.002*eye(2); 

% Guard regions: G(2) - absolute road border in 'y', G(3) - velocity
% threshold for walking
G = [0;3.5;0.10;0.10];

%Target locations
TargetState = [0,3.5;       % for approach 
               0.03,4.41;   % for wait
               0,-3.6;      % for cross
               10, 5;];     % for walkaway
          
%% System dynamics (prediction model)
sys = [];
dt = tStep;        
F = [1 0 dt 0;
     0 1 0 dt;
     0 0 1 0;
     0 0 0 1];        
C = [1 0 0 0;
     0 1 0 0];
D = [1 0;
     0 1];

% Dynamics in SS
Plant = ss(F,ones(4,1),C,0,-1,'inputname',{'w'},'outputname',{'y'});

%% define system 
sys.f = @(x,w) F * x + w;    %constant velocity model with input noise
sys.h = @(x,v) C * x + v;    %position measurement       
sys.KFplant = Plant;
sys.Q = Q;
sys.R = R; 
sys.p_thres = p_thres;
sys.G = G;
sys.dt = dt;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% %% Define Kalman filter

kf.A = F;
kf.Q = Q;
kf.R = R;
kf.H = C;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Step 1: Read the compiled and filtered data 
vehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
GapData = xlsread('GapWiseCompiledDataV6.xlsx',1);
EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');
load('StartingVelocityDistribution.mat')
load('StartTimeDistributions.mat')


%%Features data (includes pedestrian and vehicle features)
FeaturesData=load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat');

load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat');

% Read test train indices
load('HybridModelTestTrainIndices.mat')
TrackingTestIndices = TestIndices_woExtremeOutlier;

%% Load trained SVM model and test data
% load('ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel_CrossingsDataSplit.mat')
% load('SVMTestData_ExtremeOutlier_StartGapExpectedGap_CorrectDTCurbDTCW.mat')
% % load('MildOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel.mat')

%% Interesting indices

ApproachStart = EventIndices(:,4);
WaitStart = EventIndices(:,6);
CrossStart = EventIndices(:,8);
AdjustedWaitStart = WaitStart;                  % combine wait start for both approach to cross and wait to cross crossings
AdjustedWaitStart(WaitStart==0) = CrossStart(WaitStart==0)-20+1;    %2s before cross starts
AdjustedWaitStart = AdjustedWaitStart-ApproachStart;
CrossEnd = EventIndices(:,9);
AdjustedCrossEnd = CrossEnd-ApproachStart+1;

DataBinSizes = [0.5,0.1,0.2,0.2,0.2,1];
startlimit = [0,0,0,0,0,0];
endlimit = [10,1,3,2.5,5,50];
N_obs = [1,2,3,4,5,6];


TestLength = length(TrackingTestIndices);


%% Outerloop for each prediction horizon

    
pred_Horizon = 101;


%% Outerloop for each crossing starts
for ii=1:TestLength
    ind = TrackingTestIndices(ii);  %crossing index
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Step 2: Initialize system
    PedestrianPosition = FeaturesData.DataPredict{ind}.PedestrianPosition; 
    PedestrianCartesianVelocity = FeaturesData.DataPredict{ind}.PedestrianCartesianVelocity;
    PedestrianAbsoluteVelocity = FeaturesData.DataPredict{ind}.PedestrianAbsoluteVelocity;
    PedestrianAbsoluteVelocityAverage = FeaturesData.DataPredict{ind}.PedestrianAbsoluteVelocityAverage(:,1);   % average of previous 5 timesteps
    PedestrianHeading = FeaturesData.DataPredict{ind}.PedestrianHeading;
    PedestrianGaze = FeaturesData.DataPredict{ind}.GazeAtVehicleRatio(:,1);

    VehiclePosition = FeaturesData.DataPredict{ind}.VehiclePosition;
    VehicleSpeed = FeaturesData.DataPredict{ind}.VehicleSpeed;
    VehicleAcceleration = FeaturesData.DataPredict{ind}.VehicleAcceleration;
    PedestrianVehicleDistance = FeaturesData.DataPredict{ind}.PedestrianVehicleDistance;
    VehicleTimeGaptoPedestrian = FeaturesData.DataPredict{ind}.VehicleTimeGaptoPedestrian;
        
    NextVehiclePosition = FeaturesData.DataPredict{ind}.NextVehiclePosition;
    NextVehicleSpeed = FeaturesData.DataPredict{ind}.NextVehicleSpeed;
    NextVehicleAcceleration = FeaturesData.DataPredict{ind}.NextVehicleAcceleration;
    NextPedestrianVehicleDistance = FeaturesData.DataPredict{ind}.PedestrianNextVehicleDistance;
    NextVehicleTimeGaptoPedestrian = FeaturesData.DataPredict{ind}.NextVehicleTimeGaptoPedestrian;
       
    % intialize continuous and discrete states
    x0 = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    init.x = x0;
    init.q = 1;         % always starts in the approach state

    % initialize the kalman filter
    kf.x = x0;
    kf.P = eye(4)*R(1,1);
    kf.detP = det(kf.P); % Let's keep track of the noise by detP
    
    
    % duration of crossing (time steps)
    M = length(PedestrianPosition);   
    Target = sign(x0(2))*TargetState;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
    %% Step 3: Create an object of the hybrid automaton
    HybridModel = HybridAutomaton_Model_2(sys,init);
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% Discrete state (1) from file, (2) estimated from ground truth continuous state
    % (1)   
    PedestrianDiscreteState = FeaturesData.DataPredict{ind}.PedestrianDiscreteState;
    
%     % (2)
%     Target = sign(x0(2))*TargetState; 
%     State = [PedestrianPosition,PedestrianCartesianVelocity];
%     PedestrianDiscreteState(1) = 1;
%     for bb=2:M
%         PedestrianDiscreteState(bb,1) = pedestrian_action_update_meas_check(State(bb,:),PedestrianDiscreteState(bb-1),G,Target); 
%     end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% inner loop, each time step

    % initialize the trajectory for the crossing
%     Performance{ii}.Estimated_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
%     Performance{ii}.Estimated_q(1,:) = PedestrianDiscreteState(1);
    Performance{ii}.GroundTruth_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    Performance{ii}.GroundTruth_q(1,:) = PedestrianDiscreteState(1); 
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% predicting for every time step
    
    for nn=1:M

        %initialize predicted states - make sure the values in
        %Hybridmodel.x and HybridModel.q are the estimated/ground truth
        %values in the previous time step
        x_pred = zeros(pred_Horizon,4);
        q_pred = zeros(pred_Horizon,1);
        x_pred(1,:) = HybridModel.x;        
        q_pred(1) = HybridModel.q;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
%% Prediction horizon loop begins

        % state predictions for the horizon    
        for pp=2:pred_Horizon
            
            %% Pedestrian state update - constant velocity 
            
           
            %% 3) Kalman filter predict step
            
            % update the KF state with the actual hybrid state (does not
            % make a difference within a prediction horion, but needed for
            % the second time step of next horizon)
            
            kf.x = HybridModel.x;
            kf = kalmanPredict(kf);

            x_pred(pp,:) = kf.x;
            x_pred_cov(pp,:) = diag(kf.P);      %only state variance
            p_pred{pp} = kf.P;                  %full covariance matrix

            % update the model with the predicted state
            HybridModel.x = kf.x;
            
            
            
            %discrete state update based on continuous state propagation
            T = Target;
            HybridModel.pedestrian_action_update_meas(T);
            
            %update the discrete state 
            q_pred(pp) = HybridModel.q;
            
        end % Prediction horizon loop ends
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
           
        %% RMSE prediction error       
        if nn<=M-pred_Horizon+1
            pred_length = pred_Horizon;
        else
            pred_length = M-nn+2;
        end
            
       
        GTPosition = PedestrianPosition(nn:nn+pred_length-2,:);        
        GTEuclideanDistanceError = zeros(1,pred_Horizon-1);
        GTEuclideanDistanceError(1:pred_length-1) = sqrt((GTPosition(:,1)-x_pred(2:pred_length,1)).^2 + (GTPosition(:,2)-x_pred(2:pred_length,2)).^2);
        GTMSE(nn) = sum(GTEuclideanDistanceError.^2)/(pred_length-1);
        GTMHD(nn) = ModHausdorffDist(GTPosition,x_pred(2:pred_length,1:2));
               
        %update the states - based on last N observations
        N = min(2,nn);   
        zH = PedestrianPosition(nn-N+1:nn,:);    %measurement
        
        %2) kalman Filter model update
        kf.x = x_pred(2,:); %next step prediction to go with the measurement for correction step
        kf.P = p_pred{2};   %next step prediction to go with the measurement for correction step
        
        kf.z = zH(end,:);
        kf = kalmanCorrect(kf);
        
        x_corrected = kf.x;
        x_cov_corrected = kf.P;

        % update the hybrid model states based on the corrected
        % state/Ground truth state
        HybridModel.x = x_corrected;
        HybridModel.q = PedestrianDiscreteState(nn,:);                %update the discrete state to current ground truth state
        
        %% save the predictions
        Performance{ii}.Predicted_x(nn,:) = reshape(x_pred',[1,pp*4]);
        Performance{ii}.Predicted_q(nn,:) = reshape(q_pred,[1,pp]);
%         Performance{ii}.Estimated_x(nn,:) = HybridModel.x;
%         Performance{ii}.Estimated_q(nn,:) = HybridModel.q; 
        Performance{ii}.GroundTruth_x(nn,:) = [PedestrianPosition(nn,:),PedestrianCartesianVelocity(nn,:)];
        Performance{ii}.GroundTruth_q(nn,:) = PedestrianDiscreteState(nn); 
        
        Performance{ii}.Estimated_x(nn,:) = x_corrected;
        Performance{ii}.Estimated_x_cov(nn,:) = diag(x_cov_corrected);
        
        
        %% errors        
        Performance{ii}.GTMSE(nn,1) = GTMSE(nn);                                        %with Ground Truth
        Performance{ii}.GTRMSE(nn,1) = sqrt(GTMSE(nn));                                 %with Ground Truth
        Performance{ii}.GTAverageEuclideanError(nn,1) = mean(GTEuclideanDistanceError); %with Ground Truth                 %with Ground Truth
        Performance{ii}.GTEndEuclideanError(nn,1) = GTEuclideanDistanceError(end);      %with Ground Truth
        Performance{ii}.GTMHD(nn) = GTMHD(nn);
        Performance{ii}.GTEuclideanDistanceError(nn,:) = GTEuclideanDistanceError;

    end
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Transition performance
Performance{ii}.WaitCrossCheckIndices = [AdjustedWaitStart(ind):AdjustedCrossEnd(ind)];
Performance = TransitionPerformance(Performance,PedestrianDiscreteState,ii,M,pred_Horizon);
 

%% Tracking Performance - transition
Performance{ii}.MeanGTRMSE = mean(Performance{ii}.GTRMSE);
Performance{ii}.MeanGTAverageEuclideanError = mean(Performance{ii}.GTAverageEuclideanError);
Performance{ii}.MeanGTEndEuclideanError= mean(Performance{ii}.GTEndEuclideanError);
Performance{ii}.MeanGTMHD = mean(Performance{ii}.GTMHD);

Performance{ii}.TranisitionMeanGTRMSE = mean(Performance{ii}.GTRMSE(Performance{ii}.TransitionIndices));
Performance{ii}.TranisitionMeanGTAverageEuclideanError = mean(Performance{ii}.GTAverageEuclideanError(Performance{ii}.TransitionIndices));
Performance{ii}.TranisitionGTEndEuclideanError = mean(Performance{ii}.GTEndEuclideanError(Performance{ii}.TransitionIndices));
Performance{ii}.TranisitionMeanGTMHD = mean(Performance{ii}.GTMHD(Performance{ii}.TransitionIndices));




end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% filename = strcat('Performance_CV_PF_',num2str(pred_Horizon_matrix(gg)-1),'_SVM_StartGap.mat');
filename = strcat('Performance_CV_KF_',num2str(100),'_SVM_StartGap.mat');
save(filename,'Performance');

















