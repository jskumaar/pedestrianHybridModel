%% This script configures the MHP model parameters

% choose the dataset
dataset="inD";

% choose the model
% predictionModel = "MultipleHybridPedestrian";
% predictionModel = "BaselineHybrid";
predictionModel = "ConstantVelocity";

% choose the sub-models
gapModelFile = 'GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice_v2.mat';
gapModel = 'GapAcceptance_inD_8Features_FGaussianSVM_BootStrappedTwice_v2';
crossIntentModelFile = 'CrossIntent_inD_9Features_BS1_noDuration_FGaussianSVM_3s_v2.mat';
crossIntentModel = 'CrossIntent_inD_9Features_BS2_noDuration_FGaussianSVM_3s_v2';
crossIntentNoCarModelFile = 'CrossIntent_NoCar_inD_3Features_BS1_noDuration_FGaussianSVM_v3.mat';
crossIntentNoCarModel = 'CrossIntent_NoCar_inD_3Features_BS1_noDuration_FGaussianSVM_v3';


%%  configure dataset parameters
if strcmp(dataset,"inD")
    script_inD_params;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% configure MHP model variables

%a) AV parameters
Params.sensingRange = 500; % in pixels (~50 m)

% b) prediction parameters
Params.predHorizon = 6*Params.AdjustedSampFreq;      % 6 s
Params.rollOverWindow = 1*Params.AdjustedSampFreq;  % in time steps; 1s
Params.observationWindow = 3*Params.AdjustedSampFreq; % in time stepsl 3s

% c) pedestrian parameters
Params.cwHeadingThreshold = 95; % degrees
Params.swHeadingThreshold = 30; % degrees
Params.stoppingThreshold = 0.25; %speed m/s
Params.walkingThreshold = 0.1; %speed m/s 
Params.movingThreshold = 0.2; % in m/s
Params.cwDistThreshold = 240; %in pixels
Params.cwCrossThreshold = 25; % in pixels (lower than approximate width of a lane)
Params.decZone = 5; %5 m radius
Params.turnThreshold = 0.7; 
Params.goalDistThreshold = 10; %(in pixels)
Params.headingRateLimit = 50; %(degrees/s)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Kalman filter model - single differentiator point mass model
% Process and Measurement covariances
Q = 2*[0.01,  0,    0,    0;
         0,  0.01,  0,    0;
         0,   0,  0.003,  0;
         0,   0,    0,  0.003];
  
R = 0.005*eye(2);   % must be zero if using ground truth (accelerometer noise)

% System dynamics (prediction model: constnat velocity)
sys = [];
dt = Params.delta_T;        
F = [1 0 dt 0;
     0 1 0 dt;
     0 0 1 0;
     0 0 0 1];        
C = [1 0 0 0;
     0 1 0 0];
D = [1 0;
     0 1];

% Define Kalman filter
kf.A = F;
kf.Q = Q;
kf.R = R;
kf.H = C;