%% This is the main file to run the various hybrid automaton models of individual 
% pedestrians interacting with a single automated vehicle

% 1) Discrete State Transitions - various probabilistic SVM models
% 2) Contunous State Transitions - constant velocity with KF and Gaussian
% Noise
%% Updated: 07/21/20

%clear all

%% addpath of necessary directories
p1 = genpath('G:\My Drive\Research\1.Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\1.Projects\pedestrianHybridModel\datasets');

addpath(p1)
addpath(p2)

% load the SVM models
load('GapAcceptance_inD_6Features_noGaze_noTimeGap_QuadraticSVM.mat');
DiscreteModel = GapAcceptance_inD_6Features_noGaze_noTimeGap_QuadraticSVM.ClassificationSVM;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% pseudocode
%% Step 1: get the individual tracks of all agents for all scenes
% for inD dataset
% delta_T = 0.04;   % sampling rate is 25 Hz, 0.04 seconds for each data point
  SampFreq = int32(1/delta_T); 
% [tracksData, tracksMetaData, N_scenes, annotatedImage_enhanced, orthopxToMeter, cw] = inD_compile(); 
% [tracks, ~] = trackDescriptives(tracksData, N_scenes);
tracksMetaData{1}.ego_veh_gap_hist(1:size(tracksMetaData{1},1)) = {zeros(20,1)};  % for inD dataset the maximum number of gaps for a pedestrian track is '13'

% % for VAVI dataset
% tracks = VAVI_compile();


%% Step 2: For every time step in a scene, run the prediction and update loops

for scene_id = 1:N_scenes
    % Step 3: Find the time instances of the scenes
    scene_start_time = min(tracksMetaData{scene_id}.initialFrame) + 1 ;
    scene_end_time = max(tracksMetaData{scene_id}.finalFrame) + 1;
    
    %initialzie
    currentTSPedEgoData = table();
    
    % Step 4: Time loop of the scene
    for scene_time = scene_start_time : scene_end_time
        
        % Step 5: Find the active tracks for this time step
        activeTracks = find(tracksMetaData{scene_id}.initialFrame + 1 >= scene_time &... 
                            tracksMetaData{scene_id}.initialFrame + 1 <= scene_time);  
        activeCarTracks = intersect(tracks{1}.car_moving_tracks, activeTracks);
        activePedTracks = intersect(tracks{1}.ped_tracks, activeTracks);
    
        % For every active pedestrian run the prediction model
        for ped_loop_id = 1: length(activePedTracks)
            % initialize
            flag.EgoCar = false;  %flag for the presence of an ego-car
            
            ped_index = activePedTracks(ped_loop_id);
            ped_track_time_step = scene_time - tracksMetaData{scene_id}.initialFrame(ped_index); % the tracksMetaData frames start from '0' and are 1 value lower than 'scene_time'.
           
            % 1) Initialize the pedestrian according to the track id
            simulatedTracks{scene_id}{ped_index}(ped_track_time_step, :) =  tracksData{scene_id}{ped_index}(ped_track_time_step, :);
            currentPedData = simulatedTracks{scene_id}{ped_index};
            currentPedMetaData = tracksMetaData{scene_id};           
                           
            for car_loop_id = 1: length(activeCarTracks)
                car_index = activeCarTracks(car_loop_id);
                car_track_time_step = scene_time - tracksMetaData{scene_id}.initialFrame(car_index); % the tracksMetaData frames start from '0' and are 1 value lower than 'scene_time'.
                currentTSActiveCarData(car_loop_id,:) = tracksData{scene_id}{car_index}(car_track_time_step, :);
            end
            
            
        % Step 9: Update the states of the pedestrians


        % Step 10: Estimate the prediction error
        
        
        % Save all variables
       
        end
        
        
        % Step 11: Check if the pedestrians belong to a group
        if length(activePedTracks)>1
           for  ped_loop_id = 1: length(activePedTracks)
                ped_index = activePedTracks(ped_loop_id);
                groupCurrentPedData(ped_loop_id,:) = simulatedTracks{scene_id}{ped_index}(scene_time,:);            
           end
           
           % add conditions to check if they are in a group at this time
           % step.
            
            
        end
        
        % Step 11: Update the states of all the active cars
        
    
    end  % end of time loop for this scene
    
end  % end of all scenes



















%% inputs
% global tStep 
% 
% tStep = 0.1;
% p_thres = 0.50;  % probability threshold for discrete state change
% 
% waitThreshold =52.2;     %extreme threshold, (3rd quartile + 3*interquartle range; all waiting crossing (N = 403))
% out=1;

% Process and Measurement covariances
% Q = [0.01,  0,   0,    0;
%       0,  0.01,  0,    0;
%       0,   0,  0.001,  0;
%       0,   0,    0,  0.001];

% process noise calculated from data and constant velocity model propagated
% for the next time step
load('CVProcess_Noise_matrix.mat');
Q = ProcessNoise.Overall;
R = 0.002*eye(2);   % must be zero if using ground truth (accelerometer noise)

% Guard regions: G(2) - absolute road border in 'y', G(3), G(4) - minimum velocity
% threshold for walking
G = [0;3.5;0.10;0.10];

%Target locations for each discrete state; calculated from mean of the collected data
TargetState = [0,3.5;       % for approach 
               0.03,4.41;   % for wait
               0,-3.6;      % for cross
               10, 5;];     % for walkaway

% 0.0556, 4.3746 - wait position
% -0.0121, -4.4449 - wait position
% -0.0859, 3.6039 - cross position
% -0.1970, -3.5976 - cross position
           
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
GapData = xlsread('GapWiseCompiledDataV6_65535_removed.xlsx',1);
EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');

% distributions learned from collected data (this currently has the entire
% data set though, not just the ones used for the SVM model).
load('StartingVelocityDistribution.mat')
load('StartTimeDistributions.mat')

%%Features data (includes pedestrian and vehicle features)
FeaturesData=load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat');

% Load expected gap data
load('ExpectedGapData_W5_wVehDistSpeed_10_28_19.mat');

% Read test train indices
load('HybridModelTestTrainIndices.mat')
TrackingTestIndices = TestGapIndices_extreme_noHigh_dec;
TestLength = length(TrackingTestIndices);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Interesting indices

ApproachStart = EventIndices(:,4);
WaitStart = EventIndices(:,6);
CrossStart = EventIndices(:,8);
AdjustedWaitStart = WaitStart;                  % combine wait start for both approach to cross and wait to cross crossings
AdjustedWaitStart(WaitStart==0) = CrossStart(WaitStart==0)-20+1;    %2s before cross starts
AdjustedWaitStart = AdjustedWaitStart-ApproachStart;
CrossEnd = EventIndices(:,9);
AdjustedCrossEnd = CrossEnd-ApproachStart+1;
%% Discrete state transition models

%1) Load trained SVM model and test data
%load('ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel_CrossingsDataSplit.mat')
load('SVMTest_woHighDec_extreme_10_28_19.mat')
load('SVM_ExtremeOutlier_NoHighDeceleration_7Features_noTimeGap.mat')


% SVM model with probabilities
%load('SVMProbabilityModel_ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel_CrossingsDataSplit.mat')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%  Probability of transitions
load('SVMTrainData_ExtremeOutlier_StartGapExpectedGap_CorrectDTCurbDTCW.mat')

DataBinSizes = [0.5,0.1,0.3,0.5,0.5,2];
startlimit = [0,0,0,0,0,0];
endlimit = [6,1,3,7.5,5,50];

% startlimit = min(DistributionData);
% endlimit = max(DistributionData);

% select the combination of observations (all observations cannot be
% included; state explosion; either bin size should be increased or number
% of observation variables should be reduced
N_obs = [1,2,3,4,5,6];
DataBinSizes = DataBinSizes(:,N_obs);

% Limits = Limits(:,N_obs);
startlimit = startlimit(:,N_obs);
endlimit = endlimit(:,N_obs);

Prob = DiscreteTransitionProbability(SVMTrainData,DataBinSizes,N_obs,startlimit,endlimit);
ind = find(Prob.GapDistribution_Train==0);
z=length(ind)*eps;
ind2 = find(Prob.GapDistribution_Train~=0);
Prob.GapDistribution_Train(ind) = eps;
Prob.GapDistribution_Train(ind2) = Prob.GapDistribution_Train(ind2)-z/length(ind2);




pred_Horizon = 101;

%% Outerloop for each crossing starts

for ii=1:TestLength
% for ii=30
    
    ind = TrackingTestIndices(ii);  %crossing index
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
    %% Step 2: Observation variables for each crossing
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
    init.Prob = Prob;
       
    % initialize the kalman filter
    kf.x = x0;
    kf.P = eye(4)*R(1,1);
    kf.detP = det(kf.P); % Let's keep track of the noise by detP

    % duration of crossing (time steps)
    M = length(PedestrianPosition);       
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
        
    %% Step 3: Create an object of the hybrid automaton
    HybridModel = HybridAutomaton_Model_2(sys,init);
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %% Discrete state (1) from file, (2) estimated from ground truth continuous state
    % (1)   
    PedestrianDiscreteState = FeaturesData.DataPredict{ind}.PedestrianDiscreteState;
      
    %% inner loop, each time step initialization
 
    % 1)trajectory for the crossing
    
    Performance{ii}.Estimated_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    Performance{ii}.Estimated_q(1,:) = PedestrianDiscreteState(1);
    Performance{ii}.GroundTruth_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    Performance{ii}.GroundTruth_q(1,:) = PedestrianDiscreteState(1); 
    
    % 2) parameters for decision making
    HybridModel.q_decide = 1;
    CWCloseFlag = 0;
    Target = sign(x0(2))*TargetState;
    t_wait_start = -1;
    wait_start = find(PedestrianDiscreteState==2,1,'first');
    PedestrianCumulativeWaitTimeTest = 0;
    
    % 3) for transition verification
    Performance{ii}.TransitionCheck = [];
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %% predicting for every time step
    
    for nn=1:M
   
        %initialize for the time step
        VehicleSpeed_pred = VehicleSpeed(nn);
        VehicleTimeGaptoPedestrian_pred = VehicleTimeGaptoPedestrian(nn);
        VehiclePosition_pred = VehiclePosition(nn);
        PedestrianVehicleDistance_actual = PedestrianVehicleDistance(nn);
        
        NextVehicleSpeed_pred = NextVehicleSpeed(nn);
        NextVehicleTimeGaptoPedestrian_pred = NextVehicleTimeGaptoPedestrian(nn);
        NextVehiclePosition_pred = NextVehiclePosition(nn);
        NextPedestrianVehicleDistance_actual = NextPedestrianVehicleDistance(nn);
        
        %% Expected constant velocity based gap
        
        GapTest = VehicleTimeGaptoPedestrian_pred;

        %initialize predicted states
        x_pred = zeros(pred_Horizon,4);
        q_pred = zeros(pred_Horizon,1);
        
        %%for 1st time step it is from intial conditions, from next time steps, it is the estimated/Ground truth of previous time steps. 
        x_pred(1,:) = HybridModel.x;        
        q_pred(1) = HybridModel.q;
    
        % initialize decision variables for each time step
        GapStartCheckDist = PedestrianVehicleDistance_actual;   % to check when a gap starts; update to the observed value
        t_walk_start = 1000;                                    % initialize a high value; time to start walking
        walk_start_flag = 0;
        t_wait_start = -1;                                      % intialize -1, time at which wait starts
        retreat_state_start=0;
        retreat_state_reset=0;
        PedestrianCumulativeWaitTimeTest = 0;
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
        
        %% Prediction horizon loop begins

        % state predictions for the horizon    
        for pp=2:pred_Horizon
    
             T = Target(q_pred(pp-1),:);    % update target based on the predicted discrete state
             distanceToTarget = T - x_pred(pp-1,1:2);
             dist = abs(distanceToTarget(2));     % x- distance to the crosswalk             
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
        
        
            %% Pedestrian decision making - gap acceptance
            % conditions to check gap acceptance
            % (2) When predicted state is wait and a new gap is available
            % (3) When predicted state is approach, pedestrian less than 1.5m (x-position) away from crosswalk center and a new gap is available
            % (4) When predicted state is approach and pedestrian just
            % entered the 1.5m zone; CWClos flag is set to '1' only for
            % this particular time instant and reset to '0' all other times
            % (5) When predicted state is approach, pedestrian decision
            % state is approach (default value) and pedestrian is close to
            % the edge of the road (for approach to cross cases)
            
        if pp>2               
            if ((q_pred(pp-1)==2 & GapStartCheckDist<=0 & GapTest>0) | (HybridModel.q==1 & abs(x_pred(pp-1,1))<7.5 & GapStartCheckDist<=0 & GapTest>0 ) ...
                    | (HybridModel.q==1 & HybridModel.q_decide==1 & dist<0.1 & GapTest>0 ) )  

%                 %% discrete state propagation - SVM                          
%                  [predicted_decision,predicted_score] = predict(ScoreSVMModel,ObservationData);

                %% discrete state propagation - probability
                 ObservationData_binned = CombineObservations(ObservationData,DataBinSizes,startlimit,endlimit);
                 HybridModel.pedestrian_action_decision(ObservationData_binned,Target,walk_start_flag);
                 predicted_score = HybridModel.p;
                 if HybridModel.q_decide==3
                    predicted_decision = 1;
                 else
                     predicted_decision = 0;
                 end
           
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                 
                 %% for verification of discrete state transition
                 
                 timeStep = ApproachStart(ind) + nn + pp - 1 - 1 -1;
                 subjectID = ceil(ind/18);
                 crossingID = mod(ind,6);
                 crossingID(crossingID==0) = 6;
                 scenarioID = ceil(mod(ind,18)/6);
                 scenarioID(scenarioID==0) = 3;
                 
                 SubjectCheck = [[subjectID,scenarioID,crossingID,ind,nn,nn+pp-2]];
                 
                 % find the closest gap that starts corresponding to the
                 % predicted time step
                 AllgapInd = find(GapData(:,1)==subjectID & GapData(:,2)==scenarioID & GapData(:,3)==crossingID);
                 [~,gapCloseInd] = min(abs(GapData(AllgapInd,5)-timeStep));
                 gapInd = AllgapInd(gapCloseInd);
                 
                 States = [q_pred(pp-1),predicted_score(1),predicted_decision,ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap(gapInd),...
                           HybridModel.q_decide,ExpectedGapData.DiscreteState(gapInd)];
                 
                 Performance{ii}.TransitionCheck = [Performance{ii}.TransitionCheck;[SubjectCheck,ObservationData,States]];                             
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                                 
                %% reset conditions based on decision
                 currentAbsoluteVelocity = sqrt(HybridModel.x(3)^2 + HybridModel.x(4)^2);
                 distanceToTravel = Target(2,:)-[HybridModel.x(1), HybridModel.x(2)];
                 heading = atan2(distanceToTravel(2),distanceToTravel(1));
                 resetVelocity = currentAbsoluteVelocity*[cos(heading), sin(heading)];
                 
                 if HybridModel.q_decide==2
                     % Assumption: constant deceleration from current speed to reach
                     % the target; thus half the reset velocity
                     HybridModel.x(3:4) = 0.5*resetVelocity;                    
                 elseif (HybridModel.q_decide==3 & HybridModel.q==1)
                     HybridModel.x(3:4) = resetVelocity;
                 elseif (HybridModel.q_decide==3 & HybridModel.q==2)
                     
                     T = Target(HybridModel.q_decide,:);
                     TargetHeading = atan2((T(2)-HybridModel.x(2)),(T(1)-HybridModel.x(1)));
                    
                     %when there is no wait prediction
                     if t_wait_start==-1 
                         if  q_pred~=2 | q_pred(1)==2
                            t_wait_start = wait_start;   
                         else
                            temp = find(q_pred==2,1,'first'); 
                            t_wait_start = nn+temp-2;
                         end

                     end
                 
                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                 
                     
                     % randomly select a starting time from the calculated
                     % distribution which is lower than the expected gap
                     count = 1;
                     if t_walk_start==1000                           
                         while ( t_walk_start>=137  | t_walk_start<(nn + pp - t_wait_start - 2))      % 137 is the maximum from data; condition for minimum time to start walking after wait starts; condition needed when cross start needs to be sampled after wait was predicted/identified
                                count=count+1;
                                t_walk_start = int32(exprnd(StartTimeExponential.mu)/dt);        % expressed in time steps (dt=0.1)
                         if count==100
                             break
                         end
                         
                         end
                     end
                     
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

                 end                       
            end
        end            
            
        
      %% change continuous state to start walking when the walk start time has reached. No need to check for q)decide as the t_start condition probably takes care of that
                    
                    % pedestrian starts to move with a sampled starting
                    % velocity when the sampled starting time is reached
                    % within that prediction horizon
                     if t_walk_start == nn + pp - t_wait_start - 2
                         walk_start_flag=1;
                          %% random sampling from fitted distribution starting velocities when state changes to cross
                           r = normrnd(StartSpeed_10_Normal.mu,StartSpeed_10_Normal.sigma);
                           HybridModel.x(3) = r*cos(TargetHeading);     
                           HybridModel.x(4) = r*sin(TargetHeading);
                     end
                
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                        
            %% change state to zero velocity once close to target or when
            % the prediction horizon has ended
             if HybridModel.q_decide==2
                 distanceToTravel = Target(2,:)-[HybridModel.x(1), HybridModel.x(2)];
                 dist = sqrt(distanceToTravel(1)^2 + distanceToTravel(2)^2);
                 
                 if dist<0.2                
                     HybridModel.x(3:4) = [0,0];
                     HybridModel.q_decide = 1;
                     t_wait_start = nn+pp-2;
                 end
                
                 if pp==pred_Horizon
                    HybridModel.q_decide=1;
                 end
             end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
            
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
            HybridModel.pedestrian_action_update_meas(T,walk_start_flag);
            q_pred(pp) = HybridModel.q;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
                       
            %% Vehicle state and pedestrians state update - constant vehicle velocity, predicted pedestrian position
            
            VehiclePosition_pred = VehiclePosition_pred + dt*VehicleSpeed_pred;
            VehicleSpeed_pred = VehicleSpeed_pred;
            PedestrianVehicleDistance_pred = VehiclePosition_pred-x_pred(pp,1);
            VehicleTimeGaptoPedestrian_pred = PedestrianVehicleDistance_pred/abs(VehicleSpeed_pred);
            
            NextVehiclePosition_pred = NextVehiclePosition_pred + dt*NextVehicleSpeed_pred;
            NextVehicleSpeed_pred = NextVehicleSpeed_pred;
            NextPedestrianVehicleDistance_pred = NextVehiclePosition_pred-x_pred(pp,1);
            NextVehicleTimeGaptoPedestrian_pred = NextPedestrianVehicleDistance_pred/abs(NextVehicleSpeed_pred);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                      
            %Observations update
            if VehicleTimeGaptoPedestrian_pred>0
                GapTest = VehicleTimeGaptoPedestrian_pred;
            else
                GapTest = NextVehicleTimeGaptoPedestrian_pred;
            end
            
            N = min(pp,5);
            SpeedTest = mean(sqrt(x_pred(pp-N+1:pp,3).^2 + x_pred(pp-N+1:pp,4).^2));
            PedestrianDistancetoCurbTest = abs(x_pred(pp,2))-3.5;
            PedestrianDistancetoCWTest = abs(x_pred(pp,1));
            
            if q_pred(pp)==2 & t_wait_start~=-1
             % initialize cumulative waiting time when waiting
             % start time is updated
                 PedestrianCumulativeWaitTimeTest = (nn+pp-t_wait_start-2)/10;
            elseif q_pred(pp)==2 & ~isempty(wait_start)
                PedestrianCumulativeWaitTimeTest = (nn+pp-wait_start-2)/10;
            elseif q_pred(pp)==2
                temp = find(q_pred==2,1,'first');
                PedestrianCumulativeWaitTimeTest = (pp-temp-2)/10;               
            end
            
            % gaze remains consitent during the prediction horizon
            ObservationData = [GapTest,PedestrianGaze(nn),SpeedTest,PedestrianDistancetoCWTest,PedestrianDistancetoCurbTest,PedestrianCumulativeWaitTimeTest];
                     
            %update actual for next time step
            PedestrianVehicleDistance_actual = PedestrianVehicleDistance_pred;
            NextPedestrianVehicleDistance_actual = NextPedestrianVehicleDistance_pred;
                            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
            
            
            %% gap start check  distance based
            [~,temp] = min(abs([PedestrianVehicleDistance_actual,NextPedestrianVehicleDistance_actual]-GapStartCheckDist));
                       
            if (GapStartCheckDist<=0 & temp==1)     %when gap has already crossed; decision would have been checked in previous time step; update gap to next vehicle gap so that the gap is not checked again
                GapStartCheckDist = NextPedestrianVehicleDistance_actual;
            elseif (GapStartCheckDist>0 & temp==1)  %maintain gap check as current vehicle distance till the vehicle passes (refer above condition)
                GapStartCheckDist = PedestrianVehicleDistance_actual;
            elseif (GapStartCheckDist>0 & temp==2)
                GapStartCheckDist = NextPedestrianVehicleDistance_actual;
            end     
                 
            %% approach state closenes flag
            if  ( (abs(x_pred(pp-1,1))>1.5 & abs(x_pred(pp,1))<1.5))
                CWCloseFlag=1;
            else
                CWCloseFlag=0;
            end
            
            
            %% Reset state based on target
            if (abs(x_pred(pp,2))>4.5 & abs(x_pred(pp-1,2))<=4.5 & retreat_state_start==0)
                retreat_state_start=1;
            end
            
            
            %reset continuous state based on target heading when there is a change in the discrete state
            % 1) user crossed the 4.5 m threshold (approximate mean of
            % points)
            % 2) state has not been reset before during that prediction
            % horizon 
            % 3) x-velocity is below a threshold; i.e they have not already
            % started turning
            
            if (q_pred(pp)==4 & retreat_state_start==1 & retreat_state_reset==0 & x_pred(pp,3)<0.3)       
                T = Target(q_pred(pp),:);
                HybridModel.pedestrian_state_reset(T); 
                retreat_state_reset = 1;
            end
            
            % wait ends when starting to move
            if q_pred(pp)==3
                t_wait_start=-1;
            end

        end % Prediction horizon loop ends
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          

        %% Path prediction error
        
        if nn<=M-pred_Horizon+1
            pred_length = pred_Horizon;
        else
            pred_length = M-nn+2;
        end        
               
        % NOTE: time step '1' in the prediction horizon has the previous
        % time step's actual/estimated state
        
        GTPosition = PedestrianPosition(nn:nn+pred_length-2,:);
        GTEuclideanDistanceError = zeros(1,pred_Horizon-1);
        GTEuclideanDistanceError(1:pred_length-1) = sqrt((GTPosition(:,1)-x_pred(2:pred_length,1)).^2 + (GTPosition(:,2)-x_pred(2:pred_length,2)).^2);
        GTMSE(nn) = sum(GTEuclideanDistanceError.^2)/(pred_length-1);
        GTMHD(nn) = ModHausdorffDist(GTPosition,x_pred(2:pred_length,1:2));
                      
        %update the states - based on last N observations
        N = min(2,nn);   
        zH = PedestrianPosition(nn-N+1:nn,:);                         %measurement
                
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
                
        % wait ends when starting to move
        if HybridModel.q==3
            t_wait_start=-1;
        end
        
        %% save the predictions
        Performance{ii}.Predicted_x(nn,:) = reshape(x_pred',[1,pp*4]);
        Performance{ii}.Predicted_x_cov(nn,:) = reshape(x_pred_cov',[1,pp*4]);
        Performance{ii}.Predicted_q(nn,:) = reshape(q_pred,[1,pp]);
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
        
        % update discrete state based on current time step ground truth
        HybridModel.q = Performance{ii}.GroundTruth_q(nn);
                
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

filename = strcat('Performance_Hybrid_KF_',num2str(100),'_ProbDist_StartGap.mat');
%filename = strcat('Performance_Hybrid_KF_',num2str(pred_Horizon_matrix(gg)-1),'_SVM_StartGap.mat');
save(filename,'Performance');



x=1;
%end
x=1;



















