%% This is the main file to run the hybrid automaton using the SVM model for predicting state transitions

%% Updated: 10/28/19
% 1) The SVM model has been updated using 7 features (no time gap) -
% DTCurb, DTCW, Ped. Speed, Ped. Gaze ratio, Veh. distance to Ped., Veh.
% Speed, Cumulative wait time
% 2) Crossings with sudden stops and the following crossings till the end of that treatment condition (default: > 10 m/s2 at < 3m from ped.) are
% considered as anomalies and removed from training and testing data. The
% trained SVM model does not consider such anamolies.




clear all
close all


%addpath for MATLAB data, excel data, SVM models, and relevant functions
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\1. Study I Data for Modeling\Compiled Data')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\2. Mat Data')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\Scripts - HybridModel\Hybrid Automaton\Discrete Transition Models\SVM Models')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\Scripts - HybridModel\Helper Functions')
addpath('G:\My Drive\Research\Pedestrian Modelling Project\Modelling Scripts and Results\Scripts - HybridModel\Hybrid Automaton\Kalman Filter Scripts')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% inputs

tStep = 0.1;
p_thres = 0.50;  % probability threshold for discrete state change

waitThreshold =52.2;     %extreme threshold, (3rd quartile + 3*interquartle range; all waiting crossing (N = 403))
out=1;
pred_Horizon = 10/tStep + 1;     %no. of time steps for 10 s

% Process and Measurement covariances
% Q = [0.01,  0,   0,    0;
%       0,  0.01,  0,    0;
%       0,   0,  0.001,  0;
%       0,   0,    0,  0.001];

% process noise calculated from data and constant velocity model propagated
% for the enxt time step
load('CVProcess_Noise_matrix.mat');
Q = ProcessNoise.Overall;
    
R = 0.002*eye(2);   % must be zero if using ground truth (accelerometer noise)

% Guard regions: G(2) - absolute road border in 'y', G(3), G(4) - velocity
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
load('HybridModelTestTrainIndices.mat','TestIndices_woExtremeOutlier_woHighDec')
TrackingTestIndices = TestIndices_woExtremeOutlier_woHighDec;
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
%1) Load trained SVM model and test data
%load('ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel_CrossingsDataSplit.mat')
%load('SVMTest_woHighDec_extreme_10_28_19.mat')
load('SVM_ExtremeOutlier_NoHighDeceleration_7Features_noTimeGap.mat')       %latest model w/o time gap, w/o high_dec_crossings and such

% SVM model with probabilities
%load('SVMProbabilityModel_ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel_CrossingsDataSplit.mat')

% %2) Load trained LogReg model
% load('ExtremeOutlier_StartGap_NewDTCurb_LogReg_TrainedModel.mat')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Outerloop for each crossing starts

for test_crossing_no=1:TestLength
% for ii=30
    
    ind = TrackingTestIndices(test_crossing_no);  %crossing index
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
    %% Step 2: Observation variables for each crossing
    PedestrianPosition = FeaturesData.DataPredict{ind}.PedestrianPosition; 
    PedestrianCartesianVelocity = FeaturesData.DataPredict{ind}.PedestrianCartesianVelocity;
    PedestrianAbsoluteVelocity = FeaturesData.DataPredict{ind}.PedestrianAbsoluteVelocity;
    PedestrianVelocity = FeaturesData.DataPredict{ind}.PedestrianAbsoluteVelocityAverage(:,1);   % average of previous 5 timesteps
    PedestrianHeading = FeaturesData.DataPredict{ind}.PedestrianHeading;
    PedestrianGaze = FeaturesData.DataPredict{ind}.GazeAtVehicleRatio(:,1);

    VehiclePosition = FeaturesData.DataPredict{ind}.VehiclePosition;
    VehicleSpeed_instantaneous = FeaturesData.DataPredict{ind}.VehicleSpeed;
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
    init.Prob = 0;  %dummy
       
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
    
    Performance{test_crossing_no}.Estimated_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    Performance{test_crossing_no}.Estimated_q(1,:) = PedestrianDiscreteState(1);
    Performance{test_crossing_no}.GroundTruth_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    Performance{test_crossing_no}.GroundTruth_q(1,:) = PedestrianDiscreteState(1); 
    
    % 2) parameters for decision making
    HybridModel.q_decide = 1;
    CWCloseFlag = 0;
    Target = sign(x0(2))*TargetState;
    t_wait_start = -1;
    wait_start = find(PedestrianDiscreteState==2,1,'first');
    PedestrianCumulativeWaitTimeTest = 0;
    
    % 3) for transition verification
    Performance{test_crossing_no}.TransitionCheck = [];
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    %% predicting for every time step
    
    for pred_time_step=1:M
   
        %initialize for the time step
        VehicleSpeed_pred = VehicleSpeed_instantaneous(pred_time_step);
        VehicleTimeGaptoPedestrian_pred = VehicleTimeGaptoPedestrian(pred_time_step);
        VehiclePosition_pred = VehiclePosition(pred_time_step);
        PedestrianVehicleDistance_actual = PedestrianVehicleDistance(pred_time_step);
        
        NextVehicleSpeed_pred = NextVehicleSpeed(pred_time_step);
        NextVehicleTimeGaptoPedestrian_pred = NextVehicleTimeGaptoPedestrian(pred_time_step);
        NextVehiclePosition_pred = NextVehiclePosition(pred_time_step);
        NextPedestrianVehicleDistance_actual = NextPedestrianVehicleDistance(pred_time_step);
        
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
                    | (HybridModel.q==1 & HybridModel.q_decide==1 & dist<0.1) )  

%                 %% discrete state propagation - SVM                          
%                  [predicted_decision,predicted_score] = predict(ScoreSVMModel,ObservationData);
                 
                 %% discrete state propagation - LogReg   
                 %WCExpectedGapStartGap = ObservationData(1);
                 GazeRatio = ObservationData(4);
                 PedestrianVelocity = ObservationData(5);
                 DTCW = ObservationData(2);
                 DTCurb = ObservationData(3);
                 CumulativeWaitTime = ObservationData(1);
                 VehicleDistancetoPed = ObservationData(6);
                 VehicleSpeed = ObservationData(7);
                 
                 ObservationDataTable = table(CumulativeWaitTime,DTCW,DTCurb,GazeRatio,...
                                        PedestrianVelocity,VehicleDistancetoPed,VehicleSpeed);
                
                % previous SVM model w/ 6 features and w/ high dec data
                %[predicted_decision] = ExtremeOutlier_StartGap_NewDTCurb_NoGaze_LogReg_TrainedModel.predictFcn(ObservationDataTable);      %variable name 'NoGaze' but trained with gaze ratio data
                
                % new SVM model w/ 7 features (time gap split into veh.
                % distance to ped. and speed) and w/o high dec data
                [predicted_decision] = SVM_ExtremeOutlier_NoHighDeceleration_7Features_noTimeGap.predictFcn(ObservationDataTable);      %variable name 'NoGaze' but trained with gaze ratio data
                     
                 
                 
                 % assign discrete state              
                 if ( (q_pred(pp-1)==2 & predicted_decision==1) | (q_pred(pp-1)==1 & predicted_decision==1) )
                     HybridModel.q_decide = 3;
                 elseif ( (q_pred(pp-1)==1 & predicted_decision==0 | q_pred(pp-1)==2 & predicted_decision==0) )
                     HybridModel.q_decide = 2;
                 end                
                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                 
                 %% for verification of discrete state transition
                 
                 timeStep = ApproachStart(ind) + pred_time_step + pp - 1 - 1 -1;
                 subjectID = ceil(ind/18);
                 crossingID = mod(ind,6);
                 crossingID(crossingID==0) = 6;
                 scenarioID = ceil(mod(ind,18)/6);
                 scenarioID(scenarioID==0) = 3;
                 
                 SubjectCheck = [[subjectID,scenarioID,crossingID,ind,pred_time_step,pred_time_step+pp-2]];
                 
                 % find the closest gap that starts corresponding to the
                 % predicted time step
                 AllgapInd = find(GapData(:,1)==subjectID & GapData(:,2)==scenarioID & GapData(:,3)==crossingID);
                 [~,gapCloseInd] = min(abs(GapData(AllgapInd,5)-timeStep));
                 gapInd = AllgapInd(gapCloseInd);
                 
%                  States = [q_pred(pp-1),predicted_score(2),predicted_decision,ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap(gapInd),...
%                            HybridModel.q_decide,ExpectedGapData.DiscreteState(gapInd)];

% %%              For Log reg
%                     States = [q_pred(pp-1),predicted_decision,ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap(gapInd),...
%                             HybridModel.q_decide,ExpectedGapData.DiscreteState(gapInd)];
%                  
%                  Performance{test_crossing_no}.TransitionCheck = [Performance{test_crossing_no}.TransitionCheck;[SubjectCheck,ObservationData,States]];                             
%                 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                                 
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
                            t_wait_start = pred_time_step+temp-2;
                         end

                     end
                 
                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
                 
                     
                     % randomly select a starting time from the calculated
                     % distribution which is lower than the expected gap
                     count = 1;
                     if t_walk_start==1000                           
                         while ( t_walk_start>=137  | t_walk_start<(pred_time_step + pp - t_wait_start - 2))      % 137 is the maximum from data; condition for minimum time to start walking after wait starts; condition needed when cross start needs to be sampled after wait was predicted/identified
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
                     if t_walk_start == pred_time_step + pp - t_wait_start - 2
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
                     t_wait_start = pred_time_step+pp-2;
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
            if VehicleTimeGaptoPedestrian_pred>0 %current vehicle
                GapTest = VehicleTimeGaptoPedestrian_pred;
                VehDistPed = PedestrianVehicleDistance_pred;
                VehSpeed = VehicleSpeed_pred;
            else                                 %next vehicle
                GapTest = NextVehicleTimeGaptoPedestrian_pred;
                VehDistPed = NextPedestrianVehicleDistance_pred;
                VehSpeed = NextVehicleSpeed_pred;
            end
            
            N = min(pp,5);
            SpeedTest = mean(sqrt(x_pred(pp-N+1:pp,3).^2 + x_pred(pp-N+1:pp,4).^2));
            PedestrianDistancetoCurbTest = abs(x_pred(pp,2))-3.5;
            PedestrianDistancetoCWTest = abs(x_pred(pp,1));
            
            if q_pred(pp)==2 & t_wait_start~=-1
             % initialize cumulative waiting time when waiting
             % start time is updated
                 PedestrianCumulativeWaitTimeTest = (pred_time_step+pp-t_wait_start-2)/10;
            elseif q_pred(pp)==2 & ~isempty(wait_start)
                PedestrianCumulativeWaitTimeTest = (pred_time_step+pp-wait_start-2)/10;
            elseif q_pred(pp)==2
                temp = find(q_pred==2,1,'first');
                PedestrianCumulativeWaitTimeTest = (pp-temp-2)/10;               
            end
            
            % gaze remains consitent during the prediction horizon
            ObservationData = [PedestrianCumulativeWaitTimeTest, PedestrianDistancetoCurbTest, PedestrianDistancetoCWTest,...
                               PedestrianGaze(pred_time_step), SpeedTest, VehDistPed, VehSpeed];
                     
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
        
        if pred_time_step<=M-pred_Horizon+1
            pred_length = pred_Horizon;
        else
            pred_length = M-pred_time_step+2;
        end        
               
        % NOTE: time step '1' in the prediction horizon has the previous
        % time step's actual/estimated state
        
        GTPosition = PedestrianPosition(pred_time_step:pred_time_step+pred_length-2,:);
        GTEuclideanDistanceError = zeros(1,pred_Horizon-1);
        GTEuclideanDistanceError(1:pred_length-1) = sqrt((GTPosition(:,1)-x_pred(2:pred_length,1)).^2 + (GTPosition(:,2)-x_pred(2:pred_length,2)).^2);
        GTMSE(pred_time_step) = sum(GTEuclideanDistanceError.^2)/(pred_length-1);
        GTMHD(pred_time_step) = ModHausdorffDist(GTPosition,x_pred(2:pred_length,1:2));
                      
        %update the states - based on last N observations
        N = min(2,pred_time_step);   
        zH = PedestrianPosition(pred_time_step-N+1:pred_time_step,:);                         %measurement
                
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
        HybridModel.q = PedestrianDiscreteState(pred_time_step,:);                %update the discrete state to current ground truth state
                
        % wait ends when starting to move
        if HybridModel.q==3
            t_wait_start=-1;
        end
        
        %% save the predictions
        Performance{test_crossing_no}.Predicted_x(pred_time_step,:) = reshape(x_pred',[1,pp*4]);
        Performance{test_crossing_no}.Predicted_x_cov(pred_time_step,:) = reshape(x_pred_cov',[1,pp*4]);
        Performance{test_crossing_no}.Predicted_q(pred_time_step,:) = reshape(q_pred,[1,pp]);
        Performance{test_crossing_no}.GroundTruth_x(pred_time_step,:) = [PedestrianPosition(pred_time_step,:),PedestrianCartesianVelocity(pred_time_step,:)];
        Performance{test_crossing_no}.GroundTruth_q(pred_time_step,:) = PedestrianDiscreteState(pred_time_step); 
        
        Performance{test_crossing_no}.Estimated_x(pred_time_step,:) = x_corrected;
        Performance{test_crossing_no}.Estimated_x_cov(pred_time_step,:) = diag(x_cov_corrected);
        
        %% errors        
        Performance{test_crossing_no}.GTMSE(pred_time_step,1) = GTMSE(pred_time_step);                                        %with Ground Truth
        Performance{test_crossing_no}.GTRMSE(pred_time_step,1) = sqrt(GTMSE(pred_time_step));                                 %with Ground Truth
        Performance{test_crossing_no}.GTAverageEuclideanError(pred_time_step,1) = mean(GTEuclideanDistanceError); %with Ground Truth                 %with Ground Truth
        Performance{test_crossing_no}.GTEndEuclideanError(pred_time_step,1) = GTEuclideanDistanceError(end);      %with Ground Truth
        Performance{test_crossing_no}.GTMHD(pred_time_step) = GTMHD(pred_time_step);
        Performance{test_crossing_no}.GTEuclideanDistanceError(pred_time_step,:) = GTEuclideanDistanceError;
        
        % update discrete state based on current time step ground truth
        HybridModel.q = Performance{test_crossing_no}.GroundTruth_q(pred_time_step);
                
    end    
        

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Transition performance
Performance{test_crossing_no}.WaitCrossCheckIndices = [AdjustedWaitStart(ind):AdjustedCrossEnd(ind)];
Performance = TransitionPerformance(Performance,PedestrianDiscreteState,test_crossing_no,M,pred_Horizon);

%% Tracking Performance - transition
Performance{test_crossing_no}.MeanGTRMSE = mean(Performance{test_crossing_no}.GTRMSE);
Performance{test_crossing_no}.MeanGTAverageEuclideanError = mean(Performance{test_crossing_no}.GTAverageEuclideanError);
Performance{test_crossing_no}.MeanGTEndEuclideanError= mean(Performance{test_crossing_no}.GTEndEuclideanError);
Performance{test_crossing_no}.MeanGTMHD = mean(Performance{test_crossing_no}.GTMHD);

Performance{test_crossing_no}.TranisitionMeanGTRMSE = mean(Performance{test_crossing_no}.GTRMSE(Performance{test_crossing_no}.TransitionIndices));
Performance{test_crossing_no}.TranisitionMeanGTAverageEuclideanError = mean(Performance{test_crossing_no}.GTAverageEuclideanError(Performance{test_crossing_no}.TransitionIndices));
Performance{test_crossing_no}.TranisitionGTEndEuclideanError = mean(Performance{test_crossing_no}.GTEndEuclideanError(Performance{test_crossing_no}.TransitionIndices));
Performance{test_crossing_no}.TranisitionMeanGTMHD = mean(Performance{test_crossing_no}.GTMHD(Performance{test_crossing_no}.TransitionIndices));


end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% filename = strcat('Performance_Hybrid_KF_',num2str(100),'_LogReg_StartGap.mat');
filename = strcat('Performance_Hybrid_KF_',num2str(100),'_SVM_StartGap.mat');
save(filename,'Performance');




















