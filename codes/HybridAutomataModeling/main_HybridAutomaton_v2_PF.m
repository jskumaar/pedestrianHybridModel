%% This is the main file to run the  hybrid automaton
clear all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% inputs

tStep = 0.1;
p_thres = 0.50;  % probability threshold for discrete state change

% gg=10;  % no. of horizons
pred_Horizon_matrix = [10,15,20,25,30,35,40,45,50,55,60];
pred_Horizon_matrix = pred_Horizon_matrix+1;    %the first index in the prediction horizon is the previous time step's 

waitThreshold =52.2;     %extreme threshold, (3rd quartile + 3*interquartle range; all waiting crossing (N = 403))
% waitThreshold = 33.1;     %mild threshold, (3rd quartile + 1.5*interquartle range; all waiting crossing (N = 403))
% waitThreshold = 30;     %comparison threshold,
out=1;

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


%% Define particle filter
pf = robotics.ParticleFilter;

pf.StateEstimationMethod = 'mean';
pf.ResamplingMethod = 'systematic';

% StateTransitionFcn defines how particles evolve without measurement
pf.StateTransitionFcn = @PedestrianConstantVelocityStateTransition;

% MeasurementLikelihoodFcn defines how measurement affect the our estimation
pf.MeasurementLikelihoodFcn = @PedestrianConstantVelocityMeasurementLikelihood;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Step 1: Read the compiled and filtered data 

vehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
GapData = xlsread('GapWiseCompiledDataV6.xlsx',1);
EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');
ApproachStart = EventIndices(:,4);
load('StartingVelocityDistribution.mat')
load('StartTimeDistributions.mat')

%%Features data (includes pedestrian and vehicle features)
FeaturesData=load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat');

% Load expected gap data
load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat');

% Read test train indices
load('HybridModelTestTrainIndices.mat')
TrackingTestIndices = TestIndices_woExtremeOutlier;
TestLength = length(TrackingTestIndices);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Discrete state transition models

%1) Load trained SVM model and test data
load('ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel_CrossingsDataSplit.mat')
load('SVMTestData_ExtremeOutlier_StartGapExpectedGap_CorrectDTCurbDTCW.mat')

% SVM model with probabilities
load('SVMProbabilityModel_ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel_CrossingsDataSplit.mat')

% load('MildOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel.mat')

%2) for probability based predictions
DataBinSizes = [0.5,0.1,0.2,0.2,0.2,1];
startlimit = [0,0,0,0,0,0];
endlimit = [10,1,3,2.5,5,50];
N_obs = [1,2,3,4,5,6];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%% Outerloop for each prediction horizon
for gg=5
    
pred_Horizon = pred_Horizon_matrix(gg);

%% Outerloop for each crossing starts

% for ii=1:TestLength
for ii=30
    
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
    
    % initialize the particle filter
    initialize(pf, 1000, x0, Q);
    
    % duration of crossing (time steps)
    M = length(PedestrianPosition);       
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
    
    
    %% inner loop, each time step initialization
 
    % 1)trajectory for the crossing
    
%     Performance{ii}.Estimated_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
%     Performance{ii}.Estimated_q(1,:) = PedestrianDiscreteState(1);
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
%         PedVehDistDifference = PedestrianVehicleDistance(nn)-PedestrianVehicleDistance(nn-1);
        
        NextVehicleSpeed_pred = NextVehicleSpeed(nn);
        NextVehicleTimeGaptoPedestrian_pred = NextVehicleTimeGaptoPedestrian(nn);
        NextVehiclePosition_pred = NextVehiclePosition(nn);
        NextPedestrianVehicleDistance_actual = NextPedestrianVehicleDistance(nn);
        
        %% Expected constant velocity based gap
        
        GapTest = VehicleTimeGaptoPedestrian_pred;

        %initialize predicted states
        x_pred = zeros(pred_Horizon,4);
        q_pred = zeros(pred_Horizon,1);
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
             dist = abs(distanceToTarget(2));                                   % x- distance to the crosswalk
             
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
%             if ((q_pred(pp-1)==2 & GapStartCheckDist<=0) | (HybridModel.q==1 & abs(x_pred(pp-1,1))<1.5 & GapStartCheckDist<=0) | CWCloseFlag==1 ...
%                     | (HybridModel.q==1 & HybridModel.q_decide==1 & dist<0.1) )

            if ((q_pred(pp-1)==2 & GapStartCheckDist<=0 & GapTest>0) | (HybridModel.q==1 & abs(x_pred(pp-1,1))<7.5 & GapStartCheckDist<=0 & GapTest>0 ) ...
                    | (HybridModel.q==1 & HybridModel.q_decide==1 & dist<0.1) )  

                %% discrete state propagation - SVM          
                
                 [predicted_decision,predicted_score] = predict(ScoreSVMModel,ObservationData);
                         
                 % assign discrete state              
                 if ( (q_pred(pp-1)==2 & predicted_decision==1) | (q_pred(pp-1)==1 & predicted_decision==1) )
                     HybridModel.q_decide = 3;
                 elseif ( (q_pred(pp-1)==1 & predicted_decision==0 | q_pred(pp-1)==2 & predicted_decision==0) )
                     HybridModel.q_decide = 2;
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
                 
                 States = [q_pred(pp-1),predicted_score(2),predicted_decision,ExpectedGapData.WCAllGapsDecision_CrossDecisionOnRoadGap(gapInd),...
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
%                      if t_wait_start==-1
%                         t_wait_start = nn+pp-1;   % when wait prediction starts within the prediction horizon
%                      end

                    % HybridModel.x(3:4) = rand(1)*resetVelocity;
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
%                          while ( t_start>=int32(GapTest*10) | t_start<(nn + pp - t_wait_start - 1))
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
    %                     %% random starting velocities (max = 1 m/s) when state changes to cross
    %                      HybridModel.x(3) = 0.1*rand(1)*cos(TargetHeading);     
    %                      HybridModel.x(4) = rand(1)*sin(TargetHeading);

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
            
            %continuous state propagation
%             % 1) Deterministic
%             HybridModel.pedestrian_motion_cv();
%             x_pred(pp,:) = HybridModel.x;
            
            
            % 2) Particle filter predict step
            stateCorrected = HybridModel.x;             %update the PF corrected state, if there were any reset
            [statePred, covPred] = predict(pf,sys);
%             x_pred_PF(pp,:) = statePred';
%             x_pred_PF_cov(pp,:) = diag(covPred)';
            
            x_pred(pp,:) = statePred';
            x_pred_cov(pp,:) = diag(covPred)';

            HybridModel.x = statePred;
            
            
            
            
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

%             %% Observation parameters
%             [ObservationType,DataBinned,Num_levels] = CombineObservations(ObservationData,DataBinSizes,startlimit,endlimit);            
            
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
%                 retreat_state_start = 0;
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
        
        if nn==240
            x=1;
        end
        
        
        % NOTE: time step '1' in the prediction horizon has the previous
        % time step's actual/estimated state
        
        GTPosition = PedestrianPosition(nn:nn+pred_length-2,:);        
        GTEuclideanDistanceError = sqrt((GTPosition(:,1)-x_pred(2:pred_length,1)).^2 + (GTPosition(:,2)-x_pred(2:pred_length,2)).^2);
        GTMSE(nn) = sum(GTEuclideanDistanceError.^2)/(pred_length-1);
        GTMHD(nn) = ModHausdorffDist(GTPosition,x_pred(2:pred_length,1:2));
                      
        %update the states - based on last N observations
        N = min(2,nn);   
        zH = PedestrianPosition(nn-N+1:nn,:);                         %measurement
%         %1) Deterministic update
%         HybridModel.pedestrian_motion_cv_update(zH); 
                

        %2) Particle Filter model update
        [stateCorrected, covCorrected] = correct(pf, zH(end,:)',sys);
        HybridModel.x = stateCorrected;



%         HybridModel.q = Performance{ii}.Estimated_q(nn-1,:);        %update the discrete state to previous estimated state
%         HybridModel.q = Performance{ii}.GroundTruth_q(nn-1,:);      %update the discrete state to previous ground truth state
        T = Target;
%         HybridModel.pedestrian_action_update_meas(T);
        HybridModel.q = PedestrianDiscreteState(nn,:);                %update the discrete state to current ground truth state
                

        % wait ends when starting to move
        if HybridModel.q==3
            t_wait_start=-1;
        end
        
        %% save the predictions
        Performance{ii}.Predicted_x(nn,:) = reshape(x_pred',[1,pp*4]);
        Performance{ii}.Predicted_q(nn,:) = reshape(q_pred,[1,pp]);
%         Performance{ii}.Estimated_x(nn,:) = HybridModel.x;
%         Performance{ii}.Estimated_q(nn,:) = HybridModel.q; 
        Performance{ii}.GroundTruth_x(nn,:) = [PedestrianPosition(nn,:),PedestrianCartesianVelocity(nn,:)];
        Performance{ii}.GroundTruth_q(nn,:) = PedestrianDiscreteState(nn); 
        
        Performance{ii}.Estimated_x(nn,:) = stateCorrected;
        Performance{ii}.Estimated_x_cov(nn,:) = diag(covCorrected);
        
        %% errors        
        Performance{ii}.GTMSE(nn,1) = GTMSE(nn);                                        %with Ground Truth
        Performance{ii}.GTRMSE(nn,1) = sqrt(GTMSE(nn));                                 %with Ground Truth
        Performance{ii}.GTAverageEuclideanError(nn,1) = mean(GTEuclideanDistanceError); %with Ground Truth                 %with Ground Truth
        Performance{ii}.GTEndEuclideanError(nn,1) = GTEuclideanDistanceError(end);      %with Ground Truth
        Performance{ii}.GTMHD(nn) = GTMHD(nn);
        
        % update discrete state based on current time step ground truth
        HybridModel.q = Performance{ii}.GroundTruth_q(nn);
                
    end    
        

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Transition performance
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


filename = strcat('Performance_Hybrid_PF_',num2str(pred_Horizon_matrix(gg)-1),'_SVM_StartGap.mat');
save(filename,'Performance');


% clear x_pred q_pred Performance


x=1;
end
x=1;



















