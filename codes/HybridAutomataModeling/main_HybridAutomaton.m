%% This is the main file to run the  hybrid automaton
clear all


%% inputs
tStep = 0.1;
p_thres = 0.50;  % probability threshold for discrete state change
pred_Horizon = 20;

waitThreshold =52.2;     %extreme threshold, (3rd quartile + 3*interquartle range; all waiting crossing (N = 403))
% waitThreshold = 33.1;     %mild threshold, (3rd quartile + 1.5*interquartle range; all waiting crossing (N = 403))
% waitThreshold = 30;     %comparison threshold,
out=1;

% Process and Measurement covariances
Q = [0.01,  0,   0,    0;
      0,  0.01,  0,    0;
      0,   0,  0.001,  0;
      0,   0,    0,  0.001];
R = 0.001*eye(2); 

% Guard regions: G(2) - absolute road border in 'y', G(3) - velocity
% threshold for walking
G = [0;3.5;0.10;0.10];

%Target locations
TargetState = [0,3.5;
               0,3.5;
               0,-5.0;
               10, 5;];
          
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
%% Step 1: Read the compiled and filtered data 
vehicleGapTimes = xlsread('VehicleGapTimesV6.xlsx');
GapData = xlsread('GapWiseCompiledDataV6.xlsx',1);
EventIndices = xlsread('DiscreteStateEventIndicesW5.xlsx');
load('StartingVelocityDistribution.mat')
load('StartTimeDistributions.mat')


%%Features data (includes pedestrian and vehicle features)
FeaturesData=load('AllFeaturesCrossingWise_PW_5_SpeedHist_CorrectDTCurbDTCW.mat');

% load('ExpectedGapData_W5_SpeedHist.mat');
% load('ExpectedGapData_W5_SpeedHist_Correct.mat');
load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat');

% Read test train indices
load('HybridModelTestTrainIndices.mat')

%% Load trained SVM model
load('ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel.mat')
% load('MildOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel.mat')

% indices of interest
ApproachStart = EventIndices(:,4);
RetreatEnd = EventIndices(:,11);

%indices of vehicle gaps for each crossing
ind = find(diff(vehicleGapTimes(:,3))~=0);
indStart = [1;ind+1];
indEnd = [ind;length(vehicleGapTimes)];

% waiting time outliers
% gap indices for testing and training data
GapTrainIndices = [];
for mm=1:length(TrainIndices_woExtremeOutlier)
    ind = TrainIndices(mm);
    GapTrainIndices = [GapTrainIndices;[indStart(ind):indEnd(ind)]'];
end

GapTestIndices = [];
for mm=1:length(TestIndices_woExtremeOutlier)
    ind = TestIndices(mm);
    GapTestIndices = [GapTestIndices;[indStart(ind):indEnd(ind)]'];
end

% gap data for training probabilities
GapDataTrainWC = GapData(GapTrainIndices,:);


% Required for learning probability distributions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Expected Gap - 1
% ExpectedGap = ExpectedGapData.WCExpectedGapStartGap;
% WCExpectedNextVehicleGapCrossStart = ExpectedGapData.WCExpectedNextVehicleGapCrossStart;
% 
% CrossIndices = find(ExpectedGapData.VehicleGapTimes(:,4)==2 & ExpectedGapData.WCExpectedGapStartGap<20 & ExpectedGapData.WCExpectedGapCrossStart<20 & ExpectedGapData.WCExpectedGapOnRoad<20);
% NotSameGapIndices = find(ExpectedGapData.VehicleGapTimes(:,4)==1);
% NotSameGapIndicesCheck = NotSameGapIndices+1;
% [CommonIndices,~,~] = intersect(CrossIndices,NotSameGapIndicesCheck);
% 
% ExpectedGap(CommonIndices) = WCExpectedNextVehicleGapCrossStart(CommonIndices);

%% (or) copy directly from new .mat file
ExpectedGap = ExpectedGapData.ExpectedGap_bothCurrentNextVehicle;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make sure the latest variables are being copied

GazeType = ExpectedGapData.GazeRatiosGapStart(:,1);                     % Gaze ratio during last 1 second
SpeedType = ExpectedGapData.PedestrianAbsoluteVelocityAverage(:,1);
PedestrianDistancetoCW = ExpectedGapData.PedestrianDistancetoCW;
PedestrianDistancetoCurb = ExpectedGapData.PedestrianDistancetoCurb;
PedestrianCumulativeWaitTime = ExpectedGapData.VehicleGapTimes(:,10);

DistributionData = [ExpectedGap,GazeType,SpeedType,PedestrianDistancetoCurb,PedestrianDistancetoCW,PedestrianCumulativeWaitTime];
DistributionDataWCTrain = DistributionData(GapTrainIndices,:);
DistributionDataWCTest = DistributionData(GapTestIndices,:);
DataBinSizes = [0.5,0.1,0.2,0.2,0.2,1];

startlimit = [0,0,0,0,0,0];
endlimit = [10,1,3,2.5,5,50];

% startlimit = min(DistributionData);
% endlimit = max(DistributionData);

N_obs = [1:6];
DistributionDataWCTrain = DistributionDataWCTrain(:,N_obs);
DataBinSizes = DataBinSizes(:,N_obs);
startlimit = startlimit(:,N_obs);
endlimit = endlimit(:,N_obs);
%% Step 2: Calculate probability distributions
Prob_WC = WaitToCrossTrain(GapDataTrainWC,DistributionDataWCTrain,EventIndices,waitThreshold,out,DataBinSizes,startlimit,endlimit);

%Note: Wait to Cross probabilities used for evaluating approach to cross
%decisions
% init.Prob.CrossedGap = Prob_WC.AcceptedGap;
% init.Prob.CrossedGapDistribution_Train = Prob_WC.AcceptedGapDistribution_Train;

%% distributions of combined observations
init.Prob.GapDistribution_Train = Prob_WC.CombinedDistribution_Train;
init.Prob.AcceptedGap = Prob_WC.AcceptedGap;
init.Prob.AcceptedGapDistribution_Train = Prob_WC.AcceptedCombinedDistribution_Train;


N = length(TestIndices);

%% Outerloop for each crossing starts
for ii=1:1
    ind = TestIndices(ii);  %crossing index
    
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
    %% inner loop, each time step

    % initialize the trajectory for the crossing
    Performance{ii}.Estimated_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    Performance{ii}.Estimated_q(1,:) = PedestrianDiscreteState(1);
    Performance{ii}.GroundTruth_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    Performance{ii}.GroundTruth_q(1,:) = PedestrianDiscreteState(1); 
    
    % parameters for decision making
    HybridModel.q_decide = 1;
    CWCloseFlag = 0;
    walk_start_flag = 0;
    Target = sign(x0(2))*TargetState;

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% predicting for every time step
    
    for nn=2:M
        
        %initialize for the time step
%         VehicleAcceleration_pred = VehicleAcceleration(nn);
        VehicleSpeed_pred = VehicleSpeed(nn);
        VehicleTimeGaptoPedestrian_pred = VehicleTimeGaptoPedestrian(nn);
        VehiclePosition_pred = VehiclePosition(nn);
        PedestrianVehicleDistance_actual = PedestrianVehicleDistance(nn);
        PedVehDistDifference = PedestrianVehicleDistance(nn)-PedestrianVehicleDistance(nn-1);
        
        NextVehicleSpeed_pred = NextVehicleSpeed(nn);
        NextVehicleTimeGaptoPedestrian_pred = NextVehicleTimeGaptoPedestrian(nn);
        NextVehiclePosition_pred = NextVehiclePosition(nn);
        NextPedestrianVehicleDistance_actual = NextPedestrianVehicleDistance(nn);
        
        
        
        
%         NextPedVehDistDifference = NextPedestrianVehicleDistance(nn)-NextPedestrianVehicleDistance(nn-1);
                         
%         %observation features  
%         syms x
%         soln = double(real(solve(-VehicleAcceleration_pred/2*x^2 - VehicleSpeed_pred*x - PedestrianVehicleDistance_pred==0,x,'IgnoreAnalyticConstraints', true))); 
%         [~,temp] = min(abs(soln-VehicleTimeGaptoPedestrian_pred));
%         if ~isempty(temp)
%            Feature.GapTest = min(floor(soln(temp)/DataBinSizes)+1,floor(10/DataBinSizes));
%         else %(if no solution is possible)
%            Feature.GapTest = min(floor(VehicleTimeGaptoPedestrian_pred/DataBinSizes)+1,floor(10/DataBinSizes));
%         end
        
        %% Expected constant velocity based gap
        GapTest = min(floor(VehicleTimeGaptoPedestrian_pred/DataBinSizes(1))+1,floor((endlimit(1)-startlimit(1))/DataBinSizes(1)));

        %initialize predicted states
        x_pred = zeros(pred_Horizon,4);
        q_pred = zeros(pred_Horizon,1);
        x_pred(1,:) = Performance{ii}.Estimated_x(nn-1,:);
        q_pred(1) = Performance{ii}.Estimated_q(nn-1);
        
        GapStartCheck = PedestrianVehicleDistance_actual;   % to check when a gap starts; update to the observed value
        t_start = 1000;                                     % initialize a high value; time to start walking
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
%% Prediction horizon loop begins

        % state predictions for the horizon    
        for pp=2:pred_Horizon
    
             T = Target(q_pred(pp-1),:);    % update target based on the predicted discrete state
             distanceToTarget = T - x_pred(pp-1,1:2);
%              dist = sqrt(distanceToTarget(1)^2 + distanceToTarget(2)^2);      % absolute distance to the crosswalk
             dist = abs(distanceToTarget(2));                                   % x- distance to the crosswalk
             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
            %% Pedestrian decision making - gap acceptance
            % conditions to check gap acceptance
            % (1) When predicted state changes to wait from approach (may
            % not be needed as that gap would be used for checking approach to
            % cross/wait and only start of gap may be needed to be checked)
            % (2) When predicted state is wait and a new gap is available
            % (3) When predicted state is approach, pedestrian less than 1.5m (x-position) away from crosswalk center and a new gap is available
            % (4) When predicted state is approach and pedestrian just
            % entered the 1.5m zone; CWClos flag is set to '1' only for
            % this particular time instant and reset to '0' all other times
            % (5) When predicted state is approach, pedestrian decision
            % state is approach (default value) and pedestrian is close to
            % the edge of the road (for approach to cross cases)
            
        if pp>2    
            if ((q_pred(pp-2)==1 & (q_pred(pp-1)==2))|(q_pred(pp-1)==2 & GapStartCheck<=0) | (HybridModel.q==1 & abs(x_pred(pp-1,1))<1.5 & GapStartCheck<=0) | CWCloseFlag==1 ...
                    | (HybridModel.q==1 & HybridModel.q_decide==1 & dist<0.1) )             
%                  T = Target;      
%                 % discrete state propagation - probability
%                  HybridModel.pedestrian_action_decision(Feature,T) 

                % discrete state propagation - SVM             

                 
                 
                 s = predict(ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel.ClassificationSVM,DistributionDataWCTest);

                 predicted_decision = predict(ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel.ClassificationSVM,DistributionDataWCTrain(TestInd,:))
                 
                 
                 
                % reset conditions based on decision
                 currentAbsoluteVelocity = sqrt(HybridModel.x(3)^2 + HybridModel.x(4)^2);
                 distanceToTravel = Target(2,:)-[HybridModel.x(1), HybridModel.x(2)];
                 heading = atan2(distanceToTravel(2),distanceToTravel(1));
                 resetVelocity = currentAbsoluteVelocity*[cos(heading), sin(heading)];
                 
                 if HybridModel.q_decide==2
                     HybridModel.x(3:4) = 0.5*resetVelocity;
                    % HybridModel.x(3:4) = rand(1)*resetVelocity;
                 elseif (HybridModel.q_decide==3 & HybridModel.q==1)
                     HybridModel.x(3:4) = resetVelocity;
                 elseif (HybridModel.q_decide==3 & HybridModel.q==2)
                     
                     T = Target(HybridModel.q_decide,:);
                     TargetHeading = atan2((T(2)-HybridModel.x(2)),(T(1)-HybridModel.x(1)));

                     % randomly select a starting time from the calculated
                     % distribution which is lower than the expected gap
                     while t_start>=GapTest
                            t_start = int32(exprnd(StartTimeExponential.mu)/dt);        % expressed in time steps (dt=0.1)
                            t_wait_start = pp;
                     end
                     
                    % pedestrian starts to move with a sampled starting
                    % velocity when the sampled starting time is reached
                    % within that prediction horizon
                     if t_start == pp-t_wait_start
    %                     %% random starting velocities (max = 1 m/s) when state changes to cross
    %                      HybridModel.x(3) = 0.1*rand(1)*cos(TargetHeading);     
    %                      HybridModel.x(4) = rand(1)*sin(TargetHeading);

                          %% random sampling from fitted distribution starting velocities when state changes to cross
                           r = normrnd(StartSpeed_10_Normal.mu,StartSpeed_10_Normal.sigma);
                           HybridModel.x(3) = r*cos(TargetHeading);     
                           HybridModel.x(4) = r*sin(TargetHeading);
                     end
                         

                 end
                                     
            end
        end            
            
            
            % change state to zero velocity once close to target or when
            % the prediction horizon has ended
             if HybridModel.q_decide==2
                 distanceToTravel = Target(2,:)-[HybridModel.x(1), HybridModel.x(2)];
                 dist = sqrt(distanceToTravel(1)^2 + distanceToTravel(2)^2);
                 
                 if dist<0.2                
                     HybridModel.x(3:4) = [0,0];
                     HybridModel.q_decide = 1;
                 end
                
                 if pp==pred_Horizon
                    HybridModel.q_decide=1;
                 end
             end
             
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
            %% Pedestrian state update - constant velocity 
            
            %continuous state propagation
            HybridModel.pedestrian_motion_cv();
            x_pred(pp,:) = HybridModel.x;
            
            %discrete state update based on continuous state propagation
            T = Target;
            HybridModel.pedestrian_action_update_meas(T);
            
            %update the discrete state 
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
            

            %Observations update
            if VehicleTimeGaptoPedestrian_pred>0
                GapTest = min(floor(VehicleTimeGaptoPedestrian_pred/DataBinSizes)+1,floor(10/DataBinSizes));
            else
                GapTest = min(floor(NextVehicleTimeGaptoPedestrian_pred/DataBinSizes)+1,floor(10/DataBinSizes));
            end
            
            N = min(pp,5);
            SpeedTest = mean(sqrt(x_pred(pp-N+1:pp,1).^2 + x_pred(pp-N+1:pp,2).^2));
            PedestrianDistancetoCurbTest = abs(x_pred(pp,1))-3.5;
            PedestrianDistancetoCWTest = abs(x_pred(pp,2));
            PedestrianCumulativeWaitTimeTest = (t_wait_start+pp)/10;        %in seconds
            
            ObservationData = [GapTest,PedestrianGaze(nn),SpeedTest,PedestrianDistancetoCurbTest,PedestrianDistancetoCWTest,PedestrianCumulativeWaitTimeTest];

            %% Observation parameters
            [ObservationType,DataBinned,Num_levels] = CombineObservations(ObservationData,DataBinSizes,Limits);           
            Feature.GapTest = ObservationType;
            
            %update actual for next time step
            PedestrianVehicleDistance_actual = PedestrianVehicleDistance_pred;
            NextPedestrianVehicleDistance_actual = NextPedestrianVehicleDistance_pred;
            
            [~,temp] = min(abs([PedestrianVehicleDistance_actual,NextPedestrianVehicleDistance_actual]-GapStartCheck));
            
            
            if (GapStartCheck<=0 & temp==1)
                GapStartCheck = NextPedestrianVehicleDistance_actual;
            elseif (GapStartCheck>0 & temp==1)
                GapStartCheck = PedestrianVehicleDistance_actual;
            elseif (GapStartCheck>0 & temp==2)
                GapStartCheck = NextPedestrianVehicleDistance_actual;
            end            

            if  ( (abs(x_pred(pp-1,1))>1.5 & abs(x_pred(pp,1))<1.5))
                CWCloseFlag=1;
            else
                CWCloseFlag=0;
            end
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
%             %% Vehicle state update - constant acceleration
%             VehiclePosition_pred = VehiclePosition_pred + dt*VehicleSpeed_pred + 0.5*dt^2*VehicleAcceleration_pred;
%             VehicleSpeed_pred = VehicleSpeed_pred + dt*VehicleAcceleration_pred;
%             PedestrianVehicleDistance_pred = VehiclePosition_pred-x_pred(pp,1);
%             VehicleTimeGaptoPedestrian_pred = PedestrianVehicleDistance_pred/abs(VehicleSpeed_pred);                    
%             
%             %Feature update
%             soln = double(real(solve(-VehicleAcceleration_pred/2*x^2 - VehicleSpeed_pred*x - PedestrianVehicleDistance_pred==0,x,'IgnoreAnalyticConstraints', true))); 
%             [~,temp] = min(abs(soln-VehicleTimeGaptoPedestrian_pred));
%             if ~isempty(temp)
%                Feature.GapTest = min(floor(soln(temp)/DataBinSizes)+1,floor(10/DataBinSizes));
%             else
%                Feature.GapTest = min(floor(VehicleTimeGaptoPedestrian_pred/DataBinSizes)+1,floor(10/DataBinSizes));
%             end
%             
%             % change in pedestrian vehicle distance
%             PedVehDistDifference = PedestrianVehicleDistance_pred-PedestrianVehicleDistance_actual;
            %discrete state transition prediction (wait to cross, approach
            %to wait/cross
            
%            % update actual for next time step
%             PedestrianVehicleDistance_actual = PedestrianVehicleDistance_pred;
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                  
%             %% Reset state based on target
%             if ((q_pred(pp)==3 & q_pred(pp-1)==2) | (q_pred(pp)==3 & q_pred(pp-1)==1) | (q_pred(pp)==4 & q_pred(pp-1)==3))       %reset continuous state based on target heading when there is a change in the discrete state
%                 T = Target(q_pred(pp),:);
%                 HybridModel.pedestrian_state_reset(T);  
%             end
                    
             %% Reset state based on target
            if (q_pred(pp)==4 & q_pred(pp-1)==3)       %reset continuous state based on target heading when there is a change in the discrete state
                T = Target(q_pred(pp),:);
                HybridModel.pedestrian_state_reset(T);  
            end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%      
%% Prediction horizon loop ends
        end
              
       
           
        %% RMSE prediction error
        
        if nn<=M-pred_Horizon
            pred_length = pred_Horizon;
        else
            pred_length = M-nn+1;
        end
            
        ActualPosition = PedestrianPosition(nn:nn+pred_length-1,:);        
        DistanceError = sqrt((ActualPosition(:,1)-x_pred(1:pred_length,1)).^2 + (ActualPosition(:,2)-x_pred(1:pred_length,2)).^2);
        RMSE(nn) = sum(DistanceError.^2)/pred_length;
        
        
        %update the states - based on last N observations
        N = min(2,nn);   
        zH = PedestrianPosition(nn-N+1:nn,:);    %measurement
        HybridModel.pedestrian_motion_cv_update(zH); 
        
        HybridModel.q = Performance{ii}.Estimated_q(nn-1,:);        %update the discrete state to previous estimated state
        T = Target;
        HybridModel.pedestrian_action_update_meas(T);
%         
%         HybridModel.q = PedestrianDiscreteState(nn);
        %% save the predictions
        Performance{ii}.Predicted_x(nn,:) = reshape(x_pred',[1,pp*4]);
        Performance{ii}.Predicted_q(nn,:) = reshape(q_pred,[1,pp]);
        Performance{ii}.Estimated_x(nn,:) = HybridModel.x;
        Performance{ii}.Estimated_q(nn,:) = HybridModel.q; 
        Performance{ii}.GroundTruth_x(nn,:) = [PedestrianPosition(nn,:),PedestrianCartesianVelocity(nn,:)];
        Performance{ii}.GroundTruth_q(nn,:) = PedestrianDiscreteState(nn); 
        Performance{ii}.RMSE(nn,1) = RMSE(nn);      %with Ground Truth
        Performance{ii}.EndError(nn,1) = DistanceError(end);      %with Ground Truth

    end
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %% Transition prediction performance
% Actual transitions 
   WaitStart{ii} = find(PedestrianDiscreteState==2,1,'first');
   RetreatStart{ii} = find(PedestrianDiscreteState==4,1,'first');
   
   temp=[];
   for bb=2:M 
       if (PedestrianDiscreteState(bb-1)==2 & PedestrianDiscreteState(bb)==3)
           temp=[temp;bb];
       end               
   end
   CrossStart{ii} = temp(end);  
   
% Predicted transitions
WaitInstanceTemp=[];
CrossInstanceTemp=[];
RetreatInstanceTemp=[];
WaitHorizonTemp =[];
CrossHorizonTemp = [];
RetreatHorizonTemp = [];
   for bb=1:M 
        temp = find(Performance{ii}.Predicted_q(bb,:)==2 & Performance{ii}.Predicted_q(bb,1)~=2,1,'first'); 
        if ~isempty(temp)
            WaitInstanceTemp = [WaitInstanceTemp;bb];
            WaitHorizonTemp = [WaitHorizonTemp;temp];
        end
        
        temp = find(Performance{ii}.Predicted_q(bb,:)==3  & Performance{ii}.Predicted_q(bb,1)~=3,1,'first'); 
        if ~isempty(temp)
           CrossInstanceTemp = [CrossInstanceTemp;bb];
           CrossHorizonTemp = [CrossHorizonTemp;temp];
        end
        
        temp = find(Performance{ii}.Predicted_q(bb,:)==4 & Performance{ii}.Predicted_q(bb,1)~=4,1,'first'); 
        if ~isempty(temp)
            RetreatInstanceTemp = [RetreatInstanceTemp;bb];
            RetreatHorizonTemp = [RetreatHorizonTemp;temp];
        end     
   end


   WaitPred{ii}.Instances{ii} = WaitInstanceTemp;
   CrossPred{ii}.Instances{ii} = CrossInstanceTemp;
   RetreatPred{ii}.Instances{ii} = RetreatInstanceTemp;

   WaitPred{ii}.InHorizon{ii} = WaitHorizonTemp;
   CrossPred{ii}.InHorizon{ii} = CrossHorizonTemp;
   RetreatPred{ii}.InHorizon{ii} = RetreatHorizonTemp;




%    WaitPred{ii}.Instances{ii} = find(Performance{ii}.Predicted_q==2 & Performance{ii}.Predicted_q(:,1)~=2 );
%    CrossPred{ii}.Instances{ii} = find(Performance{ii}.Predicted_q==3  & Performance{ii}.Predicted_q(:,1)~=3 );
%    RetreatPred{ii}.Instances{ii} = find(Performance{ii}.Predicted_q==4  & Performance{ii}.Predicted_q(:,1)~=4);
   
% identify when in the predicition horizon the transition was predicted   
   len = length(WaitPred{ii}.Instances{ii});
   
   for jj=1:len
%         WaitPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(WaitPred{ii}.Instances{ii}(jj),:)==2,1,'first');
        WaitPred{ii}.Error{ii}(jj) = WaitStart{ii} - WaitPred{ii}.Instances{ii}(jj) - WaitPred{ii}.InHorizon{ii}(jj); 
   end
   
   len = length(CrossPred{ii}.Instances{ii});
   
   for jj=1:len
%         CrossPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(CrossPred{ii}.Instances{ii}(jj),:)==3,1,'first');
        CrossPred{ii}.Error{ii}(jj) = CrossStart{ii} - CrossPred{ii}.Instances{ii}(jj) - CrossPred{ii}.InHorizon{ii}(jj); 
   end
   
   len = length(RetreatPred{ii}.Instances{ii});
   
   for jj=1:len
%         RetreatPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(RetreatPred{ii}.Instances{ii}(jj),:)==4,1,'first');   
        RetreatPred{ii}.Error{ii}(jj) = RetreatStart{ii} - RetreatPred{ii}.Instances{ii}(jj) - RetreatPred{ii}.InHorizon{ii}(jj); 
   end
    
   
   %% earliest transition prediction and best transition prediction
   
   Performance{ii}.FirstWaitPredInstance = WaitStart{ii} - WaitPred{ii}.Instances{ii}(1);
   Performance{ii}.FirstWaitPredError = WaitPred{ii}.Error{ii}(1);
   
 
    WaitErrorPos = WaitPred{ii}.Error{ii}(WaitPred{ii}.Error{ii}>=0);
    if ~isempty(WaitErrorPos)
        [~,minErrorInd] = min(WaitErrorPos);
        minErrorInd = find(WaitPred{ii}.Error{ii}==WaitErrorPos(minErrorInd),1,'first');
    else
        [~,minErrorInd] = max(WaitPred{ii}.Error{ii});
    end
   
   Performance{ii}.BestWaitPredInstance = WaitStart{ii} - WaitPred{ii}.Instances{ii}(minErrorInd);
   Performance{ii}.BestWaitPredError = WaitPred{ii}.Error{ii}(minErrorInd);
   
   
   
   
   Performance{ii}.FirstCrossPredInstance = CrossStart{ii} - CrossPred{ii}.Instances{ii}(1);
   Performance{ii}.FirstCrossPredError = CrossPred{ii}.Error{ii}(1);
   
    CrossErrorPos = CrossPred{ii}.Error{ii}(CrossPred{ii}.Error{ii}>=0);
    if ~isempty(CrossErrorPos)
        [~,minErrorInd] = min(CrossErrorPos);
        minErrorInd = find(CrossPred{ii}.Error{ii}==CrossErrorPos(minErrorInd),1,'first');
    else
        [~,minErrorInd] = max(CrossPred{ii}.Error{ii});
    end
   
   Performance{ii}.BestCrossPredInstance = CrossStart{ii} - CrossPred{ii}.Instances{ii}(minErrorInd);
   Performance{ii}.BestCrossPredError = CrossPred{ii}.Error{ii}(minErrorInd);
   
   
   
   Performance{ii}.FirstRetreatPredInstance = RetreatStart{ii} - RetreatPred{ii}.Instances{ii}(1);
   Performance{ii}.FirstRetreatPredError = RetreatPred{ii}.Error{ii}(1);
   
    RetreatErrorPos = RetreatPred{ii}.Error{ii}(RetreatPred{ii}.Error{ii}>=0);
    if ~isempty(RetreatErrorPos)
        [~,minErrorInd] = min(RetreatErrorPos);
        minErrorInd = find(RetreatPred{ii}.Error{ii}==RetreatErrorPos(minErrorInd),1,'first');
    else
        [~,minErrorInd] = max(RetreatPred{ii}.Error{ii});
    end
   
   Performance{ii}.BestRetreatPredInstance = RetreatStart{ii} - RetreatPred{ii}.Instances{ii}(minErrorInd);
   Performance{ii}.BestRetreatPredError = RetreatPred{ii}.Error{ii}(minErrorInd);
   
   
  
   
   
   if ~isempty(WaitStart{ii})
        TransitionIndices{ii} = [WaitStart{ii}-pred_Horizon:RetreatStart{ii}+pred_Horizon];
   else
        TransitionIndices{ii} = [CrossStart{ii}-pred_Horizon:RetreatStart{ii}+pred_Horizon];
   end
   
   Performance{ii}.MeanRMSE = mean(Performance{ii}.RMSE);
   Performance{ii}.MeanFinalError = mean(Performance{ii}.EndError);
   
   Performance{ii}.TranisitionMeanRMSE = mean(Performance{ii}.RMSE(TransitionIndices{ii}));
   Performance{ii}.TranisitionMeanFinalError = mean(Performance{ii}.EndError(TransitionIndices{ii}));

end


save('Performance.mat','Performance');



%% figures
figure()
plot(Performance{1,1}.RMSE,'-r*','MarkerSize',15);hold on;
plot(Performance{1,1}.EndError,'-b*','MarkerSize',15);hold on;
plot(PedestrianAbsoluteVelocity,'-g*','MarkerSize',15);hold on;
xline(1);hold on;
% xline(EventIndices(ind,6)-EventIndices(ind,4)+1);hold on;
% xline(EventIndices(ind,8)-EventIndices(ind,4)+1);hold on;
% xline(EventIndices(ind,10)-EventIndices(ind,4)+1);
waitStart = find(PedestrianDiscreteState==2,1,'first');
if ~isempty(waitStart)
    xline(waitStart);hold on;
end
crossStart = find(PedestrianDiscreteState==3,1,'first');
if ~isempty(crossStart)
    xline(crossStart);hold on;
end
retreatStart = find(PedestrianDiscreteState==4,1,'first');
if ~isempty(retreatStart)
    xline(retreatStart);hold on;
end

ylabel('RMSE error [m], End error [m], Pedestrian velocity [m/s]')
xlabel('Time steps [0.1 s]')
title('Tracking performance')
legend('RMSE error','End error','Crossing Speed')
set(gca,'fontsize', 18)
% % To save figure
% filename = 'Tracking Performance';
% % FigH = figure('Position', get(0, 'Screensize'));
% saveas(gca,filename,'fig')
% saveas(gca,filename,'png')
% saveas(gca,filename,'eps')
% saveas(gca,filename,'emf')



















