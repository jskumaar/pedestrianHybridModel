%% This is the main file to run the  hybrid automaton
clear all


%% inputs
tStep = 0.1;
order = 2;      % order of difference operator
roundDen = 100; % for 1/100 i.e 0.01
p_thres = 0.4;  % probability threshold for discrete state change
% WS = 32;        % Window Size
pred_Horizon = 20;
binsize = 0.5;

% Process and Measurement covariances
Q = [0.01,  0,   0,    0;
      0,  0.01,  0,    0;
      0,   0,  0.001,  0;
      0,   0,    0,  0.001];
R = 0.001*eye(2); 

% Guard regions: G(2) - absolute  road border in 'y', G(3) - velocity
% threshold for walking
G = [0;3.5;0.15;0.15];

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
sys.h = @(x)  C * x + v;     %position measurement       
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

load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat');

% Read test train indices
load('HybridModelTestTrainIndices.mat')
TrackingTestIndices = TestIndices_woExtremeOutlier;

N = length(TrackingTestIndices);


%% Outerloop for each crossing starts
for ii=1:1
    ind = TrackingTestIndices(ii);  %crossing index
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Step 2: Create an object of the hybrid automaton
    %% initialize system
    PedestrianPosition = FeaturesData.DataPredict{ind}.PedestrianPosition;    
    M = length(PedestrianPosition);
    
    
    PedestrianCartesianVelocity = FeaturesData.DataPredict{ind}.PedestrianCartesianVelocity;
    PedestrianAbsoluteVelocity = FeaturesData.DataPredict{ind}.PedestrianAbsoluteVelocity;
    PedestrianHeading = FeaturesData.DataPredict{ind}.PedestrianHeading;
  
   
    
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    VehiclePosition = FeaturesData.DataPredict{ind}.VehiclePosition;
    VehicleSpeed = FeaturesData.DataPredict{ind}.VehicleSpeed;
    VehicleAcceleration = FeaturesData.DataPredict{ind}.VehicleAcceleration;
    PedestrianVehicleDistance = FeaturesData.DataPredict{ind}.PedestrianVehicleDistance;
    VehicleTimeGaptoPedestrian = FeaturesData.DataPredict{ind}.VehicleTimeGaptoPedestrian;
        
    NextVehiclePosition = FeaturesData.DataPredict{ind}.NextVehiclePosition;
    NextVehicleSpeed = FeaturesData.DataPredict{ind}.NextVehicleSpeed;
    NextVehicleAcceleration = FeaturesData.DataPredict{ind}.NextVehicleAcceleration;
    NextPedestrianVehicleDistance = FeaturesData.DataPredict{ind}.PedestrianNextVehicleDistance;
    NextVehicleTimeGaptoPedestrian = NextPedestrianVehicleDistance/abs(NextVehicleSpeed);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Hybrid automation
    x0 = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];

    init.x = x0;
    init.q = 1;

    HybridModel = HybridAutomaton_Model_2(sys,init);
    
    
       
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Discrete state (1) from file, (2) from continuous state
    % (1)
    Target = sign(x0(2))*TargetState;
    
    PedestrianDiscreteState_2 = FeaturesData.DataPredict{ind}.PedestrianDiscreteState;
    
    % (2)
    State = [PedestrianPosition,PedestrianCartesianVelocity];
    PedestrianDiscreteState(1) = 1;
    for bb=2:M
        if bb==121
            x=1;
        end
            
        PedestrianDiscreteState(bb,1) = pedestrian_action_update_meas_check(State(bb,:),PedestrianDiscreteState(bb-1),G,Target); 

    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% inner loop, each time step
    
    
    % initialize the trajectory
    CVPerformance{ii}.Estimated_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    CVPerformance{ii}.Estimated_q(1,:) = PedestrianDiscreteState(1);
    CVPerformance{ii}.GroundTruth_x(1,:) = [PedestrianPosition(1,:),PedestrianCartesianVelocity(1,:)];
    CVPerformance{ii}.GroundTruth_q(1,:) = PedestrianDiscreteState(1); 
   
    
    
    for nn=2:M
        
        if nn==127
            x=1;
        end
        %initialize for the horizon
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
        NextPedVehDistDifference = NextPedestrianVehicleDistance(nn)-NextPedestrianVehicleDistance(nn-1);
                         
%         %observation features  
%         syms x
%         soln = double(real(solve(-VehicleAcceleration_pred/2*x^2 - VehicleSpeed_pred*x - PedestrianVehicleDistance_pred==0,x,'IgnoreAnalyticConstraints', true))); 
%         [~,temp] = min(abs(soln-VehicleTimeGaptoPedestrian_pred));
%         if ~isempty(temp)
%            Feature.GapTest = min(floor(soln(temp)/binsize)+1,floor(10/binsize));
%         else
%            Feature.GapTest = min(floor(VehicleTimeGaptoPedestrian_pred/binsize)+1,floor(10/binsize));
%         end

        Feature.GapTest = min(floor(VehicleTimeGaptoPedestrian_pred/binsize)+1,floor(10/binsize));

        
        %initialize predicted states
        x_pred = zeros(pred_Horizon,4);
        q_pred = zeros(pred_Horizon,1);
        x_pred(1,:) = CVPerformance{ii}.Estimated_x(nn-1,:);
        q_pred(1) = CVPerformance{ii}.Estimated_q(nn-1);
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
%% Prediction horizon loop begins

        % state predictions for the horizon
       
        for pp=2:pred_Horizon
            
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
            %% Vehicle state update - constant velocity
            
            VehiclePosition_pred = VehiclePosition_pred + dt*VehicleSpeed_pred;
            VehicleSpeed_pred = VehicleSpeed_pred;
            PedestrianVehicleDistance_pred = VehiclePosition_pred-x_pred(pp,1);
            VehicleTimeGaptoPedestrian_pred = PedestrianVehicleDistance_pred/abs(VehicleSpeed_pred);
            
            NextVehiclePosition_pred = NextVehiclePosition_pred + dt*NextVehicleSpeed_pred;
            NextVehicleSpeed_pred = NextVehicleSpeed_pred;
            NextPedestrianVehicleDistance_pred = NextVehiclePosition_pred-x_pred(pp,1);
            NextVehicleTimeGaptoPedestrian_pred = NextPedestrianVehicleDistance_pred/abs(NextVehicleSpeed_pred);

            %Feature update
            if VehicleTimeGaptoPedestrian_pred>0
                Feature.GapTest = min(floor(VehicleTimeGaptoPedestrian_pred/binsize)+1,floor(10/binsize));
            else
                Feature.GapTest = min(floor(NextVehicleTimeGaptoPedestrian_pred/binsize)+1,floor(10/binsize));
            end
                        
            % change in pedestrian vehicle distance
            PedVehDistDifference = PedestrianVehicleDistance_pred-PedestrianVehicleDistance_actual;
            NextPedVehDistDifference = NextPedestrianVehicleDistance_pred-NextPedestrianVehicleDistance_actual;
            
            %update actual for next time step
            PedestrianVehicleDistance_actual = PedestrianVehicleDistance_pred;
            NextPedestrianVehicleDistance_actual = NextPedestrianVehicleDistance_pred;
            
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
%                Feature.GapTest = min(floor(soln(temp)/binsize)+1,floor(10/binsize));
%             else
%                Feature.GapTest = min(floor(VehicleTimeGaptoPedestrian_pred/binsize)+1,floor(10/binsize));
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
        N = min(5,nn);   
        zH = PedestrianPosition(nn-N+1:nn,:);    %measurement
        HybridModel.pedestrian_motion_cv_update(zH); 
        
        HybridModel.q = CVPerformance{ii}.Estimated_q(nn-1,:);        %update the discrete state to previous estimated state
        T = Target;
        HybridModel.pedestrian_action_update_meas(T);
%         
%         HybridModel.q = PedestrianDiscreteState(nn);
        %% save the predictions
        CVPerformance{ii}.Predicted_x(nn,:) = reshape(x_pred',[1,pp*4]);
        CVPerformance{ii}.Predicted_q(nn,:) = reshape(q_pred,[1,pp]);
        CVPerformance{ii}.Estimated_x(nn,:) = HybridModel.x;
        CVPerformance{ii}.Estimated_q(nn,:) = HybridModel.q; 
        CVPerformance{ii}.GroundTruth_x(nn,:) = [PedestrianPosition(nn,:),PedestrianCartesianVelocity(nn,:)];
        CVPerformance{ii}.GroundTruth_q(nn,:) = PedestrianDiscreteState(nn); 
        CVPerformance{ii}.RMSE(nn,1) = RMSE(nn);      %with Ground Truth
        CVPerformance{ii}.EndError(nn,1) = DistanceError(end);      %with Ground Truth

    end
     
   CVPerformance{ii}.MeanRMSE = mean(CVPerformance{ii}.RMSE);
   CVPerformance{ii}.MeanFinalError = mean(CVPerformance{ii}.EndError);
   
   
   
   WaitStart{ii} = find(PedestrianDiscreteState==2,1,'first');
   CrossStart{ii} = find(PedestrianDiscreteState==3,1,'first');
   RetreatStart{ii} = find(PedestrianDiscreteState==4,1,'first');
   
   
  
   
   
  
   
   
   if ~isempty(WaitStart{ii})
        TransitionIndices{ii} = [WaitStart{ii}-pred_Horizon:RetreatStart{ii}+pred_Horizon];
   else
        TransitionIndices{ii} = [CrossStart{ii}-pred_Horizon:RetreatStart{ii}+pred_Horizon];
   end
       
   CVPerformance{ii}.TranisitionMeanRMSE = mean(CVPerformance{ii}.RMSE(TransitionIndices{ii}));
   CVPerformance{ii}.TranisitionMeanFinalError = mean(CVPerformance{ii}.EndError(TransitionIndices{ii}));

end

save('CVPerformance.mat','CVPerformance');




%% figures
figure()
plot(CVPerformance{1,1}.RMSE,'-r*','MarkerSize',15);hold on;
plot(CVPerformance{1,1}.EndError,'-b*','MarkerSize',15);hold on;
plot(PedestrianAbsoluteVelocity,'-g*','MarkerSize',15);hold on;
xline(1);hold on;
% xline(EventIndices(ind,6)-EventIndices(ind,4)+1);hold on;
% xline(EventIndices(ind,8)-EventIndices(ind,4)+1);hold on;
% xline(EventIndices(ind,10)-EventIndices(ind,4)+1);
WaitStart = find(PedestrianDiscreteState==2,1,'first');
if ~isempty(WaitStart)
    xline(WaitStart);hold on;
end
CrossStart = find(PedestrianDiscreteState==3,1,'first');
if ~isempty(CrossStart)
    xline(CrossStart);hold on;
end
RetreatStart = find(PedestrianDiscreteState==4,1,'first');
if ~isempty(RetreatStart)
    xline(RetreatStart);hold on;
end

ylabel('RMSE error [m], End error [m], Pedestrian velocity [m/s]')
xlabel('Time steps [0.1 s]')
title('Tracking CVPerformance')
legend('RMSE error','End error','Crossing Speed')
set(gca,'fontsize', 18)
% % To save figure
% filename = 'Tracking CVPerformance';
% % FigH = figure('Position', get(0, 'Screensize'));
% saveas(gca,filename,'fig')
% saveas(gca,filename,'png')
% saveas(gca,filename,'eps')
% saveas(gca,filename,'emf')



















