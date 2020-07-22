%% This is the main file to run the  hybrid automaton
clear all


%% inputs
tStep = 0.1;
p_thres = 0.50;  % probability threshold for discrete state change


gg=11;

pred_Horizon_matrix = [10,15,20,25,30,35,40,45,50,55,60];
pred_Horizon = pred_Horizon_matrix(gg);
    
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

load('ExpectedGapData_W5_NewDTCurbDTCW_StartingTimeforCrossingGaps.mat');

% Read test train indices
load('HybridModelTestTrainIndices.mat')
TrackingTestIndices = TestIndices_woExtremeOutlier;

%% Load trained SVM model and test data
load('ExtremeOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel_CrossingsDataSplit.mat')
load('SVMTestData_ExtremeOutlier_StartGapExpectedGap_CorrectDTCurbDTCW.mat')
% load('MildOutlier_StartGap_NewDTCurb_CubicSVM_TrainedModel.mat')

% indices of interest
ApproachStart = EventIndices(:,4);
RetreatEnd = EventIndices(:,11);

DataBinSizes = [0.5,0.1,0.2,0.2,0.2,1];
startlimit = [0,0,0,0,0,0];
endlimit = [10,1,3,2.5,5,50];
N_obs = [1,2,3,4,5,6];


TestLength = length(TrackingTestIndices);





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
    
    for nn=2:M

        %initialize predicted states
        x_pred = zeros(pred_Horizon,4);
        q_pred = zeros(pred_Horizon,1);
%         x_pred(1,:) = Performance{ii}.Estimated_x(nn-1,:);
        x_pred(1,:) = Performance{ii}.GroundTruth_x(nn-1,:);
%         q_pred(1) = Performance{ii}.Estimated_q(nn-1);
        q_pred(1) = Performance{ii}.GroundTruth_q(nn-1);
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%             
%% Prediction horizon loop begins

        % state predictions for the horizon    
        for pp=2:pred_Horizon
            
            %% Pedestrian state update - constant velocity 
            
            %continuous state propagation
            HybridModel.pedestrian_motion_cv();
            x_pred(pp,:) = HybridModel.x;
            
            %discrete state update based on continuous state propagation
            T = Target;
            HybridModel.pedestrian_action_update_meas(T);
            
            %update the discrete state 
            q_pred(pp) = HybridModel.q;
            
        end % Prediction horizon loop ends
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
           
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
        
%         HybridModel.q = Performance{ii}.Estimated_q(nn-1,:);        %update the discrete state to previous estimated state
        HybridModel.q = Performance{ii}.GroundTruth_q(nn-1,:);        %update the discrete state to previous ground truth state
        T = Target;
        HybridModel.pedestrian_action_update_meas(T);
        

        
        %% save the predictions
        Performance{ii}.Predicted_x(nn,:) = reshape(x_pred',[1,pp*4]);
        Performance{ii}.Predicted_q(nn,:) = reshape(q_pred,[1,pp]);
%         Performance{ii}.Estimated_x(nn,:) = HybridModel.x;
%         Performance{ii}.Estimated_q(nn,:) = HybridModel.q; 
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
       if (PedestrianDiscreteState(bb-1)~=3 & PedestrianDiscreteState(bb)==3)
           temp=[temp;bb];
       end               
   end
   CrossStart{ii} = temp(end);  
   
% Predicted transitions
ApproachToWaitInstanceTemp=[];
ApproachToCrossInstanceTemp = [];
WaitToCrossInstanceTemp=[];
RetreatInstanceTemp=[];
ApproachToWaitHorizonTemp =[];
ApproachToCrossHorizonTemp = [];
WaitToCrossHorizonTemp = [];
RetreatHorizonTemp = [];
   for bb=1:M 
       for pp=2:pred_Horizon

        if (Performance{ii}.Predicted_q(bb,pp)==2 & Performance{ii}.Predicted_q(bb,pp-1)==1)
            ApproachToWaitInstanceTemp = [ApproachToWaitInstanceTemp;bb];
            ApproachToWaitHorizonTemp = [ApproachToWaitHorizonTemp;pp];
        end
        

        if (Performance{ii}.Predicted_q(bb,pp)==3 & Performance{ii}.Predicted_q(bb,pp-1)==1)
           ApproachToCrossInstanceTemp = [ApproachToCrossInstanceTemp;bb];
           ApproachToCrossHorizonTemp = [ApproachToCrossHorizonTemp;pp];
        end
        

        if (Performance{ii}.Predicted_q(bb,pp)==3 & Performance{ii}.Predicted_q(bb,pp-1)==2)
           WaitToCrossInstanceTemp = [WaitToCrossInstanceTemp;bb];
           WaitToCrossHorizonTemp = [WaitToCrossHorizonTemp;pp];
        end
        
        if (Performance{ii}.Predicted_q(bb,pp)==4 & Performance{ii}.Predicted_q(bb,pp-1)~=4)
            RetreatInstanceTemp = [RetreatInstanceTemp;bb];
            RetreatHorizonTemp = [RetreatHorizonTemp;pp];
        end
        
       end
   end


   ApproachToWaitPred{ii}.Instances{ii} = ApproachToWaitInstanceTemp;
   ApproachToCrossPred{ii}.Instances{ii} = ApproachToCrossInstanceTemp;
   WaitToCrossPred{ii}.Instances{ii} = WaitToCrossInstanceTemp;
   RetreatPred{ii}.Instances{ii} = RetreatInstanceTemp;

   ApproachToWaitPred{ii}.InHorizon{ii} = ApproachToWaitHorizonTemp;
   ApproachToCrossPred{ii}.InHorizon{ii} = ApproachToCrossHorizonTemp;
   WaitToCrossPred{ii}.InHorizon{ii} = WaitToCrossHorizonTemp;
   RetreatPred{ii}.InHorizon{ii} = RetreatHorizonTemp;


%    WaitPred{ii}.Instances{ii} = find(Performance{ii}.Predicted_q==2 & Performance{ii}.Predicted_q(:,1)~=2 );
%    CrossPred{ii}.Instances{ii} = find(Performance{ii}.Predicted_q==3  & Performance{ii}.Predicted_q(:,1)~=3 );
%    RetreatPred{ii}.Instances{ii} = find(Performance{ii}.Predicted_q==4  & Performance{ii}.Predicted_q(:,1)~=4);
   
 
if ~isempty(WaitStart{ii})
   len = length(ApproachToWaitPred{ii}.Instances{ii});  
   for jj=1:len
%         WaitPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(WaitPred{ii}.Instances{ii}(jj),:)==2,1,'first');
        ApproachToWaitPred{ii}.Error{ii}(jj) = WaitStart{ii} - ApproachToWaitPred{ii}.Instances{ii}(jj) - ApproachToWaitPred{ii}.InHorizon{ii}(jj)-1; 
   end
   
end


if ~isempty(ApproachToCrossPred{ii}.Instances{ii})
   len = length(ApproachToCrossPred{ii}.Instances{ii});  
   for jj=1:len
%         WaitPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(WaitPred{ii}.Instances{ii}(jj),:)==2,1,'first');
        ApproachToCrossPred{ii}.Error{ii}(jj) = CrossStart{ii} - ApproachToCrossPred{ii}.Instances{ii}(jj) - ApproachToCrossPred{ii}.InHorizon{ii}(jj)-1; 
   end
   
end


if ~isempty(WaitToCrossPred{ii}.Instances{ii})
   len = length(WaitToCrossPred{ii}.Instances{ii});  
   for jj=1:len
%         WaitPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(WaitPred{ii}.Instances{ii}(jj),:)==2,1,'first');
        WaitToCrossPred{ii}.Error{ii}(jj) = CrossStart{ii} - WaitToCrossPred{ii}.Instances{ii}(jj) - WaitToCrossPred{ii}.InHorizon{ii}(jj)-1; 
   end
end


if ~isempty(RetreatPred{ii}.Instances{ii})
   len = length(RetreatPred{ii}.Instances{ii});  
   for jj=1:len
%         WaitPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(WaitPred{ii}.Instances{ii}(jj),:)==2,1,'first');
        RetreatPred{ii}.Error{ii}(jj) = RetreatStart{ii} - RetreatPred{ii}.Instances{ii}(jj) - RetreatPred{ii}.InHorizon{ii}(jj)-1; 
   end
end

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

   %% earliest transition prediction and best transition prediction
   %% Wait transition
   
   if (~isempty(WaitStart{ii}) & ~isempty(ApproachToWaitPred{ii}.Instances{ii}))    %% actual wait, predicted wait
       Performance{ii}.FirstWaitPredInstance = WaitStart{ii} - ApproachToWaitPred{ii}.Instances{ii}(1);
       Performance{ii}.FirstWaitPredError = ApproachToWaitPred{ii}.Error{ii}(1);


        WaitErrorPos = ApproachToWaitPred{ii}.Error{ii}(ApproachToWaitPred{ii}.Error{ii}>=0);      
        if ~isempty(WaitErrorPos)        %consider only early prediction errors
            [~,minErrorInd] = min(WaitErrorPos);
            minErrorInd = find(ApproachToWaitPred{ii}.Error{ii}==WaitErrorPos(minErrorInd),1,'first');
        else
            [~,minErrorInd] = max(ApproachToWaitPred{ii}.Error{ii});        %if not available, consider the late prediction errors
        end

       Performance{ii}.BestWaitPredInstance = WaitStart{ii} - ApproachToWaitPred{ii}.Instances{ii}(minErrorInd);
       Performance{ii}.BestWaitPredError = ApproachToWaitPred{ii}.Error{ii}(minErrorInd);
   elseif (~isempty(WaitStart{ii}) & isempty(ApproachToWaitPred{ii}.Instances{ii})) %% actual wait, NO predicted wait
       
       Performance{ii}.FirstWaitPredInstance = 1e6;
       Performance{ii}.FirstWaitPredError = 1e6;

       Performance{ii}.BestWaitPredInstance = 1e6;
       Performance{ii}.BestWaitPredError =1e6;
       
   elseif (isempty(WaitStart{ii}) & ~isempty(ApproachToWaitPred{ii}.Instances{ii})) %% NO actual wait, predicted wait
       
       Performance{ii}.FirstWaitPredInstance = -1e6;
       Performance{ii}.FirstWaitPredError = -1e6;

       Performance{ii}.BestWaitPredInstance = -1e6;
       Performance{ii}.BestWaitPredError = -1e6;
   end
   
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %% Approach to Cross transition
   
   if (~isempty(CrossStart{ii}) & ~isempty(ApproachToCrossPred{ii}.Instances{ii}))    %% actual wait, predicted wait
       Performance{ii}.FirstApproachToCrossPredInstance = CrossStart{ii} - ApproachToCrossPred{ii}.Instances{ii}(1);
       Performance{ii}.FirstApproachToCrossPredError = ApproachToCrossPred{ii}.Error{ii}(1);

        CrossErrorPos = ApproachToCrossPred{ii}.Error{ii}(ApproachToCrossPred{ii}.Error{ii}>=0);
        if ~isempty(CrossErrorPos)
            [~,minErrorInd] = min(CrossErrorPos);
            minErrorInd = find(ApproachToCrossPred{ii}.Error{ii}==CrossErrorPos(minErrorInd),1,'first');
        else
            [~,minErrorInd] = max(ApproachToCrossPred{ii}.Error{ii});
        end

       Performance{ii}.BestApproachToCrossPredInstance = CrossStart{ii} - ApproachToCrossPred{ii}.Instances{ii}(minErrorInd);
       Performance{ii}.BestApproachToCrossPredError = ApproachToCrossPred{ii}.Error{ii}(minErrorInd);
   
   end
   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %% Wait to Cross transition   
   
  if (~isempty(CrossStart{ii}) & ~isempty(WaitToCrossPred{ii}.Instances{ii}))    %% actual wait, predicted wait
       Performance{ii}.FirstWaitToCrossPredInstance = CrossStart{ii} - WaitToCrossPred{ii}.Instances{ii}(1);
       Performance{ii}.FirstWaitToCrossPredError = WaitToCrossPred{ii}.Error{ii}(1);

        CrossErrorPos = WaitToCrossPred{ii}.Error{ii}(WaitToCrossPred{ii}.Error{ii}>=0);
        if ~isempty(CrossErrorPos)
            [~,minErrorInd] = min(CrossErrorPos);
            minErrorInd = find(WaitToCrossPred{ii}.Error{ii}==CrossErrorPos(minErrorInd),1,'first');
        else
            [~,minErrorInd] = max(WaitToCrossPred{ii}.Error{ii});
        end

       Performance{ii}.BestWaitToCrossPredInstance = CrossStart{ii} - WaitToCrossPred{ii}.Instances{ii}(minErrorInd);
       Performance{ii}.BestWaitToCrossPredError = WaitToCrossPred{ii}.Error{ii}(minErrorInd);
   
   end
   
   
      
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %% Cross to Retreat transition
if (~isempty(RetreatStart{ii}) & ~isempty(RetreatPred{ii}.Instances{ii}))    %% actual wait, predicted wait   
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
end
   
  
   
   %% identify transition indices
   if ~isempty(WaitStart{ii})
        TransitionIndices{ii} = [WaitStart{ii}-20:RetreatStart{ii}+20];
   else
        TransitionIndices{ii} = [CrossStart{ii}-20:RetreatStart{ii}+20];
   end
   
   Performance{ii}.MeanRMSE = mean(Performance{ii}.RMSE);
   Performance{ii}.MeanFinalError = mean(Performance{ii}.EndError);
   
   Performance{ii}.TranisitionMeanRMSE = mean(Performance{ii}.RMSE(TransitionIndices{ii}));
   Performance{ii}.TranisitionMeanFinalError = mean(Performance{ii}.EndError(TransitionIndices{ii}));

end

filename = strcat('Performance_CV_',num2str(pred_Horizon_matrix(gg)),'_SVM_StartGap.mat');

save(filename,'Performance');



% %% figures
% figure()
% % plot(Performance{1,1}.RMSE,'-r*','MarkerSize',15);hold on;
% % plot(Performance{1,1}.EndError,'-b*','MarkerSize',15);hold on;
% % plot(PedestrianAbsoluteVelocity,'-g*','MarkerSize',15);hold on;
% plot(Performance{1,1}.RMSE,'-','Color',[0.5843    0.8157    0.9882],'LineWidth',2);hold on;
% plot(Performance{1,1}.EndError,'-','Color',[0.9882    0.8157    0.5843],'LineWidth',2);hold on;
% plot(PedestrianAbsoluteVelocity,'-','Color',[0.5 0.5 0.5],'LineWidth',2);hold on;
% 
% 
% xline(1,'-','Color',[0.25 0.75 0.25],'LineWidth',2,'Label','Actual Approach Start');hold on;
% if ~isempty(waitStart)
%     xline(WaitStart{ii},'-','Color',[0.25 0.75 0.25],'LineWidth',2,'Label','Actual Wait Start','LabelVerticalAlignment','middle');hold on;
% end
% 
% if ~isempty(crossStart)
%     xline(CrossStart{ii},'-','Color',[0.25 0.75 0.25],'LineWidth',2,'Label','Actual Cross Start','LabelVerticalAlignment','middle');hold on;
% end
% 
% if ~isempty(retreatStart)
%     xline(RetreatStart{ii},'-','Color',[0.25 0.75 0.25],'LineWidth',2,'Label','Actual Retreat Start','LabelVerticalAlignment','middle');hold on;
% end
% 
% 
% if abs(Performance{ii}.BestWaitPredInstance)<1000
%     xline(WaitStart{ii} - Performance{ii}.BestWaitPredError - 1,'-','Color',[0.5 0 0.5],'LineWidth',2,'Label','Predicted Wait Start');hold on;
%     xline(WaitStart{ii} - Performance{ii}.BestWaitPredInstance,'-','Color',[0.1 0.1 0.1],'LineWidth',2,'Label','Predicted Wait Instance','LabelHorizontalAlignment','left');hold on;
% end 
% 
% if abs(Performance{ii}.BestApproachToCrossPredInstance)<1000
%     xline(CrossStart{ii} - Performance{ii}.BestApproachToCrossPredError - 1,'-','Color',[0.5 0 0.5],'LineWidth',2,'Label','Predicted Approach to Cross Start');hold on;
%     xline(CrossStart{ii} - Performance{ii}.BestApproachToCrossPredInstance,'-','Color',[0.1 0.1 0.1],'LineWidth',2,'Label','Predicted Approach to Cross Instance','LabelHorizontalAlignment','left');hold on;
% end 
%     
% if abs(Performance{ii}.BestWaitToCrossPredInstance)<1000
%     xline(CrossStart{ii} - Performance{ii}.BestWaitToCrossPredError - 1,'-','Color',[0.5 0 0.5],'LineWidth',2,'Label','Predicted Wait to Cross Start');hold on;
%     xline(CrossStart{ii} - Performance{ii}.BestWaitToCrossPredInstance,'-','Color',[0.1 0.1 0.1],'LineWidth',2,'Label','Predicted Wait to Cross Instance','LabelHorizontalAlignment','left');hold on;
% end 
%     
% if abs(Performance{ii}.BestRetreatPredInstance)<1000
%     xline(RetreatStart{ii} - Performance{ii}.BestRetreatPredError - 1,'-','Color',[0.5 0 0.5],'LineWidth',2,'Label','Predicted Retreat Start');hold on;
%     xline(RetreatStart{ii} - Performance{ii}.BestRetreatPredInstance,'-','Color',[0.1 0.1 0.1],'LineWidth',2,'Label','Predicted Retreat Instance','LabelHorizontalAlignment','left');hold on;
% end 
% 
% 
% 
% 
% 
% 
% 
% 
% ylabel('RMSE error [m], End error [m], Pedestrian velocity [m/s]')
% xlabel('Time steps [0.1 s]')
% title('Tracking performance')
% legend('RMSE error','End error','Crossing Speed')
% set(gca,'fontsize', 18)
% % % To save figure
% % filename = 'Tracking Performance';
% % % FigH = figure('Position', get(0, 'Screensize'));
% % saveas(gca,filename,'fig')
% % saveas(gca,filename,'png')
% % saveas(gca,filename,'eps')
% % saveas(gca,filename,'emf')



















