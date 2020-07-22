function [Performance] = TransitionPerformance(Performance,PedestrianDiscreteState,ii,M,pred_Horizon) 


%% Transition prediction performance

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Actual transitions 
   Performance{ii}.WaitStart = find(PedestrianDiscreteState==2,1,'first');
   Performance{ii}.RetreatStart = find(PedestrianDiscreteState==4,1,'first');
   
   % to avoid calculating a walk in wait state (but that should not happen  when
   % using the ground truth 'Pedestrian Discrete State' variable; nonetheless including this code for safety)
   temp=[];
   for bb=2:M 
       if (PedestrianDiscreteState(bb-1)~=3 & PedestrianDiscreteState(bb)==3)
           temp=[temp;bb];
       end               
   end
   Performance{ii}.CrossStart = temp(end);  

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Predicted transitions
ApproachToWaitInstanceTemp=[];
ApproachToCrossInstanceTemp = [];
WaitToCrossInstanceTemp=[];
RetreatInstanceTemp=[];
ApproachToWaitHorizonTemp =[];
ApproachToCrossHorizonTemp = [];
WaitToCrossHorizonTemp = [];
RetreatHorizonTemp = [];

% calculating the time instances and the exact locations within the
% prediction horizons at which predictions were made
for bb=1:M        % for every time step
   for pp=2:pred_Horizon    % for every time step within the prediction horizon

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

% if no predictions were made, then save the actual transitions as the
% predictions
if isempty(ApproachToWaitInstanceTemp)
    ApproachToWaitInstanceTemp = Performance{ii}.WaitStart;
    ApproachToWaitHorizonTemp = 2;
end

if isempty(ApproachToCrossInstanceTemp)
    ApproachToCrossInstanceTemp = Performance{ii}.CrossStart;
    ApproachToCrossHorizonTemp = 2;
end

if isempty(WaitToCrossInstanceTemp)
    WaitToCrossInstanceTemp = Performance{ii}.CrossStart;
    WaitToCrossHorizonTemp = 2;
end

if isempty(RetreatInstanceTemp)
    RetreatInstanceTemp = Performance{ii}.RetreatStart;
    RetreatHorizonTemp = 2;
end

% copy the transition perfromances to the variables
Performance{ii}.ApproachToWaitPred.Instances = ApproachToWaitInstanceTemp;
Performance{ii}.ApproachToCrossPred.Instances = ApproachToCrossInstanceTemp;
Performance{ii}.WaitToCrossPred.Instances = WaitToCrossInstanceTemp;
Performance{ii}.RetreatPred.Instances = RetreatInstanceTemp;

Performance{ii}.ApproachToWaitPred.InHorizon= ApproachToWaitHorizonTemp;
Performance{ii}.ApproachToCrossPred.InHorizon= ApproachToCrossHorizonTemp;
Performance{ii}.WaitToCrossPred.InHorizon = WaitToCrossHorizonTemp;
Performance{ii}.RetreatPred.InHorizon= RetreatHorizonTemp;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Transition prediction errors 
if ~isempty(Performance{ii}.WaitStart)
   len = length(Performance{ii}.ApproachToWaitPred.Instances);  
   for jj=1:len
%         WaitPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(WaitPred{ii}.Instances(jj),:)==2,1,'first');
        Performance{ii}.ApproachToWaitPred.Error(jj) = Performance{ii}.WaitStart - Performance{ii}.ApproachToWaitPred.Instances(jj) - Performance{ii}.ApproachToWaitPred.InHorizon(jj)+2; 
   end      
end

if ~isempty(Performance{ii}.ApproachToCrossPred.Instances)
   len = length(Performance{ii}.ApproachToCrossPred.Instances);  
   for jj=1:len
%         WaitPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(WaitPred{ii}.Instances(jj),:)==2,1,'first');
        Performance{ii}.ApproachToCrossPred.Error(jj) = Performance{ii}.CrossStart - Performance{ii}.ApproachToCrossPred.Instances(jj) - Performance{ii}.ApproachToCrossPred.InHorizon(jj)+2; 
   end   
end

if ~isempty(Performance{ii}.WaitToCrossPred.Instances)
   len = length(Performance{ii}.WaitToCrossPred.Instances);  
   for jj=1:len
%         WaitPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(WaitPred{ii}.Instances(jj),:)==2,1,'first');
        Performance{ii}.WaitToCrossPred.Error(jj) = Performance{ii}.CrossStart- Performance{ii}.WaitToCrossPred.Instances(jj) - Performance{ii}.WaitToCrossPred.InHorizon(jj)+2; 
   end
end

if ~isempty(Performance{ii}.RetreatPred.Instances)
   len = length(Performance{ii}.RetreatPred.Instances);  
   for jj=1:len
%         WaitPred{ii}.InHorizon{ii}(jj) = find(Performance{ii}.Predicted_q(WaitPred{ii}.Instances(jj),:)==2,1,'first');
        Performance{ii}.RetreatPred.Error(jj) = Performance{ii}.RetreatStart - Performance{ii}.RetreatPred.Instances(jj) - Performance{ii}.RetreatPred.InHorizon(jj)+2; 
   end
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% earliest transition prediction and best transition prediction

%% 1)Wait transition
if (~isempty(Performance{ii}.WaitStart) & ~isempty(Performance{ii}.ApproachToWaitPred.Instances))    %% actual wait, predicted wait
   Performance{ii}.FirstWaitPredInstance = Performance{ii}.WaitStart - Performance{ii}.ApproachToWaitPred.Instances(1);
   Performance{ii}.FirstWaitPredError = Performance{ii}.ApproachToWaitPred.Error(1);

    WaitErrorPos = Performance{ii}.ApproachToWaitPred.Error(Performance{ii}.ApproachToWaitPred.Error>=0);      
    if ~isempty(WaitErrorPos)        %consider only early prediction errors
        [~,minErrorInd] = min(WaitErrorPos);
        minErrorInd = find(Performance{ii}.ApproachToWaitPred.Error==WaitErrorPos(minErrorInd),1,'first');
    else
        [~,minErrorInd] = max(Performance{ii}.ApproachToWaitPred.Error);        %if not available, consider the late prediction errors
    end

   Performance{ii}.BestWaitPredInstance = Performance{ii}.WaitStart - Performance{ii}.ApproachToWaitPred.Instances(minErrorInd);
   Performance{ii}.BestWaitPredError = Performance{ii}.ApproachToWaitPred.Error(minErrorInd);
elseif (~isempty(Performance{ii}.WaitStart) & isempty(Performance{ii}.ApproachToWaitPred.Instances)) %% actual wait, NO predicted wait

   Performance{ii}.FirstWaitPredInstance = 1e6;
   Performance{ii}.FirstWaitPredError = 1e6;

   Performance{ii}.BestWaitPredInstance = 1e6;
   Performance{ii}.BestWaitPredError =1e6;

elseif (isempty(Performance{ii}.WaitStart) & ~isempty(Performance{ii}.ApproachToWaitPred.Instances)) %% NO actual wait, predicted wait

   Performance{ii}.FirstWaitPredInstance = -1e6;
   Performance{ii}.FirstWaitPredError = -1e6;

   Performance{ii}.BestWaitPredInstance = -1e6;
   Performance{ii}.BestWaitPredError = -1e6;
end
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) Approach to Cross transition

if (~isempty(Performance{ii}.CrossStart) & ~isempty(Performance{ii}.ApproachToCrossPred.Instances))    %% actual wait, predicted wait
   Performance{ii}.FirstApproachToCrossPredInstance = Performance{ii}.CrossStart - Performance{ii}.ApproachToCrossPred.Instances(1);
   Performance{ii}.FirstApproachToCrossPredError = Performance{ii}.ApproachToCrossPred.Error(1);

    CrossErrorPos = Performance{ii}.ApproachToCrossPred.Error(Performance{ii}.ApproachToCrossPred.Error>=0);
    if ~isempty(CrossErrorPos)
        [~,minErrorInd] = min(CrossErrorPos);
        minErrorInd = find(Performance{ii}.ApproachToCrossPred.Error==CrossErrorPos(minErrorInd),1,'first');
    else
        [~,minErrorInd] = max(Performance{ii}.ApproachToCrossPred.Error);
    end

   Performance{ii}.BestApproachToCrossPredInstance = Performance{ii}.CrossStart - Performance{ii}.ApproachToCrossPred.Instances(minErrorInd);
   Performance{ii}.BestApproachToCrossPredError = Performance{ii}.ApproachToCrossPred.Error(minErrorInd);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 3) Wait to Cross transition   

if (~isempty(Performance{ii}.CrossStart) & ~isempty(Performance{ii}.WaitToCrossPred.Instances))    %% actual wait, predicted wait
   Performance{ii}.FirstWaitToCrossPredInstance = Performance{ii}.CrossStart - Performance{ii}.WaitToCrossPred.Instances(1);
   Performance{ii}.FirstWaitToCrossPredError = Performance{ii}.WaitToCrossPred.Error(1);

    CrossErrorPos = Performance{ii}.WaitToCrossPred.Error(Performance{ii}.WaitToCrossPred.Error>=0);
    if ~isempty(CrossErrorPos)
        [~,minErrorInd] = min(CrossErrorPos);
        minErrorInd = find(Performance{ii}.WaitToCrossPred.Error==CrossErrorPos(minErrorInd),1,'first');
    else
        [~,minErrorInd] = max(Performance{ii}.WaitToCrossPred.Error);
    end

   Performance{ii}.BestWaitToCrossPredInstance = Performance{ii}.CrossStart - Performance{ii}.WaitToCrossPred.Instances(minErrorInd);
   Performance{ii}.BestWaitToCrossPredError = Performance{ii}.WaitToCrossPred.Error(minErrorInd);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 4) Cross to Retreat transition
if (~isempty(Performance{ii}.RetreatStart) & ~isempty(Performance{ii}.RetreatPred.Instances))    %% actual wait, predicted wait      
Performance{ii}.FirstRetreatPredInstance = Performance{ii}.RetreatStart - Performance{ii}.RetreatPred.Instances(1);
Performance{ii}.FirstRetreatPredError = Performance{ii}.RetreatPred.Error(1);

RetreatErrorPos = Performance{ii}.RetreatPred.Error(Performance{ii}.RetreatPred.Error>=0);
if ~isempty(RetreatErrorPos)
    [~,minErrorInd] = min(RetreatErrorPos);
    minErrorInd = find(Performance{ii}.RetreatPred.Error==RetreatErrorPos(minErrorInd),1,'first');
else
    [~,minErrorInd] = max(Performance{ii}.RetreatPred.Error);
end

Performance{ii}.BestRetreatPredInstance = Performance{ii}.RetreatStart - Performance{ii}.RetreatPred.Instances(minErrorInd);
Performance{ii}.BestRetreatPredError = Performance{ii}.RetreatPred.Error(minErrorInd);

end
  

% %% identify transition indices
% if ~isempty(Performance{ii}.WaitStart)       % 2 s before wait to 2s after wait; 2 s before cross to 2s after reaching end of crosswalk
%     Performance{ii}.TransitionIndices = unique([Performance{ii}.WaitStart-20:Performance{ii}.WaitStart+20,Performance{ii}.CrossStart-20:Performance{ii}.RetreatStart+20]);
% else
%     Performance{ii}.TransitionIndices = unique([Performance{ii}.CrossStart-20:Performance{ii}.RetreatStart+20]);
% end


Performance{ii}.TransitionIndices = unique([Performance{ii}.CrossStart-20:Performance{ii}.CrossStart+20]);



   
end