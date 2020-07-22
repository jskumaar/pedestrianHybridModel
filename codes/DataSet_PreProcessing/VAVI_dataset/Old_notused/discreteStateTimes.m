%% this script is a simplified version of 'eventsDataSegregator.m'
% identifies indices for approach, wait, cross and retreat
% appraoch separated into approach (close to wait area) and approach (in
% the wait area); similarly retreat segregated into two based on proximity
% to crosswalk; this is done to compare the behaviors when they are fully
% walking; can combine the approach if needed.

clear all
close all

[EventsData,~] = xlsread('EventsData.xlsx',1);
EventsData(:,3:8) = round([double(int32(EventsData(:,3:8)*10))/10],1);
EventsDataNew = [];
EventsIndices = [];
ActivityVelocity = [];

for kk=1:30
    for jj=1:3
        
%         %read pedestrian position data
%         [PedZohData,~] = xlsread('PedestrianDataZoh.xlsx',6*(kk-1)+jj);
%         FilterShift=6;
%         PedZohData = PedZohData(FilterShift+1:end,:);      %To remove the shift due to Zoh Filter       
%         %filtered measurement data - EWMA filter
%         alpha = 0.25;
%         a1 = [1, -(1-alpha)];
%         b1 = alpha;
%         shift = 3; %to compensate for delay introduced by the filter
%         PedZohDataEMAFiltered = PedZohData;
%         PedZohDataEMAFiltered(:,6:9) = filter(b1,a1,PedZohData(:,6:9));
%         PedZohDataEMAFiltered = PedZohDataEMAFiltered(shift:end,:);
%         PedZohDataEMAFiltered(:,3) = round([double(int32(PedZohDataEMAFiltered(:,3)*10))/10],1);
%  
        
        [PedRawData,~] = xlsread('PedestrianDataRaw.xlsx',6*(kk-1)+jj);
        
        % Pedestrian position filter
        [xZoh,TSteps] = holdFilter(PedRawData(:,4),PedRawData(:,3),Ts,tStep,1);
        [yZoh,~] = holdFilter(PedRawData(:,5),PedRawData(:,3),Ts,tStep,1);
        %% 2)WMA (Weighted Moving Average) - equal weights
        [xPosFiltWMA,~] = MwaFilter(xZoh,TSteps,5);
        [yPosFiltWMA,~] = MwaFilter(yZoh,TSteps,5);

        %% Pedstrian position filter
        %% 3) Diff WMA   
        [xVelWMA] = DifferenceOperation(xPosFiltWMA,TSteps,2);    
        [yVelWMA] = DifferenceOperation(yPosFiltWMA,TSteps,2);   

        %% 3) MWA for Diff WMA
        [xVelFiltWMA,~] = MwaFilter(xVelWMA,TSteps,5);
        [yVelFiltWMA,~] = MwaFilter(yVelWMA,TSteps,5);    
        
        
        
        
        %% events' times
        PlotEvents = EventsData(36*(kk-1)+6*(jj-1)+1:36*(kk-1)+6*(jj),3:8);
        PlotEvents = reshape(PlotEvents,[size(PlotEvents,1)*size(PlotEvents,2),1]);
        PlotEvents = [PlotEvents,reshape(repmat([1:6],6,1),[36,1])];

        WaitStart = PlotEvents(1:6,1);
        WaitEnd = PlotEvents(7:12,1);
        CrossStart = WaitEnd;
        CrossEnd = PlotEvents(19:24,1);
        
        %% identify approach start, retreat end indices based on proximity to
        %racks and balls
        temp.taskCloseInd = [];
        temp.waitStartInd = [];
        
        
        %% identify indices for retreat end      
        for ii=1:size(TSteps,1)-1
            if((abs(xPosFiltWMA(ii,1))<9.9 && abs(xPosFiltWMA(ii+1,1))>9.9))
                temp.taskCloseInd = [temp.taskCloseInd;ii];
            end
        end
      
        if  (size(temp.taskCloseInd,1)<6)   %to check if all six retreat ends have been identified
            error('Not all retreat end indices identified')
        else
            limit = 1;
            while (size(temp.taskCloseInd,1)~=6)         % to remove the indices that got captured due to noise/jitter around the ball pickup/drop area
               temp.removeInd = find(diff(temp.taskCloseInd,1)<limit);
               temp.taskCloseInd(temp.removeInd+1)=[];
               limit = limit + 1;
            end

        end
        
        %% identify indices for approach start 
             
        for ii=1:size(TSteps,1)-1
            if((abs(xPosFiltWMA(ii,1))>9.9 && abs(xPosFiltWMA(ii+1,1))<9.9))
                temp.taskCloseInd = [temp.taskCloseInd;ii];
            end
        end
        
              %to check if all five approach starts have been identified
        if  (size(temp.taskCloseInd,1)<11)   
            error('Not all approach indices identified')
        else
            limit = 1;
            if temp.taskCloseInd(end)>temp.taskCloseInd(6)  % to identify if the user has crossed back the 9.9 mm line after placing the ball and started approaching the crosswalk
                temp.taskCloseInd(end)=[];
            end
            while (size(temp.taskCloseInd,1)~=11)         
               temp.removeInd = find(diff(temp.taskCloseInd,1)<limit);
               temp.taskCloseInd(temp.removeInd+1)=[];
               limit = limit + 1;
            end

        end
        

        %adding first moving index for first approach start
        temp.approachRetreatInd = [find(diff(yPosFiltWMA(:,1))>0.10,1,'first');temp.taskCloseInd];
        temp.approachRetreatInd = sort(temp.approachRetreatInd);
        temp.approachRetreatInd = reshape(temp.approachRetreatInd,[2,6]);
        
        % actual wait start when velocity is less than 0.05 m/s; but this
        % is on noisy Zoh data!! Update this to filtered position data!
        for ii=1:6
            rVelFiltWMA = sqrt(xVelFiltWMA.^2 + yVelFiltWMA.^2);
            temp.waitStartInd(ii,1) = find(((rVelFiltWMA<0.05) & (TSteps(:,3)>WaitStart(ii))),1,'first');            
            WaitStartTime(ii,1) = TSteps(temp.waitStartInd(ii),3);
            if TSteps(temp.waitStartInd(ii),3)>CrossStart(ii)
                WaitStartTime(ii,1) = WaitStart(ii);
            end
            temp.retreatStartInd(ii,1) = find(((xPosFiltWMA(:,4)<-1) & (TSteps(:,3)>CrossEnd(ii))),1,'first');            
            RetreatStartTime(ii,1) = TSteps(temp.retreatStartInd(ii),3);            
        end
        %% indices to save
        ApproachStartTime = TSteps(temp.approachRetreatInd(1,:),3);
        RetreatEndTime = TSteps(temp.approachRetreatInd(2,:),3);                      
      
        for ii=1:6
            ApproachEndTime(ii,1) = WaitStart(ii)-0.1;
            
            ApproachInWaitAreaEndTime(ii,1) = WaitStartTime(ii)-0.1;
            if (ApproachEndTime(ii)~=ApproachInWaitAreaEndTime(ii))
               ApproachInWaitAreaStartTime(ii,1) = ApproachEndTime(ii,1)+0.1;
            else
                ApproachInWaitAreaStartTime(ii,1)=ApproachInWaitAreaEndTime(ii);
            end
            WaitEndTime(ii,1) = WaitEnd(ii)-0.1;
            CrossStartTime(ii,1) = WaitEnd(ii);
            CrossEndTime(ii,1) = CrossEnd(ii)-0.1;
            RetreatInWaitAreaStartTime(ii,1) = CrossEnd(ii); 
            RetreatInWaitAreaEndTime(ii,1) = RetreatStartTime(ii)-0.1;
        end 

       
    %% write to excel
    
    EventsDataNew = [EventsDataNew;[repmat([kk,jj],6,1),ApproachStartTime,ApproachEndTime,ApproachInWaitAreaStartTime,ApproachInWaitAreaEndTime,...
        WaitStartTime,WaitEndTime,CrossStartTime,CrossEndTime,RetreatInWaitAreaStartTime,RetreatInWaitAreaEndTime,RetreatStartTime,RetreatEndTime]];
    


    end
end


excelHeader2 = {'SubjectID','ScenarioID','ApproachStartTime','ApproachEndTime','ApproachInWaitAreaStartTime','ApproachInWaitAreaEndTime','WaitStartTime','WaitEndTime',...
                'CrossStartTime','CrossEndTime','RetreatInWaitAreaStartTime','RetreatInWaitAreaEndTime','RetreatStartTime','RetreatEndTime'};

xlswrite('DiscreteEventsDatawithApproach.xlsx',EventsDataNew,1,'A2');
xlswrite('DiscreteEventsDatawithApproach.xlsx',excelHeader2,1,'A1');      

        
        