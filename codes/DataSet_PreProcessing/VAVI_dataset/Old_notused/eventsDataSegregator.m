%% Events data segregator
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
            xVelFiltWMA
            temp.waitStartInd(ii,1) = find(((yPosFiltWMA<0.05) & (PedZohDataEMAFiltered(:,3)>WaitStart(ii))),1,'first');            
            WaitStartTime(ii,1) = PedZohDataEMAFiltered(temp.waitStartInd(ii),3);
            if PedZohDataEMAFiltered(temp.waitStartInd(ii),3)>CrossStart(ii)
                WaitStartTime(ii,1) = WaitStart(ii);
            end
            temp.retreatStartInd(ii,1) = find(((PedZohDataEMAFiltered(:,4)<-1) & (PedZohDataEMAFiltered(:,3)>CrossEnd(ii))),1,'first');            
            RetreatStartTime(ii,1) = PedZohDataEMAFiltered(temp.retreatStartInd(ii),3);            
        end
        %% indices to save
        ApproachStartTime = PedZohDataEMAFiltered(temp.approachRetreatInd(1,:),3);
        RetreatEndTime = PedZohDataEMAFiltered(temp.approachRetreatInd(2,:),3);                      
      
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
%         IndShift = FilterShift + int32((PedZohDataEMAFiltered(1,3)-PedZohData(1,3))*10);
%         
%         ApproachStartInd = int32(ApproachStartTime*10)-IndShift;
%         ApproachEndInd = int32(ApproachEndTime*10)-IndShift;
%         WaitStartInd = int32(WaitStartTime*10)-IndShift;
%         WaitEndInd = int32(WaitEndTime*10)-IndShift;
%         CrossStartInd = int32(CrossStartTime*10)-IndShift;
%         CrossEndInd = int32(CrossEndTime*10)-IndShift;
%         RetreatStartInd = int32(RetreatStartTime*10)-IndShift;
%         RetreatEndInd = int32(RetreatEndTime*10)-IndShift;
        
%         if ApproachStartInd(1)==0
%             ApproachStartInd=1;
%         else if WaitStartInd
        
       %% activity wise velocity
%        for ii=1:6
%             ApproachStartInd(ii) = find(PedZohDataEMAFiltered(:,3)==ApproachStartTime(ii));
%             ApproachEndInd(ii) = find(PedZohDataEMAFiltered(:,3)==ApproachEndTime(ii));
%             WaitStartInd(ii) = find(PedZohDataEMAFiltered(:,3)==WaitStartTime(ii));
%             WaitEndInd(ii) = find(PedZohDataEMAFiltered(:,3)==WaitEndTime(ii));
%             CrossStartInd(ii) = find(PedZohDataEMAFiltered(:,3)==CrossStartTime(ii));
%             CrossEndInd(ii) = find(PedZohDataEMAFiltered(:,3)==CrossEndTime(ii));
%             RetreatStartInd(ii) = find(PedZohDataEMAFiltered(:,3)==RetreatStartTime(ii));
%             RetreatEndInd(ii) = find(PedZohDataEMAFiltered(:,3)==RetreatEndTime(ii));       
%        end
       
%         for ii=1:6
%             approachVel{ii} = PedZohDataEMAFiltered(ApproachStartInd(ii):ApproachEndInd(ii)-1,6:7);
%             waitVel{ii} = PedZohDataEMAFiltered(WaitStartInd(ii):WaitEndInd(ii)-1,6:7);
%             crossVel{ii} =  PedZohDataEMAFiltered(CrossStartInd(ii):CrossEndInd(ii)-1,6:7);
%             retreatVel{ii} = PedZohDataEMAFiltered(RetreatStartInd(ii):RetreatEndInd(ii),6:7);
%         end
%         
%         for ii=1:6
%             meanApproachVel(ii,1) = mean(vecnorm(approachVel{ii}')');
%             meanWaitVel(ii,1) = mean(vecnorm(waitVel{ii}')');
%             meanCrossVel(ii,1) = mean(vecnorm(crossVel{ii}')');
%             meanRetreatVel(ii,1) = mean(vecnorm(retreatVel{ii}')');
%         end
%        

       
    %% write to excel
%     excelHeader = {'SubjectID','ScenarioID','Time','x-position','z-position','x-velocity','z-velocity','x-acceleration','z-acceleration'};
% 
%     xlswrite('PedestrianDataPosZohVelEwma.xlsx',PedZohDataEMAFiltered,6*(kk-1)+jj,'A2');
%     xlswrite('PedestrianDataPosZohVelEwma.xlsx',excelHeader,6*(kk-1)+jj,'A1'); 
    
    
    EventsDataNew = [EventsDataNew;[repmat([kk,jj],6,1),ApproachStartTime,ApproachEndTime,ApproachInWaitAreaStartTime,ApproachInWaitAreaEndTime,...
        WaitStartTime,WaitEndTime,CrossStartTime,CrossEndTime,RetreatInWaitAreaStartTime,RetreatInWaitAreaEndTime,RetreatStartTime,RetreatEndTime]];
    
%     EventsIndices = [EventsIndices;[repmat([kk,jj],6,1),ApproachStartInd,ApproachEndInd,WaitStartInd,...
%                 WaitEndInd,CrossStartInd,CrossEndInd,RetreatStartInd,RetreatEndInd]];
%             
%             
%     ActivityVelocity = [ActivityVelocity;[repmat([kk,jj],6,1),meanApproachVel,meanWaitVel,...
%                             meanCrossVel,meanRetreatVel]];
        

    end
end


excelHeader2 = {'SubjectID','ScenarioID','ApproachStartTime','ApproachEndTime','ApproachInWaitAreaStartTime','ApproachInWaitAreaEndTime','WaitStartTime','WaitEndTime',...
                'CrossStartTime','CrossEndTime','RetreatInWaitAreaStartTime','RetreatInWaitAreaEndTime','RetreatStartTime','RetreatEndTime'};

xlswrite('EventsDatawithApproach.xlsx',EventsDataNew,1,'A2');
% xlswrite('EventsDataNew.xlsx',EventsIndices,2,'A2');
xlswrite('EventsDatawithApproach.xlsx',excelHeader2,1,'A1');      
% xlswrite('EventsDataNew.xlsx',excelHeader2,2,'A1'); 

% excelHeader3 = {'SubjectID','ScenarioID','ApproachVelocity','WaitVelocity',...
%                 'CrossVelocity','RetreatVelcoity'};
% 
% xlswrite('ActivityVelocity.xlsx',ActivityVelocity,1,'A2');
% xlswrite('ActivityVelocity.xlsx',excelHeader3,1,'A1');
    
%     figure()
%     plot(ApproachStartInd,'*');hold on;
%     plot(ApproachEndInd,'*');hold on;
%     plot(WaitStartInd,'*');hold on;
%     plot(WaitEndInd,'*');hold on;
%     plot(CrossStartInd,'*');hold on;
%     plot(CrossEndInd,'*');hold on;
%     plot(RetreatStartInd,'*');hold on;
%     plot(RetreatEndInd,'*');hold on; 
%     legend('ApproachStartInd','ApproachEndInd','WaitStartInd','WaitEndInd','CrossStartInd',...
%             'CrossEndInd','RetreatStartInd','RetreatEndInd');
        
        
        