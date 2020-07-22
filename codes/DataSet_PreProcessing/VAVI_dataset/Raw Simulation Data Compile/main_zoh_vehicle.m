% zero order hold


clear all
close all

for kk = 1:30 %for every subject
    for jj = 1:3 %for every scenario
               
     [VehPos,~] = xlsread('VehicleDataResampled.xlsx',6*(kk-1)+jj);
     VehPos = VehPos(2:end,:);
     XPosA = [];
     XVelA = [];
     XAccA = [];
     XPosB = [];
     XVelB = [];
     XAccB = [];
     %Zoh
     N = size(VehPos,2);
     
     temp.LaneChangeInd = find(diff(VehPos(:,3))~=0);
     VehPosA = VehPos(1:temp.LaneChangeInd,:);
     VehPosB = VehPos(temp.LaneChangeInd+1:end,:);
     
     for ii=1:N-4
         %Lane A
         [PosA,TStepsA] = zohFilter(VehPosA(:,ii+4),VehPosA(:,4),0.1);
         XPosA = [XPosA,PosA];
         
         temp.XZohDiff = diff(PosA);
         VelA =  (temp.XZohDiff(1:end-1) +  temp.XZohDiff(2:end))/(2*(TStepsA(2)-TStepsA(1)));
         XVelA = [XVelA,[0;VelA;0]];
         
         temp.XZohDiff = diff([0;VelA;0]);
         AccA =  temp.XZohDiff(1:end-1) +  temp.XZohDiff(2:end)/(2*(TStepsA(2)-TStepsA(1)));
         XAccA = [XAccA,[0;AccA;0;]];
         
         
         % Lane B
         [PosB,TStepsB] = zohFilter(VehPosB(:,ii+4),VehPosB(:,4),0.1);
         XPosB = [XPosB,PosB];
         
         temp.XZohDiff = diff(PosB);
         VelB =  (temp.XZohDiff(1:end-1) +  temp.XZohDiff(2:end))/(2*(TStepsB(2)-TStepsB(1)));
         XVelB = [XVelB,[0;VelB;0]];
         
         temp.XZohDiff = diff([0;VelB;0]);
         AccB =  temp.XZohDiff(1:end-1) +  temp.XZohDiff(2:end)/(2*(TStepsB(2)-TStepsB(1)));
         XAccB = [XAccB,[0;AccB;0;]];
         
     end
             

     %% add subject and scenario details
     matLength = size(TStepsA',1)+size(TStepsB',1);    
     VehiclePositionZoh = [repmat([kk,jj],matLength,1),[TStepsA,TStepsB]',[XPosA;XPosB]];
     VehicleVelocityZoh = [repmat([kk,jj],matLength,1),[TStepsA,TStepsB]',[XVelA;XVelB]];
     VehicleAccelerationZoh = [repmat([kk,jj],matLength,1),[TStepsA,TStepsB]',[XAccA;XAccB]];

    %% write to excel
    excelHeader = {'SubjectID','ScenarioID','Time'};
    for ll=1:N-4
        excelHeader = [excelHeader, num2str(ll)];
    end
    
    xlswrite('VehiclePositionZoh.xlsx',VehiclePositionZoh,6*(kk-1)+jj,'A2');
    xlswrite('VehiclePositionZoh.xlsx',excelHeader,6*(kk-1)+jj,'A1'); 
    
    xlswrite('VehicleVelocityZoh.xlsx',VehicleVelocityZoh,6*(kk-1)+jj,'A2');
    xlswrite('VehicleVelocityZoh.xlsx',excelHeader,6*(kk-1)+jj,'A1'); 
    
    xlswrite('VehicleAccelerationZoh.xlsx',VehicleAccelerationZoh,6*(kk-1)+jj,'A2');
    xlswrite('VehicleAccelerationZoh.xlsx',excelHeader,6*(kk-1)+jj,'A1'); 


    end 
end