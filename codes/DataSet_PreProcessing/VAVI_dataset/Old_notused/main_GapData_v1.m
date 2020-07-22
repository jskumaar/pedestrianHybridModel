%% File to compile data for symbolic non-linear regression
% Split data into four actions - Approach, Wait, Cross, and Retreat
% Input Parameters: 
% a)pedestrian position, b)velocity, c)acceleration, d)heading,
% e)gaze angle, f)gaze ratio at vehicle, g)DTC, h)distance between vehicles,
% i)both lane vehicle positions, j)speeds, k)acccelerations, l)distance to
% crosswalk, m)distance between vehicle and pedestrian, n)distance to curb,
% o)time to collision

clear all
close all
MeanGPInput = [];
MeanGPOutput = [];
StdGPInput = [];
StdGPOutput= [];


%% Filter parameters        
Ts = 0.1;       % sampling time
tStep = 0.1;    % zoh filter resampling time
snip = 0;       % no.of points to remove from start after filtering
order = 2;      % order of difference operator
roundDen = 100; % for 1/100 i.e 0.01
indexShift = 9; % index increase of pedestrian from vehicle
timeInGap = 15; % within each gap interesting times from start of gap and before end of gap

%Read events data
[ActivityData,~] = xlsread('DiscreteStateEventIndicesW5.xlsx');
[GapData,~] = xlsread('VehicleGapTimesV6.xlsx');   %changed the -0.1 gap duratiosn to 0.1


%indices.end = int32(GapData(:,7)*10+1);      
%indices.Crossing = find(GapData(:,4)==1);
%indices.NewEnd = indices.end;
indices.start = GapData(:,5);
indices.NewStart = GapData(:,6);
indices.end = GapData(:,7);
indices.NewEnd = GapData(:,9);     %On road indices
% for ii=1:length(indices.Crossing)
%     ind = find(ActivityData(:,1)==GapData(indices.Crossing(ii),1)&....
%                ActivityData(:,2)==GapData(indices.Crossing(ii),2)&...
%                ActivityData(:,3)==GapData(indices.Crossing(ii),3));
%     if ActivityData(ind,8)>indices.start(indices.Crossing(ii))
%         indices.NewEnd(indices.Crossing(ii)) = ActivityData(ind,8);
%     end
% end

%% Data compile loop 
for kk=1:30              %subject ID
    for jj=1:3          % scenario ID

        %% Read Data
        %Read nearest vehicle DTC, position, next vehicle position,
        %adjacent vehicle position data
        [vehicleData,~] = xlsread('VehicleDTCSpeedDataV2.xlsx',6*(kk-1)+jj);       
        vehicleData = ceil(vehicleData*roundDen)/roundDen;
        
        %Read gaze data
        [GazeNumericData,GazeObjects] = xlsread('GazeNoDuplicate.xlsx',6*(kk-1)+jj);
        %Read pedestrian position data
        [PedestrianData,~] = xlsread('PedestrianDiscreteDataW5.xlsx',6*(kk-1)+jj);
        %pedestrianData = ceil(pedestrianData*roundDen)/roundDen;
                    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%     Pedestrian Parameters       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Gaze at Vehicle categorical vector
GazeAtVehicle = zeros(size(vehicleData,1),1);
for ii=1:size(GazeObjects,1)-1
    if (strcmp(GazeObjects{ii+1,11},'Looking at approach Vehicle'))
        GazeAtVehicle(ii) = 1;
    elseif(isempty(GazeObjects{ii+1,11}))
         GazeAtVehicle(ii) = NaN;
    end
end


%Rolling Window gaze ratio at vehicle
WindowSize = 10; % 1 seconds
window = ones(WindowSize,1)/WindowSize;
GazeAtVehicleRatio = conv(GazeAtVehicle,window,'same');


%Gaze Angle
GazeX = GazeNumericData(:,4);
GazeY = GazeNumericData(:,5);
GazeZ = GazeNumericData(:,6);

%         GazeX = MwaFilter(GazeNumericData(:,4),GazeNumericData(:,3),5);
%         GazeY = MwaFilter(GazeNumericData(:,5),GazeNumericData(:,3),5);
%         GazeZ = MwaFilter(GazeNumericData(:,6),GazeNumericData(:,3),5);
% %         GazeAngle = atan2(GazeZ,GazeX);
% %GazeAngleFilt = MwaFilter(GazeAngle,GazeNumericData(:,3),5);
% 
% figure()
% gazeXPlot = plot(TSteps,GazeX,'.','MarkerSize',10);
% figure()
% gazeYPlot = plot(TSteps,GazeY,'.','MarkerSize',10);
% figure()
% gazeZPlot = plot(TSteps,GazeZ,'.','MarkerSize',10);
        
%% Gaze FFT - removing Nans
GazeZNaN =  GazeZ;
GazeZNaN(isnan(GazeZNaN))=0; 
GazeXNaN =  GazeX;
GazeXNaN(isnan(GazeXNaN))=0; 
% TStepsNaN = TSteps;
% TStepsNaN(isnan(GazeZ))=[]; 

% Fs = 1/Ts;
% %%Time specifications:
% N = length(TStepsNaN);
% %%Fourier Transform:
% Xfft = fftshift(fft(GazeZNaN));
% %%Frequency specifications:
% dF = Fs/N;                      % hertz
% yf = -Fs/2:dF:Fs/2-dF;           % hertz
% %%Plot the spectrum:
% figure;
% plot(yf,abs(Xfft)/N);
% xlabel('Frequency (in hertz)');
% title('Magnitude Response');

%% 3)Butterworth
[b,a] = butter(6,0.2);      %In FFT most information is in +/- 1 Hz range
GazeZNaNButterFilt = filter(b,a,GazeZNaN);
GazeXNaNButterFilt = filter(b,a,GazeXNaN);
GazeAngle = atan2(GazeZNaNButterFilt,GazeXNaNButterFilt); 
% figure()
% plot(TStepsNaN,GazeZNaNButterFilt,'.','MarkerSize',10);  
        
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Vehicle Parameters
% no filtering for now - just use the raw data calculated
        
%% Compile all data
%RegressionOutputFullData = [xPosFiltEWMA.data,yPosFiltEWMA.data,xVelFiltWMA.data,yVelFiltWMA.data,....
%                           xAccFiltWMA.data,yAccFiltWMA.data,velWMA,heading,AccWMA,AccNor,AccTan,TurnRate];
% RegressionInputFullData = [GazeAngle,GazeAtVehicleRatio,vehicleData(:,5:10)];
        
%% gap wise parameter calculations
%

for ii=1:6 % each crossing
    
GapStart = find(GapData(:,1)==kk & GapData(:,2)==jj & GapData(:,3)==ii,1,'first');
GapEnd = find(GapData(:,1)==kk & GapData(:,2)==jj & GapData(:,3)==ii,1,'last');
shortCount = 0;
longCount = 0;
cumWait = 0;
    for GapNumber=GapStart:GapEnd
         indStart = indices.start(GapNumber);
         indEnd = indices.NewEnd(GapNumber);
        
        %gap duration
         GapDuration(GapNumber,1) = double(indEnd-indStart)/10;
         
        %gap counters
        if (GapDuration(GapNumber)<4 && GapNumber~=GapEnd)
            longGapCounter(GapNumber,1) = longCount;
            shortGapCounter(GapNumber,1) = shortCount+1;
            shortCount = shortCount+1;
        elseif (GapDuration(GapNumber)>=4 && GapNumber~=GapEnd)
            shortGapCounter(GapNumber,1) = shortCount;
            longGapCounter(GapNumber,1) = longCount+1;
            longCount = longCount+1;
        elseif (GapDuration(GapNumber) + vehicleData(indEnd,5)/abs(vehicleData(indEnd,7))<5)
            longGapCounter(GapNumber,1) = longCount;
            shortGapCounter(GapNumber,1) = shortCount+1;
            shortCount = shortCount+1;
        else
            shortGapCounter(GapNumber,1) = shortCount;
            longGapCounter(GapNumber,1) = longCount+1;
            longCount = longCount+1;            
        end
                 
%         %cumulative wait time
%         cumWait = GapData(:,9);
         
        %1a) crossing direction,1 - rack to ball, 2 - ball to rack
        if mod(ii,2)==1
          crossDirection(GapNumber) = 1;
        else
          crossDirection(GapNumber) = 2;
        end
    
        %2) pedestrian speed, 1 - mean, 2 - std deviation, 3 - at start, 4 - at
        %end
        speed{1}(GapNumber) = nanmean(PedestrianData(indStart:indEnd,8));
        speed{2}(GapNumber) = nanstd(PedestrianData(indStart:indEnd,8));
        speed{3}(GapNumber) = PedestrianData(indStart,8);
        speed{4}(GapNumber) = PedestrianData(indEnd,8);         
        if (indEnd-indStart)>timeInGap
            speed{5}(GapNumber) = nanmean(PedestrianData(indStart:indStart+timeInGap-1,8));
            speed{6}(GapNumber) = nanmean(PedestrianData(indEnd-timeInGap+1:indEnd,8));
        end
        
        speed{5}(2645) = 0;
        speed{6}(2645) = 0;
    
        %3) Gaze ratio
        gaze{1}(GapNumber) = nanmean(GazeAtVehicleRatio(indStart:indEnd));
        gaze{2}(GapNumber) = nanstd(GazeAtVehicleRatio(indStart:indEnd));
        gaze{3}(GapNumber) = GazeAtVehicleRatio(indStart);
        gaze{4}(GapNumber) = GazeAtVehicleRatio(indEnd); 
        if (indEnd-indStart)>timeInGap
            gaze{5}(GapNumber) = nanmean(GazeAtVehicleRatio(indStart:indStart+timeInGap-1));
            gaze{6}(GapNumber) = nanmean(GazeAtVehicleRatio(indEnd-timeInGap+1:indEnd));
        end
        gaze{5}(2645) = 0;
        gaze{6}(2645) = 0;
        %3) Gaze angle
        gazeAng{1}(GapNumber) = nanmean(GazeAngle(indStart:indEnd));
        gazeAng{2}(GapNumber) = nanstd(GazeAngle(indStart:indEnd));
        gazeAng{3}(GapNumber) = GazeAngle(indStart);
        gazeAng{4}(GapNumber) = GazeAngle(indEnd); 
        if (indEnd-indStart)>timeInGap
            gazeAng{5}(GapNumber) = nanmean(GazeAngle(indStart:indStart+timeInGap-1));
            gazeAng{6}(GapNumber) = nanmean(GazeAngle(indEnd-timeInGap+1:indEnd));
        end
        gazeAng{5}(2645) = 0;
        gazeAng{6}(2645) = 0;         
        %3a) gaze for entire duration
        gazeDur(GapNumber) = double(nansum(GazeAtVehicle(indStart:indEnd))/(indEnd-indStart+1));   %not a rolling window; overal gaze ratio for that duration

        %3b) distance to curb
        dtc{1}(GapNumber) = nanmean(abs(PedestrianData(indStart:indEnd,5))-3.5);
        dtc{2}(GapNumber) = nanstd(abs(PedestrianData(indStart:indEnd,5))-3.5);
        dtc{3}(GapNumber) = abs(PedestrianData(indStart,5))-3.5;
        dtc{4}(GapNumber) = abs(PedestrianData(indEnd,5))-3.5;
        if (indEnd-indStart)>timeInGap
            dtc{5}(GapNumber) = nanmean(abs(PedestrianData(indStart:indStart+timeInGap-1,5))-3.5);
            dtc{6}(GapNumber) = nanmean(abs(PedestrianData(indEnd-timeInGap+1:indEnd,5))-3.5);
        end
        dtc{5}(2645) = 0;
        dtc{6}(2645) = 0; 
        %3c) distance to crosswalk
        rDist = sqrt(PedestrianData(:,4).^2+(abs(PedestrianData(:,5))-3.5).^2);
        dtCW{1}(GapNumber) = nanmean(rDist(indStart:indEnd));
        dtCW{2}(GapNumber) = nanstd(rDist(indStart:indEnd));
        dtCW{3}(GapNumber) = rDist(indStart);
        dtCW{4}(GapNumber) = rDist(indEnd); 
        if (indEnd-indStart)>timeInGap
            dtCW{5}(GapNumber) = nanmean(rDist(indStart:indStart+timeInGap-1));
            dtCW{6}(GapNumber) = nanmean(rDist(indEnd-timeInGap+1:indEnd));
        end
        dtCW{5}(2645) = 0;
        dtCW{6}(2645) = 0; 
        
        %4) vehicle speed same lane
        VehSpeed{1}(GapNumber) = nanmean(abs(vehicleData(indStart:indEnd,7)));
        VehSpeed{2}(GapNumber) = nanstd(abs(vehicleData(indStart:indEnd,7)));
        VehSpeed{3}(GapNumber) = abs(vehicleData(indStart,7));
        VehSpeed{4}(GapNumber) = abs(vehicleData(indEnd,7)); 
        if (indEnd-indStart)>timeInGap
            VehSpeed{5}(GapNumber) = nanmean(abs(vehicleData(indStart:indStart+timeInGap-1,7)));
            VehSpeed{6}(GapNumber) = nanmean(abs(vehicleData(indEnd-timeInGap+1:indEnd,7)));
        end
        VehSpeed{5}(2645) = 0;
        VehSpeed{6}(2645) = 0; 
        
        %5) vehicle acceleration same lane
        VehAcc{1}(GapNumber) = nanmean(-vehicleData(indStart:indEnd,8));
        VehAcc{2}(GapNumber) = nanstd(-vehicleData(indStart:indEnd,8));
        VehAcc{3}(GapNumber) = -vehicleData(indStart,8);
        VehAcc{4}(GapNumber) = -vehicleData(indEnd,8); 
        if (indEnd-indStart)>timeInGap
            VehAcc{5}(GapNumber) = nanmean(-vehicleData(indStart:indStart+timeInGap-1,8));
            VehAcc{6}(GapNumber) = nanmean(-vehicleData(indEnd-timeInGap+1:indEnd,8));
        end
        VehAcc{5}(2645) = 0;
        VehAcc{6}(2645) = 0; 
        
        %6) vehicle gap same lane
        VehGap{1}(GapNumber) = nanmean(vehicleData(indStart:indEnd,9)-vehicleData(indStart:indEnd,6));
        VehGap{2}(GapNumber) = nanstd(vehicleData(indStart:indEnd,9)-vehicleData(indStart:indEnd,6));
        VehGap{3}(GapNumber) = vehicleData(indStart,9)-vehicleData(indStart,6);
        VehGap{4}(GapNumber) = vehicleData(indEnd,9)-vehicleData(indEnd,6);
        if (indEnd-indStart)>timeInGap
            VehGap{5}(GapNumber) = nanmean(vehicleData(indStart:indStart+timeInGap-1,9)-vehicleData(indStart:indStart+timeInGap-1,6));
            VehGap{6}(GapNumber) = nanmean(vehicleData(indEnd-timeInGap+1:indEnd,9)-vehicleData(indEnd-timeInGap+1:indEnd,6));
        end
        VehGap{5}(2645) = 0;
        VehGap{6}(2645) = 0; 
        
        
        %6a) vehicle ttc (time to crosswalk) same lane
        VehTTC{1}(GapNumber) = nanmean(vehicleData(indStart:indEnd,6)./abs(vehicleData(indStart:indEnd,7)));
        VehTTC{2}(GapNumber) = nanstd(vehicleData(indStart:indEnd,6)./abs(vehicleData(indStart:indEnd,7)));
        VehTTC{3}(GapNumber) = vehicleData(indStart,6)./abs(vehicleData(indStart,7));
        VehTTC{4}(GapNumber) = vehicleData(indEnd,6)./abs(vehicleData(indEnd,7)); 
        if (indEnd-indStart)>timeInGap
            VehTTC{5}(GapNumber) = nanmean(vehicleData(indStart:indStart+timeInGap-1,6)./abs(vehicleData(indStart:indStart+timeInGap-1,7)));
            VehTTC{6}(GapNumber) = nanmean(vehicleData(indEnd-timeInGap+1:indEnd,6)./abs(vehicleData(indEnd-timeInGap+1:indEnd,7)));
        end
        VehTTC{5}(2645) = 0;
        VehTTC{6}(2645) = 0; 
        
        
        %6a) vehicle ttcol (time to collision) same lane
        VehTTCol{1}(GapNumber) = nanmean(vehicleData(indStart:indEnd,5)./abs(vehicleData(indStart:indEnd,7)));
        VehTTCol{2}(GapNumber) = nanstd(vehicleData(indStart:indEnd,5)./abs(vehicleData(indStart:indEnd,7)));
        VehTTCol{3}(GapNumber) = vehicleData(indStart,5)./abs(vehicleData(indStart,7));
        VehTTCol{4}(GapNumber) = vehicleData(indEnd,5)./abs(vehicleData(indEnd,7)); 
        if (indEnd-indStart)>timeInGap
            VehTTCol{5}(GapNumber) = nanmean(vehicleData(indStart:indStart+timeInGap-1,5)./abs(vehicleData(indStart:indStart+timeInGap-1,7)));
            VehTTCol{6}(GapNumber) = nanmean(vehicleData(indEnd-timeInGap+1:indEnd,5)./abs(vehicleData(indEnd-timeInGap+1:indEnd,7)));
        end
        VehTTCol{5}(2645) = 0;
        VehTTCol{6}(2645) = 0;         
       
        %7) vehicle distance to pedestrian
        VehDTP{1}(GapNumber) = nanmean(vehicleData(indStart:indEnd,5));
        VehDTP{2}(GapNumber) = nanstd(vehicleData(indStart:indEnd,5));
        VehDTP{3}(GapNumber) = vehicleData(indStart,5);
        VehDTP{4}(GapNumber) = vehicleData(indStart,5);
        if (indEnd-indStart)>timeInGap
            VehDTP{5}(GapNumber) = nanmean(vehicleData(indStart:indStart+timeInGap-1,5));
            VehDTP{6}(GapNumber) = nanmean(vehicleData(indEnd-timeInGap+1:indEnd,5));
        end
        VehDTP{5}(2645) = 0;
        VehDTP{6}(2645) = 0;    
        
        %8) vehicle distance to pedestrian, next lane
%         VehDTPL2{1}(GapNumber) = nanmean(vehicleData(indStart:indEnd,10)-vehicleData(indStart:indEnd,6)+vehicleData(indStart:indEnd,5));
%         VehDTPL2{2}(GapNumber) = nanstd(vehicleData(indStart:indEnd,10)-vehicleData(indStart:indEnd,6)+vehicleData(indStart:indEnd,5));
%         VehDTPL2{3}(GapNumber) = vehicleData(indStart,10)-vehicleData(indStart,6)+vehicleData(indStart,5);
%         VehDTPL2{4}(GapNumber) = vehicleData(indStart,10)-vehicleData(indEnd,6)+vehicleData(indEnd,5);
%         if (indEnd-indStart)>timeInGap
%             VehDTPL2{5}(GapNumber) = nanmean(vehicleData(indStart:indStart+timeInGap-1,10)-vehicleData(indStart:indStart+timeInGap-1,6)+vehicleData(indStart:indStart+timeInGap-1,5));
%             VehDTPL2{6}(GapNumber) = nanmean(vehicleData(indEnd-timeInGap+1:indEnd,10)-vehicleData(indEnd-timeInGap+1:indEnd,6)+vehicleData(indEnd-timeInGap+1:indEnd,5));
%         end
        VehDTPL2{1}(GapNumber) = nanmean(vehicleData(indStart:indEnd,10)-PedestrianData(indStart:indEnd,4));
        VehDTPL2{2}(GapNumber) = nanstd(vehicleData(indStart:indEnd,10)-PedestrianData(indStart:indEnd,4));
        VehDTPL2{3}(GapNumber) = vehicleData(indStart,10)-PedestrianData(indStart,4);
        VehDTPL2{4}(GapNumber) = vehicleData(indStart,10)-PedestrianData(indEnd,4);
        if (indEnd-indStart)>timeInGap
            VehDTPL2{5}(GapNumber) = nanmean(vehicleData(indStart:indStart+timeInGap-1,10)-PedestrianData(indStart:indStart+timeInGap-1,4));
            VehDTPL2{6}(GapNumber) = nanmean(vehicleData(indEnd-timeInGap+1:indEnd,10)-PedestrianData(indEnd-timeInGap+1:indEnd,4));
        end
        VehDTPL2{5}(2645) = 0;
        VehDTPL2{6}(2645) = 0;          
        
        %9) vehicle speed adjacent lane
        VehSpeedAdj{1}(GapNumber) = nanmean(abs(vehicleData(indStart:indEnd,11)));
        VehSpeedAdj{2}(GapNumber) = nanstd(abs(vehicleData(indStart:indEnd,11)));
        VehSpeedAdj{3}(GapNumber) = abs(vehicleData(indStart,11));
        VehSpeedAdj{4}(GapNumber) = abs(vehicleData(indEnd,11)); 
        if (indEnd-indStart)>timeInGap
            VehSpeedAdj{5}(GapNumber) = nanmean(abs(vehicleData(indStart:indStart+timeInGap-1,11)));
            VehSpeedAdj{6}(GapNumber) = nanmean(abs(vehicleData(indEnd-timeInGap+1:indEnd,11)));
        end
        VehSpeedAdj{5}(2645) = 0;
        VehSpeedAdj{6}(2645) = 0;
        
        
        %10) vehicle acceleration adjacent lane
        VehAccAdj{1}(GapNumber) = nanmean(-vehicleData(indStart:indEnd,12));
        VehAccAdj{2}(GapNumber) = nanstd(-vehicleData(indStart:indEnd,12));
        VehAccAdj{3}(GapNumber) = -vehicleData(indStart,12);
        VehAccAdj{4}(GapNumber) = -vehicleData(indEnd,12); 
        if (indEnd-indStart)>timeInGap
            VehAccAdj{5}(GapNumber) = nanmean(-vehicleData(indStart:indStart+timeInGap-1,12));
            VehAccAdj{6}(GapNumber) = nanmean(-vehicleData(indEnd-timeInGap+1:indEnd,12));
        end
        VehAccAdj{5}(2645) = 0;
        VehAccAdj{6}(2645) = 0;        
        
      
    end
    
    
    end
    x=1;
end

    
end

       
x=1;       

%11) Actual Gap Durations
ActualGap = (GapData(:,6)-GapData(:,5))/10;
TTCGap = GapDuration + double(VehTTCol{4})';

%
excelData = [GapData(:,1:12),GapDuration,ActualGap,TTCGap,shortGapCounter,longGapCounter,double(crossDirection)',gazeDur'];
excelData = [excelData,double(speed{1})',double(speed{2})',double(speed{3})',double(speed{4})',double(speed{5})',double(speed{6})'];
excelData = [excelData,double(gaze{1})',double(gaze{2})',double(gaze{3})',double(gaze{4})',double(gaze{5})',double(gaze{6})'];    
excelData = [excelData,double(gazeAng{1})',double(gazeAng{2})',double(gazeAng{3})',double(gazeAng{4})',double(gazeAng{5})',double(gazeAng{6})'];    
excelData = [excelData,double(dtc{1})',double(dtc{2})',double(dtc{3})',double(dtc{4})',double(dtc{5})',double(dtc{6})'];       
excelData = [excelData,double(dtCW{1})',double(dtCW{2})',double(dtCW{3})',double(dtCW{4})',double(dtCW{5})',double(dtCW{6})'];    
excelData = [excelData,double(VehDTP{1})',double(VehDTP{2})',double(VehDTP{3})',double(VehDTP{4})',double(VehDTP{5})',double(VehDTP{5})'];
excelData = [excelData,double(VehSpeed{1})',double(VehSpeed{2})',double(VehSpeed{3})',double(VehSpeed{4})',double(VehSpeed{5})',double(VehSpeed{6})']; 
excelData = [excelData,double(VehAcc{1})',double(VehAcc{2})',double(VehAcc{3})',double(VehAcc{4})',double(VehAcc{5})',double(VehAcc{6})'];
excelData = [excelData,double(VehDTPL2{1})',double(VehDTPL2{2})',double(VehDTPL2{3})',double(VehDTPL2{4})',double(VehDTPL2{5})',double(VehDTPL2{6})'];
excelData = [excelData,double(VehSpeed{1})',double(VehSpeed{2})',double(VehSpeed{3})',double(VehSpeed{4})',double(VehSpeed{5})',double(VehSpeed{6})']; 
excelData = [excelData,double(VehAcc{1})',double(VehAcc{2})',double(VehAcc{3})',double(VehAcc{4})',double(VehAcc{5})',double(VehAcc{6})'];
excelData = [excelData,double(VehGap{1})',double(VehGap{2})',double(VehGap{3})',double(VehGap{4})',double(VehGap{5})',double(VehGap{6})'];
excelData = [excelData,double(VehTTC{1})',double(VehTTC{2})',double(VehTTC{3})',double(VehTTC{4})',double(VehTTC{5})',double(VehTTC{6})'];
excelData = [excelData,double(VehTTCol{1})',double(VehTTCol{2})',double(VehTTCol{3})',double(VehTTCol{4})',double(VehTTCol{5})',double(VehTTCol{6})'];


excelHeader = {'SubjectID','ScenarioID','Crossing ID','Crossing Decision','Gap Start Index',...
               'Gap End Index','Gap Start (Wait Start)','Gap End (Cross Start)','On Road','Cumulative Wait time',...
               'Approach to Wait/Cross Gap Decision','Cross from wait Gap Decision',...
               'GapDuration','ActualGap','TTCGap','ShortGapCounter','LongGapCounter','Cross Direction','Gaze Ratio Entire Duration','Pedestrian Speed','','','','','','Moving Window Gaze Ratio','','','','','',...
               'Gaze Angle','','','','','','Pedestrian distance to curb','','','','','','Pedestrian distance to CW','','','','','',...
               'Same Lane Vehicle distance to pedestrian','','','','','','Same Lane Vehicle Speed','','','','','','Same Lane Vehicle Acceleration','','','','','',...
               'Adjacent Lane Vehicle distance to pedestrian','','','','','','Adjacent Lane Vehicle Speed','','','','','','Adjacent Lane Vehicle Acceleration','','','','','',...
               'Same Lane Vehicle Distance Gap','','','','','','Same Lane Vehicle time to CW','','','','','','Same Lane Vehicle time to collision','','','','',''};


xlswrite('GapWiseCompiledDataV5.xlsx',excelData,1,'A2');
xlswrite('GapWiseCompiledDataV5.xlsx',excelHeader,1,'A1');


% p = randperm(360,72);
% xtrain = [MeanGPInput(p,:)];
% xtest = MeanGPInput;
% xtest(p,:)=[];
% 
% ytrain = MeanGPOutput(p,10);
% ytest = MeanGPOutput(:,10);
% ytest(p)=[];