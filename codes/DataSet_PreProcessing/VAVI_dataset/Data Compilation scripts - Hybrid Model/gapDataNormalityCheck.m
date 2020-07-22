%% check normality of the Gap wise
clear all

GapData  = xlsread('GapWiseCompiledDataV5ManuallyAdjusted.xlsx',1);

 CumulativeWait = GapData(:,10);    
 GapDuration = GapData(:,15);  
 CrossDirection = GapData(:,18);
 GazeRatioEntireDuration = GapData(:,19);
 PedestrianSpeed = GapData(:,20);
 MovingWindowGazeRatio = GapData(:,26);
 PedestrianDistanceToCurb = GapData(:,38);
 PedestrianDistanceToCW = GapData(:,44);
 SameLaneVehicleDistanceToPedestrian = GapData(:,50);
 SameLaneVehicleSpeed = GapData(:,56);
 SameLaneVehicleAcceleration = GapData(:,62);
 AdjacentLaneVehicleDistanceToPedestrian = GapData(:,68);
 AdjacentLaneVehicleSpeed = GapData(:,74);
 AdjacentLaneVehicleAcceleration = GapData(:,80);
 SameLaneVehicleDistanceGap = GapData(:,86);
 SameLaneVehicleTimeToCW = GapData(:,92);
 SameLaneVehicleTimeToCollision = GapData(:,98);
 
 h_CumulativeWait = adtest(CumulativeWait)