function output = createTable(input)

%% Caution: Before using this function make sure the variables and the input column numbers match!!



 CrossFromWaitGapDecision = input(:,12);
 CrossFromWaitGapDecision(CrossFromWaitGapDecision==1)=0;
 CrossFromWaitGapDecision(CrossFromWaitGapDecision==2|CrossFromWaitGapDecision==3)=1;
 
 CumulativeWait = input(:,10);    
 GapDuration = input(:,15);  
 CrossDirection = input(:,18);
 GazeRatioEntireDuration = input(:,19);
 PedestrianSpeed = input(:,20);
 MovingWindowGazeRatio = input(:,26);
 PedestrianDistanceToCurb = input(:,38);
 PedestrianDistanceToCW = input(:,44);
 SameLaneVehicleDistanceToPedestrian = input(:,50);
 SameLaneVehicleSpeed = input(:,56);
 SameLaneVehicleAcceleration = input(:,62);
 AdjacentLaneVehicleDistanceToPedestrian = input(:,68);
 AdjacentLaneVehicleSpeed = input(:,74);
 AdjacentLaneVehicleAcceleration = input(:,80);
 SameLaneVehicleDistanceGap = input(:,86);
 SameLaneVehicleTimeToCW = input(:,92);
 SameLaneVehicleTimeToCollision = input(:,98);
 SubjectID = input(:,1);

 output = table(SubjectID,CrossFromWaitGapDecision,CumulativeWait,GapDuration,...
                                    CrossDirection,GazeRatioEntireDuration,PedestrianSpeed,MovingWindowGazeRatio,...
                                    PedestrianDistanceToCurb,PedestrianDistanceToCW,SameLaneVehicleDistanceToPedestrian,...
                                    SameLaneVehicleSpeed,SameLaneVehicleAcceleration,AdjacentLaneVehicleDistanceToPedestrian,...
                                    SameLaneVehicleDistanceGap,SameLaneVehicleTimeToCW,SameLaneVehicleTimeToCollision);
 

end