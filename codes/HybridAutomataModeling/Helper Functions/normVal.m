%% norm function

function output = normVal(input,mean,sigma)

 CrossFromWaitGapDecision = input(:,12);
 CrossFromWaitGapDecision(CrossFromWaitGapDecision==1)=0;
 CrossFromWaitGapDecision(CrossFromWaitGapDecision==2|CrossFromWaitGapDecision==3)=1;

 CumulativeWait = (input(:,10)-mean(3))/sigma(3);    
 GapDuration = (input(:,15)-mean(4))/sigma(4);  
 CrossDirection = input(:,18);
 GazeRatioEntireDuration = (input(:,19)-mean(6))/sigma(6);
 PedestrianSpeed = (input(:,20)-mean(7))/sigma(7);
 MovingWindowGazeRatio = (input(:,26)-mean(8))/sigma(8);
 PedestrianDistanceToCurb = (input(:,38)-mean(9))/sigma(9);
 PedestrianDistanceToCW = (input(:,44)-mean(10))/sigma(10);
 SameLaneVehicleDistanceToPedestrian = (input(:,50)-mean(11))/sigma(11);
 SameLaneVehicleSpeed = (input(:,56)-mean(12))/sigma(12);
 SameLaneVehicleAcceleration = (input(:,62)-mean(13))/sigma(13);
 AdjacentLaneVehicleDistanceToPedestrian = (input(:,68)-mean(14))/sigma(14);
 SameLaneVehicleDistanceGap = (input(:,86)-mean(15))/sigma(15);
 SameLaneVehicleTimeToCW = (input(:,92)-mean(16))/sigma(16);
 SameLaneVehicleTimeToCollision = (input(:,98)-mean(17))/sigma(17);
 SubjectID = input(:,1);
 
 output= table(SubjectID,CrossFromWaitGapDecision,CumulativeWait,GapDuration,...
                                    CrossDirection,GazeRatioEntireDuration,PedestrianSpeed,MovingWindowGazeRatio,...
                                    PedestrianDistanceToCurb,PedestrianDistanceToCW,SameLaneVehicleDistanceToPedestrian,...
                                    SameLaneVehicleSpeed,SameLaneVehicleAcceleration,AdjacentLaneVehicleDistanceToPedestrian,...
                                    SameLaneVehicleDistanceGap,SameLaneVehicleTimeToCW,SameLaneVehicleTimeToCollision);                              

                               

end




