%% this function gives the reset/goal locations of the various discrete states for the inD dataset


function [reset] = goalsinD()

% these were manually calculated from the figure; pixel values in global
% x-y coordinates

%rotation matrices for the different crosswalks
cw1_rot = 14;
cw2_rot = 10;
cw3_rot = 23;
cw4_rot = 50;

% order of lanes/crosswalks
% 1-West Right, 2- West Left, 3 - East Left, 4-East right
% 5-South Right, 6- South Left, 7- North Right, 8-North Left


% reset states
reset.wait.velocity = zeros(8,2);  % zero x- and y-velocity for all crosswalks and both lanes
reset.wait.heading = [cw1_rot - 90;
                      cw1_rot + 90;
                      cw2_rot + 90;
                      cw2_rot - 90;
                      cw3_rot - 180;
                      cw3_rot;
                      cw4_rot;
                      cw4_rot - 180];

reset.approach.goal = [620, -255;
                       640, -330;
                       410, -365;
                       380, -290;
                       525, -380;
                       450, -405;
                       400, -185;
                       455, -135];
                   
reset.walkaway.goal = [620, -185;
                       950, -270;
                       130, -455;
                       100, -345;
                       610, -560;
                       520, -580;
                       320, -150;
                       440, -110];                   

reset.walkaway.er_cw1_final = [920, -120];                  


reset.carCW.goal = [625, -275;
                    630, -295;
                    400, -340;
                    385, -315;
                    505, -385;
                    480, -395;
                    425, -175;
                    445, -165];

                
reset.carLane.goal  = [980, -205;
                       995, -235;
                       110, -420;
                       100, -390;
                       575, -565;
                       545, -575;
                       375, -130;
                       415, -120];




end