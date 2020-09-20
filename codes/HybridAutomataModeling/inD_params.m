%% This script is used to initialize the parameters for the inD dataset

%% inD dataset variables

% a) dataset
N_Scenes = 12;
Params.raw_T = 0.04;   % sampling rate is 25 Hz, 0.04 seconds for each data point
Params.SampFreq = int32(1/Params.raw_T);
recordingMetaData = readtable(strcat(num2str(18),'_recordingMeta.csv'));
Params.orthopxToMeter = recordingMetaData.orthoPxToMeter;
Params.scaleFactor = 12;   % this was calculated manually; used in pixels to m conversion

Params.AdjustedSampFreq = 5;
Params.delta_T = 0.2;
Params.reSampleRate = Params.SampFreq/Params.AdjustedSampFreq;

% b) intersection and crosswalk params
% intersection environment limits; manually calculated from the maximum and
% minimum xCenter yCenter values for a few tracks (18, 19)
Params.xMin = 100;  %in pixels
Params.xMax = 950;  %in pixels
Params.yMin = -100; %in pixels (absolute value minimum)
Params.yMax = -560;  %in pixels

% load background images
annotatedImage = imread(strcat('annotated_',num2str(18),'_background.png'));        %using the same background image for all scenes as it roughly remains the same; also segmenting all images takes time.
imgSize = size(annotatedImage);

% for better visualization, increase the grayscale value of the different regions
annotatedImageEnhanced = annotatedImage;
for ii = 1:imgSize(1)
    for jj = 1:imgSize(2)
        if (annotatedImage(ii,jj)==2) %Road
            annotatedImageEnhanced(ii,jj) = 50;
        end
        if (annotatedImage(ii,jj)==4 || annotatedImage(ii,jj)==5 || annotatedImage(ii,jj)==6) %unmarked crosswalk
            annotatedImageEnhanced(ii,jj) = 100;
        end        
        if (annotatedImage(ii,jj)==3) %Sidewalk
            annotatedImageEnhanced(ii,jj) = 150;
        end        
        if (annotatedImage(ii,jj)==1) %marked crosswalk
            annotatedImageEnhanced(ii,jj) = 200;
        end
    end
end
 
% % center of the road
% imgSize = size(annotatedImageEnhanced);
% roadPixels = [];
% for ii = 1:imgSize(1)
%     for jj = 1:imgSize(2)
%         if annotatedImageEnhanced(ii,jj)==50
%             roadPixels = [roadPixels; [jj, -ii]];
%         end
%     end
% end
% roadCenter = (mean(roadPixels));
% annotatedImageEnhanced(-int32(roadCenter(2)), int32(roadCenter(1))) = 255;
% annotatedImageEnhanced_w_Tracks = annotatedImageEnhanced;
Params.imgSize = imgSize;
%identify the centers of the crosswalks; use the knowledge of the
%scene; we know that there are two crosswalks and they are on the
%opposite ends of the intersection; use x = 500 as the mid-point for x to
%differentiate the two crosswalks       
[cw.y1,cw.x1] = find(double(annotatedImage==1));
[cw.y2,cw.x2] = find(double(annotatedImage==4));
[cw.y3,cw.x3] = find(double(annotatedImage==5));
[cw.y4,cw.x4] = find(double(annotatedImage==6));

cw.center_y = -([mean(cw.y1), mean(cw.y2), mean(cw.y3), mean(cw.y4)]);
cw.center_x = ([mean(cw.x1), mean(cw.x2), mean(cw.x3), mean(cw.x4)]);
cw.centerLatOffset = ([27, 24, 30, 27]);     %this was calculated manually from the images

% crosswalk angles
cw.theta = [-14, -10, -23, -50];  % this was calculated manually
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% goals and resetStatess for discrete states

% all these were manually calculated from the figure; pixel values in global
% x-y coordinates


% order of lanes/crosswalks
% 1-West Right, 2- West Left, 3 - East Left, 4-East right
% 5-South Right, 6- South Left, 7- North Right, 8-North Left

% resetStates continuous states for the different discrete states
% a) Approach
resetStates.approach.goal = [620, -255;
                       640, -330;
                       410, -365;
                       380, -290;
                       525, -380;
                       450, -405;
                       400, -185;
                       455, -135];
                                
% wait
resetStates.wait.velocity = zeros(8,2);  % zero x- and y-velocity for all crosswalks and both lanes
resetStates.wait.heading = [cw.theta(1) - 90;
                      cw.theta(1) + 90;
                      cw.theta(2) + 90;
                      cw.theta(2) - 90;
                      cw.theta(3) - 180;
                      cw.theta(3);
                      cw.theta(4);
                      cw.theta(4) - 180];
                   
resetStates.walkaway.goal =   [620, -185;
                               950, -270;
                               130, -455;
                               100, -345;
                               610, -560;
                               520, -580;
                               320, -150;
                               440, -110];                   

resetStates.walkaway.erCw1Final = [920, -120];                  

resetStates.carCW.goal = [625, -275;
                    630, -295;
                    400, -340;
                    385, -315;
                    505, -385;
                    480, -395;
                    425, -175;
                    445, -165];

                
resetStates.carLane.goal  = [980, -205;
                       995, -235;
                       110, -420;
                       100, -390;
                       575, -565;
                       545, -575;
                       375, -130;
                       415, -120];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

              



