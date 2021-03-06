%% This script is used to initialize the parameters for the inD dataset

%% inD dataset variables

% load compiled and formatted dataset
% load('tracksData_reSampled_v11.mat')
% load('inD_trackDescriptives_v3.mat') 

% a) dataset
N_Scenes = 12;
Params.raw_T = 0.04;   % sampling rate is 25 Hz, 0.04 seconds for each data point
Params.SampFreq = (1/Params.raw_T);
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
% % 
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
%  
% % center of the road
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

Params.imgSize = imgSize;
Params.roadOrientation = [10, -166, -67, 140];
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
resetStates.approach.goal =   [620, -255;
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
                   
resetStates.walkaway.goal =   [620, -170;
                               950, -270;
                               130, -455;
                               100, -345;
                               610, -560;
                               520, -580;
                               320, -150;
                               440, -110];                   

resetStates.walkaway.erCw1Final = [920, -120];                  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% old values
%bounding box (calcualed using temp_test_rotation.m)
% bb_approach  = [617.567740324617,-265.912566740758,627.270697587377,-263.493347784762,615.148521368620,-256.209609477998,624.851478631380,-253.790390522002;
%                 637.567740324617,-340.912566740758,647.270697587377,-338.493347784762,635.148521368620,-331.209609477998,644.851478631380,-328.790390522002;
%                 406.812443011608,-375.716318418457,416.660520541730,-373.979836641788,405.075961234939,-365.868240888335,414.924038765061,-364.131759111665;
%                 376.812443011608,-300.716318418457,386.660520541730,-298.979836641787,375.075961234939,-290.868240888335,384.924038765061,-289.131759111665;
%                 517.748607107922,-388.509835552155,513.841295823029,-379.304787017631,526.953655642446,-384.602524267262,523.046344357554,-375.397475732738;
%                 442.748607107922,-413.509835552155,438.841295823029,-404.304787017631,451.953655642446,-409.602524267262,448.046344357554,-400.397475732738;
%                 397.402346118730,-195.874382479622,389.741901687540,-189.446506382757,403.830222215595,-188.213938048433,396.169777784405,-181.786061951567;
%                 452.402346118730,-145.874382479623,444.741901687540,-139.446506382757,458.830222215595,-138.213938048433,451.169777784405,-131.786061951567];

% bb_walkaway = [617.567740324617,-195.912566740758,627.270697587377,-193.493347784762,615.148521368620,-186.209609477998,624.851478631380,-183.790390522002;
%                947.567740324617,-280.912566740758,957.270697587377,-278.493347784762,945.148521368620,-271.209609477998,954.851478631380,-268.790390522002;
%                126.812443011608,-465.716318418457,136.660520541730,-463.979836641787,125.075961234939,-455.868240888335,134.924038765061,-454.131759111665;
%                96.8124430116082,-355.716318418457,106.660520541730,-353.979836641788,95.0759612349389,-345.868240888335,104.924038765061,-344.131759111665;
%                602.748607107922,-568.509835552155,598.841295823029,-559.304787017631,611.953655642446,-564.602524267262,608.046344357554,-555.397475732738;
%                512.748607107922,-588.509835552155,508.841295823029,-579.304787017631,521.953655642446,-584.602524267262,518.046344357554,-575.397475732738;
%                317.402346118730,-160.874382479622,309.741901687540,-154.446506382757,323.830222215595,-153.213938048433,316.169777784405,-146.786061951567;
%                437.402346118730,-120.874382479623,429.741901687540,-114.446506382757,443.830222215595,-113.213938048433,436.169777784405,-106.786061951567];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% new values

bb_approach =  [624.851478631380,-253.790390522002,615.148521368620,-256.209609477998,622.432259675383,-244.087433259242,612.729302412623,-246.506652215238;
                644.851478631380,-328.790390522002,635.148521368620,-331.209609477998,647.270697587377,-338.493347784762,637.567740324617,-340.912566740758;
                414.924038765061,-364.131759111665,405.075961234939,-365.868240888335,416.660520541730,-373.979836641788,406.812443011608,-375.716318418457;
                384.924038765061,-289.131759111665,375.075961234939,-290.868240888335,383.187556988392,-279.283681581543,373.339479458270,-281.020163358213;
                523.046344357554,-375.397475732738,526.953655642446,-384.602524267262,532.251392892078,-371.490164447845,536.158704176971,-380.695212982369;
                448.046344357554,-400.397475732738,451.953655642446,-409.602524267262,438.841295823029,-404.304787017631,442.748607107922,-413.509835552155;
                396.169777784405,-181.786061951567,403.830222215595,-188.213938048433,389.741901687540,-189.446506382757,397.402346118730,-195.874382479622;
                451.169777784405,-131.786061951567,458.830222215595,-138.213938048433,457.597653881271,-124.125617520378,465.258098312460,-130.553493617243];
            
bb_walkaway =  [624.851478631380,-170.790390522002,615.148521368620,-170.209609477998,622.432259675383,-160.087433259242,612.729302412623,-160.506652215238;
                954.851478631380,-268.790390522002,945.148521368620,-271.209609477998,957.270697587377,-278.493347784762,947.567740324617,-280.912566740758;
                134.924038765061,-454.131759111665,125.075961234939,-455.868240888335,136.660520541730,-463.979836641787,126.812443011608,-465.716318418457;
                104.924038765061,-344.131759111665,95.0759612349389,-345.868240888335,103.187556988392,-334.283681581543,93.3394794582696,-336.020163358213;
                608.046344357554,-555.397475732738,611.953655642446,-564.602524267262,617.251392892078,-551.490164447845,621.158704176971,-560.695212982369;
                518.046344357554,-575.397475732738,521.953655642446,-584.602524267262,508.841295823029,-579.304787017631,512.748607107922,-588.509835552155;
                316.169777784405,-146.786061951567,323.830222215595,-153.213938048433,309.741901687540,-154.446506382757,317.402346118730,-160.874382479622;
                436.169777784405,-106.786061951567,443.830222215595,-113.213938048433,442.597653881270,-99.1256175203776,450.258098312460,-105.553493617243];


bb_walkaway_erCw1Final = [917.567740324617,-130.912566740758,927.270697587377,-128.493347784762,915.148521368620,-121.209609477998,924.851478631380,-118.790390522002];
            

resetStates.approach.goal = [resetStates.approach.goal, bb_approach];
resetStates.walkaway.goal = [resetStates.walkaway.goal, bb_walkaway];
resetStates.walkaway.erCw1Final = [resetStates.walkaway.erCw1Final, bb_walkaway_erCw1Final];
resetStates.wait.goal = resetStates.approach.goal([2,1,4,3,6,5,8,7],:);
resetStates.approachReset.goal = resetStates.approach.goal([8,5,6,7,2,3,4,1],:);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Needed for pedestrian lane calculation %%%%%%%%%%%%%%%%

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
                   
%%%%%%%%%%%%%%%%%%%%
% sidewalk region not originally annotated
Params.sidewalk.xmin = [650, 730]; 
Params.sidewalk.xmax = [1000, 1000];
Params.sidewalk.ymin = [-150, -270];
Params.sidewalk.ymax = [-250, -320];


