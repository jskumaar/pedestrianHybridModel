%% This portion of the script plots the predicted trajectories on top of the bird's eye view of the intersection.
% Note: Includes predictions from MHP, HBase and CV models and ground truth
% trajectories.
% Note: Trajectory legend: White for ground truth, green for MHP trajectories, red for constant
% velocity and blue for HBase
% Note: 

% save predicted probability on the grid to new variables
temp_prob2D_MHP(:,:,1) = prob2D_MHP{sceneId}{car_index}{pedTrackId}{predictionTimeStep};
temp_prob2D_MHP_map = temp_prob2D_MHP;
temp_prob2D_MHP_map(temp_prob2D_MHP_map>1) = 1;

temp_prob2D_HBase(:,:,1) = prob2D_HBase{sceneId}{car_index}{pedTrackId}{predictionTimeStep};
temp_prob2D_HBase_map = temp_prob2D_HBase;
temp_prob2D_HBase_map(temp_prob2D_HBase_map>1) = 1;

temp_prob2D_CV(:,:,1) = prob2D_CV{sceneId}{car_index}{pedTrackId}{predictionTimeStep};
temp_prob2D_CV_map = temp_prob2D_CV;
temp_prob2D_CV_map(temp_prob2D_CV_map>1) = 1;

% ground truth probabilites on the grid
GT_image = zeros([img_size(1), img_size(2)]);
for ii=1:length(GTTrajectory)                                      
    ind = int32([-GTTrajectory(ii,2), GTTrajectory(ii,1)]/(orthopxToMeter*scaleFactor));
    GT_image(ind(1),ind(2)) = 255 ;
end
% denote the start point of the
% trajectory with a circle
ind = int32([-GTTrajectory(1,2), GTTrajectory(1,1)]/(orthopxToMeter*scaleFactor));
GT_image = insertShape(GT_image, 'FilledCircle', [ind(2), ind(1), 2],'Color','yellow','Opacity',1);
GT_image_gray = rgb2gray(GT_image);
% all trajectories combined
img_combined = temp_prob2D_MHP + temp_prob2D_HBase + temp_prob2D_CV + GT_image_gray;
% identify the region of interest
ind_first = find(img_combined~=0,1,'first');
ind_last = find(img_combined~=0,1,'last');
crop_x_min = floor(ind_first/img_size(1)); 
crop_x_max = floor(ind_last/img_size(1));
crop_y_min = mod(ind_first,img_size(1)); 
crop_y_max = mod(ind_last,img_size(1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Overlay predicted tarjectories on top of the background

% Note: Color selection. White for ground truth, green for MHP
% trajectories, red for constant velocity and blue for HBase
% Note: Background has some transparency
imshow(intersection_img)
alpha(0.5)
hold on
green = cat(3, zeros(size(temp_prob2D_MHP)), ones(size(temp_prob2D_MHP)), zeros(size(temp_prob2D_MHP)));
prob_pred_image = imshow(green);
hold on
set(prob_pred_image, 'AlphaData', temp_prob2D_MHP_map)
blue = cat(3, zeros(size(temp_prob2D_MHP)), zeros(size(temp_prob2D_MHP)), ones(size(temp_prob2D_MHP)));
prob_pred_image_2 = imshow(blue);
hold on
set(prob_pred_image_2, 'AlphaData', temp_prob2D_HBase_map)
red = cat(3, ones(size(temp_prob2D_MHP)),zeros(size(temp_prob2D_MHP)), zeros(size(temp_prob2D_MHP)));
prob_pred_image_3 = imshow(red);
hold on;
set(prob_pred_image_3, 'AlphaData', temp_prob2D_CV_map)
%                                     white = cat(3, ones(size(temp_prob2D_MHP)),ones(size(temp_prob2D_MHP)), ones(size(temp_prob2D_MHP)));
yellow = cat(3, ones(size(temp_prob2D_MHP)),ones(size(temp_prob2D_MHP)), zeros(size(temp_prob2D_MHP)));
prob_pred_image_4 = imshow(yellow);
set(prob_pred_image_4, 'AlphaData', 0.6*GT_image_gray)
%%%%%%%%%%
% crop and zoom image to include the relevant trajectories; region of
% interest is 150 X 150 pixels
x_ind = [max(1,crop_x_min-25)]; % start of x-axis
y_ind = [max(1,crop_y_min-25)]; % start of y-axis
h = gcf;
set(h.Children, 'Xlim',[x_ind, x_ind+150]);
set(h.Children, 'Ylim',[y_ind, y_ind+150]);
% save cropped image with background and trajectories
saveas(gcf,strcat('Trajectory_', num2str(sceneId),'_',num2str(car_index),'_',num2str(pedTrackId),'_',num2str(predictionTimeStep),'.png'));


% %%%%%%%%%%%%%%%%%%%
% % overlay images w/o transparency
% prob2D_MHP_map = cat(3, zeros(size(temp_prob2D_MHP)), (temp_prob2D_MHP), zeros(size(temp_prob2D_MHP)));
% green = cat(3, zeros(size(temp_prob2D_MHP)), ones(size(temp_prob2D_MHP)), zeros(size(temp_prob2D_MHP)));
% figure1 = figure;
% ax1 = axes('Parent',figure1);
% ax2 = axes('Parent',figure1);
% 
% prob_pred_image = imshow(prob2D_MHP_map);
% [a,map,alpha] = imread(prob_pred_image);
% I = imshow(a,'Parent',ax2);
% set(I,'AlphaData',alpha);
% imshow('18_background.png','Parent',ax1);
