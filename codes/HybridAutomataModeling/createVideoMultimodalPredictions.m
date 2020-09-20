%% run predictions on a picture


% setup
p1 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\codes');
p2 = genpath('G:\My Drive\Research\Projects\pedestrianHybridModel\datasets');
addpath(p1)
addpath(p2)


% load background image
backgroundImage = imread(strcat(num2str(18),'_background.png'));
imshow(backgroundImage)
axis on;

% discretize space into 0.2 X 0.2 grids (2 X 2 pixels)
Igrid = imageGrid(backgroundImage, 'vertLines', 585, 'horzLines', 390, 'lineWidth', 1, 'lineStyle', ':', 'method', 'burn');
figure
imshow(Igrid)


% load predictions





% do the predicted probability calculation






