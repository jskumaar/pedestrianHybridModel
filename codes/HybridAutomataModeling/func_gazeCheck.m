%% This function checks if the pedestrian is looking at the vehicle or not


function isLooking = func_gazeCheck(carPosPixels, pedPosPixels, car_heading, ped_heading)


% convert heading from cell to double
%  if  iscell(car_heading_cell)
%     car_heading = double(cell2mat(car_heading_cell));
%  else
%     car_heading = car_heading_cell;
%  end
% 
%  if  iscell(ped_heading_cell)
%      ped_heading = double(cell2mat(ped_heading_cell));
%  else
%      ped_heading = ped_heading_cell;
%  end


% fixed params
scale_down_factor = 12;
orthopxToMeter = 0.0081;

% adjustable params
gazeRadii = 30; % in m
gazeRadiiPixels = gazeRadii/(scale_down_factor*orthopxToMeter);
gazeView = 45;

% vehicle bounding box for gaze evaluation; add a rectangle surrounding the
% center of the car; assumed length of car is 5m and width of car is 2m;
% add 2m along the length and 1 m along the width on each side of the
% vehicle for the gaze.

carLength = 5;
carWidth = 2;
carLengthBox = 1.5;
carWidthBox = 0.75;

carOverallLength_pixels = (carLength/2 + carLengthBox)/(scale_down_factor*orthopxToMeter);
carOverallWidth_pixels = (carWidth/2 + carWidthBox)/(scale_down_factor*orthopxToMeter);

rot_car = [cosd(car_heading), -sind(car_heading); sind(car_heading), cosd(car_heading)];

carEdgePoints = carPosPixels + (rot_car*[-carOverallLength_pixels, carOverallWidth_pixels;
                                         carOverallLength_pixels, carOverallWidth_pixels;
                                         carOverallLength_pixels, -carOverallWidth_pixels;
                                        -carOverallLength_pixels, -carOverallWidth_pixels]')';

theta_1 = ped_heading + gazeView/2;
rot1 = [cosd(theta_1), -sind(theta_1); sind(theta_1), cosd(theta_1)];
theta_2 = ped_heading - gazeView/2;
rot2 = [cosd(theta_2), -sind(theta_2); sind(theta_2), cosd(theta_2)];
pedVisionPoints = pedPosPixels +  [[0, 0];
                                    (rot1*[gazeRadiiPixels, 0]')';
                                    (rot2*[gazeRadiiPixels, 0]')'];
                   
                                
% % create polygon objects for the car and gaze region and check intersection
% carBox = polyshape([carEdgePoints(:,1), carEdgePoints(:,2)]);                        
% pedVisionBox = polyshape(pedVisionPoints(:,1), pedVisionPoints(:,2));                                
% intersectingRegion = intersect(pedVisionBox, carBox);          

% check intersection of points
[in,on] = inpolygon(carEdgePoints(:,1),carEdgePoints(:,2),pedVisionPoints(:,1),pedVisionPoints(:,2));
if ~isempty(in) || ~isempty(on)
    isLooking = true;
else
    isLooking = false;   
end


end