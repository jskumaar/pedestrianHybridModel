
function cw_ind = updateCloseCW(pixel_pos, cw, cw_cross_threshold, cw_dist_threshold, heading_threshold)


% angle between pedestrian and the crosswalks
ped_cw1_angle = atan2(double([cw.center_y(1) - pixel_pos(2)]), double([cw.center_x(1) - pixel_pos(1)]))*180/pi;
ped_cw2_angle = atan2(double([cw.center_y(2) - pixel_pos(2)]), double([cw.center_x(2) - pixel_pos(1)]))*180/pi;  
ped_cw3_angle = atan2(double([cw.center_y(3) - pixel_pos(2)]), double([cw.center_x(3) - pixel_pos(1)]))*180/pi;  
ped_cw4_angle = atan2(double([cw.center_y(4) - pixel_pos(2)]), double([cw.center_x(4) - pixel_pos(1)]))*180/pi;  

% distance between pedestrian and crosswalk (in pixels)
dist_cw1 = sqrt(double(cw.center_x(1) - pixel_pos(1))^2 + double(cw.center_y(1) - pixel_pos(2))^2);
dist_cw2 = sqrt(double(cw.center_x(2) - pixel_pos(1))^2 + double(cw.center_y(2) - pixel_pos(2))^2); 
dist_cw3 = sqrt(double(cw.center_x(3) - pixel_pos(1))^2 + double(cw.center_y(3) - pixel_pos(2))^2);
dist_cw4 = sqrt(double(cw.center_x(4) - pixel_pos(1))^2 + double(cw.center_y(4) - pixel_pos(2))^2);

dist_cw_temp = [inf, inf, inf, inf]; 

% when the heading angle is within +/- heading_threshold and pedestrian is close to a crosswalk, 
% then pedestrian is approaching one of the crosswalks 
if  ( ( abs(ped_cw1_angle - ped_head) <= heading_threshold && dist_cw1 < cw_dist_threshold ) || dist_cw1 < cw_cross_threshold )
    dist_cw_temp(1) = dist_cw1;
end
if ( ( abs(ped_cw2_angle - ped_head) <= heading_threshold && dist_cw2 < cw_dist_threshold ) || dist_cw2 < cw_cross_threshold )
    dist_cw_temp(2) = dist_cw2;
end
if ( ( abs(ped_cw3_angle - ped_head) <= heading_threshold && dist_cw3 < cw_dist_threshold ) || dist_cw3 < cw_cross_threshold )
    dist_cw_temp(3) = dist_cw3;   
end
if ( ( abs(ped_cw4_angle - ped_head) <= heading_threshold && dist_cw4 < cw_dist_threshold ) || dist_cw4 < cw_cross_threshold )
    dist_cw_temp(4) = dist_cw4;
end

% distance to the closest crosswalk when headed towards that
% crosswalk
[dist_cw, cw_ind] = min(dist_cw_temp);
% when the closest distance to a headed cw is 'inf', then change the
% cw index to zero
if dist_cw==inf
   cw_ind = 0;
end