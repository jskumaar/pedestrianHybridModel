% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% pedVisionPoints = [0, 0;
%                    300, 0;
%                    260, 200];
%                
% carBoxPoints = [278, 100;
%                 330, 100;
%                 330, 150;
%                 278, 150];
%             
%             
% ms = 12;
%             
% % figure()
% % 
% % for ii=1:length(pedVisionPoints)
% %    plot(pedVisionPoints(ii,1), pedVisionPoints(ii,2), 'b*', 'MarkerSize', ms);hold on;
% % end
% % 
% % for ii=1:length(carBoxPoints)
% %    plot(carBoxPoints(ii,1), carBoxPoints(ii,2), 'r*', 'MarkerSize', ms);hold on;
% % end
% 
% %% a)
% tic
% carBox = polyshape(carBoxPoints(:,1), carBoxPoints(:,2));
% pedBox = polyshape(pedVisionPoints(:,1), pedVisionPoints(:,2));
% intersectRegion = intersect(carBox, pedBox);
% if ~isempty(intersectRegion.Vertices)
%     isLooking = true;
% else
%     isLooking = false;   
% end
% disp('polyshape')
% isLooking
% toc
% 
% %%b) 
% tic
% [in,on] = inpolygon(carBoxPoints(:,1),carBoxPoints(:,2),pedVisionPoints(:,1),pedVisionPoints(:,2));
% if ~isempty(in) || ~isempty(on)
%     isLooking = true;
% else
%     isLooking = false;   
% end
% disp('inpoly')
% isLooking
% toc
%     
% 
% % c)
% tic
% pedBox = polyshape(pedVisionPoints(:,1), pedVisionPoints(:,2));
% [TFin,TFon] = isinterior(pedBox, carBoxPoints);
% if ~isempty(TFin) || ~isempty(TFon)
%     isLooking = true;
% else
%     isLooking = false;   
% end
% disp('isinterior')
% isLooking
% toc
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% lane = "east_right";
% % a)
% tic
% if lane=="east_right"
%     x=1
% else
%     x=2
% end
% disp('==')
% toc
% 
% % b)
% tic
% if strcmp(lane,"east_right")
%     x=1
% else
%     x=2
% end
% disp('strcmp')
% toc
% 
% 
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% 
% % a) is better
% tic
% for ii=1:1000
%     scaleFactor = Params.scaleFactor;
%     orthopxToMeter = Params.orthopxToMeter;
%     del_t = Params.delta_T;
%     reSampleRate = Params.reSampleRate;
%        
%     a = scaleFactor*orthopxToMeter*del_t*reSampleRate;
%     b = scaleFactor*orthopxToMeter*del_t*reSampleRate;
%     c = scaleFactor*orthopxToMeter*del_t*reSampleRate;
%     d = scaleFactor*orthopxToMeter*del_t*reSampleRate;
%     e = scaleFactor*orthopxToMeter*del_t*reSampleRate;
% end
% disp('separate variables')
% toc
% 
% % b)
% 
% tic
% for ii=1:1000       
%     a = Params.scaleFactor*Params.orthopxToMeter*Params.delta_T*Params.reSampleRate;
%     b = Params.scaleFactor*Params.orthopxToMeter*Params.delta_T*Params.reSampleRate;
%     c = Params.scaleFactor*Params.orthopxToMeter*Params.delta_T*Params.reSampleRate;
%     d = Params.scaleFactor*Params.orthopxToMeter*Params.delta_T*Params.reSampleRate;
%     e = Params.scaleFactor*Params.orthopxToMeter*Params.delta_T*Params.reSampleRate;
% end
% disp('struct variables')
% toc
% 
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % a)
% tic
% for ii=1:100
%     carData.recordingId(end) = carData.recordingId(end-1);
%     carData.trackId(end) = carData.trackId(end-1);
%     carData.xCenter(end) = carData.xCenter(end-1);
%     carData.yCenter(end) = carData.yCenter(end-1);
%     carData.xVelocity(end) = carData.xVelocity(end-1);
%     carData.yVelocity(end) = carData.yVelocity(end-1);
% end
% disp('table')
% toc
% 
% % b) is better
% carStruct = table2struct(carData);
% tic
% for ii=1:100
%     carStruct(end).recordingId = carStruct(end-1).recordingId;
%     carStruct(end).trackId = carStruct(end-1).trackId;
%     carStruct(end).xCenter = carStruct(end-1).xCenter;
%     carStruct(end).yCenter = carStruct(end-1).yCenter;
%     carStruct(end).xVelocity = carStruct(end-1).xVelocity;
%     carStruct(end).yVelocity = carStruct(end-1).yVelocity;
% end
% disp('struct')
% toc
% 
% % c) is better
% carStruct_2 = table2struct(carData, 'ToScalar',true);
% tic
% for ii=1:100
%     carStruct_2.recordingId(end) = carStruct_2.recordingId(end-1);
%     carStruct_2.trackId(end) = carStruct_2.trackId(end-1);
%     carStruct_2.xCenter(end) = carStruct_2.xCenter(end-1);
%     carStruct_2.yCenter(end) = carStruct_2.yCenter(end-1);
%     carStruct_2.xVelocity(end) = carStruct_2.xVelocity(end-1);
%     carStruct_2.yVelocity(end) = carStruct_2.yVelocity(end-1);
% end
% disp('scalar struct')
% toc
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 
% %a) is better
% tic
% for ii=1:10000
%     carData(end+1, :) = carData(end, :); 
% end
% disp('end')
% toc
% 
% % b)
% tic
% for ii=1:10000
%     carData = [carData; carData(end, :)]; 
% end
% disp('append')
% toc

% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

car.a = 1;
car.b = 1;

car(end+1,:) = car(end,:);


