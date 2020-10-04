%% plot 


N = size(predictionTrajectoryMatrix,1);
prediction = reshape(predictionTrajectoryMatrix(:, 3:end)', [2,N*30])';
figure()
for ii=1:N
    plot(prediction((N-1)*30 + 1: N*30,1), prediction((N-1)*30 + 1: N*30,2), '*', 'MarkerSize', 8); hold on;
%     plot(prediction(31:60,ii), prediction(31:60,2), 'r*', 'MarkerSize', 8); hold on;    
end
 

