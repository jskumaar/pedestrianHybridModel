%% 

kk=3;
jj=1;

[PedZohData,~] = xlsread('PedestrianDataZoh.xlsx',6*(kk-1)+jj);
[PedRawData,~] = xlsread('PedestrianDataRaw.xlsx',6*(kk-1)+jj);
[VehicleZohData,~] = xlsread('VehiclePositionZoh.xlsx',6*(kk-1)+jj);


xZoh = PedZohData(:,4);
yZoh = PedZohData(:,5);
TSteps = PedZohData(:,3);
%% 2)WMA (Weighted Moving Average) - equal weights
[xPosFiltWMA,~] = MwaFilter(xZoh,TSteps,5);
[yPosFiltWMA,~] = MwaFilter(yZoh,TSteps,11);
figure()
ind = [1:length(TSteps)];
plot(PedZohData(ind,4),'.','MarkerSize',10);hold on; grid on;
plot(xPosFiltWMA(ind),'.','MarkerSize',10);hold on;
figure()
plot(diff(xPosFiltWMA(ind)),'.','MarkerSize',10);hold on; grid on;



error = abs(xPosFiltWMA-PedZohData(:,4));

figure()
plot(error,'.','MarkerSize',10)


[VehicleFiltWMA,~] = MwaFilter(VehicleZohData(:,4),TSteps,5);

figure()
plot(VehicleFiltWMA,'.','MarkerSize',10);hold on; grid on;
plot(VehicleZohData(:,4),'.','MarkerSize',10);hold on;




error = abs(VehicleFiltWMA-VehicleZohData(:,4));

figure()
plot(error,'.','MarkerSize',10)