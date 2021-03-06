clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');

if exist ('OCTAVE_VERSION', 'builtin') 
    addpath('AHRS_Octave');
    pkg load signal
end
% -------------------------------------------------------------------------
% Select dataset (comment in/out)

if exist ('OCTAVE_VERSION', 'builtin') 

    pkg load signal
	filePath = 'Datasets/straightLine_CalInertialAndMag.csv';
    %slow
    %filePath = './Datasets/lpSlowLine.csv'
    %startTime = 2;
    %stopTime = 30;

    %mid
    %filePath = './Datasets/lpMidLine.csv'
    %startTime = 0.5;
    %stopTime = 25;

    %normal
    %filePath = './Datasets/lpLine.csv'
    %startTime = 2;
    %stopTime = 24;

    %rot
    %filePath = './Datasets/rotSlow.csv'
    %startTime = 2;
    %stopTime = 88;
    %startTime = 20.2;
    %stopTime = 27.5;

    %200rot
    filePath = './Datasets/gyrtest200.csv'
    startTime = 2;
    stopTime = 80;
else
	filePath = 'Datasets/straightLine_CalInertialAndMag.csv';
    %slow
    %filePath = './Datasets/lpSlowLine.csv'
    %startTime = 2;
    %stopTime = 30;

    %mid
    %filePath = './Datasets/lpMidLine.csv'
    %startTime = 0.5;
    %stopTime = 25;

    %normal
    %filePath = './Datasets/lpLine.csv'
    %startTime = 2;
    %stopTime = 24;

    %rot
    %filePath = './Datasets/rotSlow.csv'
    %startTime = 2;
    %stopTime = 88;
    %startTime = 20.2;
    %stopTime = 27.5;

    %200rot
    filePath = './Datasets/gyrtest200.csv'
    startTime = 2;
    stopTime = 80;
end


% -------------------------------------------------------------------------
% Import data


samplePeriod = 1/200;
xIMUdata = dlmread(filePath,',',1,0);
time = xIMUdata(:,2);
accX = xIMUdata(:,4);
accY = xIMUdata(:,5);
accZ = xIMUdata(:,6);
gyrX = xIMUdata(:,7);
gyrY = xIMUdata(:,8);
gyrZ = xIMUdata(:,9);

qW = xIMUdata(:,16);
qX = xIMUdata(:,17);
qY = xIMUdata(:,18);
qZ = xIMUdata(:,19);
%tmpNum = 0.0025;
%time = [];
%gyrX = [];
%gyrY = [];
%gyrZ = [];
%accX = [];
%accY = [];
%accZ = [];
%for i = 1:1:size(xIMUdata,1)
%    if xIMUdata(i,2) - tmpNum >= 0.039
%	    time = [time;xIMUdata(i,2)];
%	    gyrX = [gyrX;xIMUdata(i,7)];
%	    gyrY = [gyrY;xIMUdata(i,8)];
%	    gyrZ = [gyrZ;xIMUdata(i,9)];
%	    accX = [accX;xIMUdata(i,4)];
%	    accY = [accY;xIMUdata(i,5)];
%	    accZ = [accZ;xIMUdata(i,6)];
 %       tmpNum = xIMUdata(i,2);
  %      if i < 100
  %          disp(tmpNum);
  %      end
  %  end
%end

clear('xIMUdata');

% -------------------------------------------------------------------------
% Manually frame data

%startTime ~ stopTimeのセルの行番号を1にする
indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);

time = time(indexSel);
gyrX = gyrX(indexSel, :);
gyrY = gyrY(indexSel, :);
gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :);
accY = accY(indexSel, :);
accZ = accZ(indexSel, :);

qW = qW(indexSel, :);
qX = qX(indexSel, :);
qY = qY(indexSel, :);
qZ = qZ(indexSel, :);




% -------------------------------------------------------------------------
% Detect stationary periods

% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% HP filter accelerometer data
%正規化されたカットオフ周波数 Wn をもつ n 次のローパス デジタル バタワース フィルター
%filtCutOff = 0.001;
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');

%ゼロ位相デジタルフィルター 遅延をなくす
acc_magFilt = filtfilt(b, a, acc_mag);

% Compute absolute value
acc_magFilt = abs(acc_magFilt);

% LP filter accelerometer data
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);

% Threshold detection
stationary = acc_magFilt < 0.05;

% -------------------------------------------------------------------------
% Plot data raw sensor data and stationary periods

figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Sensor Data');
ax(1) = subplot(3,1,1);
    hold on;
    plot(time, gyrX, 'r');
    plot(time, gyrY, 'g');
    plot(time, gyrZ, 'b');
    title('Gyroscope');
    xlabel('Time (s)');
    ylabel('degree per second');
    legend('X', 'Y', 'Z');
    hold off;
ax(2) = subplot(3,1,2);
    hold on;
    plot(time, accX, 'r');
    plot(time, accY, 'g');
    plot(time, accZ, 'b');

    %plot(time, acc_mag, 'r');
    plot(time, double(stationary), 'k', 'LineWidth', 2);%octave
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('X', 'Y', 'Z', 'stationary');
    hold off;
ax(3) = subplot(3,1,3);
    hold on;
    %plot(time, accX, 'r');
    %plot(time, accY, 'g');
    %plot(time, accZ, 'b');
    plot(time, acc_magFilt, ':k');
    plot(time, double(stationary), 'k', 'LineWidth', 2);%octave
    title('Accelerometer Magnitude Filtered');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    %legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
    legend('Filtered', 'Stationary');
    hold off;
if ~exist('OCTAVE_VERSION','builtin')
    linkaxes(ax,'x');
end
% -------------------------------------------------------------------------
% Compute orientation

quat = [qW qX qY qZ];

% Initial convergence



initPeriod = 2;
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);

initquat = [mean(qW(indexSel)) mean(qX(indexSel)) mean(qY(indexSel)) mean(qZ(indexSel))];


AHRSStruct = AHRS_Octave('SamplePeriod', samplePeriod, 'Kp',1, 'KpInit', 1);
% Initial convergence
for i = 1:10000
    disp(AHRSStruct.q);
	AHRSStruct = UpdateIMU(AHRSStruct,[0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
end

disp(AHRSStruct.q);

%
%	% For all data
for t = 1:length(time)
	if(stationary(t))
		AHRSStruct.Kp = 0.5;
	else
		AHRSStruct.Kp = 0;
	end
	AHRSStruct = UpdateIMU(AHRSStruct,deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
    quat(t,:) = AHRSStruct.Quaternion;
end
% -------------------------------------------------------------------------
% Compute translational accelerations

% Rotate body accelerations to Earth frame
%quat = [qW qX qY qZ];
acc = quaternRotate([accX accY accZ], quaternConj(quat));

angle = [ones(size(acc,1)) zeros(size(acc,1),2)];
angle = quaternRotate(angle, quaternConj(quat));

% Plot quaternion
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Quaternion');
hold on;
scatter(angle(:,1), angle(:,2), 'r');
%plot(time, angle(:,2), 'g');
%plot(time, angle(:,3), 'b');
title('Angle');
xlabel('Time (s)');
ylabel('Angle');
legend('X', 'Y', 'Z');
hold off;

% % Remove gravity from measurements
 %acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     % unnecessary due to velocity integral drift compensation

% Convert acceleration measurements to m/s/s
acc = acc * 9.81;

quat = [qW qX qY qZ];
% Plot quaternion
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Quaternion');
hold on;
plot(time, quat(:,1), 'r');
plot(time, quat(:,2), 'g');
plot(time, quat(:,3), 'b');
plot(time, quat(:,4), 'p');
title('Quaternion');
xlabel('Time (s)');
ylabel('Quaternion');
legend('W','X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Plot translational accelerations
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Accelerations');
hold on;
plot(time, acc(:,1), 'r');
plot(time, acc(:,2), 'g');
plot(time, acc(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational velocities

acc(:,3) = acc(:,3) - 9.81;

% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
    %vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    vel(t,:) = vel(t-1,:) + acc(t,:) * (time(t)-time(t-1));
    if(stationary(t) == 1)
        vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
    end
end


% Compute integral drift during non-stationary periods
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == -1);
stationaryEnd = find([0; diff(stationary)] == 1);
for i = 1:numel(stationaryEnd)
    driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
    enum = 1:(stationaryEnd(i) - stationaryStart(i));
    drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
    velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
end

% Remove integral drift
vel = vel - velDrift;

% Plot translational velocity
figure('Position', [9 39 900 300], 'Number', 'off', 'Name', 'Velocity');
hold on;
plot(time, vel(:,1), 'r');
plot(time, vel(:,2), 'g');
plot(time, vel(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Compute translational position

% Integrate velocity to yield position
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * (time(t)- time(t-1));    % integrate velocity to yield position
end

planeNorm = sqrt(pos(:,1).*pos(:,1) + pos(:,2).*pos(:,2));
disp("posMax(m)"),disp(max(planeNorm))

% Plot translational position
figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Position');
hold on;
plot(time, pos(:,1), 'r');
plot(time, pos(:,2), 'g');
plot(time, pos(:,3), 'b');
plot(time, planeNorm, 'p');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z', 'plane');
hold off;

% -------------------------------------------------------------------------
% Plot 3D foot trajectory

% 
figure('Position', [9 39 900 600], 'Number', 'off', 'Name', 'Position3d');
%plot3(pos(:,1),pos(:,2),pos(:,3), 'r*');
plot(pos(:,1),pos(:,2), 'r*');

title('Position3d');
xlabel('xPosition (m)');
ylabel('yPosition (m)');
%%zlabel('zPosition (m)');
legend('pos');

% % Remove stationary periods from data to plot
% posPlot = pos(find(~stationary), :);
% quatPlot = quat(find(~stationary), :);
posPlot = pos;
quatPlot = quat;

% Extend final sample to delay end of animation
extraTime = 20;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end, 2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];



% Create 6 DOF animation

if ~exist ('OCTAVE_VERSION', 'builtin')
    SamplePlotFreq = 4;
    Spin = 120;
    SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                    'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                    'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
                    'AxisLength', 0.1, 'ShowArrowHead', false, ...
                    'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                    'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
end
