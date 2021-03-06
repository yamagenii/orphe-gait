clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');
%OCTAVE 3.6.4 couldn't handle some of the Matlab features used in the script (e.g. class declarations -> ximu_matlab_library didn't work)
%Modified the code to enable running it with Octave on Windows platform
%OCTAVE
if exist ('OCTAVE_VERSION', 'builtin') 
	addpath('AHRS_Octave');
end

% -------------------------------------------------------------------------
% Select dataset (comment in/out)
%OCTAVE
if exist ('OCTAVE_VERSION', 'builtin') 

    pkg load signal
	filePath = 'Datasets/straightLine_CalInertialAndMag.csv';
else
	filePath = 'Datasets/straightLine';
end 
startTime = 6;
stopTime = 26;
%startTime = 13;
%topTime = 13.5;

 %filePath = 'Datasets/stairsAndCorridor_CalInertialAndMag.csv';
 %startTime = 5;
 %stopTime = 53;

% filePath = 'Datasets/spiralStairs';
% startTime = 4;
% stopTime = 47;

% -------------------------------------------------------------------------
% Import data

samplePeriod = 1/256;
if exist ('OCTAVE_VERSION', 'builtin') 
	xIMUdata = dlmread(filePath,',',1,0);
    tmpNum = 1;
	time = [];
	gyrX = [];
	gyrY = [];
	gyrZ = [];
	accX = [];
	accY = [];
	accZ = [];
    for i = 1:1:size(xIMUdata,1)
        if xIMUdata(i,1) - tmpNum >= 4
	        time = [time;xIMUdata(i,1)*1/256];
	        gyrX = [gyrX;xIMUdata(i,2)];
	        gyrY = [gyrY;xIMUdata(i,3)];
	        gyrZ = [gyrZ;xIMUdata(i,4)];
	        accX = [accX;xIMUdata(i,5)];
	        accY = [accY;xIMUdata(i,6)];
	        accZ = [accZ;xIMUdata(i,7)];
            tmpNum = xIMUdata(i,1);
            if i < 100
                disp(tmpNum);
            end
        end
    end
	time256 = xIMUdata(:,1)*1/256;
	gyrX256 = xIMUdata(:,2);
	gyrY256 = xIMUdata(:,3);
	gyrZ256 = xIMUdata(:,4);
	accX256 = xIMUdata(:,5);
	accY256 = xIMUdata(:,6);
	accZ256 = xIMUdata(:,7);
else
	xIMUdata = xIMUdataClass(filePath, 'InertialMagneticSampleRate', 1/samplePeriod);
	time = xIMUdata.CalInertialAndMagneticData.Time;
	gyrX = xIMUdata.CalInertialAndMagneticData.Gyroscope.X;
	gyrY = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y;
	gyrZ = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z;
	accX = xIMUdata.CalInertialAndMagneticData.Accelerometer.X;
	accY = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y;
	accZ = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z;
end 
clear('xIMUdata');
% -------------------------------------------------------------------------
% Manually frame data

% startTime = 0;
% stopTime = 10;

indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
time = time(indexSel);
gyrX = gyrX(indexSel, :);
gyrY = gyrY(indexSel, :);
gyrZ = gyrZ(indexSel, :);
accX = accX(indexSel, :);
accY = accY(indexSel, :);
accZ = accZ(indexSel, :);

%indexSel = find(sign(time256-startTime)+1, 1) : find(sign(time256-stopTime)+1, 1);
%time = time256(indexSel);
%gyrX = gyrX256(indexSel, :);
%gyrY = gyrY256(indexSel, :);
%gyrZ = gyrZ256(indexSel, :);
%accX = accX256(indexSel, :);
%accY = accY256(indexSel, :);
%accZ = accZ256(indexSel, :);
% -------------------------------------------------------------------------
% Detect stationary periods

% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

% HP filter accelerometer data
filtCutOff = 0.001;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
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

figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Sensor Data');
ax = subplot(2,1,1);
   hold on;
   plot(time, gyrX, 'r');
   plot(time, gyrY, 'g');
   plot(time, gyrZ, 'b');
%    title('Gyroscope');
   xlabel('Time (s)');
   ylabel('Angular velocity (^\circ/s)');
   legend('X', 'Y', 'Z');
   hold off;
ax = subplot(2,1,2);
   hold on;
   plot(time, accX, 'r');
    plot(time, accY, 'g');
    plot(time, accZ, 'b');
    plot(time, acc_magFilt, ':k');
    plot(time, stationary, 'k', 'LineWidth', 2);
    title('Accelerometer');
    xlabel('Time (s)');
    ylabel('Acceleration (g)');
    legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
    hold off;
if ~exist ('OCTAVE_VERSION', 'builtin')
	linkaxes(ax,'x');	%Octave 3.6.4 had not implemented linkaxes
end

% -------------------------------------------------------------------------
% Compute orientation

quat = zeros(length(time), 4);
% Initial convergence
initPeriod = 2;
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);

if ~exist ('OCTAVE_VERSION', 'builtin')
	AHRSalgorithm = AHRS('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);
	for i = 1:2000
		AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
	end

	% For all data
	for t = 1:length(time)
		if(stationary(t))
			AHRSalgorithm.Kp = 0.5;
		else
			AHRSalgorithm.Kp = 0;
		end
		AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
		quat(t,:) = AHRSalgorithm.Quaternion;
	end
else	%classdef wasn't implemented in Octave 3.6.4
	AHRSStruct = AHRS_Octave('SamplePeriod', 1/256, 'Kp', 1, 'KpInit', 1);
	% Initial convergence
	initPeriod = 2;
	indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
	for i = 1:2000
		AHRSStruct = UpdateIMU(AHRSStruct,[0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
	end

    disp(AHRSStruct.q);
	% For all data
	for t = 1:length(time)
		if(stationary(t))
			AHRSStruct.Kp = 0.5;
		else
			AHRSStruct.Kp = 0;
		end
		AHRSStruct = UpdateIMU(AHRSStruct,deg2rad([gyrX(t) gyrY(t) gyrZ(t)]), [accX(t) accY(t) accZ(t)]);
		quat(t,:) = AHRSStruct.Quaternion;
	end
end

% -------------------------------------------------------------------------
% Compute translational accelerations

% Rotate body accelerations to Earth frame
acc = quaternRotate([accX accY accZ], quaternConj(quat));

% % Remove gravity from measurements
% acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     % unnecessary due to velocity integral drift compensation

% Convert acceleration measurements to m/s/s
acc = acc * 9.81;

% Plot quaternion
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Quaternion');
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
% Plot translational accelerations
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Accelerations');
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
    vel(t,:) = vel(t-1,:) + acc(t,:) * (time(t,:)-time(t-1,:));
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
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name', 'Velocity');
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
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;    % integrate velocity to yield position
end

% Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot(time, pos(:,1), 'r');
plot(time, pos(:,2), 'g');
plot(time, pos(:,3), 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;

% -------------------------------------------------------------------------
% Plot 3D foot trajectory

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



if ~exist ('OCTAVE_VERSION', 'builtin')
% Create 6 DOF animation
SamplePlotFreq = 4;
Spin = 120;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot), 1)], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, ...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));
end
