% Verify raw measurement data (UWB ranging and wheel encoders)
% Read and Parse UWB range and SLAM data
clc; close all; clear all;
format long;
set(0,'defaultAxesFontSize',16)
%% User input variables
% ---------------------
% All UWB IDs = anchorIDs + robotIDs
UWBIDs = [0 1 2 4]; 
% Robot IDs
robotIDs = [1 4];
% 1, read data from file; 0, load data from "data.mat"
readfile_flag = 1;
% Figure plot flags
plot_imu_flag = 0;
plot_odom_flag = 0;
% ---------------------
%% Read data from file
nRobot = length(robotIDs);
nUWB = length(UWBIDs);
anchorIDs = setdiff(UWBIDs, robotIDs);
nAnchor = length(anchorIDs);

if readfile_flag

UWBFilename = '/home/qin/Documents/range_only_CL/datasets/test.txt';
% [UWBFilename, filePath] = uigetfile('*.txt','Select UWB hex data file');
% addpath(filePath);

[range_m, imu_m, odom_m] = hexdumpUWB(UWBFilename, UWBIDs, robotIDs);


for i = 1:nRobot
    odom_m(i).v_x = odom_m(i).Delta(:,1) ./ odom_m(i).int * 1000;
    odom_m(i).v_y = odom_m(i).Delta(:,2) ./ odom_m(i).int * 1000;
    odom_m(i).omega = odom_m(i).Delta(:,3) ./ odom_m(i).int * 1000;
    robot(i).id = robotIDs(i);
    robot(i).v_m = odom_m(i).v_x;
    robot(i).omega_m = odom_m(i).omega;
    robot(i).t_m = odom_m(i).t;
end

% Save data to workspace
% Pre-process, outlier reject
for i = 1:nRobot
    odom_len = length(odom_m(i).v_x);
    for j = 1:odom_len
        if abs(robot(i).v_m(j)) > 1e6
            disp("Outlier v:");
            disp(robot(i).v_m(j));
            robot(i).v_m(j) = robot(i).v_m(j-1);
        end
        if abs(robot(i).omega_m(j)) > 1e6
            disp("Outlier omega:");
            disp(robot(i).omega_m(j));
            robot(i).omega_m(j) = robot(i).omega_m(j-1);
        end
    end
end
pairs = nUWB*(nUWB-1)/2;
% simple range rejection
for i = 1:pairs
    index = find(range_m(i).range > 1e5);
    if (sum(index) > 0)
        disp('UWB outiler:');
        disp(range_m(i).range(index));
    end
    range_m(i).range(index) = range_m(i).range(index-1);
end

save('data.mat', 'robot', 'range_m');

else
    % Load data from workspcae
    load('data.mat');
end

% plot test
for i = 1:nRobot
    figure;
    subplot(2,3,1);
    plot(odom_m(i).Delta(:,1));
    title(['Robot ',num2str(robot(i).id),' Odom dx']);
    subplot(2,3,2);
    plot(odom_m(i).Delta(:,2));
    title('Odom dy');
    subplot(2,3,3);
    plot(odom_m(i).Delta(:,3));
    title('Odom dphi');
    subplot(2,3,4);
    plot(odom_m(i).int);
    title('Odom int');
    subplot(2,3,5);
    plot(odom_m(i).t);
    title('Odom t');
end

%% Plot UWB range data

% anchor-anchor
anchorPairs = nAnchor*(nAnchor-1)/2;
if anchorPairs ~= 0
    k = 1;
    figure; 
    for i = 1:nAnchor
        for j = i+1:nAnchor
            for p = 1:pairs
                if anchorIDs(i) == range_m(p).pair(1) && anchorIDs(j) == range_m(p).pair(2)
                    subplot(anchorPairs,2,k);
                    plot(range_m(p).t,range_m(p).range,'.');
                    title(['anchor ', num2str(anchorIDs(i)), ' and anchor ', num2str(anchorIDs(j))]);
                    ylabel('UWB range/m'); xlabel('t/s');
                    dt = range_m(p).t(2:end) - range_m(p).t(1:end-1);
                    subplot(anchorPairs,2,k+1);
                    plot(dt,'.');
                    title('UWB range \Delta t');
                    ylabel('t/s');
                    k = k + 2;
                end
            end
        end
    end
end
% robot-robot
robotPairs = nRobot * (nRobot - 1) / 2;
k = 1;
figure;
for i = 1:nRobot
    for j = i+1:nRobot
        for p = 1:pairs
            if robotIDs(i) == range_m(p).pair(1) && robotIDs(j) == range_m(p).pair(2)
                subplot(robotPairs,2,k);
                plot(range_m(p).t,range_m(p).range,'.');
                title(['robot ', num2str(robotIDs(i)), ' and robot ', num2str(robotIDs(j))]);
                ylabel('UWB range/m'); xlabel('t/s');
                dt = range_m(p).t(2:end) - range_m(p).t(1:end-1);
                subplot(robotPairs,2,k+1);
                plot(dt,'.');
                title('UWB range \Delta t');
                ylabel('t/s');
                k = k + 2;
            end
        end
    end
end
% robot-anchor
robotAnchorPairs = nRobot * nAnchor;
k = 1;
figure;
for i = 1:nRobot
    for j = 1:nAnchor
        for p = 1:pairs
            robotID = robotIDs(i);
            anchorID = anchorIDs(j);
            if robotID < anchorID
                if robotID == range_m(p).pair(1) && anchorID == range_m(p).pair(2)
                    subplot(robotAnchorPairs,1,k);
                    plot(range_m(p).t,range_m(p).range,'.');
                    title(['robot ', num2str(robotID), ' and anchor ', num2str(anchorID)]);
                    ylabel('UWB range/m'); xlabel('t/s');
                    k = k + 1;
                end
            else
                if robotID == range_m(p).pair(2) && anchorID == range_m(p).pair(1)
                    subplot(robotAnchorPairs,1,k);
                    plot(range_m(p).t,range_m(p).range,'.');
                    title(['anchor ', num2str(anchorID), ' and robot ', num2str(robotID)]);
                    ylabel('UWB range/m'); xlabel('t/s');
                    k = k + 1;
                end
            end
        end
    end
end
%% Plot Odom data
if plot_odom_flag 
figure;
k = 1;
for r = 1:nRobot
    dt = odom_m(r).t(2:end) - odom_m(r).t(1:end-1);
    subplot(nRobot, 2, k);
    plot(odom_m(r).t, odom_m(r).Delta(:,1),'.');
    title(['Odom ', num2str(odom_m(r).id),' dx']);
    subplot(nRobot, 2, k+1);
    plot(dt,'.');
    title('\Delta t'); ylabel('t/s');
    k = k+2;
end
end
%% Plot IMU data
if plot_odom_flag 
figure;
k = 1;
for r = 1:nUWB
    dt = imu_m(r).t(2:end) - imu_m(r).t(1:end-1);
    subplot(nUWB, 2, k);
    plot(imu_m(r).t, imu_m(r).data(:,1)/8192.0,'.');
    title(['IMU ',num2str(imu_m(r).id),' a\_x']);
    subplot(nUWB, 2, k+1);
    plot(dt,'.');
    title('\Delta t'); ylabel('t/s');
    k = k+2;
end
end

