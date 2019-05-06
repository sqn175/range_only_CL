% Read and Parse UWB range and SLAM data
clc; close all; clear all;
format long;
set(0,'defaultAxesFontSize',16)
%% User input variables
% ---------------------
% All UWB IDs = anchorIDs + robotIDs
UWBIDs = [0 1 2 4]; 
% row: x,y coordinate of static UWB anchors
anchorPos = [2.5305,-0.80421;
            0.4084, -1.20842]; 
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
    
[UWBFilename, filePath] = uigetfile('*.txt','Select UWB hex data file');
addpath(filePath);

% [gtFileName, filePath] = uigetfile('*.csv','Select SLAM files in the order of robotIDs',...
%     'MultiSelect', 'on');
gtFileName = {'1.csv','4.csv'};

[range_m, imu_m, odom_m] = hexdumpUWB(UWBFilename, UWBIDs, robotIDs);

for i = 1:nRobot
     gt = importSLAMData(gtFileName{i});
     robot(i).x = gt.VarName2; 
     robot(i).y = gt.VarName3;
     [~,~,robot(i).phi] = quart2rpy([gt.VarName5, gt.VarName6, gt.VarName7, gt.VarName8]);
     robot(i).t = gt.VarName1 / 1e6;
     robot(i).id = robotIDs(i);
     robot(i).x0 = robot(i).x(1);
     robot(i).y0 = robot(i).y(1);
     robot(i).phi0 = robot(i).phi(1);
    % associate odom with robot

    odom_m(i).v_x = odom_m(i).Delta(:,1) ./ odom_m(i).int * 1000;
    odom_m(i).v_y = odom_m(i).Delta(:,2) ./ odom_m(i).int * 1000;
    odom_m(i).omega = odom_m(i).Delta(:,3) ./ odom_m(i).int * 1000;
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
%% Compare odom and SLAM
% Velocity from SLAM 
for i = 1:nRobot
    robot(i).v = vecnorm( [robot(i).x(2:end) - robot(i).x(1:end-1), ...
                    robot(i).y(2:end) - robot(i).y(1:end-1)]' )' ./ ...
                    (robot(i).t(2:end) - robot(i).t(1:end-1));
    len = length(robot(i).phi);
    for j = 1:len-1 
        dphi = robot(i).phi(j+1) - robot(i).phi(j);
        if dphi >= 1.5*pi
            dphi = dphi - 2*pi;
        elseif dphi <= -1.5*pi
            dphi = dphi + 2*pi;
        end
        robot(i).omega(j) = dphi / (robot(i).t(j+1) - robot(i).t(j));
    end
end
% Velocity from Odom
for r = 1:nRobot
    figure;
    subplot(2,1,1); hold on;
    plot(robot(r).t(2:end), robot(r).v);
    plot(robot(r).t_m, robot(r).v_m);
    title(['robot ', num2str(robot(r).id), ' v']);
    xlabel('t/s'); ylabel('v(m/s)');
    legend('SLAM','Odom');
    subplot(2,1,2); hold on;
    plot(robot(r).t(2:end), robot(r).omega);
    plot(robot(r).t_m, robot(r).omega_m);
    title(['robot ', num2str(robot(r).id), ' omega']);
    xlabel('t/s'); ylabel('omega(rad/s)');
    legend('SLAM','Odom');
end

%% Compare UWB and SLAM
pairs = 0;
for id1 = 1:nUWB
    for id2 = id1+1:nUWB
        pairs = pairs + 1;
        range_slam(pairs).pair = [UWBIDs(id1), UWBIDs(id2)];
        % Associate range_slam with range_m
        % What is the pair id in range_m
        for pair_idx = 1:length(range_m)
            if range_m(pair_idx).pair(1) == range_slam(pairs).pair(1) && ...
                    range_m(pair_idx).pair(2) == range_slam(pairs).pair(2)
                break;
            end
        end
        
        [lia1_r, r1] = ismember(UWBIDs(id1), robotIDs);
        [lia2_r, r2] = ismember(UWBIDs(id2), robotIDs);
        % Case1: robot-to-robot
        if lia1_r && lia2_r
            % Synchronize timestamp of robot 2 to robot 1
            % and timestamp of uwb ranging mesurements to robot 1
            % Requirement: data rate of latter is higher or equal than prior one
            i = 1; j = 1; k = 1;
            r1Len = length(robot(r1).t);
            r2Len = length(robot(r2).t);
            uwbLen = length(range_m(pair_idx).t);
            
            if robot(r1).t(i) <= robot(r2).t(j)
               while robot(r1).t(i) <= robot(r2).t(j) && i < r1Len && j < r2Len
                    i = i + 1;
                end
            end
            
            if robot(r1).t(i) <= range_m(pair_idx).t(k)
                while robot(r1).t(i) <= range_m(pair_idx).t(k) && i < r1Len && k < uwbLen
                    i = i + 1;
                end
            end
            
            idx = 1;
            while i < r1Len && j < r2Len-1 && k < uwbLen - 3
                while robot(r1).t(i) > robot(r2).t(j)
                    j = j + 1;
                    if j >= r2Len-1
                        break;
                    end
                end
                r1t = robot(r1).t(i);
                r2x = interp1(robot(r2).t(j-1:j+1), robot(r2).x(j-1:j+1), r1t);
                r2y = interp1(robot(r2).t(j-1:j+1), robot(r2).y(j-1:j+1), r1t);
                r1x = robot(r1).x(i);
                r1y = robot(r1).y(i);
                range_slam(pairs).r_gt(idx) = norm([r1x-r2x, r1y-r2y]);
                range_slam(pairs).t(idx) = r1t;
                
                while robot(r1).t(i) > range_m(pair_idx).t(k)
                    k = k + 1;
                    if k >= uwbLen - 3
                        break;
                    end
                end
                range_slam(pairs).r_uwb(idx) = mean(range_m(pair_idx).range(k-2:k+2));
                
                while robot(r1).t(i) <= max(range_m(pair_idx).t(k), robot(r2).t(j))
                    i = i + 1;
                    if i >= r1Len
                        break;
                    end
                end
                idx = idx + 1;
            end
        % Case2: anchor-to-anchor
        elseif ~lia1_r && ~lia2_r
            if nAnchor ~= 2
                error('Not implemented.');
            end
            tmprange = norm(anchorPos(1,:)-anchorPos(2,:));
            range_slam(pairs).r_uwb = range_m(pair_idx).range(1:50:end)';
            range_slam(pairs).r_gt = ones(1,length(range_slam(pairs).r_uwb))*tmprange;
            range_slam(pairs).t = range_m(pair_idx).t(1:50:end)';
        % Case3: robot-to-anchor
        else
            if lia1_r && ~lia2_r
                [~, loc_anc] = ismember( UWBIDs(id2), anchorIDs);
                loc_1 = r1;
            elseif ~lia1_r && lia2_r
                [~, loc_anc] = ismember( UWBIDs(id1), anchorIDs);
                loc_1 = r2;
            end
            i = 1;  k = 1;
            r1Len = length(robot(loc_1).t);
            uwbLen = length(range_m(pair_idx).t);
            
            if robot(loc_1).t(i) <= range_m(pair_idx).t(k)
                while robot(loc_1).t(i) <= range_m(pair_idx).t(k) && i < r1Len && k < uwbLen
                    i = i + 1;
                end
            end
            
            idx = 1;
            while i < r1Len &&  k < uwbLen - 3
                r1x = robot(loc_1).x(i);
                r1y = robot(loc_1).y(i);
                r1t = robot(loc_1).t(i);
                range_slam(pairs).r_gt(idx) = norm([r1x-anchorPos(loc_anc,1), ...
                                                    r1y-anchorPos(loc_anc,2)]);
                range_slam(pairs).t(idx) = r1t;
                
                while robot(loc_1).t(i) > range_m(pair_idx).t(k)
                    k = k + 1;
                    if k >= uwbLen - 3
                        break;
                    end
                end
                range_slam(pairs).r_uwb(idx) = mean(range_m(pair_idx).range(k-2:k+2));
                
                while robot(loc_1).t(i) <= range_m(pair_idx).t(k)
                    i = i + 1;
                    if i >= r1Len
                        break;
                    end
                end
                idx = idx + 1;
            end
        end
    end
end

% fit
x = [];
y = [];

for p = 1:pairs
    if p==2 
        continue;
    end
    x = [x range_slam(p).r_uwb];
    y = [y range_slam(p).r_gt];
end
% x = [range_slam(1).r_uwb range_slam(4).r_uwb];
% y = [range_slam(1).r_gt range_slam(4).r_gt];
% x = [range_slam(3).r_uwb range_slam(6).r_uwb];
% y = [range_slam(3).r_gt range_slam(6).r_gt];

% f = fit(x(150:end)',y(150:end)','poly2');
% coef = coeffvalues(f);
% x_fit = x.^2*coef(1) + coef(2)*x + coef(3);
% RMSE = sqrt(mean((y - x_fit).^2))
% S = std(y-x_fit)

% plot
for p = 1:pairs
    figure; hold on;
    plot(range_slam(p).t, range_slam(p).r_gt);
    tmprange = range_m(p).range;
%     range_fit = coef(1)*tmprange.^2 + coef(2)*tmprange + coef(3);
%     range_fit = tmprange.^2*coef(1) + coef(2)*tmprange + coef(3);
    range_fit = tmprange + 0.5158;
    plot(range_m(p).t, range_fit );
    xlabel('time(s)');
    ylabel('range(m)');
    legend('SLAM groundtruth','UWB ranging');
    id1 = range_slam(p).pair(1);
    id2 = range_slam(p).pair(2);
    [lia1_r, r1] = ismember(id1, robotIDs);
    [lia2_r, r2] = ismember(id2, robotIDs);
    title_str = 'Range: ';
    if lia1_r
        title_str = [title_str,'robot ',num2str(id1),' to '];
    else
        title_str = [title_str,'anchor ',num2str(id1),' to '];
    end
    if lia2_r
        title_str = [title_str,'robot ',num2str(id2)];
    else
        title_str = [title_str,'anchor ',num2str(id2)];
    end
    title(title_str);
end
