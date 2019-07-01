% Verify the RCL results
clear all; close all; clc;
addpath('../Matlab-VerifyData/');
addpath('./icp');

baseDir = 'C:\\Users\\Qin Shi\\Seafile\\私人资料库\\2018_MRL\\ION GNSS+ 2019\\论文数据\\res0618\\';

fileRCLRes = strcat(baseDir, 'res.txt');

robotIds = [1 3 4]; % 1 red, 3 green, 4 yellow
colors = {[244, 71, 65]/256, [20, 122, 20]/256, [201, 176, 56]/256};

anchors = [0, 0;
           2.42,0];
%%
nRobot = length(robotIds);
Rt = [0.6, 2.5, 0];

for r = 1:nRobot
    robot(r).id = robotIds(r);
    robot(r).color = cell2mat(colors(r));
    robot(r).t = [];
    robot(r).x = [];
    robot(r).y = [];
    robot(r).phi = [];
    % Transform from groundtruth to RCL
end

res = importRCLRes(fileRCLRes);
len = height(res);

for i = 1:len
    r = find(res.id(i) == robotIds);
    robot(r).t(end+1) = res.t(i);
    robot(r).x(end+1) = res.x(i);
    robot(r).y(end+1) = res.y(i);
    robot(r).phi(end+1) = res.phi(i);
end

figure; hold on; title('Vehicle trajectories'); 
axis equal; grid on; set(gca,'GridLineStyle','--');
xlabel('x (m)'); ylabel('y (m)');
box on;
for r = 1:nRobot
    plot(robot(r).x, robot(r).y, 'DisplayName', ['Vehicle ', num2str(robot(r).id)], ...
        'Color', robot(r).color, 'LineWidth', 1.6);
end
plot(anchors(:,1), anchors(:,2), 'd','DisplayName', 'Static Vehicles', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
legend show

%% Read gt
for r = 1:nRobot
    filename = strcat(baseDir, num2str(robot(r).id), '_SLAM.txt');
    gtRes = importSLAMfile(filename);
    robot(r).t_gt = gtRes.t / 1e6;
    robot(r).x_gt = gtRes.x; 
    robot(r).y_gt = gtRes.y;
    [~, ~, robot(r).phi_gt] = quart2rpy([gtRes.qw, gtRes.qx, gtRes.qy, gtRes.qz]);
    robot(r).phi_gt = robot(r).phi_gt;
end

%%
for r = 1:nRobot
    error(r).id = robotIds(r);
    error(r).resRCL = [];
    error(r).resGt = [];
    error(r).err = [];
    error(r).t = [];
    error(r).ape = []; % absolute position error
    error(r).i = []; 
end

% Point cloud to be aligned
D = [];
M = [];

for r = 1:nRobot
    item = robot(r);
    % Synchronize timestamp of RCL to SLAM
    lenRCL = length(item.t);
    lenGt = length(item.t_gt);
    if (item.t(1) <= item.t_gt(1))
        error('Not implemented');
    end
    
    % proceed to the first coinciding timestamp where t_gt is ahead
    iRCL = 1; iGt = 1;
    while (iGt <= lenGt && item.t_gt(iGt) <= item.t(iRCL))
        iGt = iGt + 1;
    end
    error(r).i(1) = iGt;
    error(r).i(3) = iRCL;
    
    while (iGt <= lenGt)
        while (iRCL <= lenRCL && item.t(iRCL) < item.t_gt(iGt))
            iRCL = iRCL + 1;
        end
        if (iRCL >= lenRCL)
            break;
        end
        % Time synchronized
        resRCL = [item.x(iRCL), item.y(iRCL), item.phi(iRCL)];
        resGt = [item.x_gt(iGt), item.y_gt(iGt), item.phi_gt(iGt)];
        % Alignment
        D(:,end+1) = [item.x(iRCL); item.y(iRCL); 0];
        M(:,end+1) = [item.x_gt(iGt); item.y_gt(iGt); 0];
        
        error(r).resRCL(end+1, :) = resRCL;
        error(r).t(end+1) = item.t(iRCL);
        error(r).resGt(end+1, :) = resGt;
%         error(r).err(end+1, :) =  resGt - resRCL;
%         error(r).ape(end+1) = norm(error(r).err(end,1:2));
        iGt = iGt + 1;
    end
    
    error(r).i(2) = iGt-1;
    error(r).i(4) = iRCL-1;
end

%% Alignment using ICP
% align results to groundtruth
[Ricp Ticp ER t] = icp(M, D, 15);

diff_phi = rotm2eul(Ricp, 'xyz');
n = 475;
Dicp = Ricp * D + repmat(Ticp, 1, n);
% Plot model points blue and transformed points red
error(1).xyerr = Dicp(1:2,:) - M(1:2, :);
error(1).xyape = vecnorm(error(r).xyerr);

error(1).phierr = error(r).resRCL(:,3) + diff_phi(3) - error(r).resGt(:, 3);
tmpLen = length(error(1).phierr);
for i = 1:tmpLen
    if error(1).phierr(i) > pi
        error(1).phierr(i) = error(1).phierr(i) - 2*pi;
    elseif error(1).phierr(i) < -pi
        error(1).phierr(i) = error(1).phierr(i) + 2*pi;
    end
end
error(1).phiape = abs(error(1).phierr);

figure;
subplot(2,2,1);
plot3(M(1,:),M(2,:),M(3,:),'bo',D(1,:),D(2,:),D(3,:),'r.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Red: z=sin(x)*cos(y), blue: transformed point cloud');

% Plot the results
subplot(2,2,2);
plot3(M(1,:),M(2,:),M(3,:),'bo',Dicp(1,:),Dicp(2,:),Dicp(3,:),'r.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('ICP result');
% align gt to res and compute the error
for r = 1:nRobot
    tmpLen = length(robot(r).x);
    for i=1:tmpLen
        xyz = [robot(r).x(i); robot(r).y(i); 0];
        aligned_xyz = Ricp * xyz + Ticp;
        robot(r).x(i) = aligned_xyz(1);
        robot(r).y(i) = aligned_xyz(2);
        robot(r).phi = robot(r).phi + diff_phi(3);
    end
end

for r = 1:nRobot
    % plot trajectory
    figure; hold on; title(['Vehicle ', num2str(error(r).id), ' trajectory']);
    xlabel('x (m)'); ylabel('y (m)'); grid on; set(gca,'GridLineStyle','--');
    box on; axis equal;
    plot(robot(r).x(error(r).i(3) : error(r).i(4)), ...
        robot(r).y(error(r).i(3) : error(r).i(4)), ...
        'LineWidth', 1.6, 'Color', robot(r).color);
    plot(robot(r).x_gt(error(r).i(1) : error(r).i(2)), ...
        robot(r).y_gt(error(r).i(1) : error(r).i(2)), ...
        '--k', 'LineWidth', 1.6);
    legend('Ours', 'Reference');
    
    % plot error
    figure; hold on; title(['Vehicle ', num2str(error(r).id), ': APE (m)']);
    xlabel('Time (sec)'); ylabel('APE (m)'); grid on; set(gca,'GridLineStyle','--');
    box on;
    avg = mean(error(r).xyape);
    rmse = sqrt(mean((error(r).xyape).^2));
    stder = std(error(r).xyape);
%     rectangle('Position',[robot(r).t(1), avg - stder, ...
%                         robot(r).t(end)-robot(r).t(1), 2*stder], ...
%             'FaceColor',[182, 227, 237]/256,'EdgeColor',[171, 198, 204]/256);

    plot(error(r).t, error(r).xyape, 'Color', robot(r).color, 'LineWidth', 1.6);
    plot([error(r).t(1), error(r).t(end)], [rmse, rmse], 'b', 'LineWidth', 1.6);
    plot([error(r).t(1), error(r).t(end)], [avg, avg], 'm', 'LineWidth', 1.6);
    legend('APE (m)', 'rmse', 'mean');
    
    % plot heading error
    figure; hold on; title(['Vehicle ', num2str(error(r).id), ': Heading error (m)']);
    xlabel('Time (sec)'); ylabel('Heading error (rad)'); grid on; set(gca,'GridLineStyle','--');
    box on;
    avg = mean(error(r).phiape);
    rmse = sqrt(mean((error(r).phiape).^2));
    stder = std(error(r).phiape);
%     rectangle('Position',[robot(r).t(1), avg - stder, ...
%                         robot(r).t(end)-robot(r).t(1), 2*stder], ...
%             'FaceColor',[182, 227, 237]/256,'EdgeColor',[171, 198, 204]/256);

    plot(error(r).t, error(r).phiape, 'Color', robot(r).color, 'LineWidth', 1.6);
    plot([error(r).t(1), error(r).t(end)], [rmse, rmse], 'b', 'LineWidth', 1.6);
    plot([error(r).t(1), error(r).t(end)], [avg, avg], 'm', 'LineWidth', 1.6);
    legend('APE (m)', 'rmse', 'mean');
end

