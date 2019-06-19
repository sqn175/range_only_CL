% Verify the RCL results
clear all; close all; clc;
addpath('../Matlab-VerifyData/');

baseDir = '/home/qin/Downloads/res0618/';

fileRCLRes = strcat(baseDir, 'res.txt');

robotIds = [1 3 4]; % 1 red, 3 green, 4 yellow
colors = {[244, 71, 65]/256, [20, 122, 20]/256, [201, 176, 56]/256};

anchors = [0, 0;
           3.6,0];
%%
nRobot = length(robotIds);
Rt = [0, 1.2, 0];

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
plot(anchors(:,1), anchors(:,2), 'd','DisplayName', 'Anchors', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
legend show

%% Read gt
for r = 1:nRobot
    filename = strcat(baseDir, num2str(robot(r).id), '_SLAM.txt');
    gtRes = importSLAMfile(filename);
    robot(r).t_gt = gtRes.t / 1e6;
    robot(r).x_gt = gtRes.x + Rt(1); 
    robot(r).y_gt = gtRes.y + Rt(2);
    [~, ~, robot(r).phi_gt] = quart2rpy([gtRes.qw, gtRes.qx, gtRes.qy, gtRes.qz]);
    robot(r).phi_gt = robot(r).phi_gt + Rt(3);
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
        
        error(r).resRCL(end+1, :) = resRCL;
        error(r).t(end+1) = item.t(iRCL);
        error(r).resGt(end+1, :) = resGt;
        error(r).err(end+1, :) =  resGt - resRCL;
        error(r).ape(end+1) = norm(error(r).err(end,1:2));
        iGt = iGt + 1;
    end
    
    error(r).i(2) = iGt-1;
    error(r).i(4) = iRCL-1;
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
    legend('Results', 'Reference');
    
    % plot error
    figure; hold on; title(['Vehicle ', num2str(error(r).id), ': APE (m)']);
    xlabel('Time (sec)'); ylabel('APE (m)'); grid on; set(gca,'GridLineStyle','--');
    box on;
    avg = mean(error(r).ape);
    rmse = sqrt(mean((error(r).ape).^2));
    stder = std(error(r).ape);
%     rectangle('Position',[robot(r).t(1), avg - stder, ...
%                         robot(r).t(end)-robot(r).t(1), 2*stder], ...
%             'FaceColor',[182, 227, 237]/256,'EdgeColor',[171, 198, 204]/256);

    plot(error(r).t, error(r).ape, 'Color', robot(r).color, 'LineWidth', 1.6);
    plot([error(r).t(1), error(r).t(end)], [rmse, rmse], 'b', 'LineWidth', 1.6);
    plot([error(r).t(1), error(r).t(end)], [avg, avg], 'm', 'LineWidth', 1.6);
    legend('APE (m)', 'rmse', 'mean');
end

