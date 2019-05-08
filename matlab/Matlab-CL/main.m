clear all; close all; clc
addpath('utils');
rng(1); % Generate random numbers that are repeatable
set(0,'defaultAxesFontSize',16)
set(0, 'DefaultLineLineWidth', 2);
set(0,'DefaultLineMarkerSize',12);
%% The groundtruth and measurements
sim_freq = 20;  % Groundtruth simulation frequency in Hz.
dt_sec = 1/sim_freq;
% ����ֻ��SLAM����õ�?1�Ż�������У������?
coef = [-0.0645072297986758,1.14230147991211,0.515768398211774];
% We set world frame coincide with the reference frame of robot 1
% robot1, trajectory expressed in its own reference frame
% Wheel encoder noise
sigma_v = 0.05;
sigma_omega = 0.05;
% UWB ranging measurements sigma
sigma_uwb = 0.09;

dataOri = 2; % 1, from simulation, 2, from dataset
if dataOri == 1
    nRobots = 2;
    % anchors
    nAnchors = 2;
    anchorPos = [0.05,-1.062;
                2.6,2.034];
    for a = 1:nAnchors
        anchor(a).id = a + nRobots - 1;
        anchor(a).x = anchorPos(a,1);
        anchor(a).y = anchorPos(a,2);
    end
    sim_t = 0: dt_sec: 120;
    % robots
    for r = 1:nRobots
        robot(r).id = r - 1; % 0 based index
        [p, v, a, j] = trajectoryGen(sim_t, r);
        robot(r).x = p(1,:); robot(r).y = p(2,:);
        robot(r).x0 = robot(r).x(1); robot(r).y0 = robot(r).y(1); 
        [robot(r).phi, robot(r).v_m, robot(r).omega_m] ...
            = WheelEncoder(p, v, a, j, sigma_v, sigma_omega);
        robot(r).phi0 = robot(r).phi(1);
        robot(r).t = sim_t;
    end
    range_m = UWBRangeSim(sigma_uwb, robot, anchor);
elseif dataOri == 2
    UWBIDs = [0 1 2 4];
    anchorPos = [-0.5010,-1.4105;
                 -0.5010, 0.81263];
    robotIDs = [1 4];
    anchorIDs = setdiff(UWBIDs, robotIDs);
    nAnchors = length(anchorIDs);
    for a = 1:nAnchors
        anchor(a).id = anchorIDs(a);
        anchor(a).x = anchorPos(a,1);
        anchor(a).y = anchorPos(a,2);
    end
    nRobots = length(robotIDs);
    nUWB = length(UWBIDs);

    load('data190313.mat');
%     robot(1).v_m(877) = 0;
%     robot(1).omega_m(877) = 0;
%     save('data20190301.mat', 'robot', 'range_m');

    for i = 1:size(range_m, 2)
        tmprange = range_m(i).range;
        range_m(i).range = tmprange + 0.5158;
%         range_m(i).range = coef(1)*tmprange.^2 + coef(2)*tmprange + coef(3);
    end
    % �ض����ݼ����ض��������������ê��֮��Ĳ���?,���Ұ���ê����׼�����?
    % ���ֵ����?
    range_tmp = range_m;
    range_m(1) = range_tmp(5);
    range_m(2) = range_tmp(1);
    range_m(3) = range_tmp(4);
    range_m(4) = range_tmp(3);
    range_m(5) = range_tmp(6);
    range_m(6) = range_tmp(2);
    
    % �������������ϵ����С����ת���������ֵ�?
    for r = 1:nRobots
        robot(r).omega_m = -robot(r).omega_m;
    end
end

sim_len = length(robot(1).omega_m);

% figure test
figure;  axis equal
hold on;
for r = 1:nRobots
    plot(robot(r).x0, robot(r).y0,':s','DisplayName',['robot', num2str(robot(r).id)]);
end
for r = 1:nAnchors
    plot(anchor(r).x, anchor(r).y,'--o','DisplayName',['anchor', num2str(anchor(r).id)]);
end
legend show
% figure test end

%% Initialization
[state_ini, mds_pos] = initializer(robot, range_m);
for a = 1:nAnchors
    anchor(a).x_est = mds_pos(a,1);
    anchor(a).y_est = mds_pos(a,2);
%     anchor(a).x_est = anchorPos(a,1);
%     anchor(a).y_est = anchorPos(a,2);
end
baseline = norm(mds_pos(1,:) - mds_pos(2,:));

%% Linear motion detection
close all
window_len = 40;
heading = zeros(nRobots,sim_len);
for r = 1:nRobots
    odom_len = length(robot(r).v_m);
    omega_energy = zeros(1,odom_len);
    v_energy = zeros(1,odom_len);
    flag = zeros(1,odom_len);
    start_j = 0;
    for j = window_len : odom_len - 2
        omega_energy(j) = sum(robot(r).omega_m(j-window_len+1:j).^2) / window_len;
        v_energy(j) = robot(r).v_m(j);
        if (omega_energy(j) < 2e-3 && v_energy(j) > 0.1 && v_energy(j-window_len+1) > 0.1)
            if start_j == 0
                start_j = j-window_len+1; 
                flag(start_j:j-1) = 1;
            end
            flag(j) = 1;
        else
            if flag(j-1) == 1
                % distance to first anchor
                d1 = range_m(2*r).range(5*start_j:5*(j-1));
                % distance to second anchor
                d2 = range_m(2*r+1).range(5*start_j:5*(j-1));
                % fit to a line
                coef = polyfit(1:5*(j-1)-5*start_j + 1, d1', 1);
                d1_start = coef(1) * 1 + coef(2);
                d1_end = coef(1) * (5*(j-1)-5*start_j+1) + coef(2);
                
                coef = polyfit(1:5*(j-1)-5*start_j + 1, d2', 1);
                d2_start = coef(1) * 1 + coef(2);
                d2_end = coef(1) * (5*(j-1)-5*start_j+1) + coef(2);
                
%                 d1 = [mean(range_m(2*r).range(5*(start_j)-2:5*(start_j)+2)),...
%                       mean(range_m(2*r+1).range(5*(start_j)-2:5*(start_j)+2))];
%                 d2 = [mean(range_m(2*r).range(5*j-2:5*j+2)),...
%                       mean(range_m(2*r+1).range(5*j-2:5*j+2))];
%                 heading(r,start_j) = heading_est([d1;d2],3.2);
                d = [d1_start, d2_start;
                     d1_end, d2_end];
                heading(r,start_j) = heading_est(d,baseline);
                start_j = 0;
            end
        end
    end
    figure; hold on;
    plot(robot(r).omega_m);
    plot(robot(r).v_m);
    plot(omega_energy);
    plot(v_energy);
    plot(range_m(2*r).range(1:5:end));
    plot(range_m(2*r+1).range(1:5:end));
    plot(0.4*flag);
    legend('omega_m','v_m','omega_e','v_e','range to 0','range to 2','flag');
    figure;
    plot(heading(r,:),'.');
end

%% Odometry
% Integration
vState_int = nan(nRobots,3,sim_len);
for r = 1:nRobots
    state_int = State(robot(r).x0, robot(r).y0, robot(r).phi0);
%     state_int = State(0, 0, 0);
    vState_int(r,:,1) = state_int.serialize;
    for k = 2:sim_len
        [~, state_int] = state_int.propagate(robot(r).v_m(k-1), ...
            robot(r).omega_m(k-1),robot(r).v_m(k), robot(r).omega_m(k), ...
            dt_sec, false);
        vState_int(r,:,k) = state_int.serialize;
    end
end

% plot results
% for r = 1:nRobots
%     x_int = reshape(vState_int(r,1,:),[1,sim_len]); 
%     y_int = reshape(vState_int(r,2,:),[1,sim_len]);
%     phi_int = reshape(vState_int(r,3,:),[1,sim_len]);
% 
%     figure;
%     subplot(2,3,[1 2 3]); hold on; axis equal
%     plot(robot(r).x, robot(r).y,'g.');
%     plot(robot(r).x(1), robot(r).y(1),'-s','MarkerSize',10,...
%         'MarkerEdgeColor','green',...
%         'MarkerFaceColor','green');
%     plot(x_int, y_int,'k.');
%     xlabel('x/m'); ylabel('y/m')
%     legend('groundtruth','start point','odometry');
%     title(['trajectory of robot ' num2str(r)]);
%     subplot(2,3,4); hold on;
%     plot(robot(r).x,'g.');
%     plot(x_int,'k.');
%     ylabel('m');
%     legend('groundtruth','odometry');
%     title('x');
%     subplot(2,3,5); hold on;
%     plot(robot(r).y,'g.');
%     plot(y_int,'k.');
%     ylabel('m');
%     legend('groundtruth','odometry');
%     title('y');
%     subplot(2,3,6); hold on;
%     plot(robot(r).phi,'g.');
%     plot(phi_int,'k.');
%     ylabel('rad');
%     legend('groundtruth','odometry');
%     title('yaw');
% end
% clear x_int y_int phi_int 

%% Extrinsic calibration 
% Extrinsic calibration for robot 1 and 2
% p_int_1 = [reshape(vState_int(1,1,:),[1,sim_len]);...
%            reshape(vState_int(1,2,:),[1,sim_len])]; 
% p_int_2 = [reshape(vState_int(2,1,:),[1,sim_len]);...
%            reshape(vState_int(2,2,:),[1,sim_len])];
% p_int_3 = [reshape(vState_int(3,1,:),[1,sim_len]);...
%    reshape(vState_int(3,2,:),[1,sim_len])];
% ind = 1:500:2001;
% [phi_m, p_m] = fivepoint(p_int_1(:,ind), p_int_2(:,ind), ...
%             range_m(1).range(ind));
% phi_gt = robot(2).phi0 /pi * 180
% phi_m = phi_m / pi *180
% p_gt = [robot(2).x0 robot(2).y0]
% p_m
% ind = 1:500:2001;
% [phi_m, p_m] = fivepoint(p_int_1(:,ind), p_int_3(:,ind), ...
%             range_m(2).range(ind));
% phi_gt = robot(3).phi0 /pi * 180
% phi_m = phi_m / pi *180
% p_gt = [robot(3).x0 robot(3).y0]
% p_m
%% Kalman
P_all = eye(3*nRobots);
Q = [sigma_v^2, sigma_v^2, sigma_omega^2];
Q_all = diag(repmat(Q,[1,nRobots]));
if dataOri == 1 
    % the range between two static anchors are not simulated, but the datasets
    % contains the data
    nRange = size(range_m,2);
elseif dataOri == 2
    nRange = size(range_m,2) - nAnchors*(nAnchors-1)/2;
end

% Integration
vState_est = nan(nRobots,3,sim_len);
for r = 1:nRobots
    if dataOri == 1 
        state_est(r) = State(robot(r).x0, robot(r).y0, robot(r).phi0);
    elseif dataOri == 2
        state_est(r) = State(state_ini(r,:)');
%         state_est(r) = State(robot(r).x0, robot(r).y0, robot(r).phi0);
    end
    % Save the stuffs 
    vState_est(r,:,1) = state_est(r).serialize;
end

for k = 2:sim_len
    if k == 2830
        k = k;
    end
    Q_all = diag(repmat(Q,[1,nRobots]));
    % 1. State and covariance propagation
    Phi_all = zeros(3*nRobots, 3*nRobots);
    for r = 1:nRobots
        [Phi, state_est(r)] = state_est(r).propagate(robot(r).v_m(k-1), ...
            robot(r).omega_m(k-1),robot(r).v_m(k), robot(r).omega_m(k), ...
            dt_sec, true);
        
        Phi_all(1+3*(r-1):3*r, 1+3*(r-1):3*r) = Phi;
        Q_all(1+(r-1)*3,1+(r-1)*3)= Q_all(1+(r-1)*3,1+(r-1)*3)*cos(thetatmp)^2;
        Q_all(2+(r-1)*3,2+(r-1)*3) = Q_all(2+(r-1)*3,2+(r-1)*3)*sin(thetatmp)^2;
    end
    P_all = Phi_all * P_all * Phi_all' + dt_sec^2*Q_all;
    
    % 2. State update with ranging data
    H = zeros(nRange, 3*nRobots);
    y_k = nan(nRange,1);
    y_k_p = nan(nRange,1);
    for i = 1:nRange
        % range data in datasets is 100Hz
        if dataOri == 1
            y_k(i) = range_m(i).range(k);
        elseif dataOri == 2
            if 5*k+2 > length(range_m(1).t)
                break;
            end
            y_k(i) = mean(range_m(i).range(5*k-2:5*k+2));
        end
        id1 = range_m(i).pair(1); 
        id2 = range_m(i).pair(2); 
        [lia1, locb1] = ismember(id1, robotIDs);
        [lia2, locb2] = ismember(id2, robotIDs);
        if lia1==1 && lia2==1 % robot-robot range
            dxy = [state_est(locb1).x_ - state_est(locb2).x_; ...
                   state_est(locb1).y_ - state_est(locb2).y_];
            y_k_p(i) = norm(dxy);
            e = dxy' ./ y_k_p(i);
            H(i, 1+3*(locb1-1):3*locb1-1) = e;
            H(i, 1+3*(locb2-1):3*locb2-1) = -e;
        elseif lia1==1 && lia2~=1
            [~, locb2] = ismember(id2, anchorIDs);
            dxy = [state_est(locb1).x_ - anchor(locb2).x_est; ...
                   state_est(locb1).y_ - anchor(locb2).y_est];
            y_k_p(i) = norm(dxy);
            e = dxy' ./ y_k_p(i);
            H(i, 1+3*(locb1-1):3*locb1-1) = e;
        elseif lia1~=1 && lia2==1
            [~, locb1] = ismember(id1, anchorIDs);
            dxy = [state_est(locb2).x_ - anchor(locb1).x_est; ...
                   state_est(locb2).y_ - anchor(locb1).y_est];
            y_k_p(i) = norm(dxy);
            e = dxy' ./ y_k_p(i);
            H(i, 1+3*(locb2-1):3*locb2-1) = e;
        else
            error('Not implemented');
        end
        
%         if (r2 <= nRobots)
%             dxy = [state_est(r1).x_ - state_est(r2).x_; ...
%                    state_est(r1).y_ - state_est(r2).y_];
%             y_k_p(i) = norm(dxy);
%             e = dxy' ./ y_k_p(i);
%             H(i, 1+3*(r1-1):3*r1-1) = e;
%             H(i, 1+3*(r2-1):3*r2-1) = -e;
%         else
%             dxy = [state_est(r1).x_ - anchor(r2-nRobots).x_est; ...
%                    state_est(r1).y_ - anchor(r2-nRobots).y_est];
%             y_k_p(i) = norm(dxy);
%             e = dxy' ./ y_k_p(i);
%             H(i, 1+3*(r1-1):3*r1-1) = e;
%         end
    end
    R = diag(sigma_uwb^2 * ones(nRange, 1));
    
    % absolute heading
    for r = 1:nRobots
        if heading(r,k) ~= 0
            y_k = [y_k; heading(r,k)];
            y_k_p = [y_k_p; state_est(r).phi_];
            tmpJ = zeros(1,3*nRobots);
            tmpJ(3*r) = 1;
            H = [H; tmpJ];
            R_heading = 1;
            R = blkdiag(R,R_heading);
        end
    end

    K = P_all*H'*(H*P_all*H' + R)^-1;
    dr = y_k - y_k_p;
    delta_x = K*dr;
    
    % Correct
    for r = 1:nRobots
        state_est(r) = state_est(r).correct(delta_x(1+3*(r-1):3*r));
            
        % Save the stuffs 
        vState_est(r,:,k) = state_est(r).serialize;
    end
    %P_all = P_all - K*H*P_all;   
    P_all = (eye(3*nRobots)-K*H)*P_all*(eye(3*nRobots)-K*H)' + K*R*K';
end
%% 
% Plot estimation results
figure; hold on; axis equal; grid minor;
c_map = hsv(nRobots+1);
% ind = 1:200;
for r = 1:nRobots
%     x_est = reshape(vState_est(r,1,:),[1,sim_len]); 
%     y_est = reshape(vState_est(r,2,:),[1,sim_len]);
    legendstr{r} = ['Robot #', num2str(robotIDs(r))];
    %plot(x_est(ind), y_est(ind), '.','Color',c_map(r,:));
%     traj(r) = animatedline(x_est,y_est,'Color','r','LineWidth',3);
end
for a = 1:nAnchors
    anchor_xest(a) = anchor(a).x_est;
    anchor_yest(a) = anchor(a).y_est;
    text(anchor(a).x_est, anchor(a).y_est, ['Anchor #', num2str(anchorIDs(a))],'fontsize',16,...
         'VerticalAlignment','bottom','horizontalalign','center');
end
plot(anchor_xest, anchor_yest ,'s','MarkerSize',10,...
        'MarkerEdgeColor',c_map(nRobots+1,:),...
        'MarkerFaceColor',c_map(nRobots+1,:),...
        'DisplayName','Anchors');
title('Estimated trajectories');
traj1 = animatedline('Color',c_map(1,:),'DisplayName', ['Robot #', num2str(robotIDs(1))]);
traj2 = animatedline('Color',c_map(2,:),'DisplayName', ['Robot #', num2str(robotIDs(2))]);
legend show
xlabel('x(m)'); ylabel('y(m)'); %title('Collaborative localization');
for k = 1:sim_len
    addpoints(traj1, vState_est(1, 1, k),vState_est(1, 2, k));
    addpoints(traj2, vState_est(2, 1, k),vState_est(2, 2, k));
    drawnow
end

% plot results
errorpos_est = nan(1,nRobots);
errorphi_est = nan(1,nRobots);
errorpos_int = nan(1,nRobots);
errorphi_int = nan(1,nRobots);


for r = 1:nRobots
    phi_ini = state_ini(r,3) - robot(r).phi0;
    R = [cos(phi_ini) -sin(phi_ini);
         sin(phi_ini) cos(phi_ini)];
    t = state_ini(r,1:2)' - R*[robot(r).x0, robot(r).y0]';
%     R = eye(2);
%     t = [0;0];
    
    x = reshape(vState_int(r,1,:),[1,sim_len]); 
    y = reshape(vState_int(r,2,:),[1,sim_len]);
    phi = reshape(vState_int(r,3,:),[1,sim_len]);
    xy_int = R*[x;y] + t;
    x_int = xy_int(1,:);
    y_int = xy_int(2,:);
    phi_int = phi + phi_ini;
    phi_int(phi_int > pi) = phi_int(phi_int > pi) - 2*pi;
    phi_int(phi_int < -pi) = phi_int(phi_int < -pi) + 2*pi;
    
    x_est = reshape(vState_est(r,1,:),[1,sim_len]); 
    y_est = reshape(vState_est(r,2,:),[1,sim_len]);
    phi_est = reshape(vState_est(r,3,:),[1,sim_len]);

%     errorpos_est(r) = sqrt(mean(vecnorm([x_est - robot(r).x; y_est - robot(r).y]).^2));
%     errorphi_est(r) = sqrt(mean(abs(phi_est - robot(r).phi).^2));
%     errorpos_int(r) = sqrt(mean(vecnorm([x_int - robot(r).x; y_int - robot(r).y]).^2));
%     errorphi_int(r) = sqrt(mean(abs(phi_int - robot(r).phi).^2));
    x = robot(r).x';
    y = robot(r).y';
    phi = robot(r).phi;
    xy_gt = R*[x;y] + t;
    x_gt = xy_gt(1,:);
    y_gt = xy_gt(2,:);
    phi_gt = phi + phi_ini;
    phi_gt(phi_gt > pi) = phi_gt(phi_gt > pi) - 2*pi;
    phi_gt(phi_gt < -pi) = phi_gt(phi_gt < -pi) + 2*pi;

    figure; grid minor;
    subplot(2,3,[1 2 3]); hold on;
    plot(x_gt, y_gt,'-g');
    plot(x_gt(1), y_gt(1),'-s','MarkerSize',10,...
        'MarkerEdgeColor','green',...
        'MarkerFaceColor','green');
    plot(x_int, y_int,'k');
    plot(x_est, y_est,'r');
    xlabel('x(m)'); ylabel('y(m)')
    legend('groundtruth','start point','odometry','estimated');
    title(['trajectory of robot ' num2str(robot(r).id)]);
    subplot(2,3,4); hold on;
    plot(1:10:10*length(x_gt)-1,x_gt,'g.');
%     plot(robot(r).x,'g.');
    plot(x_int,'k.');
    plot(x_est,'r.');
    ylabel('m');
    legend('groundtruth','odometry','est');
    title('x');
    subplot(2,3,5); hold on;
    plot(1:10:10*length(y_gt)-1,y_gt,'g.');
%     plot(robot(r).y,'g.');
    plot(y_int,'k.');
    plot(y_est,'r.');
    ylabel('m');
    legend('groundtruth','odometry','est');
    title('y');
    subplot(2,3,6); hold on;
    plot(1:10:10*length(phi_gt)-1,phi_gt,'g.');
%     plot(robot(r).phi,'g.');
    plot(phi_int,'k.');
    plot(phi_est,'r.');
    ylabel('rad');
    legend('groundtruth','odometry','est');
    title('yaw');
end
clear x_int y_int phi_int 

% plot all trajectories with groundtruth
figure; hold on; axis equal; grid minor;
h = zeros(1,2*nRobots+nAnchors); % plot handles
legendstr = cell(1,2*nRobots);
c_map = hsv(nRobots+1);
for r = 1:nRobots
    h(2*(r-1)+1) = plot(robot(r).x, robot(r).y,'Color',c_map(r,:));
    x_est = reshape(vState_est(r,1,:),[1,sim_len]); 
    y_est = reshape(vState_est(r,2,:),[1,sim_len]);
    h(2*r) = plot(x_est(1:10:end), y_est(1:10:end), 'x','Color',c_map(r,:),...
            'LineWidth',2,...
            'MarkerSize',6);
    
    legendstr{2*r-1} = strcat("groundtruth ",num2str(robot(r).id));
    legendstr{2*r} = strcat("estimates ",num2str(robot(r).id));
end
for a = 1:nAnchors
    h(2*nRobots + nAnchors) = plot(anchor(a).x_est, anchor(a).y_est ,'-s','MarkerSize',10,...
        'MarkerEdgeColor',c_map(nRobots+1,:),...
        'MarkerFaceColor',c_map(nRobots+1,:));
    text(anchor(a).x, anchor(a).y - 0.4, ['Anchor ', num2str(a)],'fontsize',16,...
         'VerticalAlignment','bottom','horizontalalign','center');
end
legend(h(1:2*nRobots),cellstr(legendstr),'NumColumns',2,'Orientation','horizontal');
xlabel('x(m)'); ylabel('y(m)'); %title('Collaborative localization');

% plot errors
figure; grid minor;
error = [errorpos_est;errorpos_int]';
hB = bar(error);
xlabel('Vehicles'); ylabel('RMSE(m)'); %title('position error');
barWidth = hB.BarWidth;
hT = [];
for i=1:length(hB) 
  hT = [hT text(hB(i).XData+hB(i).XOffset,hB(i).YData,num2str(hB(i).YData.','%.3f'), ...
        'VerticalAlignment','bottom','horizontalalign','center', 'fontsize', 16)];    
end
legend('Collaboration','Odometry');

%% test
a0 = [anchor(1).x_est, anchor(1).y_est];
a2 = [anchor(2).x_est, anchor(2).y_est];
r1 = [vState_est(1,1,end-1),vState_est(1,2,end-1)];
r4 = [vState_est(2,1,end-1),vState_est(2,2,end-1)];
d04=norm(a0-r4)
d01=norm(a0-r1)
d02=norm(a0-a2)
d12=norm(r1-a2)
d42=norm(r4-a2)
d14=norm(r1-r4)