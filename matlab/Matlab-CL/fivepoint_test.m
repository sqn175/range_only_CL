
clc; close all; clear all;

set(0,'defaultAxesFontSize',18)
% traj1 = [0,0; 0.03,0.01; 0.05,0.18; 0.14,0.24; 0.2,0.3]';
% traj2 = [0,0; 0.09,0.01; 0.14,0.02; 0.07,0.05; 0.1, 0.1]';
traj1 = [0,0; 0.3,0.5; 1.2,1; 3,1.9; 5,2.1]';
traj2 = [0,0; -1,-0.5; 0.5,-1; 1.2, -2; 2, -0.1]';
figure; hold on;
scatter(traj1(1,:), traj1(2,:), '*');
scatter(traj2(1,:), traj2(2,:), 'x');

% phi = 0.3
phi = [0 0 0.3]; 
phi(3)
% C12 = [cos(phi) sin(phi); -sin(phi) cos(phi)];
R12 = Kinematics.eulerZXY2rotm(phi);
C12 = R12(1:2,1:2);

theta = -0.8; % rad
rho = 1;
p21 = rho*[cos(theta);sin(theta)]

len = length(traj1);

for i = 1:len
    traj2_w(:,i) = C12 *traj2(:,i) + p21;
end

figure; hold on;
scatter(traj1(1,:), traj1(2,:), '*');
scatter(traj2_w(1,:), traj2_w(2,:), 'x');
legend('robot0', 'robot1');

% inter-robot distance measurements
pd = zeros(2, len);
for i = 1:len
    pd(:,i) = traj2_w(:,i) - traj1(:,i);
end
d = vecnorm(pd);
sigma = 0.1;
d = d + random('norm',0,sigma,1,length(d));
[phi_m, p21_m] = fivepoint(traj1, traj2, d)



