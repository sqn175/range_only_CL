function [phi21, p21] = fivepoint(traj1, traj2, d)
% Calculate the relative pose of robot2 w.r.t robot1.
%
% function [phi21, p21] = fivepoint(traj1, traj2, d)
% Inputs:
%   traj1    :  2x5 odometry-based trajectory of robot1 expressed in local
%               frame.
%   traj2    :  2x5 odometry-based trajectory of robot2 expressed in local
%               frame.
%   d        :  1x5 distance measurements between robot1 and robot2.
%
% Outputs: relative pose
%   phi21    :  Relative orientation
%   p21      :  Relative position
%
% Ref: Zhou X S, Roumeliotis S I. Robot-to-robot relative pose estimation
%      from range measurements[J]. IEEE Transactions on Robotics, 2008, 
%      24(6): 1379.
%
% ---------------------------------------------------------
% Copyright (c) 2018, Qin Shi
% ---------------------------------------------------------

validateattributes(traj1, {'double'}, {'2d','ncols', 5});
validateattributes(traj2, {'double'}, {'2d','ncols', 5});
validateattributes(d, {'double'},{'vector'});
if length(d) ~= 5
    error('Input parameter d must have length of 5');
end

figure; hold on;
plot(traj1(1,:), traj1(2,:), '-*');
plot(traj1(1,:), traj2(2,:), '-x');
xlabel('x/m'); ylabel('y/m');
legend('robot0', 'robot1');

for i = 1:4
    po1(:,i) = traj1(:,i+1) - traj1(:,1);
end
for i = 1:4
    po2(:,i) = traj2(:,i+1) - traj2(:,1);
end

rho = d(1);
A = zeros(4, 7);
for i = 1:4
    A(i,1) = - dot(po1(:,i), po2(:,i));
    A(i,2) = po1(1,i)*po2(2,i) - po1(2,i)*po2(1,i);
    A(i,3) = - rho * po1(1,i);
    A(i,4) = - rho * po1(2,i);
    A(i,5) = rho * po2(1,i);
    A(i,6) = rho * po2(2,i);
    a0 = 0.5 * (d(i+1)^2 - rho^2 - dot(po1(:,i)', po1(:,i)) - dot(po2(:,i)', po2(:,i)) );
    A(i,7) = - a0;
end

xn = null(A, 'r');

r = xn(:,1);
s = xn(:,2);
t = xn(:,3);
a = zeros(6,3);
for i = 1:6
    a(i,1) = r(i) - t(i)*r(7)/t(7);
    a(i,2) = s(i) - t(i)*s(7)/t(7);
    a(i,3) = t(i)/t(7);
end

beta = zeros(5,5);
gamma = zeros(5,1);
for i = 1:3
    r1 = 2*i-1; r2 = 2*i;
    beta(i,1) = a(r1, 1)^2 + a(r2, 1)^2;
    beta(i,2) = a(r1, 2)^2 + a(r2, 2)^2;
    beta(i,3) = 2 * (a(r1, 1)*a(r1, 2) + a(r2, 1)*a(r2, 2));
    beta(i,4) = 2 * (a(r1, 1)*a(r1, 3) + a(r2, 1)*a(r2, 3));
    beta(i,5) = 2 * (a(r1, 2)*a(r1, 3) + a(r2, 2)*a(r2, 3));
    
    gamma(i) = 1 - a(r1, 3)^2 - a(r2, 3)^2;
end

beta(4,1) = a(1,1)*a(3,1) + a(2,1)*a(4,1);
beta(4,2) = a(1,2)*a(3,2) + a(2,2)*a(4,2);
beta(4,3) = a(1,1)*a(3,2) + a(1,2)*a(3,1) + a(2,1)*a(4,2) + a(2,2)*a(4,1);
beta(4,4) = a(1,1)*a(3,3) + a(1,3)*a(3,1) + a(2,1)*a(4,3) + a(2,3)*a(4,1) - a(5,1);
beta(4,5) = a(1,2)*a(3,3) + a(1,3)*a(3,2) + a(2,2)*a(4,3) + a(2,3)*a(4,2) - a(5,2);
gamma(4) = a(5,3) - a(1,3)*a(3,3) - a(2,3)*a(4,3);

beta(5,1) = a(1,1)*a(4,1) - a(2,1)*a(3,1);
beta(5,2) = a(1,2)*a(4,2) - a(2,2)*a(3,2);
beta(5,3) = a(1,1)*a(4,2) + a(1,2)*a(4,1) - a(2,1)*a(3,2) - a(2,2)*a(3,1);
beta(5,4) = a(1,1)*a(4,3) + a(1,3)*a(4,1) - a(2,1)*a(3,3) - a(2,3)*a(3,1) - a(6,1);
beta(5,5) = a(1,2)*a(4,3) + a(1,3)*a(4,2) - a(2,2)*a(3,3) - a(2,3)*a(3,2) - a(6,2);
gamma(5) = a(6,3) - a(1,3)*a(4,3) + a(2,3)*a(3,3);

lambdas = beta \ gamma;
lambda1 = lambdas(4);
lambda2 = lambdas(5);
lambda3 = (1 - lambda1*r(7) - lambda2*s(7)) / t(7);

x = lambda1 * r + lambda2 * s + lambda3 * t;

phi21 = asin(x(2));
theta_m = asin(x(4));
p21 = d(1)*[cos(theta_m) sin(theta_m)];
end