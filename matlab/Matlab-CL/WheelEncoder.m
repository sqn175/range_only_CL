function [theta, v_norm, omega_z] = WheelEncoder(p,v,a,j, sigma_v, sigma_omega)
sim_len = length(v);

% Speed along the path
v_norm = vecnorm(v);

if sum(v_norm) == 0 % static cast
    theta = zeros(1,sim_len);
    omega_z = zeros(1,sim_len);
elseif sum(vecnorm(a) == 0) % linear velocity
    theta = atan2(v(2,1), v(1,1))*ones(1,sim_len);
    omega_z = zeros(1,sim_len);
else
    T = v ./ v_norm; % for check
    
    a_cross_v = cross(a,v);
    v_cross_a = cross(v,a);

    N = cross(v, a_cross_v) ./ (v_norm .* vecnorm(a_cross_v));
    B = v_cross_a ./ (vecnorm(v_cross_a));

    euler = nan(3,sim_len);
    for i = 1:sim_len
        TNB(:,:,i) = [T(:,i) N(:,i) B(:,i)]';
    %     disp(TNB'*TNB)
        euler(:,i) =  Kinematics.rotm2eulerZXY(TNB(:,:,i));
    end
    theta = euler(3,:);

    kappa = vecnorm(v_cross_a) ./ (v_norm.^3);
    tau = dot(v_cross_a, j) ./ (vecnorm(v_cross_a).^2);

    % Angular velocity expressed in the Frenet-Serret frame
    omega = [v_norm .* tau; zeros(1,sim_len); v_norm .* kappa];
    omega_z = omega(3,:);
end

% add noise
v_noise = random('Normal', 0, sigma_v, 1, sim_len);
omega_noise = random('Normal', 0, sigma_omega, 1, sim_len);

v_norm = v_norm + v_noise;
omega_z = omega_z + omega_noise;
% 
% subplot(1,4,[1 2]); hold on;
% plot3(p(1,2:end),p(2,2:end), p(3,2:end),'k.');
% plot3(p(1,1),p(2,1), p(3,1),'-s','MarkerSize',10,...
%     'MarkerEdgeColor','black',...
%     'MarkerFaceColor','black');
% ind = 1:50:sim_len;
% quiver3(p(1,ind),p(2,ind), p(3,ind),...
%     reshape(TNB(1,1,ind),[1, length(ind)]),...
%     reshape(TNB(1,2,ind),[1, length(ind)]),...
%     reshape(TNB(1,3,ind),[1, length(ind)]),'g');
% quiver3(p(1,ind),p(2,ind),p(3,ind),...
%     reshape(TNB(2,1,ind),[1, length(ind)]),...
%     reshape(TNB(2,2,ind),[1, length(ind)]),...
%     reshape(TNB(2,3,ind),[1, length(ind)]),'b');
% quiver3(p(1,ind),p(2,ind), p(3,ind),...
%     reshape(TNB(3,1,ind),[1, length(ind)]),...
%     reshape(TNB(3,2,ind),[1, length(ind)]),...
%     reshape(TNB(3,3,ind),[1, length(ind)]),'r');
% daspect([1 1 1])
% xlabel('x/m'); ylabel('y/m'); zlabel('z/m');
% title('Trajectory with frenet frame');
% 
% subplot(1,4,3); hold on;
% plot(kappa); title('\kappa, curvature');
% 
% subplot(1,4,4); hold on;
% plot(tau); title('\tau, torsion');
end