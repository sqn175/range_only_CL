function [state_ini, anchor_pos] = initializer(robot, range_m)
    % During initialization, the robots remain static, we use the range
    % measurements to determine the topology of the robots and the static
    % anchors. A MDS method is used. We then use the 5point method to
    % determine the relative pose of the robots. 
    
    % MDS
    n = size(robot, 2) + 2; % 2 static UWB anchors
    nRobot = size(robot,2);
    dt_sec = range_m(1).t(2) - range_m(1).t(1);
    irange = 1;
    M = zeros(n);
    
    range_tmp = range_m;
    range_m(1) = range_tmp(6);
    range_m(2) = range_tmp(2);
    range_m(3) = range_tmp(4);
    range_m(4) = range_tmp(3);
    range_m(5) = range_tmp(5);
    range_m(6) = range_tmp(1);
    
    for i = 1:n-1
        for j = i+1 : n
            M(i,j) = mean(range_m(irange).range(1:1000));
            M(j,i) = M(i,j);
            irange = irange + 1;
        end
    end
    
    % test
    M = [0, 2.03334, 1.01929, 2.57519;
				2.03334, 0000000, 2.14298, 2.04346;
				1.01929, 2.14298, 0000000, 1.66843;
				2.57519, 2.04346, 1.66843, 0000000];
    % test end
    [Loc, iter] = MDS_Adam(M, 1e-5, 500);
    
    % figure test
    figure; hold on; title('MDS-Adam');axis equal
    ids = [0 2 1 4];
    for i = 1:4
        plot(Loc(i,1),Loc(i,2),'.','DisplayName',num2str(ids(i)));
    end
    legend show
    % figure test end
        
    state_ini = zeros(nRobot, 3);
    for i = 1:nRobot
        state_ini(i,1:2) = Loc(i+2,:);
    end
    anchor_pos = Loc(1:2, :);
    
    % Ԥ���趨��
%     state_ini(1,3) = -0.4468;
%     state_ini(2,3) = -0.1145;
%     state_ini(1,3) = -0.3276;
%     state_ini(2,3) = -0.1027;
    state_ini(1,3) = -2.971;
    state_ini(2,3) = -3.079;

    % 5 point
    % Integration
%     int_len = 2100;
%     vState_int = nan(nRobot,3,int_len);
%     for r = 1:nRobot
%         state_int = State(0, 0, 0);
%         vState_int(r,:,1) = state_int.serialize;
%         for k = 2:int_len
%             [~, state_int] = state_int.propagate(robot(r).v_m(k-1), ...
%                 robot(r).omega_m(k-1),robot(r).v_m(k), robot(r).omega_m(k), ...
%                 dt_sec, false);
%             vState_int(r,:,k) = state_int.serialize;
%         end
%     end

    % Extrinsic calibration 
    % Extrinsic calibration for robot 1 and 2
%     p_int_1 = [reshape(vState_int(1,1,:),[1,int_len]);...
%                reshape(vState_int(1,2,:),[1,int_len])]; 
%     p_int_2 = [reshape(vState_int(2,1,:),[1,int_len]);...
%                reshape(vState_int(2,2,:),[1,int_len])];
%     p_int_3 = [reshape(vState_int(3,1,:),[1,int_len]);...
%                 reshape(vState_int(3,2,:),[1,int_len])];
%     ind = 1:20:81;
%     [phi_m, p_m] = fivepoint(p_int_1(:,ind), p_int_2(:,ind), ...
%                 range_m(1).range(ind));
%     phi_gt = robot(2).phi0 /pi * 180
%     phi_m = phi_m / pi *180
%     p_gt = [robot(2).x0 robot(2).y0]
%     p_m
%     ind = 1:500:2001;
%     [phi_m, p_m] = fivepoint(p_int_1(:,ind), p_int_3(:,ind), ...
%                 range_m(2).range(ind));
%     phi_gt = robot(3).phi0 /pi * 180
%     phi_m = phi_m / pi *180
%     p_gt = [robot(3).x0 robot(3).y0]
%     p_m
end