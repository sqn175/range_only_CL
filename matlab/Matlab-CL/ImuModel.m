classdef ImuModel
    properties
        acc_cov_;    % acceleration white Gaussion noise covariance
        gyro_cov_;   % gyroscope white Gaussion noise covariance
        b_acc_cov_;  % gyroscope bias white Gaussion noise covariance
        b_gyro_cov_; % gyroscope bias white Gaussion noise covariance
        b_a0_;
        b_g0_;
    end
    
    methods  
        function o = ImuModel(acc_cov, gyro_cov, b_acc_cov, b_gyro_cov, b_a0, b_g0)
            validateattributes(acc_cov, {'double'}, {'nonnegative', 'scalar'});
            validateattributes(gyro_cov, {'double'}, {'nonnegative', 'scalar'});
            validateattributes(b_acc_cov, {'double'}, {'nonnegative', 'scalar'});
            validateattributes(b_gyro_cov, {'double'}, {'nonnegative', 'scalar'});
            
            o.acc_cov_ = acc_cov;
            o.gyro_cov_ = gyro_cov;
            o.b_acc_cov_ = b_acc_cov;
            o.b_gyro_cov_ = b_gyro_cov;
            o.b_a0_ = b_a0;
            o.b_g0_ = b_g0;
        end
        
        function [acc_noise, gyro_noise] = gennoise(o, len)
            acc_noise  = random('Normal', 0, sqrt(o.acc_cov_), 3, len);
            gyro_noise = random('Normal', 0, sqrt(o.gyro_cov_), 3, len);
        end
        
        function [b_a, b_g] = genbias(o, len, step)
            validateattributes(len, {'numeric'}, {'scalar','integer','positive'});
            validateattributes(step, {'double'}, {'positive', 'scalar'});
            
            b_a_noise = random('Normal', 0, sqrt(o.b_acc_cov_), 3, len);
            b_g_noise = random('Normal', 0, sqrt(o.b_gyro_cov_), 3, len);
            b_a = zeros(3,len);
            b_g = zeros(3,len);
            b_a(:,1) = o.b_a0_;
            b_g(:,1) = o.b_g0_;
            for i = 2:len
                rate_a = (b_a_noise(:,i) + b_a_noise(:,i-1)) / 2;
                b_a(:,i) = b_a(:,i-1) + step*rate_a;
                
                rate_g = (b_g_noise(:,i) + b_g_noise(:,i-1)) / 2;
                b_g(:,i) = b_g(:,i-1) + step*rate_g;
            end
        end
        
        function cov = covm(o)
            cov = diag([o.acc_cov_*ones(3,1); o.gyro_cov_*ones(3,1); ...
                       o.b_acc_cov_*ones(3,1); o.b_gyro_cov_*ones(3,1)]);
        end
        
        function sigmas2 = sigmas2(o)
            sigmas2 = [o.acc_cov_ o.gyro_cov_ o.b_acc_cov_ o.b_gyro_cov_];
        end
        
        function [acc_m, gyro_m] = genImuMeas(o, a, euler, euler_rate, freq)
            len = length(a);
            acc_true = zeros(3,len);
            gyro_true = zeros(3,len);
            g = [0 0 -9.8]';
            for i = 1:len
                C_bn = Kinematics.eulerZXY2rotm(euler(:,i));
                acc_true(:,i) = C_bn * (a(:,i) - g);

                J = Kinematics.eulerZXY2jm(euler(:,i));
                gyro_true(:,i) = J * euler_rate(:,i);
            end
            
            [acc_noise, gyro_noise] = o.gennoise(len);
            [b_a, b_g] = o.genbias(len, 1/freq);
            acc_m = acc_true + b_a + acc_noise;
            gyro_m = gyro_true + b_g + gyro_noise;
        end
    end
    
end