function [p, v, a, j] = trajectoryGen(t, id)
sim_len = length(t);
switch id
    case 1
        r1 = 4;  m = 0.2*pi;
        px = r1*sin(m*t);
        vx = m*r1*cos(m*t);
        ax = -m*m*r1*sin(m*t);
        jx = -m*m*m*r1*cos(m*t);

        r2 = 2.6; n = 0.12*pi;
        py = -r2*cos(n*t) +r2*cos(n*t(1)); 
        vy = n*r2*sin(n*t); 
        ay = n*n*r2*cos(n*t);
        jy = -n*n*n*r2*sin(n*t); 

        pz = zeros(1,sim_len);
        vz = zeros(1,sim_len);
        az = zeros(1,sim_len);
        jz = zeros(1,sim_len);

        % Expressed in world reference frame
        p = [px; py; pz]; % Position
        v = [vx; vy; vz]; % velocity, derivative of position
        a = [ax; ay; az]; % accelation, derivative of velocity
        j = [jx; jy; jz]; % jerk, derivative of accelaration
    case 2
        r1 = 2;  m = -0.1*pi;
        px = r1*sin(m*t+pi/3);
        vx = m*r1*cos(m*t+pi/3);
        ax = -m*m*r1*sin(m*t+pi/3);
        jx = -m*m*m*r1*cos(m*t+pi/3);

        r2 = 6; n = 0.1*pi;
        py = -r2*cos(n*t+pi/3); 
        vy = n*r2*sin(n*t+pi/3); 
        ay = n*n*r2*cos(n*t+pi/3);
        jy = -n*n*n*r2*sin(n*t+pi/3); 

        pz = zeros(1,sim_len);
        vz = zeros(1,sim_len);
        az = zeros(1,sim_len);
        jz = zeros(1,sim_len);

        % Expressed in world reference frame
        p = [px; py; pz]; % Position
        v = [vx; vy; vz]; % velocity, derivative of position
        a = [ax; ay; az]; % accelation, derivative of velocity
        j = [jx; jy; jz]; % jerk, derivative of accelaration
    case 4
        
        px = zeros(1,sim_len) - 2;
        vx = zeros(1,sim_len);
        ax = zeros(1,sim_len);
        jx = zeros(1,sim_len);

        py =  -3 + 0.1*t; 
        vy = 0.01*ones(1,sim_len); 
        ay = zeros(1,sim_len);
        jy = zeros(1,sim_len); 

        pz = zeros(1,sim_len);
        vz = zeros(1,sim_len);
        az = zeros(1,sim_len);
        jz = zeros(1,sim_len);

        % Expressed in world reference frame
        p = [px; py; pz]; % Position
        v = [vx; vy; vz]; % velocity, derivative of position
        a = [ax; ay; az]; % accelation, derivative of velocity
        j = [jx; jy; jz]; % jerk, derivative of accelaration
    case 3
        r1 = 5;  m = 0.03*pi;
        px = r1*sin(m*t-pi/3);
        vx = m*r1*cos(m*t-pi/3);
        ax = -m*m*r1*sin(m*t-pi/3);
        jx = -m*m*m*r1*cos(m*t-pi/3);

        r2 = 5; n = 0.03*pi;
        py = -r2*cos(n*t-pi/3); 
        vy = n*r2*sin(n*t-pi/3); 
        ay = n*n*r2*cos(n*t-pi/3);
        jy = -n*n*n*r2*sin(n*t-pi/3); 

        pz = zeros(1,sim_len);
        vz = zeros(1,sim_len);
        az = zeros(1,sim_len);
        jz = zeros(1,sim_len);

        % Expressed in world reference frame
        p = [px; py; pz]; % Position
        v = [vx; vy; vz]; % velocity, derivative of position
        a = [ax; ay; az]; % accelation, derivative of velocity
        j = [jx; jy; jz]; % jerk, derivative of accelaration
    otherwise
        error(['Trajectory of robot ', num2str(id), ' not defined.']);
end