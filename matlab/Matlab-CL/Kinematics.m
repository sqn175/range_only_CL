classdef Kinematics
    methods(Static = true)
    
        function rotm = eulerZXY2rotm(eul)
            % converts ZXY sequence Euler angles in radians, eul, to the corresponding ..
            % rotation matrix, rotm. When using the rotation matrix, ..
            % premultiply it with the coordinates to be rotated (as opposed to postmultiplying) 
            x = eul(1); % pitch, rotation angle around x-axis
            y = eul(2); % roll, y-axis
            z = eul(3); % yaw, z-axis
            sx = sin(x); 
            cx = cos(x);
            sy = sin(y);
            cy = cos(y);
            sz = sin(z);
            cz = cos(z);
            % Z-X-Y euler angle sequence 
            rotm = zeros(3,3);
            rotm(1,1) = cz*cy - sz*sx*sy;
            rotm(1,2) = sz*cy + cz*sx*sy;
            rotm(1,3) = -cx*sy;
            rotm(2,1) = -sz*cx;
            rotm(2,2) = cz*cx;
            rotm(2,3) = sx;
            rotm(3,1) = cz*sy + sz*sx*cy;
            rotm(3,2) = sz*sy - cz*sx*cy;
            rotm(3,3) = cx*cy;
        end
        
        function jm = eulerZXY2jm(eul)
            % Input: euler angles
            % Output: The transition matrix from Euler rates to the 
            % body-frame angular velocity vector.
            x = eul(1); % pitch, rotation angle around x-axis
            y = eul(2); % roll, y-axis
            sx = sin(x); 
            cx = cos(x);
            sy = sin(y);
            cy = cos(y);
            % Z-X-Y euler angle sequence 
            jm = [cy 0 -sy*cx;
                  0 1 sx;
                  sy 0 cx*cy];
        end
        
        function rotm = rodrigues(u, theta)
        % Inputs: 
        %    k - Rotation axis (does not need to be unit vector)
        %    theta - Rotation angle in radians; positive according to 
        %           right-hand (screw) rule
        %  Outputs:
        %    rotm - corresponding rotation matrix
            rotm = cos(theta)*eye(3) + sin(theta)*skew(theta) + ...
                u*u'*(1-cos(theta));
        end
        
        % https://www.geometrictools.com/Documentation/EulerAngles.pdf
        function euler = rotm2eulerZXY(R)
            R = R';
            if (R(3,2) < 0.998)
                if (R(3,2) > -0.998)
                    x = asin(R(3,2));
                    z = atan2(-R(1,2), R(2,2));
                    y = atan2(-R(3,1), R(3,3));
                else % Singularity
                    disp('singularity');
                    x = -pi/2;
                    z = -atan2(R(1,3), R(1,1));
                    y = 0;
                end
            else % Singularity
                 disp('singularity');
                 x = pi/2;
                 z = atan2(R(1,3), R(1,1));
                 y = 0;
            end
            euler = [x y z];
        end
        
    end
end