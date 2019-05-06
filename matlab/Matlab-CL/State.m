
classdef State
    properties
        % Navigation states
        x_;   % x coordinate expressed in world reference
        y_;   % y coordinate expressed in world reference
        phi_;   % orientation
    end
    
    methods
        function o = State(x, y, phi)
            if nargin == 0
                % Provide values for superclass constructor
                % and initialize other inputs
                o.x_ = NaN;
                o.y_ = NaN;
                o.phi_ = NaN;
            elseif nargin == 1
                validateattributes(x, {'double'}, {'column','nrows',3});
                o.x_ = x(1);
                o.y_ = x(2);
                o.phi_ = x(3);
            else
                validateattributes(x, {'double'}, {'scalar'});
                validateattributes(y, {'double'}, {'scalar'});
                validateattributes(phi, {'double'}, {'scalar'});
                o.x_ = x;
                o.y_ = y;
                o.phi_ = phi;
            end
        end
        
        function s = char(o)
            s = ['[x y phi]: [' num2str(o.x_)  ',' num2str(o.y_) ...
                 ',' num2str(o.phi_),']'];
        end
        
        function disp(o)
            disp(char(o))
        end
        
        function [Phi, s] = propagate(o, v_0, omega_0, v_1, omega_1, delta_sec, Phi_flag)
            int_method = 2;
            Phi = eye(3);
            % PROPAGATE propagate the states   
            if int_method == 1  % zeroth order integration
                state_der = o.dot(v_0, omega_0);
                s = o + state_der * delta_sec;
                if Phi_flag
                    Phi = delta_sec * jacobian(o, v_0, omega_0);
                end
            elseif  int_method == 2 % midpoint
                v_mid = (v_0 + v_1) / 2;
                omega_mid = (omega_0 + omega_1) / 2;
                state_der = o.dot(v_mid, omega_mid);
                s = o + state_der * delta_sec;
                if Phi_flag
                    Phi = delta_sec * jacobian(o, v_mid, omega_mid);
                end
            end
        end
        
        function o = correct(o, delta)
            o.x_ = o.x_ + delta(1);
            o.y_ = o.y_ + delta(2);
            phi = o.phi_ + delta(3);
            if phi > pi
                phi = phi - 2*pi;
            elseif phi < -pi
                phi = phi + 2*pi;
            end
            o.phi_ = phi;
        end
        
        function Fc = jacobian(o, v_m, omega_m)
            % error state jacobian
            Fc = [ 0 0 -v_m*sin(o.phi_);
                   0 0 v_m*cos(o.phi_);
                   0 0 0               ];
        end
        
        function s = mtimes(s1, s2)
            s = State();
            if isa(s1, 'State') && isa(s2, 'double')
                s.x_ = s1.x_ * s2;
                s.y_ = s1.y_ * s2;
                s.phi_ = s1.phi_ * s2;
            elseif isa(s1, 'double') && isa(s2, 'State')
                s.x_ = s2.x_ * s1;
                s.y_ = s2.y_ * s1;
                s.phi_ = s2.phi_ * s1;
            else
                error('Not defined');
            end
        end
        
       function s = plus(s1, s2)
            s = State();
            if isa(s1, 'State') && isa(s2, 'State')
                s.x_ = s1.x_ + s2.x_;
                s.y_ = s1.y_ + s2.y_;
                phi = s1.phi_ + s2.phi_;
                if phi > pi
                    phi = phi - 2*pi;
                elseif phi < -pi
                    phi = phi + 2*pi;
                end
                s.phi_ = phi;
            else
                error('Not defined');
            end
        end
        
        % State derivative
        function od = dot(o, v_m, omega_m)
            od = State();
            
            % DO NOT change the Gravity value
            od.x_ = v_m * cos(o.phi_);
            od.y_ = v_m * sin(o.phi_);
            od.phi_ = omega_m;
        end
        
        function v = serialize(s)
            v = [s.x_; s.y_; s.phi_]; 
        end
        
    end
end



