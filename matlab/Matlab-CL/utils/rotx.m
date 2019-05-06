%ROTX Rotation about X axis
%
% R = ROTX(THETA) is an SO(3) rotation matrix (3x3) representing a rotation of THETA 
% radians about the x-axis.
%
% R = ROTX(THETA, 'deg') as above but THETA is in degrees.
%
% See also ROTY, ROTZ, ANGVEC2R, ROT2.


function R = rotx(t, deg)

    if nargin > 1 && strcmp(deg, 'deg')
        t = t *pi/180;
    end
    
    ct = cos(t);
    st = sin(t);
    R = [
        1   0     0
        0   ct    st
        0   -st   ct
        ];