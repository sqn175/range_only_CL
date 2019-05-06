%ROTY Rotation about Y axis
%
% R = ROTY(THETA) is an SO(3) rotation matrix (3x3) representing a rotation of THETA 
% radians about the y-axis.
%
% R = ROTY(THETA, 'deg') as above but THETA is in degrees.
%
% See also ROTX, ROTZ, ANGVEC2R, ROT2.


function R = roty(t, deg)
    if nargin > 1 && strcmp(deg, 'deg')
        t = t *pi/180;
    end
    ct = cos(t);
    st = sin(t);
    R = [
        ct  0   -st
        0   1   0
       st  0   ct
       ];