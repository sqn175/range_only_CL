% test Quaternion class
w = 0.685460562; x = 0.0; y = 0.0; z= 0.728109757;
q = Quaternion([w x y z]); q = q.unit;
euler(1) = atan2(2*(w*x+y*z), 1-2*(x^2 + y^2));
euler(2) = asin(2*(w*y-z*x));
euler(3) = atan2(2*(w*z+x*y), 1-2*(z^2+y^2));

% Define an absolute tolerance
tol = 1e-8; 

%% Test 1: quaternion to euler angles
angles = Kinematics.rotm2eulerZXY(t2r(q2tr(q))');
assert(abs(angles(1)-euler(1)) <= tol)
assert(abs(angles(2)-euler(2)) <= tol)
assert(abs(angles(3)-euler(3)) <= tol)