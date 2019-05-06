% Shi Qin 201880/03/28, not tested
function euler = q2ZYXeuler(q)
    euler = zeros(1,3);
    q = double(q);
    s = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    
    euler(1) = atan2(2*(s*x+y*z), 1-2*(x^2+y^2));
    euler(2) = asin(2*(s*y-z*x));
    euler(3) = atan2(2*(s*z+x*y), 1-2*(y^2+z^2));
end