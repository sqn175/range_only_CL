function [roll, pitch, yaw] = quart2rpy(quart)
    n = size(quart, 1);
    roll = nan(n,1); pitch = nan(n,1); yaw = nan(n,1);
    for i =1:n
        w = quart(i,1);
        x = quart(i,2);
        y = quart(i,3);
        z = quart(i,4);

        roll(i) = atan2(2*(w*x+y*z), 1-2*(x*x+y*y));
        pitch(i)= asin(2*(w*y-z*x));
        yaw(i) = atan2(2*(w*z+x*y), 1-2*(z*z+y*y));
    end
end