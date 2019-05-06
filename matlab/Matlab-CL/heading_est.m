function phi = heading_est(d, baseline)
    pos = zeros(2,2);
    for i = 1:2
        cos_alpha = (d(i,1)^2 + baseline^2 - d(i,2)^2) / (2*baseline*d(i,1));
        if cos_alpha > 1
            cos_alpha = 1;
        elseif cos_alpha < -1
            cos_alpha = -1;
        end
        alpha = acos(cos_alpha);
        pos(i,:) = [d(i,1)*cos_alpha, d(i,1)*sin(alpha)];
    end
       
    dxy = [pos(2,2) - pos(1,2), pos(2,1) - pos(1,1)];
    phi = atan2(dxy(1), dxy(2));
end