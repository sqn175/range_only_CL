function range_m = UWBRangeSim(sigma_uwb, robots, anchors)
    if nargin == 3
        nAnchors = length(anchors);
    elseif nargin == 2
        nAnchors = 0;
    end
    nRobots = length(robots);
    len = length(robots(1).t);
    pairs = 1;
    range_m = struct('pair', zeros(1,2), 'range', zeros(1,len));
    for i = 1:nRobots
        % inter-robot ranges
        for j = i+1:nRobots
            range_m(pairs).pair = [robots(i).id, robots(j).id];
            for k = 1:len
                dxy = [robots(i).x(k) - robots(j).x(k);...
                    robots(i).y(k) - robots(j).y(k)];
                range_m(pairs).range(k) = norm(dxy);
            end
            range_noise = random('Normal', 0, sigma_uwb, 1, len);
            range_m(pairs).range = range_m(pairs).range + range_noise;
            range_m(pairs).t = robots(i).t;
            pairs = pairs + 1;
        end
        % robot-anchor ranges
        for j = 1 : nAnchors
            range_m(pairs).pair = [robots(i).id, anchors(j).id];
            for k = 1:len
                dxy = [robots(i).x(k) -  anchors(j).x;...
                    robots(i).y(k) - anchors(j).y];
                range_m(pairs).range(k) = norm(dxy);
            end
            range_noise = random('Normal', 0, sigma_uwb, 1, len);
            range_m(pairs).range = range_m(pairs).range + range_noise;
            range_m(pairs).t = robots(i).t;
            pairs = pairs + 1;
        end
    end
    % anchor-anchor ranges
    for i = 1 : nAnchors
        for j = i+1: nAnchors
            range_m(pairs).pair = [anchors(i).id, anchors(j).id];
            dxy = [anchors(i).x - anchors(j).x;...
                   anchors(i).y - anchors(j).y];
            range_m(pairs).range = norm(dxy) * ones(1, len);
            range_noise = random('Normal', 0, sigma_uwb, 1, len);
            range_m(pairs).range = range_m(pairs).range + range_noise;
            range_m(pairs).t = robots(i).t;
            pairs = pairs + 1;
        end
    end
end