function [range_m, imu_m, odom_m] = hexdumpUWB(filename, UWBIDs, robotIDs)

nUWB = length(UWBIDs);
nRobot = length(robotIDs);

for r = 1:nUWB
    imu_m(r).id = UWBIDs(r);
    imu_m(r).t = [];
    imu_m(r).data = [];
end
for r = 1:nRobot
    odom_m(r).id = robotIDs(r);
    odom_m(r).t = [];
    odom_m(r).int = [];
    odom_m(r).Delta = [];
end

pairs = 0;
for i = 1:nUWB
    for j = i+1:nUWB
        pairs = pairs + 1; 
        range_m(pairs).pair = [UWBIDs(i),UWBIDs(j)];
        range_m(pairs).range = [];
        range_m(pairs).t = [];
    end
end

fid = fopen(filename,'r');
%fseek(fid, 500, 'bof'); % Skip the first 1000 bytes

while (true)
    [head, cnt] = fread(fid, 2, 'uint8');
    if cnt ~= 2
        disp('Read to file end');
        break;
    end
    
    if head(1) == 165 && head(2) == 90
        % Message type, 0x01 IMU; 0x02: Odom; 0x03 UWB range
        [msgLen,cnt] = fread(fid, 1, 'uint8'); 
        if cnt~= 1 
            break; 
        end
        fseek(fid, msgLen+1, 'cof');
        if ftell(fid) ~= -1
            fseek(fid, -(msgLen+1), 'cof');
%             [bin, cnt] = fread(fid, msgLen+1, 'uint8');
%             disp('Message:');
%             disp(dec2hex(bin));
%             fseek(fid, -(msgLen+1), 'cof');
        else
            disp('Read to file end');
            break;
        end
        msgType = fread(fid, 1, 'uint8');
        timestamp = fread(fid, 1, 'uint32');
        timestamp = timestamp / 1000; % Convert form ms to sec
        switch msgType
            case 1 % IMU
                anchorId = fread(fid, 1, 'uint8');
                accgyro = fread(fid, 6, 'int16');
                for r = 1:nUWB
                    if anchorId == imu_m(r).id
                        imu_m(r).t(end+1) = timestamp;
                        imu_m(r).data(end+1, :) = accgyro';
                    end
                end
            case 2 % Odom
                anchorId = fread(fid, 1, 'uint8');
                odomInt = fread(fid, 1, 'uint8');
                odomDelta = fread(fid, 3, 'float32');
                
                for r = 1:nRobot
                    if anchorId == odom_m(r).id
                        odom_m(r).t(end+1) = timestamp;
                        odom_m(r).int(end+1) = odomInt;
                        odom_m(r).Delta(end+1, :) = odomDelta';
                    end
                end
            case 3 % UWB range
                anchorIds = fread(fid, 2, 'uint8');
                range = fread(fid, 1, 'double');
                for p = 1:pairs
                    if anchorIds(1) == range_m(p).pair(1) && anchorIds(2) == range_m(p).pair(2)
                        range_m(p).range(end+1) = range;
                        range_m(p).t(end+1) = timestamp;
                    end
                end
            otherwise
                disp(['Invalid message type: ', num2str(msgType)]);
        end
        tail = fread(fid, 1, 'uint8');
        if tail ~= 221
            disp('Invalid message tail');
        end
    else
         fseek(fid, -1, 'cof');
    end

end
fclose(fid);
end