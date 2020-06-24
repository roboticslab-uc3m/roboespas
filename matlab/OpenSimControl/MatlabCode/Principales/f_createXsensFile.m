function f_createXsensFile(trial, IMUpath)
    %Create Xsens file for the biceps (arm) and FCR (forearm) sensors (1 and 4)
    t = trial.DelsysSensors.Trial.Sensor1.IMU.Timestamps;
    fs = 1/(t(2)-t(1));
    for i = 1:3:4
        sensor = sprintf('Sensor%d', i);
        filename = sprintf('%s/%s%s.txt',IMUpath,trial.Conditions.NameData,sensor);
        fid = fopen(filename, 'wt');
        fprintf(fid, '// Start Time: Unknown\n');
        fprintf(fid, '// Update Rate: %.4fHz\n', fs);
        fprintf(fid, '// Filter Profile: human (46.1)\n// Option Flags: AHS Disabled ICC Disabled\n// Firmware Version: 4.0.2\n');
        fprintf(fid, 'PacketCounter\tAcc_X\tAcc_Y\tAcc_Z\tGyr_X\tGyr_Y\tGyr_Z\tMag_X\tMag_Y\tMag_Z\tMat[1][1]\tMat[2][1]\tMat[3][1]\tMat[1][2]\tMat[2][2]\tMat[3][2]\tMat[1][3]\tMat[2][3]\tMat[3][3]\n');
        acc = trial.DelsysSensors.Trial.(sensor).IMU.Acc';
        gyro = trial.DelsysSensors.Trial.(sensor).IMU.Gyro';
        mag = trial.DelsysSensors.Trial.(sensor).ImuAll(7:9,:)'./(2.35058086e11); % 1 a.u. = 235051.8086 T
        R = quat2rotm(trial.DelsysSensors.Trial.(sensor).IMU.Orientations);
        for j = 1:max(size(acc))
            fprintf(fid, '%d\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\n', j,...
                acc(j,1), acc(j,2), acc(j,3), gyro(j,1), gyro(j,2), gyro(j,3), mag(j,1), mag(j,2), mag(j,3),...
                R(1,1,j), R(2,1,j), R(3,1,j), R(1,2,j), R(2,2,j), R(3,2,j), R(1,3,j), R(2,3,j), R(3,3,j));
        end
        fclose(fid);
    end
end

