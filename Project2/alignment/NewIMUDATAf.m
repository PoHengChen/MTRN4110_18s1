% Author: Po-Heng Chen , Z5014392
% Program: Solution for RD, S1.2018, Project02.PartA
% This function return a new IMU struct with new rows 10 to 12 which
% storing roll, pitch & yaw

function IMU = NewIMUDATAf(MyFile)
    load(MyFile) ;

    times_imu = double(IMU.times)/10000;
    times_imu = times_imu - times_imu(1);
    N = IMU.N;
    k = 180/pi; 

    % Manage yaw_rate bias.................................................
    count = 0;
    for t = 1:N
        if abs(IMU.DATAf(4,t))> 0.01 || abs(IMU.DATAf(5,t))> 0.01 || abs(IMU.DATAf(6,t))> 0.01, count = count + 1;end
        if count > 150,break;end
    end
    IMU.static_time = times_imu(t);
    IMU.static_time = 17;
%     ii = times_imu < IMU.static_time;
    ii = times_imu < IMU.static_time;
    bias_row = mean(IMU.DATAf(4,ii));
    bias_pitch = mean(IMU.DATAf(5,ii));
    bias_yaw = mean(IMU.DATAf(6,ii));
    IMU.DATAf(4,:) = IMU.DATAf(4,:) - bias_row ;
    IMU.DATAf(5,:) = IMU.DATAf(5,:) - bias_pitch ;
    IMU.DATAf(6,:) = IMU.DATAf(6,:) - bias_yaw ;

    % Intergrate gyroscope rate
%     IMU.DATAf = {IMU.DATAf;zeros(3,N)}; %B = {B1;B2};
    IMU.DATAf(10:12,1) = [0 0 0]';
    for i = 1:N-1
        IMU.DATAf(10:12,i+1) = IntegrateOneStepOfAttitude( IMU.DATAf(4:6,i), times_imu(i+1)-times_imu(i), IMU.DATAf(10:12,i) );
    end

end

% =========================== IMU.comment ================================
% Accelerations X,Y,Z, expressed in local frame : IMU.DATAf(1:3,:) 
% Angular rates (local roll, pitch and yaw rates): IMU.DATAf(4:6,:) 
% Units: Accelerations in Gravities, Angular rates in radians/second
% time units:  1 count =0.1ms; "times" expressed via class "uint32"; you may need to convert it to real format (e.g. "double")
% Author: Jose Guivant, 2016; for MTRN4110, S1.2016
% ========================================================================