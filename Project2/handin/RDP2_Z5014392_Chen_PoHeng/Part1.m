%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project02.PartA

% =========================== IMU.comment ================================
% Accelerations X,Y,Z, expressed in local frame : IMU.DATAf(1:3,:) 
% Angular rates (local roll, pitch and yaw rates): IMU.DATAf(4:6,:) 
% Units: Accelerations in Gravities, Angular rates in radians/second
% time units:  1 count =0.1ms; "times" expressed via class "uint32"; you may need to convert it to real format (e.g. "double")
% Author: Jose Guivant, 2016; for MTRN4110, S1.2016
% ========================================================================

% Storing value...........................................................
clear;
MyFile = 'IMU_data.mat';  
load(MyFile) ;

times_imu = double(IMU.times)/10000;
times_imu = times_imu - times_imu(1);
N = IMU.N;
k = 180/pi; 

% Manage yaw_rate bias.................................................
ii = times_imu < 17;
bias_row = mean(IMU.DATAf(4,ii));
bias_pitch = mean(IMU.DATAf(5,ii));
bias_yaw = mean(IMU.DATAf(6,ii));
IMU.DATAf(4,:) = IMU.DATAf(4,:) - bias_row ;
IMU.DATAf(5,:) = IMU.DATAf(5,:) - bias_pitch ;
IMU.DATAf(6,:) = IMU.DATAf(6,:) - bias_yaw ;

% Intergrate gyroscope rate
Attitude = zeros(3,N-1);
Attitude(:,1) = [0 0 0]';
for i = 1:N-1
    Attitude(:,i+1) = IntegrateOneStepOfAttitude( IMU.DATAf(4:6,i), times_imu(i+1)-times_imu(i), Attitude(:,i) );
end

% Plotting
figure(141) ; clf ;
% ======================= Angular Rate (Degree/Second)=====================
subplot(211) ; hold on ; grid on ; zoom on ;
plot(times_imu,IMU.DATAf(4,:)*k,'b');
plot(times_imu,IMU.DATAf(5,:)*k,'r');
plot(times_imu,IMU.DATAf(6,:)*k,'g');
legend({'Wx (roll rate)','Wy (pitch rate)','Wz (yaw rate)'});
xlabel('time (in seconds)') ; ylabel('angular rates (degrees/sec)');

% ========================== Angle (Degree) ===============================
subplot(212) ; hold on ; grid on ; zoom on ;
plot(times_imu,Attitude(1,:)*k,'b');
plot(times_imu,Attitude(2,:)*k,'r');
plot(times_imu,Attitude(3,:)*k,'g');
legend({'(roll)','(pitch)','(yaw)'});
xlabel('time (in seconds)') ; ylabel('angle (degrees)');

function NewAttitude  = IntegrateOneStepOfAttitude( gyros, dt, CurrentAttitude ) 
    % for a small delta time , dt  
    % CurrentAttitude is the current (initial) attitude, in radians  
    % gyros:vector with the gyros measurements, scaled in rad/sec 

    ang = CurrentAttitude ;  % current global Roll, Pitch, Yaw  (at time t) 
    wx = gyros(1);   %local roll rate 
    wy = gyros(2);   %local pitch rate 
    wz = gyros(3);   %local yaw rate

    %----------------------------------
    cosang1=cos(ang(1)) ; 
    cosang2=cos(ang(2)) ; 
    sinang1=sin(ang(1)) ; 

    roll = ang(1) + dt * (wx + (wy*sinang1 + wz*cosang1)*tan(ang(2))) ; %(*) 
    pitch = ang(2) + dt * (wy*cosang1 - wz*sinang1)     ; 
    yaw = ang(3) + dt * ((wy*sinang1 + wz*cosang1)/cosang2)    ; %(*) 
    %----------------------------------

    NewAttitude= [roll,pitch,yaw]';  % new global Roll, Pitch, Yaw (at time t+dt) 
return;
end