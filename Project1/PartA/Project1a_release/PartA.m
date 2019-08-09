%Author: Po-Heng Chen , Z5014392

%Program: Solution for RD, S1.2018, Project01.PartA

%..........................................................................
MyFile = '.\ImuData\IMU_002\IMUdata.mat';  
load(MyFile) ;

times = IMU.times ;     
times = times - times(1) ;  % just, in order to refer to t0=0 (not necessary).

k = 180/pi; % constant, useful for converting radian to degrees.

figure(1) ; clf() ; hold on ; grid on ; zoom on ;
plot(times,IMU.Gyros(3,:)*k,'b');
legend({'Wz (yaw rate)'}) ; xlabel('time (in seconds)') ; ylabel('angular rates (degrees/sec)');

figure(2) ; clf() ; hold on ; grid on ; zoom on ;
Attitude = zeros(3,length(times)-1);
Attitude(:,1) = [0 0 0];

%biased plot
for i = 1:length(times)-1
    Attitude(:,i+1) = IntegrateOneStepOfAttitude( IMU.Gyros(:,i), IMU.times(i+1)-IMU.times(i), Attitude(:,i) );
end;
plot(times,Attitude(3,:)*k,'b');

% =========================================================================
% =========================== Bias Caculation =============================
% =========================================================================
% ii = times <= 5 ;
bias = mean(IMU.Gyros(3,times <= 8 )) ;
IMU.Gyros(3,:) = IMU.Gyros(3,:) - bias ;

%unbiased plot
NewAttitude = zeros(3,length(times)-1);
NewAttitude(:,1) = [0 0 0];
for i = 1:length(times)-1
    NewAttitude(:,i+1) = IntegrateOneStepOfAttitude( IMU.Gyros(:,i), IMU.times(i+1)-IMU.times(i), NewAttitude(:,i) );
end;
    
plot(times,NewAttitude(3,:)*k,'r');
xlabel('time (in seconds)'); ylabel('angle (degrees)');

function NewAttitude  = IntegrateOneStepOfAttitude( gyros, dt, CurrentAttitude ) 
    % for a small delta time , dt  
    % CurrentAttitude is the current (initial) attitude, in radians  
    % gyros:vector with the gyros measurements, scaled in rad/sec 

    ang = CurrentAttitude ;  % current global Roll, Pitch, Yaw  (at time t) 
%     wx = gyros(1);   %local roll rate
%     wy = gyros(2);   %local pitch rate

    % As the sensor was rotated just on the z-plane
    % hence, there will be no rotation on roll and pitch
    wx = 0;   %local roll rate 
    wy = 0;   %local pitch rate 
    wz = gyros(3);   %local yaw rate

    %----------------------------------
    cosang1=cos(ang(1)) ; 
    cosang2=cos(ang(2)) ; 
    sinang1=sin(ang(1)) ; 

    roll  = ang(1) + dt * (wx + (wy*sinang1 + wz*cosang1)*tan(ang(2))) ; %(*) 
    pitch = ang(2) + dt * (wy*cosang1 - wz*sinang1)     ; 
    yaw   = ang(3) + dt * ((wy*sinang1 + wz*cosang1)/cosang2)    ; %(*) 
    %----------------------------------

    NewAttitude= [roll,pitch,yaw];  % new global Roll, Pitch, Yaw (at time t+dt) 
return;
end