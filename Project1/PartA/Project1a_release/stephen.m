
MyFile = '.\ImuData\IMU_001\IMUdata.mat';  
figure(1);
flag = 1;
drawing(MyFile,flag);
MyFile2 = '.\ImuData\IMU_002\IMUdata.mat';  
flag = 2;
drawing(MyFile2,flag);


function drawing(MyFile,flag)

load(MyFile) ;

times = IMU.times;      
times = times - times(1) ;  % just, in order to refer to t0=0 (not necessary).

k = 180/pi; % constant, useful for converting radian to degrees.
%Remove Bias 
TimeFix = 5;
Frequency = 200;

Bias_roll = mean(IMU.Gyros(1,1:TimeFix*Frequency));
Bias_pitch = mean(IMU.Gyros(2,1:TimeFix*Frequency));
Bias_yaw = mean(IMU.Gyros(3,1:TimeFix*Frequency));

plotnum = 1+(flag-1)*2;

subplot(4,1,plotnum);
hold on ; grid on ; zoom on ;
plot(times,IMU.Gyros(3,:)*k,'b');
title('Gyros Z (deg/sec)');
xlabel('time (in seconds)'); ylabel('angular rates (degrees/sec)');
plotnum = 2+(flag-1)*2;
subplot(4,1,plotnum) ; hold on ; grid on ; zoom on ;
Attitude = zeros(3,length(times)-1);
Attitude(:,1) = [0 0 0];
Edit_Attitude = Attitude;

for i = 1:length(times)-1
    Attitude(:,i+1) = IntegrateOneStepOfAttitude( IMU.Gyros(:,i), IMU.times(i+1)-IMU.times(i), Attitude(:,i) );
    
end;

for i = 1:length(times)-1
    
     Edit_Gyros = [IMU.Gyros(1,i)-Bias_roll;IMU.Gyros(2,i)-Bias_pitch;(IMU.Gyros(3,i)-Bias_yaw)];
    Edit_Attitude(:,i+1) = IntegrateOneStepOfAttitude( Edit_Gyros, IMU.times(i+1)-IMU.times(i), Edit_Attitude(:,i) );
    
end;

% for i = 1: length(times)-1
%     
% end
%IntegrateOneStepOfAttitude( IMU.Gyros(:,1), IMU.times(i+1)-IMU.times(i), [] );
plot(times,Attitude(3,:)*k,'b');
hold on 
plot(times,Edit_Attitude(3,:)*k,'r');
hold off
legend('Yaw Position with Bias', 'Yaw Position without Bias');
title('Estimated Yaw (deg)');
xlabel('time (in seconds)'); ylabel('angle (degrees)');
end
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

    NewAttitude= [roll,pitch,yaw];  % new global Roll, Pitch, Yaw (at time t+dt) 
return;
end