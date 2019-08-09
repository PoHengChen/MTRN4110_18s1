%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project02.PartB

% =========================== CR.comment ==================================
% "CR.N": length of the sequence of images.
% "CR.H(1,i)": sample time of image #i. (Class "uint32". 1 unit = 0.1ms.)
% "CR.R(:,:,i)": depth image #i. (class "uint16"; 1 unit = 1mm)
% =========================================================================

% =========================================================================
% =========               Camera Laser Data                      ==========
% =========================================================================

if ~exist('file','var'),   file =  'DepthData01.mat' ;   end;    
load(file);        % here we load the file, specified by the caller.
L = CR.N;              % how many images are this dataset?  
times = double(CR.H(1,:))/10000;
times = times - times(1);
API = IniAPIGetPointCloudFromDepth();
API.SetProjectionConstants(1,340,0.001848, 240,0.001865); 
fprintf('(using API ver=[%.1f])\n',API.Info.version);
T = [];

flag1 = 0;   
xRoI = [95 120]; % Horizontal
yRoI = [60 100]; % Vertical
regress_k = 0.973;
for i=1:L           
    Depth = CR.R(:,:,i);  
    [xx,yy,zz]=API.ConvertDepthsTo3DPoints(Depth,0.1) ;
    xxR = xx(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  yyR = yy(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  zzR = zz(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  
    xr = xxR(1:end)';yr = yyR(1:end)';zr = zzR(1:end)';

    X = [ones(size(xr)) xr yr]; 
    [b, bint, r, rint, stats] = regress(zr,X); %first regress is aim to use return value "stats" to check if regression correct

    % Calculate pitch and roll
    normal = [-b(2) -b(3)  1  ];
    ver = [0 0 1 ];
    if stats(1) > regress_k
        pitch = real(asin(normal(1)));
        roll = real(asin(normal(2) / -cos(pitch)));
    end
    
    if i == 1
        T = [roll;pitch];
        Camera = [roll;pitch];
    else
        T = [T(1,:),roll;T(2,:),pitch];
    end

end
T = [T(1,:) - Camera(1);T(2,:) - Camera(2)];

% =========================================================================
% =========               IMU Gyroscope Data                     ==========
% =========================================================================
MyFile = 'IMU_data.mat';  
load(MyFile) ;

times_imu = double(IMU.times)/10000;
times_imu = times_imu - times_imu(1);
N = IMU.N;
k = 180/pi; 

% Manage yaw_rate bias.................................................
ii = times_imu < 15;
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

% =========================================================================
% ==========         Plotting of Platform Pitch & Roll          ===========
% =========================================================================

figure(113) ; clf ;
% ========================== Pitch ========================================
subplot(211); hold on ; grid on ; zoom on ;
xlabel('time (in seconds)') ; ylabel('degrees'); title('Pitch (Robot repect to Platform)');
% T(2,:) = - T(2,:);
plot(times, T(2,:)*k);
Attitude(2,:) = - Attitude(2,:);
plot(times_imu, Attitude(2,:)*k, 'r');
legend('Camera - 3D Image processing','IMU - Gyroscope');

% ========================== Roll =========================================
subplot(212) ; hold on ; grid on ; zoom on ;
xlabel('time (in seconds)') ; ylabel('degrees'); title('Roll (Robot repect to Platform)');
plot(times,T(1,:)*k);
plot(times_imu, Attitude(1,:)*k, 'r');
legend('Camera - 3D Image processing','IMU - Gyroscope');

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