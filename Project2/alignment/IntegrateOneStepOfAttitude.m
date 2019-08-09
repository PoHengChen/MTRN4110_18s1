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