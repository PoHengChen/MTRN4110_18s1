
% For Project 1. Part A
% This piece of code just shows how to load a Matlab data file that contains 
% IMU data (according to the structure format we use in MTRN4110, for OFF-LINE processing).
% The example also uses the time and gyros' measurements.
% Jose Guivant - MTRN4110 - Session1/2018



% Read all the comments in this program, before solving Part A.
% Use the help in Matlab for learning the functionality of certain functions used in this example,
% such as: load() ; plot() ; 

%---------------------------------------------------
% Here we load the file of interest.

% This is the name of the file I want to use.
MyFile = '.\ImuData\IMU_002\IMUdata.mat';  
% All the files related to certain type of data (e.g. from a particular
% sensor), have the same filename (but are located in different folders). 


load(MyFile) ;
% Now after I load the data, it will be in a variable named IMU
% (because it was saved using that name.)

%---------------------------------------------------

times = IMU.times;      
% IMU.times is an array that contains the timestamps of the measurements.
% Each sample has an associated timestamp (sample time)

times = times - times(1) ;  % just, in order to refer to t0=0 (not necessary).

% IMU is a structure, whose field "Gyros" contains the Gyros' measurements
% IMU.Gyros(1,:) is Wx  (local Roll rate)
% IMU.Gyros(2,:) is Wy  (local Pitch rate)
% IMU.Gyros(3,:) is Wz  (local Yaw rate)
% All measurements are expressed in radians/second

%---------------------------------------------------
% -- Here we plot the 3 angular rates (local, from gyros) ------


figure(1) ;  clf() ; hold on ;
k = 180/pi; % constant, useful for converting radian to degrees.
plot(times,IMU.Gyros(1,:)*k,'b');
plot(times,IMU.Gyros(2,:)*k,'r');
plot(times,IMU.Gyros(3,:)*k,'g');
legend({'Wx (roll rate)','Wy (pitch rate)','Wz (yaw rate)'});
xlabel('time (in seconds)');
ylabel('angular rates (degrees/sec)');
grid on ;
zoom on;
disp('Done, see the plots in figure 1');

% You can use the help in Matlab for learning the functionality of certain 
% functions used in this example, such as:
% plot ; hold ; clf ; figure ; xlabel, ylabel ; grid ; zoom ; legend ;


%-----------------------------------------------------
% Jose Guivant - MTRN4110 - S1.2018
% Questions: email j.guivant@unsw.edu.ar or via Moodle's forum.
%-----------------------------------------------------








