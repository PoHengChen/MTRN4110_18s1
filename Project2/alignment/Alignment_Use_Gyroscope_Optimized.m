%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project01.PartA

function Gyroscope(file)
    if ~exist('file','var'),   file =  'DepthData01.mat' ;   end;    
    load(file);        % here we load the file, specified by the caller.
    
    IMU = NewIMUDATAf('IMU_data.mat');
    L = CR.N;
    
    % Declare time
    times_CR = double(CR.H(1,:))/10000;
%      times_CR = times_CR - times_CR(1);
    
    times_IMU = double(IMU.times(1,:))/10000;
%      times_IMU = times_IMU - times_IMU(1);
    
    % Setting Depth Image
    API = IniAPIGetPointCloudFromDepth();
    API.SetProjectionConstants(1,340,0.001848, 240,0.001865); 
    
    % Declaring variable for polygon on depth
    xRoI = [95 120];
    yRoI = [60 100];
    v = [yRoI(2),xRoI(2);yRoI(1),xRoI(2);yRoI(1),xRoI(1);yRoI(2),xRoI(1)];
    f = [1,2,3,4];
    
    regress_k = 0.973;
    CreateFigureFlag = 0;
    k = 180/pi;
    j = 1;
    CR_pitch_array = [];
    CR_roll_array = [];
    
    for i=1:L % index of CR (update slower, low frequency)         
        Depth = CR.R(:,:,i);  
        [xx,yy,zz] = API.ConvertDepthsTo3DPoints(Depth,0.1);
        xxR = xx(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  yyR = yy(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  zzR = zz(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  
        xr = xxR(1:end)';yr = yyR(1:end)';zr = zzR(1:end)';
        
        X = [ones(size(xr)) xr yr]; 
        [b, ~, ~, ~, stats] = regress(zr,X); 
        if stats(1)>regress_k
        normal = [-b(2) -b(3)  1  ]; 
        end
        % Getting (Camera's) roll and pitch        (using matrix operation)
        p = real(asin(normal(1)));
        r = real(asin(normal(2) / -cos(p)));
        
        if isempty(CR_pitch_array)
            CR_pitch_array = p;
            CR_roll_array = r;
        else
            CR_pitch_array = [CR_pitch_array(1,:),p];
            CR_roll_array = [CR_roll_array(1,:),r];
        end
        
        if times_CR(i)-times_CR(1) < IMU.static_time
            roll_CR = mean(CR_roll_array);
            pitch_CR = mean(CR_pitch_array);
        end
        
        % loop to the IMU index which close to CR index
        while times_IMU(j) < times_CR(i),j = j+1; end
        % move to the cloestest one
        if abs(times_IMU(j-1) - times_CR(i)) < abs(times_IMU(j) - times_CR(i)), j = j-1; end
        
        roll = IMU.DATAf(10,j) + roll_CR;
        pitch = -IMU.DATAf(11,j) + pitch_CR;
        
        % Updating rotated coordinate
        xx = xx * cos(-pitch) + zz * sin(-pitch);
        zz = xx * -sin(-pitch) + zz * cos(-pitch);
        yy = yy * cos(-roll) - zz * sin(-roll);
        zz = yy * sin(-roll) + zz * cos(-roll);      
        
        if CreateFigureFlag
%             set(h1,'cdata',Depth);
            set(h2,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));      %ROI(pink)
            
            set(h4a,'xdata',times_CR(1,1:i)-times_CR(1,1),'ydata',k * -(CR_pitch_array(1,:)-pitch_CR));
            set(h4b,'xdata',times_CR(1,1:i)-times_CR(1,1),'ydata',k * (CR_roll_array(1,:)-roll_CR));
            set(h5a,'xdata',times_IMU(1,1:j)-times_IMU(1,1),'ydata',k * IMU.DATAf(11,1:j));
            set(h5b,'xdata',times_IMU(1,1:j)-times_IMU(1,1),'ydata',k * IMU.DATAf(10,1:j));
        else   %First time: we create Matlab graphical objects
            CreateFigureFlag = 1;
            % ===================== Figure 1 =============================
            figure(22) ; clf() ; 
            
%             subplot(211) ; h1 = imagesc(Depth,[100,2000]); % image, scalling color for values in rpitche from 100mm to 2000.
%             patch('Faces',f,'Vertices',v,'FaceColor','m','faceAlpha',0.5);
%             set(gca(),'xdir','reverse');
%             colormap gray ; zoom on ; title('Depth, shown as an image');
%             subplot(212) ;
              axis([0,200,-125,125,-30,60]); rotate3d on; grid on ; hold on;
            xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');title('3D Points Cloud (view from camera)');
            h2 = plot3(xx(1:end),yy(1:end),zz(1:end),'.b','markersize',1);
            
            % ===================== Figure 2 =============================
            figure(831) ; clf() ; 
            subplot(211) ; axis([0,160,-3,3]); grid on ; hold on;
            xlabel('time (second)'); ylabel('Degree');title('Pitch (Camera respect to platform)');
            h4a = plot(0,0,'b');
            h5a = plot(0,0,'r');
            legend('Camera - 3D Image processing','IMU - Gyroscope');
            
            subplot(212) ; axis([0,160,-4,4]); grid on ; hold on;
            xlabel('time (second)'); ylabel('Degree');title('Roll (Camera respect to platform)');
            h4b = plot(0,0,'b');
            h5b = plot(0,0,'r');
            legend('Camera - 3D Image processing','IMU - Gyroscope');
            
        end
        
            pause(0.05);
    end;
    disp('Done....');
 return;
 
% =========================== CR.comment =================================
% "CR.N": length of the sequence of images.
% "CR.H(1,i)": sample time of image #i. (Class "uint32". 1 unit = 0.1ms.)
% "CR.R(:,:,i)": depth image #i. (class "uint16"; 1 unit = 1mm)
% ========================================================================

% ============================ IMU.comment ===============================
% Accelerations X,Y,Z, expressed in local frame : IMU.DATAf(1:3,:) 
% Angular rates (local roll, pitch and yaw rates): IMU.DATAf(4:6,:) 
% Units: Accelerations in Gravities, Angular rates in radians/second
% time units:  1 count =0.1ms; "times" expressed via class "uint32"; you may need to convert it to real format (e.g. "double")
% ========================================================================