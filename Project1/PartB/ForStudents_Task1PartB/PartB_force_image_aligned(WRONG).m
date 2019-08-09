%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project01.PartA

% =========================== CR.comment =================================
% "CR.N": length of the sequence of images.
% "CR.H(1,i)": sample time of image #i. (Class "uint32". 1 unit = 0.1ms.)
% "CR.R(:,:,i)": depth image #i. (class "uint16"; 1 unit = 1mm)
% ========================================================================

function main(file)
    if ~exist('file','var'),   file =  'DepthData01.mat' ;   end;    
    load(file);        % here we load the file, specified by the caller.
    L = CR.N;
    
    % Declare time
    times = double(CR.H(1,:))/10000;
    times = times - times(1);
    
    API = IniAPIGetPointCloudFromDepth();
    API.SetProjectionConstants(1,340,0.001848, 240,0.001865); 
    
    CreateFigureFlag = 0;
    xRoI = [95 120];
    yRoI = [60 100];
    pitch_array = [];
    roll_array = [];
    k = 180/pi;
    regress_k = 0.99;
    for i=1:L           
        Depth = CR.R(:,:,i);  
        [xx,yy,zz] = API.ConvertDepthsTo3DPoints(Depth,0.1);
        xxR = xx(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  yyR = yy(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  zzR = zz(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  
        xr = xxR(1:end)';yr = yyR(1:end)';zr = zzR(1:end)';
        
        % Check noise       (using "regress" function, parameter "stats"). It will return a number which close to 1 if regress quility is high
        X = [ones(size(xr)) xr yr]; 
        [b, ~, ~, ~, stats] = regress(zr,X); disp(stats(1));
            % Store first b
            if i == 1
               first_normal =  [-b(2) -b(3)  1  ];
            end
        % if noise occured, update first normal
        if stats(1) > regress_k 
            normal = [-b(2) -b(3)  1  ]; 
        else
            normal = first_normal;
        end       
        
        % Getting roll and pitch        (using matrix operation)
        pitch = real(asin(normal(1)));
        roll = real(asin(normal(2) / -cos(pitch)));
        if isempty(pitch_array) 
            pitch_array = pitch;
            roll_array = roll;
        else 
            pitch_array = [pitch_array(1,:),pitch];
            roll_array = [roll_array(1,:),roll];
        end
        
        
        % Updating rotated coordinate
        xx = xx * cos(-pitch) + zz * sin(-pitch);
        zz = xx * -sin(-pitch) + zz * cos(-pitch);
        yy = yy * cos(-roll) - zz * sin(-roll);
        zz = yy * sin(-roll) + zz * cos(-roll);
        
        % Updating rotated ROI & normal 
        xxR = xx(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  yyR = yy(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  zzR = zz(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  
        xr = xxR(1:end)';yr = yyR(1:end)';zr = zzR(1:end)';
        X = [ones(size(xr)) xr yr];
        b = regress(zr,X);
            % store first rotated b
            if i == 1
               first_rotated_normal =  [-b(2) -b(3)  1  ];
            end
        % if noise occured, update first n
        if stats(1) > regress_k 
            n = [-b(2) -b(3)  1  ]; 
        else
            n = first_rotated_normal;
        end
        
        % Declaring variable for polygon on depth
        v = [yRoI(2),xRoI(2);yRoI(1),xRoI(2);yRoI(1),xRoI(1);yRoI(2),xRoI(1)];
        f = [1,2,3,4];
        
        if CreateFigureFlag
            set(h1,'cdata',Depth);
            set(h2b,'xdata',xxR(1:end),'ydata',yyR(1:end),'zdata',zzR(1:end));  %blue
            set(h2,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));      %ROI(pink)
            set(h3,'xdata',xxR(end/2),'ydata',yyR(end/2),'zdata',zzR(end/2),'udata',n(1),'vdata',n(2),'wdata',n(3));   %normal (red)
            
            set(h4a,'xdata',times(1,1:i),'ydata',k * pitch_array(1,:));
            set(h4b,'xdata',times(1,1:i),'ydata',k * roll_array(1,:));
        else   %First time: we create Matlab graphical objects
            CreateFigureFlag = 1;
            % ===================== Figure 1 =============================
            figure(1) ; clf() ; 
            
            subplot(211) ; h1 = imagesc(Depth,[100,2000]); % image, scalling color for values in rpitche from 100mm to 2000.
            patch('Faces',f,'Vertices',v,'FaceColor','m','faceAlpha',0.5);
            set(gca(),'xdir','reverse');
            colormap gray ; zoom on ; title('Depth, shown as an image');
            
            subplot(212) ;  axis([0,200,-125,125,-30,60]); rotate3d on; grid on ; hold on;
            xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');title('3D Points Cloud (view from camera)');
            
            h2b = plot3(xxR,yyR,zzR,'*m','markersize',3);
            h2 = plot3(xx(1:end),yy(1:end),zz(1:end),'.b','markersize',1);
            h3 = quiver3(xxR(end/2),yyR(end/2),zzR(end/2),n(1),n(2),n(3),40,'LineWidth', 1,'Color','k'); % scale = 50
            
            % ===================== Figure 2 =============================
            figure(2) ; clf() ; 
            subplot(211) ; axis([0,160,-20,-10]); grid on ; hold on;
            xlabel('time (second)'); ylabel('Degree');title('Pitch (Camera respect to platform)');
            h4a = plot(0,0,'b');
            
            subplot(212) ; axis([0,160,-2,2]); grid on ; hold on;
            xlabel('time (second)'); ylabel('Degree');title('Roll (Camera respect to platform)');
            h4b = plot(0,0,'b');
        end
        
        pause(0.1);
    end;
    disp('Done....');
 return;