%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project01.PartA

% =========================== CR.comment =================================
% "CR.N": length of the sequence of images.
% "CR.H(1,i)": sample time of image #i. (Class "uint32". 1 unit = 0.1ms.)
% "CR.R(:,:,i)": depth image #i. (class "uint16"; 1 unit = 1mm)
% ========================================================================
function main(file)
    if ~exist('file','var'),   file =  'DepthData01.mat' ;   end;    
    load(file);     % here we load the file, specified by the caller.
    L = CR.N;       % how many images are this dataset?  

    API = IniAPIGetPointCloudFromDepth();
    API.SetProjectionConstants(1,340,0.001848, 240,0.001865); 
    fprintf('(using API ver=[%.1f])\n',API.Info.version);
    flag1 = 0;      % just a flag, for indicating certain initialization. 
    
    xRoI = [95 120];
    yRoI = [65 95];
    LineOfInterest = 40;
    
    for i = 1:L             % just a loop..
        Depth = CR.R(:,:,i);  %get a copy of depth image #i;
        [xx,yy,zz]=API.ConvertDepthsTo3DPoints(Depth,0.1) ;  
        xxR = xx(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  yyR = yy(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  zzR = zz(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  
        xr = xxR(1:end)';yr = yyR(1:end)';zr = zzR(1:end)';
        xxH = xx(LineOfInterest,:); yyH = yy(LineOfInterest,:); zzH = zz(LineOfInterest,:);
        ranges = GetRanges(xxH,yyH);
        Poles = ExtractOOIs(ranges,xxH,yyH);
        
        
        % Check noise       (using "regress" function, parameter "stats"). It will return a number which close to 1 if regress quility is high
        X = [ones(size(xr)) xr yr]; 
        [b, bint, r, rint, stats] = regress(zr,X); 
        if stats(1) > 0.97, normal = [-b(2) -b(3)  1  ]; end 
        
        % Getting roll and pitch        (using matrix operation)
        pitch = real(-asin(normal(1)));
        roll = real(-asin(normal(2) / -cos(pitch)));
        
        % Updating rotated coordinate
        xx = xx * cos(pitch) + zz * sin(pitch);
        zz = xx * -sin(pitch) + zz * cos(pitch);
        yy = yy * cos(roll) - zz * sin(roll);
        zz = yy * sin(roll) + zz * cos(roll);
        
        % Updating rotated ROI & normal & LineOfInterest
        xxR = xx(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  yyR = yy(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  zzR = zz(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  
        xr = xxR(1:end)';yr = yyR(1:end)';zr = zzR(1:end)';
        X = [ones(size(xr)) xr yr];
        b = regress(zr,X);
        if stats(1) > 0.973, n = [-b(2) -b(3)  1  ]; end
        xxH = xx(LineOfInterest,:); yyH = yy(LineOfInterest,:); zzH = zz(LineOfInterest,:);

        % Declaring variable for polygon on depth
        v = [yRoI(2),xRoI(2);yRoI(1),xRoI(2);yRoI(1),xRoI(1);yRoI(2),xRoI(1)];
        f = [1,2,3,4];
        
        if flag1  % 
            
            set(h1,'cdata',Depth);
            set(h2b,'xdata',xxR(1:end),'ydata',yyR(1:end),'zdata',zzR(1:end));  %blue
            set(h2,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));      %ROI(pink)
            set(h3,'xdata',xxR(end/2),'ydata',yyR(end/2),'zdata',zzR(end/2),'udata',n(1),'vdata',n(2),'wdata',n(3));   %normal (red)
            set(h4,'xdata',xxH,'ydata',yyH,'zdata',zzH);
        else   %First time: we create Matlab graphical objects
            flag1=1;
            figure(1) ; clf() ;  
            
            subplot(211) ; 
            h1 = imagesc(Depth,[100,2000]); % image, scalling color for values in range from 100mm to 2000.
            hold on;
            plot([0,160],LineOfInterest+[0,0],'--r');
            patch('Faces',f,'Vertices',v,'FaceColor','m','faceAlpha',0.5);
            set(gca(),'xdir','reverse');    
            colormap gray ; zoom on;   title('Depth, shown as an image');
            
            subplot(212);
            axis([0,200,-125,125,-30,60]); xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
            rotate3d on; grid on; hold on; title('3D Points Cloud (view from camera)');
            
            h2b = plot3(xxR,yyR,zzR,'*m','markersize',3);
            h2 = plot3(xx(1:end),yy(1:end),zz(1:end),'.b','markersize',1);
            h3 = quiver3(xxR(end/2),yyR(end/2),zzR(end/2),n(1),n(2),n(3),40,'LineWidth', 1,'Color','r'); % scale = 50
            h4 = plot3(xxH,yyH,zzH,'.r','markersize',5);
        end;
        pause(0.1); 
    end;
    disp('Done....');
 return;
end

% This fucntion return 1 x 160
 function ranges = GetRanges(X,Y)
    ranges = [];
    for i = 1:160
        temp = pdist([0 0;X(i) Y(i)]);
        if isempty(ranges)
            ranges(1,:) = temp;
        else
            ranges = [ranges(1,:),temp];
        end
    end
    return;
 end
 
 % This function return the index which belong poles
 function r = ExtractOOIs(ranges,X,Y)
    r.N = 0;
    r.Sizes   = []; 
    count = 0;

    for i = 1:length(ranges)-1

        if abs(ranges(i)-ranges(i+1)) > 0.05 %&&count >2
        
            diameter = sqrt((X(i-count)-X(i))^2+(Y(i-count)-Y(i))^2);
            
            if  diameter <= 0.2 && diameter >= 0.05 
                    
                if isempty(r.Sizes)       %If value is empty
                    r.Sizes = diameter;
                else
                    r.Sizes = [r.Sizes,diameter];
                end
                
                r.N = r.N + 1;
            end
            count = 0;    
        else
            count = count + 1;
        end
    end

 return;
 end