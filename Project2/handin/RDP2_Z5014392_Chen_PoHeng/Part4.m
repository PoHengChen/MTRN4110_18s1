%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project02.PartD

% =========================== CR.comment =================================
% "CR.N": length of the sequence of images.
% "CR.H(1,i)": sample time of image #i. (Class "uint32". 1 unit = 0.1ms.)
% "CR.R(:,:,i)": depth image #i. (class "uint16"; 1 unit = 1mm)
% ========================================================================
function main(file)
    if ~exist('file','var'),   file =  'DepthData01.mat' ;   end    
    load(file);     % here we load the file, specified by the caller.
    L = CR.N;       % how many images are this dataset?  

    API = IniAPIGetPointCloudFromDepth();
    API.SetProjectionConstants(1,340,0.001848, 240,0.001865); 
    fprintf('(using API ver=[%.1f])\n',API.Info.version);
    flag1 = 0;      % just a flag, for indicating certain initialization. 
    
    xRoI = [95 120];
    yRoI = [60 100];
    LineOfInterest = 46;
    regress_k = 0.973;
    
    for i = 1:L             % just a loop..
        Depth = CR.R(:,:,i);  %get a copy of depth image #i;
        [xx,yy,zz]=API.ConvertDepthsTo3DPoints(Depth,0.1) ;  
        xxR = xx(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  yyR = yy(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  zzR = zz(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  
        xxx = xx;  yyy = yy;  zzz = zz;
        xr = xxR(1:end)';yr = yyR(1:end)';zr = zzR(1:end)';
        xxH = xx(LineOfInterest,:); yyH = yy(LineOfInterest,:); zzH = zz(LineOfInterest,:);
        
        
        
        % Check noise       (using "regress" function, parameter "stats"). It will return a number which close to 1 if regress quility is high
        X = [ones(size(xr)) xr yr]; 
        [b, ~, ~, ~, stats] = regress(zr,X); 
        if stats(1) > regress_k, normal = [-b(2) -b(3)  1  ]; end 
        
        % Getting roll and pitch        (using matrix operation)
        pitch = real(asin(normal(1)));
        roll = real(asin(normal(2) / -cos(pitch)));
        
        % Updating rotated coordinate
        xx = xx * cos(-pitch) + zz * sin(-pitch);
        zz = xx * -sin(-pitch) + zz * cos(-pitch);
        yy = yy * cos(-roll) - zz * sin(-roll);
        zz = yy * sin(-roll) + zz * cos(-roll);
        
        % Updating rotated ROI & normal & LineOfInterest
        xxR = xx(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  yyR = yy(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  zzR = zz(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  
        xr = xxR(1:end)';yr = yyR(1:end)';zr = zzR(1:end)';
        X = [ones(size(xr)) xr yr];
        b = regress(zr,X);
        if stats(1) > regress_k, n = [-b(2) -b(3)  1  ]; end
        xxH = xx(LineOfInterest,:); yyH = yy(LineOfInterest,:); zzH = zz(LineOfInterest,:);

        % Getting index of poles
        ranges = GetRanges(xxH,yyH);
        Poles_index = GetPolesIndex(ranges);
        
        % Filtering dangerous index from poles index
%         dan = [];
%         for u = 1:length(Poles_index)
%            if pdist([0 0; xxH(Poles_index(u)) yyH(Poles_index(u))])<60
%               if isempty(dan)
%                  dan = Poles_index(u); 
%               else
%                  dan = [dan(1,:),Poles_index(u)];
%               end
%            end
%         end
%         Poles_index = dan;
%         OOIs = ExtractOOIs(ranges,xxH,yyH);
%         iii = abs(xx-xxH)<3 ;
        X1R = xx(1:end);
        Y1R = yy(1:end);
        Z1R = zz(1:end);
        a = [];
        for i = 1:length(Poles_index)
            iii = sqrt((X1R-xxH(Poles_index(i))).^2 + (Y1R-yyH(Poles_index(i))).^2)<1.5;
            % Store index
            for j = 1:length(X1R)
                if iii(j)>0
                    if isempty(a)
                        a = j;
                    elseif ~ismember(i,a)
                        a = [a(1,:),j];
                    end
                end
            end
        end
        
        % Declaring variable for polygon on depth
        v = [yRoI(2),xRoI(2);yRoI(1),xRoI(2);yRoI(1),xRoI(1);yRoI(2),xRoI(1)];
        f = [1,2,3,4];
        
        if flag1  % 
            
            set(h1,'cdata',Depth);
            set(h2b,'xdata',xxR(1:end),'ydata',yyR(1:end),'zdata',zzR(1:end));  %blue
            set(h2,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));      %ROI(pink)
            set(h3,'xdata',xxR(end/2),'ydata',yyR(end/2),'zdata',zzR(end/2),'udata',n(1),'vdata',n(2),'wdata',n(3));   %normal (red)
            set(h4a,'xdata',xxH,'ydata',yyH,'zdata',zzH);
            set(h4b,'xdata',xxH,'ydata',yyH,'zdata',zzH);
            set(h4c,'xdata',xxH(Poles_index),'ydata',yyH(Poles_index),'zdata',zzH(Poles_index));
            set(h5,'xdata',xxx(1:end),'ydata',yyy(1:end),'zdata',zzz(1:end));      %ROI(pink)
            set(h6,'xdata',X1R(a),'ydata',Y1R(a),'zdata',Z1R(a)); 
        else   %First time: we create Matlab graphical objects
            flag1=1;
            figure(1) ; clf() ;  
            % Depth plot
            subplot(221) ; 
            h1 = imagesc(Depth,[100,2000]); % image, scalling color for values in range from 100mm to 2000.
            hold on;
            plot([0,160],LineOfInterest+[0,0],'--r');
            patch('Faces',f,'Vertices',v,'FaceColor','m','faceAlpha',0.5);
            set(gca(),'xdir','reverse');    
            colormap gray ; zoom on;   title('Depth, shown as an image');
            
            % Aligend Camera 3D plot
            subplot(222);
            axis([0,200,-125,125,-30,60]); xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
            rotate3d on; grid on; hold on; title('3D Points Cloud (view from camera)');
            h2b = plot3(xxR,yyR,zzR,'*m','markersize',3);
            h2 = plot3(xx(1:end),yy(1:end),zz(1:end),'.b','markersize',1);
            h3 = quiver3(xxR(end/2),yyR(end/2),zzR(end/2),n(1),n(2),n(3),40,'LineWidth', 1,'Color','k'); % scale = 50
            h4a = plot3(xxH,yyH,zzH,'.r','markersize',5);
            
            % Line_Of_Interset
            subplot(223);
            axis([0,200,-125,125,-30,60]); xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
            rotate3d on; grid on; hold on; title('3D Points Cloud (view from camera)');
            h4b = plot3(xxH,yyH,zzH,'.b','markersize',5);
            h4c = plot3(xxH(Poles_index),yyH(Poles_index),zzH(Poles_index),'*r','markersize',10);
            
            % Orginal Camera 3D plot
            subplot(224);
            axis([0,200,-125,125,-30,60]); xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
            rotate3d on; grid on; hold on; title('Unrotated 3D Points Cloud (view from camera)');
            h5 = plot3(xxx(1:end),yyy(1:end),zzz(1:end),'.b','markersize',1);
            
            % Obstacle detection plot
            figure(22) ; 
            axis([0,200,-125,125,-30,60]); xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
            rotate3d on; grid on; hold on; title('3D Points Cloud (view from camera)');
            h2b = plot3(xxR,yyR,zzR,'*m','markersize',3);
            h2 = plot3(xx(1:end),yy(1:end),zz(1:end),'.b','markersize',1);
            h3 = quiver3(xxR(end/2),yyR(end/2),zzR(end/2),n(1),n(2),n(3),40,'LineWidth', 1,'Color','k'); % scale = 50
            h4a = plot3(xxH,yyH,zzH,'.r','markersize',5);
            h6 = plot3(X1R(a),Y1R(a),Z1R(a),'*r','markersize',5);
            
        end
        pause(0.1); 
    end
    disp('Done....');
 return;
end

% This fucntion return 1 x 160
 function ranges = GetRanges(X,Y)
    ranges = [];
    for i = 1:length(X)
        temp = pdist([0 0; X(i) Y(i)]);
        if isempty(ranges)
            ranges(1,:) = temp;
        else
            ranges = [ranges(1,:),temp];
        end
    end
    return;
 end
 
 % This function return the index which belong poles
 function r = GetPolesIndex(ranges,X,Y)
    r = [];
    
    
    i = length(ranges)-1;
    temp = [];
    while i >= 1
        
        % Manage ranges = 0 && Detecting first pixel of each pole
        if ranges(i)==0 && ranges(i+1)>0
            temp = ranges(i+1);
            i = i-1;
            continue;
        elseif ranges(i)==0 && ranges(i+1)==0
            i = i-1;
            continue;
        elseif ranges(i)>0 && ranges(i+1)==0
            if isempty(temp)
                i = i-1;
                continue;
            else
                ranges(i+1) = temp;
            end
        else
            % normal case
        end
        
        % Store index
        if (ranges(i+1)-ranges(i)) > 10 
            if isempty(r)   %If value is empty
                r = i;
            else
                r = [r(1,:),i];
            end
        
        % Collect poles
            j = i - 1;
            while (abs(ranges(j) - ranges(i)) < 3) && (j >= 1)
                r = [r(1,:),j];
                j = j-1; 
            end
            i = j;
            
        end
        i = i-1;
    end
    return;
 end
 
 function r = GetDangerousIndex(ranges,X,Y)
    r = [];
    
    i = length(ranges)-1;
    temp = [];
    while i >= 1
        
        % Manage ranges = 0 && Detecting first pixel of each pole
        if ranges(i)==0 && ranges(i+1)>0
            temp = ranges(i+1);
            i = i-1;
            continue;
        elseif ranges(i)==0 && ranges(i+1)==0
            i = i-1;
            continue;
        elseif ranges(i)>0 && ranges(i+1)==0
            if isempty(temp)
                i = i-1;
                continue;
            else
                ranges(i+1) = temp;
            end
        else
            % normal case
        end
        
        % Store index
        if (ranges(i+1)-ranges(i)) > 10 
            if isempty(r)   %If value is empty
                r = i;
            else
                r = [r(1,:),i];
            end
        
        % Collect poles
            j = i - 1;
            while (abs(ranges(j) - ranges(i)) < 3) && (j >= 1)
                r = [r(1,:),j];
                j = j-1; 
            end
            i = j;
            
        end
        i = i-1;
    end
    return;
 end
 
 function r = ExtractOOIs(ranges,X,Y)
    r.N = 0;
    r.Centers = [];
    r.Sizes   = []; 
    r.Color = [];
    count = 0;

    for i = 1:length(ranges)-1

        if abs(ranges(i)-ranges(i+1)) > 0.05 %&&count >2
            
            % Center
            x = mean(X((i-count):i));
            y = mean(Y((i-count):i));
            
            % Diameter (Size)
            diameter = sqrt((X(i-count)-X(i))^2+(Y(i-count)-Y(i))^2);
            
            % Color
            color = 0;
            if sum(intensities((i-count):i))~=0, color = 1; end
            
            if  diameter <= 0.2 && diameter >= 0.05 && color == 1
                    
                if isempty(r.Centers)       %If value is empty
                    r.Centers(1,:) = x;
                    r.Centers(2,:) = y;
                    r.Color = color;
                    r.Sizes = diameter;
                else
                    r.Centers = [r.Centers(1,:),x;r.Centers(2,:),y];
                    r.Color = [r.Color,color];
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
 