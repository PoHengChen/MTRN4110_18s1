%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project04.PartA

% =============================================================
function main(file) 
    load('DepthData01.mat');    
    load('NavMap.mat');
    IMU = NewIMUDATAf('IMU_data.mat');
    times_CR  = double(CR.H(1,:))/10000;
    times_IMU = double(IMU.times(1,:))/10000;
    L = CR.N;       % how many images are this dataset?  
    % ===================================
    % define OG
    MyContext.x1 = -50 ;MyContext.x2 = 350 ;
    MyContext.y1 = -50 ;MyContext.y2 = 250 ;
    MyContext.Nx = 200 ;
    MyContext.Ny = 200 ;
        MyContext.M = ones(MyContext.Ny,MyContext.Nx,'uint8') ; % initial at zero
        MyContext.Cx = MyContext.Nx/(MyContext.x2-MyContext.x1) ;
        MyContext.Cy = MyContext.Ny/(MyContext.y2-MyContext.y1) ;
    % ========================
    API = IniAPIGetPointCloudFromDepth();
    API.SetProjectionConstants(1,340,0.001848, 240,0.001865); 
    fprintf('(using API ver=[%.1f])\n',API.Info.version);
    flag1 = 0;      % just a flag, for indicating certain initialization. 
    
    xRoI = [95 120]; yRoI = [60 100];
    regress_k = 0.973;
    LineOfInterest = 46;
    
    % Declaring variable for polygon on depth
    v = [yRoI(2),xRoI(2);yRoI(1),xRoI(2);yRoI(1),xRoI(1);yRoI(2),xRoI(1)];
    f = [1,2,3,4];
    
    j = 1;
    Pose = [25 105 89 * pi/180]';
    Pose_History = zeros(3,CR.N - 1);
    OG = 1;
    c = [0;0];
%     OG_History = zeros();
    global LM;
    for i = 1:L             % just a loop..
        disp(i);
        Depth = CR.R(:,:,i);  %get a copy of depth image #i;
        [xx,yy,zz]=API.ConvertDepthsTo3DPoints(Depth,0.1) ;  
        xxR = xx(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  yyR = yy(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  zzR = zz(xRoI(1):xRoI(2),yRoI(1):yRoI(2));  
        xxx = xx;  yyy = yy;  zzz = zz; % Original(unrotated) XYZ coordinate
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
        OOIs = ExtractOOIs(ranges(Poles_index),xxH(Poles_index),yyH(Poles_index),Poles_index);
        OOIs = PlotOOIs(OOIs,Pose);% obtain global centers coordinate
        Pose_History = [Pose_History,Pose];
        
% OG ====================================               
    %Set ================================       
            heading = Pose(3);
            rotate = heading - 0 ; %disp(rotate);
            xu = xxH*cos(rotate)-yyH*sin(rotate);
            yu = xxH*sin(rotate)+yyH*cos(rotate);

            % Translation
            xw = xu + Pose(1);
            yw = yu + Pose(2);    

            c = [c(1,:),xw;c(2,:),yw];
    ii = find((xw>=MyContext.x1)&(xw<MyContext.x2)&(yw>=MyContext.y1)&(yw<MyContext.y2));
    xw = xw(ii) ; yw=yw(ii) ; 
    ix = floor((xw'-MyContext.x1)*MyContext.Cx)+1 ;
    iy = floor((yw'-MyContext.y1)*MyContext.Cy)+1 ;
    ixy = sub2ind(size(MyContext.M),iy,ix) ; % index of occupied
    MyContext.M(ixy) = 20 ;
    
    %Erase ==============================
            xrr = xx*cos(rotate)-yy*sin(rotate);
            yrr = xx*sin(rotate)+yy*cos(rotate);

            % Translation
            xrr = xrr + Pose(1);
            yrr = yrr + Pose(2);    
            
    zrr = zz + 30;
    cc = find(zrr<2);
    xww = xrr(cc);
    yww = yrr(cc);
    
    ii = find((xww>=MyContext.x1)&(xww<MyContext.x2)&(yww>=MyContext.y1)&(yww<MyContext.y2));
    xww = xww(ii) ; yww=yww(ii) ; 
    ixi = floor((xww'-MyContext.x1)*MyContext.Cx)+1 ;
    iyi = floor((yww'-MyContext.y1)*MyContext.Cy)+1 ;
    ixyi = sub2ind(size(MyContext.M),iyi,ixi) ; % index of clear
%     MyContext.M(ixyi)=10;
% =========================================

    % if the index in clear belong previous found index of occupied,graudally darq it to black
    if ismember(ixy,ixyi), MyContext.M(ixy) = MyContext.M(ixy) - 1; end
    MyContext.M(ixyi) = 5;
     
        if flag1  % 
            % ======================== figure 1 ===========================
            % ======================== figure 2 ===========================
            set(h8 ,'xdata',xxH,'ydata',yyH);      %ROI(pink)
            set(h4d,'xdata',xxH(Poles_index),'ydata',yyH(Poles_index),'zdata',zzH(Poles_index));
            set(h8a,'xdata',OOIs.Centers(1,:),'ydata',OOIs.Centers(2,:));
            % ======================== ======== ===========================
            set(h13a,'xdata',Landmarks.xy(1,:),'ydata',Landmarks.xy(2,:)); %LM
            set(h13b,'xdata',Pose(1),'ydata',Pose(2),'udata',40*cos(Pose(3)),'vdata',40*sin(Pose(3))); %quiver
            set(h13c,'xdata',Pose(1),'ydata',Pose(2));  %car
            set(h13d,'xdata',Pose_History(1,:) ,'ydata',Pose_History(2,:));  % Trajectory
            set(h14 ,'xdata',OOIs.Centers(3,:) ,'ydata',OOIs.Centers(4,:) ); % OOIs 
            set(h14a,'xdata',Landmarks.xy(1,OG),'ydata',Landmarks.xy(2,OG)); % OG
            set(h15a,'xdata',c(1,:),'ydata',c(2,:)); %  hold on;
            % ======================== figure 2 ===========================
            set(h16 ,'cdata',MyContext.M) ;
        else   %First time: we create Matlab graphical objects
            
            flag1=1;
            
            figure(3); clf ; 
            h16 = imagesc(MyContext.M) ; colormap gray ;
            set(gca,'ydir','normal','xdir','normal') ;
            % ======================== figure 2 ===========================
            figure(2) ;  clf ;
            subplot(211) ; 
            h13a = plot(Landmarks.xy(1,:),Landmarks.xy(2,:),'*'); hold on;
            ax=axis() ; ax=ax+[-10,10,-10,10] ; axis(ax); xlabel('X (cm)') ; ylabel('Y (cm)') ; title('Landmarks and initial pose of robot');
            h13b = quiver(Pose(1),Pose(2),40*cos(Pose(3)),40*sin(Pose(3)),'r');
            h13c = plot(Pose(1),Pose(2),'*r');
            h13d = plot(Pose_History(1,:),Pose_History(2,:),'m.','Markersize',1);
            h14  = plot(0,0,'*g');
            h14a = plot(0,0,'.b','Markersize',20);
            h15a = plot(0,0,'.b');
            % ======================== ======= ============================            % Poles 2D (XY coordinate) plot
            subplot(212) ; 
            axis([0,200,-125,125]); xlabel('X (cm)'); ylabel('Y (cm)');
            grid on; hold on; title(' Poles X Y coordinate');
            h8 = plot(xxH,yyH,'.b');
            h4d = plot3(xxH(Poles_index),yyH(Poles_index),zzH(Poles_index),'*r','markersize',5);
            h8a = plot(OOIs.Centers(1,:),OOIs.Centers(2,:),'*g','markersize',10);
            % ======================== figure 1 ===========================
        end
        
        % data association
        [OOIs,LM,OG] = PrintID(OOIs,Landmarks,OG);
        % Pose update by observation
        
        options =  optimset('TolFun',1e-3,'TolX',1);  
        % accepted tolerance of 0.001 in the cost function.
        [Pose(1),Pose(2)] = fminsearch(@MyCostFunction,[Pose(1),Pose(2)],options);
%         temp = Pose;
%         
%         % draw car
%         if OOIs.N == 0      ,   0;
%         elseif OOIs.N == 1  ,   [Pose(1), Pose(2)] = Linearization(OOIs,LM,Pose);
%         elseif OOIs.N > 1   ,   [Pose(1), Pose(2)] = Triangulation(OOIs,LM,Pose);
%         end
%         
%         % continuous aid (used when No.AOOIs is changed)
%         distance = sqrt((Pose(1) - temp(1))^2 + (Pose(2) - temp(2))^2);
%         if distance > 8.8
%            Pose(1) = temp(1) + 1.3*cos(temp(3));
%            Pose(2) = temp(2) + 1.3*sin(temp(3));
%         end
        
        % loop to the IMU index which close to CR index
        while times_IMU(j) < times_CR(i),j = j+1; end
        Pose(3) = IMU.DATAf(12,j);
        
        pause(0.025);
    end
    disp('Done....');
 return;
end

% ============================================
function cost = MyCostFunction(z)
    global LM;
    r = EvaluatedSystemOfEquations(z);
    err = LM - r;
    cost = sum( abs(err) );
return;
end

function r = EvaluatedSystemOfEquations(X)
    global LM;
    dxs = LM.Centers(1,:) - X(1);
    dys = LM.Centers(2,:) - X(2);
    r = sqrt(dxs.*dxs + dys.*dys);
return;
end

% ============================================
 function [x,y] = Triangulation(OOIs,LM,Pose)
    a = sqrt(OOIs.Centers(1,1)^2 + OOIs.Centers(2,1));
    b = sqrt(OOIs.Centers(1,end)^2 + OOIs.Centers(2,end));
    c = sqrt((OOIs.Centers(1,1)-OOIs.Centers(1,end))^2 + (OOIs.Centers(2,1)-OOIs.Centers(2,end))^2);
    theta = acos((a^2 + c^2 - b^2)/(2*a*c));
    [vx] = [LM.Centers(1,end) - LM.Centers(1,1)];
    [vy] = [LM.Centers(2,end) - LM.Centers(2,1)];
    [vx2] = [vx*cos(-theta) - vy*sin(-theta)];
    [vy2] = [vx*sin(-theta) + vy*cos(-theta)];
    x = LM.Centers(1,1) + (a/norm([vx2,vy2])*(vx2));
    y = LM.Centers(2,1) + (a/norm([vx2,vy2])*(vy2));
 return;
 end
 
 function [x,y] = Linearization(OOIs,LM,Pose)
    vx = [LM.Centers(1)-Pose(1)];
    vy = [LM.Centers(2)-Pose(2)];
    D = sqrt((LM.Centers(1) - Pose(1))^2 + (LM.Centers(2) - Pose(2))^2);
    d = sqrt(OOIs.Centers(1)^2 + OOIs.Centers(2)^2);
    k = (D-d)/D;
    x = Pose(1) + k*vx;
    y = Pose(2) + k*vy;
 return;
 end
 
 % ===========================================\
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
 function index = GetPolesIndex(ranges)
    index = [];
    
    
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
            if isempty(index)   %If value is empty
                index = i;
            else
                index = [index(1,:),i];
            end
        
        % Collect poles
            j = i - 1;
            while (abs(ranges(j) - ranges(i)) < 3) && (j >= 1)
                index = [index(1,:),j];
                j = j-1; 
            end
            i = j;
            
        end
        i = i-1;
    end
    return;
 end
 
 function r = ExtractOOIs(ranges,X,Y,Poles_index)
    r.N = 0;
    r.Centers = [];
    r.Sizes   = []; 
%     r.Ranges = [];
    count = 0;

    for i = 1:length(ranges)-1

        if abs(Poles_index(i)-Poles_index(i+1)) > 1 || i == length(ranges)-1% index should be consecutive if they belong same poles
            
            % Center
            x = mean(X((i-count):i));
            y = mean(Y((i-count):i));
            
            % Diameter (Size)
            diameter = sqrt((X(i-count)-X(i))^2+(Y(i-count)-Y(i))^2);
                    
            % Range
%              = sqrt(x^2 + y^2);
            
                if isempty(r.Centers)       %If value is empty
                    r.Centers(1,:) = x;
                    r.Centers(2,:) = y;
                    r.Sizes = diameter;
                else
                    r.Centers = [r.Centers(1,:),x;r.Centers(2,:),y];
                    r.Sizes = [r.Sizes,diameter];
                end
                
                r.N = r.N + 1;
            count = 0;    
        else
            count = count + 1;
        end
    end

 return;
 end
 
 function OOIs = PlotOOIs(OOIs,Pose)
    if OOIs.N<1,  return ; end;
    
    xx = OOIs.Centers(1,:);
    yy = OOIs.Centers(2,:);
    
    % Rotation
    heading = Pose(3);
    rotate = heading - 0 ; %disp(rotate);
    x = xx*cos(rotate)-yy*sin(rotate);
    y = xx*sin(rotate)+yy*cos(rotate);
    
    % Translation
    x = x + Pose(1);
    y = y + Pose(2);
    
    % return global coordinate
    OOIs.Centers(3:4,:) = [x;y];
return;
 end
 
 function [OOIs,LM,OG] = PrintID(OOIs,Landmarks,OG)
    count = 0;
    tempCenter = [];
    tempUsedLM = [];
    for k = 1:OOIs.N
%         if OOIs.Centers(3,k)>200 || OOIs.Centers(4,k)>200 || OOIs.Centers(3,k)<-5 || OOIs.Centers(4,k)<-5,continue;end;
        for h = 1:Landmarks.n
            d = sqrt((OOIs.Centers(3,k)-Landmarks.xy(1,h))^2+(OOIs.Centers(4,k)-Landmarks.xy(2,h))^2);
            if d < 25    % threshold of DA
                % store occupied index of LM
                if ismember(OG,h)==0
                    OG = [OG(1,:),h];
                end
                
                if isempty(tempCenter)
                    tempCenter = OOIs.Centers(:,k);
                    tempUsedLM = Landmarks.xy(:,h);
                else
                    tempCenter = [tempCenter,OOIs.Centers(:,k)];
                    tempUsedLM = [tempUsedLM,Landmarks.xy(:,h)];
                end
                count = count + 1;
                break; 
            end;
        end
    end
    OOIs.N = count;
    OOIs.Centers = tempCenter;
    LM.Centers = tempUsedLM;
return;
 end
