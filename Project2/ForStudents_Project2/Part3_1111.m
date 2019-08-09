%Author: Po-Heng Chen , Z5014392
%Program: Solution for RD, S1.2018, Project01.PartA

% =========================== CR.commen5t =================================
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
    
    LineOfInterest = 60;
    for i = 1:L             % just a loop..
        Depth = CR.R(:,:,i);  %get a copy of depth image #i;
        [xx,yy,zz]=API.ConvertDepthsTo3DPoints(Depth,0.1) ;  
        xxR = xx(105:120,65:95);  yyR = yy(105:120,65:95);  zzR = zz(105:120,65:95);
        xr = xxR(1:end)';yr = yyR(1:end)';zr = zzR(1:end)';
        xxH = xx(LineOfInterest,:); yyH = yy(LineOfInterest,:); zzH = zz(LineOfInterest,:);
        ranges = GetRanges(xxH,yyH);
        Poles = ExtractOOIs(ranges,xxH,yyH);
        
        if flag1,  % 
            set(h1,'cdata',Depth);
            set(h3,'xdata',xxH,'ydata',yyH,'zdata',zzH);
        else,   %First time: we create Matlab graphical objects
            flag1=1;
            figure(1) ; clf() ;  
            
            subplot(211) ; 
            h1 = imagesc(Depth,[100,2000]); % image, scalling color for values in range from 100mm to 2000.
            set(gca(),'xdir','reverse');    
            colormap gray ; zoom on;   title('Depth, shown as an image');
            
            subplot(212);
            axis([0,200,-125,125,-30,60]); xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
            rotate3d on; grid on; hold on; title('3D Points Cloud (view from camera)');
            
            h3 = plot3(xxH,yyH,zzH,'.r','markersize',5);
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
 