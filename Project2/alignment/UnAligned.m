% ExampleUseDepthDataFromFileD.m : 
function main(file)
    
    if ~exist('file','var'),   file =  'DepthData01.mat' ;   end;   
    load(file); 
    L = CR.N;            
    API = IniAPIGetPointCloudFromDepth();
    API.SetProjectionConstants(1,340,0.001848, 240,0.001865); 
    fprintf('(using API ver=[%.1f])\n',API.Info.version);
    
    flag1 = 0;                   
    LineOfInterest = 60;
    for i=1:L             % just a loop..
        
        Depth = CR.R(:,:,i);  
        [xx,yy,zz]=API.ConvertDepthsTo3DPoints(Depth,0.1) ;  
        xxH = xx(LineOfInterest,:); yyH = yy(LineOfInterest,:); zzH = zz(LineOfInterest,:);
        xxV = xx(:,80);  yyV = yy(:,80);    zzV = zz(:,80);
        
        if flag1,  % 
%                 set(h1,'cdata',Depth);
                set(h2,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));
                set(h3,'xdata',xxH,'ydata',yyH,'zdata',zzH);
                set(h3b,'xdata',xxV,'ydata',yyV,'zdata',zzV);
        else,   %First time: we create Matlab graphical objects
            flag1=1;
            figure(1) ; clf() ;  
            
%             subplot(211) ; h1 = imagesc(Depth,[100,2000]); 
%             % image, scalling color for values in range from 100mm to 2000.
%             
%             set(gca(),'xdir','reverse');    
%             % this way, that image looks better (for human brains..)
%             colormap gray ; 
%             zoom on;   title('Depth, shown as an image');
%             % Ask Matlab (using command help), about all these functions:
%             %  set(),gca(),subplot(),imagesc(),colormap(),etc.
%             
%             subplot(212);
            h2 = plot3(xx(1:end),yy(1:end),zz(1:end),'.b','markersize',1);
            axis([0,200,-125,125,-30,60]); 
            rotate3d on; grid on;
            xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
            title('3D Points Cloud (view from camera)');
            hold on;
            h3 = plot3(xxH,yyH,zzH,'.r','markersize',5);
            h3b = plot3(xxV,yyV,zzV,'.g','markersize',5);
        end;
        
        pause(0.05);     
    end;
    
    disp('Done....');
 return;