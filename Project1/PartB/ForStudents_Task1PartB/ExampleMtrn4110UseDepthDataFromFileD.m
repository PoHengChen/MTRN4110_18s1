%................................................................
% ExampleUseDepthDataFromFileD.m : 

% This example shows how to use Depth data, in an off-line fashion.

% We also convert the Depth image for obtaiing a 3D points cloud (using an API), and we show it in a figure.
% The example also shows you how to implement a basic animation for visualizing the data.

% In addition, we focus our attention on one of the lines of the image (one of the horizontal
% lines of the image. We simulate a laser scanner.

% You should also pay attention to many of the Matlab's functions I am
% using (*). Visualization is not needed by the robot brain, but it is
% needed by us, to verify that our processing is performing well.

% There are less than 40 lines of code. The program looks big due to the
% comments; 90% of the code is just for plotting data.
% (btw: Comments are there to be read.)


% Jose Guivant. For MTRN4110, S1.2018
%................................................................


function main(file)
    
    % Default file name, if caller does not especify it.
    if ~exist('file','var'),   file =  'DepthData01.mat' ;   end;    

    
    load(file);        % here we load the file, specified by the caller.
    
    % The data has name "CR", when you load the file. 
    % You can change it, if you want, e.g. -->  MyData=CR ; clear CR;
    % The data is stored in a structure, whose relevant fields are:
    %   field "N"     : number of images in this dataset.
    %   field "R"     : a big matrix containing all the Depth images
    %   field "t"     : a vector containing all the images' sample times
    %   field "comment" : Useful comments.

    
    % We use the data now.
    L = CR.N;              % how many images are this dataset?  
    

    API = IniAPIGetPointCloudFromDepth();
    % This is a small API, for corverting depth data into 3D points. 
    % Here the variable API is an instance of that API.

    %...............................
    % Fistly, we call this method of the API, for setting certain calibration constants. 
    % These "constants" are used in the calculation of 3D points, for the
    % depth values of the image's pixels.
    % Keep these values I propose, although those could be improved after calibration.
    % The values are expressed for images 640x480, but internally adapted
    % to our resolution of 160x120 (I subsampled the original images for saving space and processing cost;
    % but they still cover the full FoV, as the original full resolution images did.)
    API.SetProjectionConstants(1,340,0.001848, 240,0.001865); 
    fprintf('(using API ver=[%.1f])\n',API.Info.version);
    
    % The internals of this API are not of interest for us, because we
    % focus on the application. 
    
    
    
    %...............................
    flag1 = 0;                    % just a flag, for indicating certain initialization. 

    LineOfInterest = 60;
    % I choose this one (horizontal line 60). If the camera is kept pointing ahead the robot,
    % horizontally, then this line is the one that corresponds to the
    % horizontal plane (the image is 120 pixels, vertically). So, we may
    % use this line of the Depth image, to "emulate" a 2D laser scanner.
    
    for i=1:L,             % just a loop..
        
        Depth = CR.R(:,:,i);  %get a copy of depth image #i;
         
        % Call API method for obtaining its associated 3D points.
        % scale factor =0.1, so the resulting 3D points are expressed in Cm.
        
        %ConvertDepthsTo3DPoints(Depth,scale) ;   <--- Depth is a depth
        %image; scale is the scaling factor from mm to desired units.
        
        [xx,yy,zz]=API.ConvertDepthsTo3DPoints(Depth,0.1) ;  
        % The 3D points are reported via 3 matrixes, of the same size as the Depth image, 
        % but containing the coordinates (x,y,z) of each pixel.
        % e.g. suppose pixel (i,j) : whose depth is Depth(i,j), will have its 
        % associated 3d point at (xx(i,j),yy(i,j),zz(i,j))
        % The variables xx,yy,zz are class single ( single precision, for real numbers)
        % As, in this case, we specified scale=0.1, the contents of xx,yy,zz will be given in CM
        % btw: see note (**), about coordinate frame conventions.
     
        
        % If you want to see the 3D points associated to the ROW
        % "LineOfInterest", you just extract it, in this way:
        xxH = xx(LineOfInterest,:); yyH = yy(LineOfInterest,:); zzH = zz(LineOfInterest,:);
        % those correspond to one horizontal line of the camera ( "LineOfInterest" ).
        
        
        % and here, I get the vertical line #80
        xxV = xx(:,80);  yyV = yy(:,80);    zzV = zz(:,80);
        
        % Now, this part of the example is just for visualizing the data.
        % We plot "Depth" as an image (in grayscale), and its associated 3D points..
        

        %Attention:  xx,yy,zz are still including "faulty points".
        %You should remove those bad pixels, before processing them.
        
        
        if flag1,  % 
                set(h1,'cdata',Depth);
                set(h2,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));
                set(h3,'xdata',xxH,'ydata',yyH,'zdata',zzH);
                set(h3b,'xdata',xxV,'ydata',yyV,'zdata',zzV);
        else,   %First time: we create Matlab graphical objects
            flag1=1;
            figure(1) ; clf() ;  
            
            subplot(211) ; h1 = imagesc(Depth,[100,2000]); 
            % image, scalling color for values in range from 100mm to 2000.
            
            set(gca(),'xdir','reverse');    
            % this way, that image looks better (for human brains..)
            colormap gray ; 
            zoom on;   title('Depth, shown as an image');
            % Ask Matlab (using command help), about all these functions:
            %  set(),gca(),subplot(),imagesc(),colormap(),etc.
            
            subplot(212);
            h2 = plot3(xx(1:end),yy(1:end),zz(1:end),'.b','markersize',1);
            axis([0,200,-125,125,-30,60]); 
            rotate3d on; grid on;
            xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
            title('3D Points Cloud (view from camera)');
            hold on;
            h3 = plot3(xxH,yyH,zzH,'.r','markersize',5);
            h3b = plot3(xxV,yyV,zzV,'.g','markersize',5);
        end;
        
        pause(0.1);     
        %here, we pause for approximately 100ms, before showing next image.
    end;
    
    disp('Done....');
 return;

%................................................................
% Note(*) Use Matlab's help for details about the following functions:
% load ; pause ; disp; figure ; clf ; plot3; image or imagesc ; axis ;

% (**)
% Note about the coordinate frame convention we use for 3D imagery:
        % for the camera we use the following coordinate frame convention:
            % X>0  : semiaxis pointing ahead the camera  (forward)
            % Y>0  : semiaxis pointing to the left of the camera  (left)
            % Z>0  : semiaxis pointing UP
        
            % Yaw>0 rotates respect to Z, from +X axis to +Y axis (i.e. to the left)
            % Pitch >0 rotates respect to Y, from +Z to +X ( i.e. nodding down)
            % Roll >0 rotates respect to X, from +Y to +Z (inclinating to Rigth-Down)


%................................................................
% Questions: via MTRN4110's Moodle forum or email the lecturer: <j.guivant@unsw.edu.au>
%................................................................

 
 