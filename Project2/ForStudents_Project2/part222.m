%Author: ZIQI GUO , Z3446153
%Program: Solution for RD, S1.2018, Project02.part2

function main(file)
    
    % Default file name, if caller does not especify it.
    if ~exist('file','var'),   file =  'DepthData01.mat' ;   end;    

    file2 = 'IMU_data.mat'; 
    load(file);        % here we load the file, specified by the caller.
    load(file2);
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
    
    
    times = CR.H(1,:)/10000;
    
    %...............................
    flag1 = 0;                    % just a flag, for indicating certain initialization. 

    LineOfInterest = 60;
    % I choose this one (horizontal line 60). If the camera is kept pointing ahead the robot,
    % horizontally, then this line is the one that corresponds to the
    % horizontal plane (the image is 120 pixels, vertically). So, we may
    % use this line of the Depth image, to "emulate" a 2D laser scanner.
    x_ROI = 90:120;
    y_ROI = 80:100;
    mean_z = zeros(5,31);
    RollStore = [];
    PitchStore = [];
    
    for i=1:L,             % just a loop..
        
        Depth = CR.R(:,:,i);  %get a copy of depth image #i;
        [xx,yy,zz]=API.ConvertDepthsTo3DPoints(Depth,0.1) ;   
        xxi = xx(x_ROI,y_ROI);
        yyi = yy(x_ROI,y_ROI);
        zzi = zz(x_ROI,y_ROI); 
        %-----------Section 5 --------------------
        %--------------delete error---------
        deleted = Depth(x_ROI,y_ROI)==0;
        xxi(deleted) = [];
        yyi(deleted) = [];
        zzi(deleted) = [];

%         %----------Compare z value with standard z--------------
%         if i>5
%             mask = (zzi<=mean_z_standard+2)&(zzi>=mean_z_standard-2);
%             xxi = xxi(mask);
%             yyi = yyi(mask);
%             zzi = zzi(mask);
%         end
        %=============================================================
        v = [100,120;80,120;80,90;100,90];
%         v=[115,72;82,72;82,109;115,109];
        f = [1,2,3,4];
        %-------------------------------------------------------------------
       
        
        %----------------Section (3)- Obtain the inclination---------------
        %Linear Regression
        x1 = xxi(:); 
        x2 = yyi(:);
        y = zzi(:);

        X = [ones(size(x1)) x1 x2];
        %Normal 
        %b(1) is C,b(2) is A, b(3) is B

        [b,bint,r,rint,stats] = regress(y,X);
        % Save the previous b
        
        if stats(1)>0.97
           b_prior = b; 
        else 
            b = b_prior;
            
        end
        %Create two vector
        a = [b(2),b(3),-1];
        b1 = [b(2),b(3),0];
        %Normal vector is [A;B;-1]

        % Now, this part of the example is just for visualizing the data.
        % We plot "Depth" as an image (in grayscale), and its associated 3D points..


        %Attention:  xx,yy,zz are still including "faulty points".
        %You should remove those bad pixels, before processing them.
        %
        xmean = mean([min(xxi),max(xxi)]);
        %ymean = mean(yyi(yyi>0));
        ymean = mean([min(yyi),max(yyi)]);
        zmean = mean([min(zzi),max(zzi)]);
        z = b(1) + b(2)*xmean +  b(3)*ymean;
        u = b(2)*(-20);v1 = b(3)*(-20);w = -1*(-20);

        if flag1,  % 
% %                 set(h1,'cdata',Depth);
% %                 set(h2,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));
% %                 set(h3,'xdata',x1(1:end),'ydata',x2(1:end),'zdata',y(1:end));
% %                 %set(h3,'xdata',xxH,'ydata',yyH,'zdata',zzH);
% %                % set(h3b,'xdata',xxV,'ydata',yyV,'zdata',zzV);
% %                 set(h4,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));
% %                 set(h5,'xdata',x1(1:end),'ydata',x2(1:end),'zdata',y(1:end));
% %                 set(h6,'xdata',xmean,'ydata',ymean,'zdata',zmean);
                Pitch = asin(b(2))*180/pi;
                Roll = asin(b(3)/(-cos(asin(b(2)))))*180/pi;
% %                 rotate(h4,[1,0,0],Roll);
% %                 rotate(h4,[0,1,0],Pitch);
% %                 rotate(h5,[1,0,0],Roll);
% %                 rotate(h5,[0,1,0],Pitch); 
% %                 rotate(h4,[-b(3), 1, -b(2)],90-acos(dot(a, b1) / (norm(a) * norm(b1)))*180/pi);
% %                 rotate(h5,[-b(3), 1, -b(2)],90-acos(dot(a, b1) / (norm(a) * norm(b1)))*180/pi);
        else,   %First time: we create Matlab graphical objects
            flag1=1;
% %             figure(1) ; clf() ;  
% % 
% %             subplot(211) ; h1 = imagesc(Depth,[100,2000]); 
% %             % image, scalling color for values in range from 100mm to 2000.
% %             %-----------Section 2. Plot the Bondaries of the ROI ----------
% %             patch('Faces',f,'Vertices',v,'FaceColor','red','faceAlpha',0.5);
% %             set(gca(),'xdir','reverse');    
% %             %-------------------------------------------------------------
% %             % this way, that image looks better (for human brains..)
% %             colormap gray ; 
% %             zoom on;   title('Depth, shown as an image');
% %             % Ask Matlab (using command help), about all these functions:
% %             %  set(),gca(),subplot(),imagesc(),colormap(),etc.
% % 
% %             subplot(212);
% %             %------------plot the normal vector------------
% % 
% %             h6 = quiver3(xmean,ymean,zmean,u,v1,w, 'linewidth', 1*3, 'color', 'g');
% %             hold on;
% %             h3 = plot3(x1(1:end),x2(1:end),y(1:end),'.r');hold on;
% %             h2 = plot3(xx(1:end),yy(1:end),zz(1:end),'.b','markersize',1);
% % 
% %             axis([0,200,-125,125,-30,60]); 
% %             rotate3d on; grid on;
% %             xlabel('X (cm)'); ylabel('Y (cm)'); zlabel('Z (cm)');
% %             title('3D Points Cloud (view from camera)');
% %             hold off;
% %            % h3 = plot3(xxH,yyH,zzH,'.r','markersize',5);
% %            % h3b = plot3(xxV,yyV,zzV,'.g','markersize',5);
% %            %------------Section Part 4--------------------------
% %            figure(2),clf();
% %            axis([0,200,-125,125,-50,100]);  hold on;
% %            %Get the roll and pitch
% % %
% %            h4 = plot3(xx(1:end),yy(1:end),zz(1:end),'.b','markersize',1);
% %            hold on;
% %            h5 = plot3(x1(1:end),x2(1:end),y(1:end),'.r');

           Pitch = real(asin(b(2))*180/pi);
           Roll = real(asin(b(3)/(-cos(asin(b(2)))))*180/pi);
% %            %Rotate Roll and Pitch 
% %            rotate(h4,[1,0,0],Roll);
% %            rotate(h4,[0,1,0],Pitch);
% %            rotate(h5,[1,0,0],Roll);
% %            rotate(h5,[0,1,0],Pitch); 
%            %Store Roll and Pitch Value
          
           %---------------OPTION 2 USING DOT PRODUCT----------------
%            rotate(h4,[-b(3), 1, -b(2)],90-acos(dot(a, b1) / (norm(a) * norm(b1)))*180/pi);
%            rotate(h5,[-b(3), 1, -b(2)],90-acos(dot(a, b1) / (norm(a) * norm(b1)))*180/pi);
           %------------------------------------------------------------------------
        end
        pause(0.1); %Is that for frequency not lower than 1HZ?    
         %here, we pause for approximately 100ms, before showing next image.
        RollStore = [RollStore,Roll];
        PitchStore = [PitchStore,Pitch];
       
    end
    
    figure(2),clf()
    plot(times,RollStore);
    title('Roll Rotation Angle');
    xlabel('times(sec)');
    ylabel('Angular Rate(Degree)');
    disp('Done....');
 return;


 