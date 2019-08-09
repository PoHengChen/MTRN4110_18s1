            set(h1,'cdata',Depth);
            set(h2b,'xdata',xxR(1:end),'ydata',yyR(1:end),'zdata',zzR(1:end));  %blue
            set(h2,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));      %ROI(pink)
            set(h3,'xdata',xxR(end/2),'ydata',yyR(end/2),'zdata',zzR(end/2),'udata',n(1),'vdata',n(2),'wdata',n(3));   %normal (red)
            set(h4a,'xdata',xxH,'ydata',yyH,'zdata',zzH);
            set(h4b,'xdata',xxH,'ydata',yyH,'zdata',zzH);
            set(h4c,'xdata',xxH(Poles_index),'ydata',yyH(Poles_index),'zdata',zzH(Poles_index));
            set(h5,'xdata',xxx(1:end),'ydata',yyy(1:end),'zdata',zzz(1:end));      %ROI(pink)

% ==========================================================================
            % Depth plot
            figure(1) ; clf() ;  
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