
function [] = MTRN4110S12018Lab()
    clc; clear ; close all ; dbstop if error ;
    global fig; fig=[];
    global img;
    img.V=300; img.U=400; img.L=256; img.N=img.V*img.U;
    [fn,pn,bn]=uigetfile('..//Images//*.jpg');
    if bn==1 % bn = 1 means it's a .m file
        JPG=imread([pn fn]);
        [img,RGB] = MyImResize(img,JPG);
        % ======= object detection ==========
%         [Fx,Fy] = gradient(RGB);
%         GRD = gradient(RGB); 
%         fig(end+1) = MyFigure();
%         MyImShow(GRD);
        
        
%         % ====================================
%         FL = JPG(:,:,1)-JPG(:,:,2)-JPG(:,:,3);
%         figure(66);
%         surf(flipud(FL),'edgecolor','none');
%         colorbar;
%         % ======== Filtering ===========
%         HSV = rgb2hsv(RGB);
%         gry = HSV(:,:,3);
%         H = fspecial('gaussian',9,5);%segma(std)=0.5
%         gry_f = imfilter(gry,H);
%         figure(89) ; imshow(gry_f);
%         gry1_n = imnoise(gry_f,'salt & pepper');
%         figure(34) ; imshow(gry1_n);
        % ======== Matching Histogram ========
        %RGB = MyHistogramSpecification(RGB);
        
        % ======= Modified Auto-Contrast ======
        %RGB = MyAutoContrastAdj(RGB);
        
        % ======== Histogram Equalization ======
        %RGB = histeq(RGB(:,:,3));
        
        fig(end+1) = MyFigure(); 
    %     MyImShow(JPG);
        MyImShow(RGB);
    end
end

function [RGB] = MyMedianFiliter(RGB)
    global img;
    HSV = rgb2hsv(RGB);
    gry = HSV(:,:,3);
    RGB = medfilt2(RGB);
    figure(212); 
    imshow(RGB);
end

function [RGB] = MyHistogramSpecification(RGB)
    global img;
    HSV = rgb2hsv(RGB);
    gry = HSV(:,:,3);
    [hx,ix] = hist(gry(:),256);
    plot(ix,hx,'r') ; hold on ;
    hx2 = sin(linspace(0,1,img.L)*pi);
%         plot(hx2/256,'b');
    RGB = histeq(gry,hx2);
end

function [jmg] = MyAutoContrastAdj(jmg)
    low = 0.5; high = 1; 
    R_min = min(jmg(:,:,1));R_max = max(jmg(:,:,1));
    G_min = min(jmg(:,:,2));G_max = max(jmg(:,:,2));
    B_min = min(jmg(:,:,3));B_max = max(jmg(:,:,3));
    jmg(:,:,1) = R_min + (jmg(:,:,1) - R_min).*((R_max-R_min)/(high-low));
    jmg(:,:,2) = G_min + (jmg(:,:,2) - G_min).*((G_max-G_min)/(high-low));
    jmg(:,:,3) = B_min + (jmg(:,:,3) - B_min).*((B_max-B_min)/(high-low));
end

function [fig] = MyFigure()
    global fig;
    fig = figure('units','normalized');
    set(gcf,'position',[rand*0.1+0.2 rand*0.1+0.2 0.3 0.3]);
end

function [] = MyImShow(jmg)
    global fig img;
    figure(fig(end+1)); imshow(jmg); drawnow;
    set(gca,'position',[0 0 1 1]);
end

function [img,jmg] = MyImResize(img,jmg)
    [v,u,w]=size(jmg); % 512*768*3
    if v < u
        jmg=imresize(jmg,[img.V img.U],'bilinear');
    elseif v > u
        jmg=imresize(jmg,[img.U img.V],'bilinear');
    else
        jmg=imresize(jmg,[img.U img.U],'bilinear');
    end;
    [v,u,w]=size(jmg); %300*400*3
    img.V=v; 
    img.U=u; 
    img.N=img.V*img.U;
    if max(jmg(:)) > 1
        jmg=double(jmg)/img.L;
    end;
end