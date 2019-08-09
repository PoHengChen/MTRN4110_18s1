
function [] = MTRN4110S12018Lab()
    clc; clear ; close all ; dbstop if error ;
    global fig; fig=[];
    global img;
    img.V=300; img.U=400; img.L=256; img.N=img.V*img.U;
    [fn,pn,bn]=uigetfile('..//Images//*.jpg');
    if bn==1 % bn = 1 means it's a .m file
        JPG = imread([pn fn]); %fig(end+1) = MyFigure(); MyImShow(JPG);
        [img,RGB] = MyImResize(img,JPG);
%%  B. Computer Vision
        % ====================================
        % == Tree-dimensional Construction ===
        % ====================================
        MyStereo();
        % ====================================
        % ========= Object Tracking ==========
        % ====================================
%         MyKeyPoint(RGB)
        % ====================================
        % ========= Object Detection =========
        % ====================================
%         MyGradient(RGB);
%         MyEdge(RGB);
%         MyHoughLine(RGB);
%         MyFindCircle(RGB);
%         MyFindCorner(RGB);
%%  A. Image Processing
        % ====================================
        % ========= Image Segmentation =======
        % ====================================
%         MySimpleThreshold(RGB);
%         MyOtus(RGB);
%         MyISO(RGB);
%         MyMaxEntropy(RGB);
%         MyBernsenThresh(RGB,5);
%         MyNiblackThresh(RGB,21,0.2);
%         MyWavelet1(RGB);
        % ====================================
        % ========= Image Filtering ==========
        % ====================================
%         Myfiltering(RGB);    
%         MyMedianFiliter(RGB);
%         MyMedian_Difference(RGB,9);
        % ====================================
        % ======== Image Enhancement =========
        % ====================================
%         MyHistogramSpecification(RGB);
%         MyAutoContrastAdj(RGB,0,1);
    end
end

%% ==== Three-dimensional Construction ====
function []=MyStereo()
    global fig;
    % Load the stereoParameters object.
    load('handshakeStereoParams.mat');
    % Visualize camera extrinsics.
    showExtrinsics(stereoParams);
    % Create System Objects for reading and displaying the video
    videoFileLeft = 'handshake_left.avi';
    videoFileRight = 'handshake_right.avi';
    readerLeft=vision.VideoFileReader(videoFileLeft,'VideoOutputDataType','uint8');
    readerRight=vision.VideoFileReader(videoFileRight,'VideoOutputDataType','uint8');
    player = vision.VideoPlayer('Position', [20, 400, 850, 650]);
    % Read and Rectify Video Frames
    frameLeft = readerLeft.step();
    frameRight = readerRight.step();
    [frameLeftRect, frameRightRect] = ...
        rectifyStereoImages(frameLeft, frameRight, stereoParams);
    fig(end+1)=MyFigure();
    imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
    title('Rectified Video Frames');
    % Compute Disparity
    frameLeftGray = rgb2gray(frameLeftRect);
    frameRightGray = rgb2gray(frameRightRect);
    disparityMap = disparity(frameLeftGray, frameRightGray);
    fig(end+1)=MyFigure();
    imshow(disparityMap, [0, 64]);
    title('Disparity Map');
    colormap jet; colorbar;
    % Reconstruct the 3-D Scene
    point3D = reconstructScene(disparityMap, stereoParams);
    % Limit the range of Z and X for display.
    z = point3D(:,:,3);
    z(z < 0 | z > 8000) = NaN;
    x = point3D(:,:,1);
    x(x < -3000 | x > 3000) = NaN;
    point3D(:,:,3) = z; point3D(:,:,1) = x;
    fig(end+1)=MyFigure();
    pcshow(point3D, frameLeftRect, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
    xlabel('X (mm)');ylabel('Y (mm)');zlabel('Z (mm)');
    set(gca, 'CameraViewAngle',10, 'CameraUpVector',[0 -1 0],...
        'CameraPosition',[16500 -13852 -49597], 'DataAspectRatio',[1 1 1]);
    title('Reconstructed 3-D Scene');
    % Detect People in the Left Image
    % Create the people detector object. Limit the minimum object size for speed.
    peopleDetector = vision.PeopleDetector('MinSize', [166 83]);
    % Detect people.
    bboxes = peopleDetector.step(frameLeftGray);
    % Determine The Distance of Each Person to the Camera
    % Find the centroids of detected people.
    centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), ...
        round(bboxes(:, 2) + bboxes(:, 4) / 2)];
    % Find the 3-D world coordinates of the centroids.
    centroidsIdx = sub2ind(size(disparityMap), centroids(:, 2), centroids(:, 1));
    X = point3D(:, :, 1); Y = point3D(:, :, 2); Z = point3D(:, :, 3);
    centroids3D = [X(centroidsIdx)'; Y(centroidsIdx)'; Z(centroidsIdx)'];
    % Find the distances from the camera in meters.
    dists = sqrt(sum(centroids3D .^ 2)) / 1000;
    % Display the detected people and their distances.
    labels = cell(1, numel(dists));
    for i = 1:numel(dists)
        labels{i} = sprintf('%0.2f meters', dists(i));
    end
    fig(end+1)=MyFigure();
    imshow(insertObjectAnnotation(frameLeftRect, 'rectangle', bboxes, labels));
    title('Detected People');
    % Process the Rest of the Video
    while ~isDone(readerLeft) && ~isDone(readerRight)
        % Read the frames.
        frameLeft = readerLeft.step();
        frameRight = readerRight.step();
        % Rectify the frames.
        [frameLeftRect, frameRightRect] = ...
        rectifyStereoImages(frameLeft, frameRight, stereoParams);
        % Convert to grayscale.
        frameLeftGray = rgb2gray(frameLeftRect);
        frameRightGray = rgb2gray(frameRightRect);
        % Compute disparity.
        disparityMap = disparity(frameLeftGray, frameRightGray);
        % Reconstruct 3-D scene.
        point3D = reconstructScene(disparityMap, stereoParams);
        % Detect people.
        bboxes = peopleDetector.step(frameLeftGray);
        if ~isempty(bboxes)
            % Find the centroids of detected people.
            centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), ...
            round(bboxes(:, 2) + bboxes(:, 4) / 2)];
            % Find the 3-D world coordinates of the centroids.
            centroidsIdx = sub2ind(size(disparityMap), ...
            centroids(:, 2), centroids(:, 1));
            X = point3D(:, :, 1);
            Y = point3D(:, :, 2);
            Z = point3D(:, :, 3);
            centroids3D = [X(centroidsIdx), Y(centroidsIdx),...
            Z(centroidsIdx)];
            % Find the distances from the camera in meters.
            dists = sqrt(sum(centroids3D .^ 2, 2)) / 1000;
            % Display the detect people and their distances.
            labels = cell(1, numel(dists));
            for i = 1:numel(dists)
                labels{i} = sprintf('%0.2f meters', dists(i));
            end
            dispFrame = insertObjectAnnotation(frameLeftRect, ...
            'rectangle', bboxes, labels);
            else
            dispFrame = frameLeftRect;
        end
        % Display the frame.
        step(player, dispFrame);
    end
    % Clean up
    reset(readerLeft);
    reset(readerRight);
end

%% =========== Object Tracking ============
function []=MyKeyPoint(jmg)
    global fig img;
    gry=rgb2gray(jmg);
    points=detectSURFFeatures(gry,...
        'MetricThreshold',1000,'NumOctaves',3,'NumScaleLevels',4);
    [features,valid_points]=extractFeatures(gry, points);
    fig(end+1)=MyFigure(); MyImShow(gry); hold on;
    plot(valid_points.selectStrongest(10),'showOrientation',true);
end

%% ========= Object Dtection ========
function [Fx,Fy] = MyGradient(RGB)
    global fig;
    gry = rgb2gray(RGB);
    
    [Fx,Fy] = gradient(RGB);
    fig(end+1) = MyFigure(); MyImShow(Fx);
    fig(end+1) = MyFigure(); MyImShow(Fy);
    
    [Gmag,Gdir] = imgradient(gry);
    
    % Gradient magnitude
    fig(end+1) = MyFigure(); MyImShow(Gmag);
    % Gradient angle (orientation)
    fig(end+1) = MyFigure(); MyImShow(Gdir);

end

function MyEdge(RGB)
    global fig;
    gry = rgb2gray(RGB);
    jmg = edge(gry,'Sobel');    fig(end+1) = MyFigure();  MyImShow(jmg);
    jmg = edge(gry,'Prewitt');  fig(end+1) = MyFigure();  MyImShow(jmg);
    jmg = edge(gry,'Roberts');  fig(end+1) = MyFigure();  MyImShow(jmg);
    jmg = edge(gry,'Log');      fig(end+1) = MyFigure();  MyImShow(jmg);
    jmg = edge(gry,'zerocross');fig(end+1) = MyFigure();  MyImShow(jmg);
    
    % "Canny Edge detection" work well on curvatured edge
    % but still not satisfactory?????
    jmg = edge(gry,'Canny');    fig(end+1) = MyFigure();  MyImShow(jmg);
end

function MyHoughLine(jmg)
    global fig;
    gry=rgb2gray(jmg); z=5; gry=gry(z:end-z,z:end-z);
    edg=edge(gry,'canny');
    fig(end+1)=MyFigure(); MyImShow(edg);
    
    [H,theta,rho] = hough(edg); 
    fig(end+1) = MyFigure(); imshow(imadjust(mat2gray(H)),[],'XData',theta,'YData',rho,'InitialMagnification','fit');
    xlabel('\theta (degrees)'), ylabel('\rho');axis on, axis normal, hold on;
    axis on, axis normal, hold on;
    % H:==== maximum number of peak (if larger, more line will be detect)
    % threshold:==== (if larger, less point and line will be obtain )
    P=houghpeaks(H,9,'threshold',ceil(1*max(H(:))));
    
    x=theta(P(:,2)); y=rho(P(:,1));
    plot(x,y,'s','color','black');
    
    % ==== FillGap:==== (if larger, less line detected)
    % the distance between the line segments is less than the value specified,
    % the houghlines function merges the line segments into a single line segment.
    % ==== MinLength:==== (manage the line length)
    % Minimum line length, specified as a positive real scalar. 
    % houghlines discards lines that are shorter than the value specified.
    lines = houghlines(edg,theta,rho,P,'FillGap',5,'MinLength',15);
    
    fig(end+1)=MyFigure(); imshow(gry), hold on;
    set(gca,'position',[0 0 1 1]);
    for k = 1:length(lines)
        xy = [lines(k).point1; lines(k).point2];
        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
        % Plot beginnings and ends of lines
        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
    end
end

function MyFindCircle(jmg)
    global fig;
    gry=rgb2gray(jmg); Rmin=15; Rmax=30;
    % specify the detected circles dadius range
    [centers, radii, metric] = imfindcircles(gry,[Rmin Rmax]);
    Thr=0.5; 
    if min(metric)>Thr
        C=length(radii); centersStrong = centers(1:C,:);
        radiiStrong = radii(1:C); metricStrong = metric(1:C);
        viscircles(centersStrong, radiiStrong,'EdgeColor','y');
    end
    
end

function MyFindCorner(jmg)
    global fig ;
    gry=rgb2gray(jmg); jmg=zeros(size(gry));
    Thr=graythresh(gry); jmg(gry>Thr)=1;
    
    % qualitylevel (Minimum accepted quality of corners):
    % if larger, less corner will be detected
    % Sensitivityfactor (Sensitivity factor used in the Harris detection algorithm): 
    % if small, good to detect sharp corner 
    C = corner(jmg,'qualitylevel',0.3,'Sensitivityfactor',0.02);
    fig(end+1)=MyFigure(); MyImShow(gry);
    hold on; plot(C(:,1),C(:,2),'r*');
end

%% ======== Image Segmentation ========
function MySimpleThreshold(RGB)
    % one threshold value for all pixel (simple, not effective)
    global fig;
    gry = rgb2gray(RGB);
    q = 0.5*(min(gry(:)) + max(gry(:)));% mid-range value
    jmg = zeros(size(gry));
    jmg(find(gry > q)) = 1;
    
    fig(end+1) = MyFigure(); MyImShow(jmg); 
end

% similar to IOSDATA
function MyOtus(RGB)
    % a) each as narrow as possible (have minimal variance)
    % b) their centers (means) are most distant from each other
    % threshold is adaptive to image content
    global fig;
    gry = rgb2gray(RGB);
    q = graythresh(gry);
    jmg = zeros(size(gry));
    jmg(find(gry > q)) = 1;
    
    fig(end+1) = MyFigure(); MyImShow(jmg); 
end
% similar to Otus
function MyISO(jmg)
% make initial guess to find q
%automatic adapt to image content
    global fig;
    gry = uint8(rgb2gray(jmg)*255); 
    Qold = 128;
    G1 = gry(find(gry <= Qold)); 
    G2 = gry(find(gry >  Qold));
    m1 = mean(G1);
    m2 = mean(G2);
    Qnew = floor(mean([m1 m2]));
    while abs(Qold - Qnew) > 0 % the loop is making correction
        Qold = Qnew;
        G1 = gry(find(gry <= Qold));
        G2 = gry(find(gry >  Qold));
        m1 = mean(G1);
        m2 = mean(G2);
        Qnew = floor(mean([m1 m2]));
    end
    jmg = zeros(size(gry));
    jmg(find(gry > Qnew)) = 1;
    
    fig(end+1) = MyFigure(); MyImShow(jmg); 
end

function MyMaxEntropy(jmg)
    global fig
    gry = uint8(rgb2gray(jmg)*255);
    for k = 1:255
       G1 = gry(find(gry <= k));
       G2 = gry(find(gry >  k));
       if(length(G1) > 0 & length(G2) > 0)
          E1 = entropy(G1);
          E2 = entropy(G2);
          sumE(k) = E1 + E2;
       end
    end
    [mx,ix] = max(sumE);
    jmg = zeros(size(gry));
    jmg(find(gry > ix)) = 1;
    
    fig(end+1) = MyFigure(); MyImShow(jmg); 
end

function MyBernsenThresh(jmg,sze)
    global fig;
    global img;
    gry = rgb2gray(jmg);
    jmg = zeros(size(gry));
    s = ceil(sze/2);
    for v = s:img.V -s
       for u = s:img.U - s
          R = gry(v-s+1:v+s-1,u-s+1:u+s-1);
          mi = min(R(:));
          mx = max(R(:));
          Q = (mi + mx)/2;
          jmg(v,u) = gry(v,u) > Q;
       end
    end
    
    fig(end+1) = MyFigure(); MyImShow(jmg); 
 end

function MyNiblackThresh(jmg,sze,k)
    global fig;
    global img;
    gry = rgb2gray(jmg);
    jmg = zeros(size(gry));
    s = ceil(sze/2);
    for v = s:img.V - s
        for u = s:img.U - s
            gR = gry(v-s+1:v+s-1,u-s+1:u+s-1);
            mn = mean(gR(:));
            sd = std(gR(:));
            Q = mn + k*sd;
            jmg(v,u) = gry(v,u) > Q;
        end
    end
    fig(end+1) = MyFigure(); MyImShow(jmg); 
end

function MyWavelet1(jmg)
    global fig;
    gry = rgb2gray(jmg);
    fig(end+1 )= MyFigure(); MyImShow(gry);% figure 1
    W = 2; 
    ahvd = {'a','h','v','d'}; wname='db5'; % wavelet name
    [C,S] = wavedec2(gry,W,wname); % decomposition
    for w = 1:W
        fig(end+1)=MyFigure(); % figure 2(high freq), figure 3(low freq)
        set(gcf,'position',[rand*0.1+0.2 rand*0.1+0.1 0.5 0.7]);
        for x=1:4
            if x==1
                % approximation coefficient 
                eval(sprintf('%s%d=appcoef2(C,S,''%s'',%d);',ahvd{x},w,wname,w));
            else
                % detail coefficient 
                eval(sprintf('%s%d=detcoef2(''%s'',C,S,%d);',ahvd{x},w,ahvd{x},w));
            end
            % extended pseudocolor matrix scaling
            subplot(2,2,x); eval(sprintf('image(wcodemat(%s%d,256));',ahvd{x},w));
        end
    end
    for w=1:W
        % single-level inverse discrete 2-D wavelet transform
        eval(sprintf('gry=idwt2(a%d,h%d,v%d,d%d,''%s'');',w,w,w,w,wname));
        fig(end+1)=MyFigure(); MyImShow(gry); % figure 4(high freq), figure 5(low freq)
    end;
    jmg=waverec2(C,S,wname); % multilevel 2-D wavelet reconstruction
end

%% ======== Image Filtering ========
function Myfiltering(RGB)
   % filtering has 2 purpose
   %    1. remove noise
   %    2. highlight wanted features
    global fig;
        HSV = rgb2hsv(RGB); % H:hue, S:saturation, V:brightness 
        h = HSV(:,:,1);
        s = HSV(:,:,2);
        v = HSV(:,:,3);
        
        figure;surf(flipud(v),'edgecolor','none');rotate3d on;
        
        % smoother (blurrer)
%         H = fspecial('average',5);
%         HSV(:,:,3) = imfilter(v,H);
%         figure;surf(flipud(HSV(:,:,3)),'edgecolor','none');rotate3d on;
%         RGB = hsv2rgb(HSV);
%         fig(end+1) = MyFigure(); MyImShow(RGB); 
        
        % sharpness is not removed as much as average filter
        H = fspecial('gaussian',3,.5);%segma(std)=0.5
        HSV(:,:,3) = imfilter(v,H);
        figure;surf(flipud(HSV(:,:,3)),'edgecolor','none');rotate3d on;
        RGB = hsv2rgb(HSV);
        fig(end+1) = MyFigure(); MyImShow(RGB); 
        
        H = fspecial('gaussian',9,.5);%segma(std)=0.5
        HSV(:,:,3) = imfilter(v,H);
        figure;surf(flipud(HSV(:,:,3)),'edgecolor','none');rotate3d on;
        RGB = hsv2rgb(HSV);
        fig(end+1) = MyFigure(); MyImShow(RGB); 
        
        % sharpening edge
%         H = fspecial('laplacian',0.2);%alpah(shape of laplacian)=0.2
%         HSV(:,:,3) = imfilter(v,H);
%         figure;surf(flipud(HSV(:,:,3)),'edgecolor','none');rotate3d on;
%         RGB = hsv2rgb(HSV);
%         fig(end+1) = MyFigure(); MyImShow(RGB); 
        
%         colorbar;

% If some of the filter coefficients are negative, the filter calculation can be in-
% terpreted as the difference of two sums: the weighted sum of all pixels with
% associated positive coefficients minus the weighted sum of pixels with negative
% coefficients in the filter region RH, that is
end

function MyMedianFiliter(RGB)
    global fig;
    HSV = rgb2hsv(RGB); % H:hue, S:saturation, V:brightness 
    h = HSV(:,:,1);
    s = HSV(:,:,2);
    v = HSV(:,:,3);
    fig(end+1) = MyFigure(); MyImShow(v); 
    
    v_n = imnoise(v,'salt & pepper',0.2);
    HSV(:,:,3) = v_n;
    figure;surf(flipud(HSV(:,:,3)),'edgecolor','none');rotate3d on;
    fig(end+1) = MyFigure(); MyImShow(v_n); 
    
    % the size implemented in Median Filter will effect:
    % 1. blurring
    % 2. noise remove
    % (when size is larger: noise remove ability will be better but blurrer)
    v_f = medfilt2(v_n,[9 9]); % default 3x3
    HSV(:,:,3) = v_f;
    figure;surf(flipud(HSV(:,:,3)),'edgecolor','none');rotate3d on;
    fig(end+1) = MyFigure(); MyImShow(v_f); 
end

function MyMedian_Difference(RGB,filter_Size)
    global fig;
    HSV = rgb2hsv(RGB); % H:hue, S:saturation, V:brightness 
    h = HSV(:,:,1);
    s = HSV(:,:,2);
    v = HSV(:,:,3);
    fig(end+1) = MyFigure(); MyImShow(v); 
    
    v_n = imnoise(v,'salt & pepper',0.2);
    HSV(:,:,3) = v_n;
    figure;surf(flipud(HSV(:,:,3)),'edgecolor','none');rotate3d on;
    fig(end+1) = MyFigure(); MyImShow(v_n); 
    
    % the size implemented in Median Filter will effect:
    % 1. blurring
    % 2. noise remove
    v_f = medfilt2(v_n,[filter_Size filter_Size]); % default 3x3
    HSV(:,:,3) = v_f;
    figure;surf(flipud(HSV(:,:,3)),'edgecolor','none');rotate3d on;
    fig(end+1) = MyFigure(); MyImShow(v_f); 
    
    H = fspecial('laplacian',0.2);%alpah(shape of laplacian)=0.2
    HSV(:,:,3) = imfilter(v_f,H);
    figure;surf(flipud(HSV(:,:,3)),'edgecolor','none');rotate3d on;
    fig(end+1) = MyFigure(); MyImShow(HSV(:,:,3)); 
%     RGB = hsv2rgb(HSV);
%     fig(end+1) = MyFigure(); MyImShow(RGB); 
end

%% ======== Image Enhancement ========
function MyHistogramSpecification(RGB)
    global img,
    global fig;
    HSV = rgb2hsv(RGB);
%     gry = HSV(:,:,3);
    gry = RGB(:,:,3);
    fig(end+1) = MyFigure(); MyImShow(gry); % == original
    [hx,ix] = hist(gry(:),256);
    figure(44);
    plot(ix,hx) ; hold on ;
    % positive bell shape curve (middle intensities will be focused on)
    hx2 = sin(linspace(0,1,img.L)*pi);
        plot(size(hx2(:),2),hx2,'r');
    gryeq = histeq(gry,hx2);
    fig(end+1) = MyFigure(); MyImShow(gryeq); % == histogram matching
    
    % Gamma Correction
    % As r > 1 ,0 < a < 1 , a^r < a (darker image)
    gry_gamma1 = gry.^2;
    fig(end+1) = MyFigure(); MyImShow(gry_gamma1); % == gamma correction

end

function [jmg] = MyAutoContrastAdj(jmg,minimum,maximum)
     % scale 0 ~ 255 to 0 ~ 1 (0: darkness / 255: brightness)
     % the input of minimum and maximum will effect which ranges will the
     % new image be scale to.
     % i.e. scale to ranges 0.5~1, the image will become darker
    global fig;
    
    % find min and max in R intensity
    R_temp = jmg(:,:,1);
    R_temp = R_temp(1:end);
    R_low  = min(R_temp);
    R_high = max(R_temp);
    jmg(:,:,1) = minimum + (jmg(:,:,1) - R_low).*((maximum-minimum)./(R_high-R_low));
    
    % find min and max in G intensity
    G_temp = jmg(:,:,1);
    G_temp = G_temp(1:end);
    G_low  = min(G_temp);
    G_high = max(G_temp);
    jmg(:,:,2) = minimum + (jmg(:,:,2) - G_low).*((maximum-minimum)./(G_high-G_low));
    
    % find min and max in B intensity
    B_temp = jmg(:,:,1);
    B_temp = B_temp(1:end);
    B_low  = min(B_temp);
    B_high = max(B_temp);
    jmg(:,:,3) = minimum + (jmg(:,:,3) - B_low).*((maximum-minimum)./(B_high-B_low));

    % find min and max in R,G,B intensity together
%     temp = jmg(:,:,1);
%     temp = temp(1:end);
%     low  = min(temp);
%     high = max(temp);
%     jmg(:,:,:) = minimum + (jmg(:,:,:) - low).*((maximum-minimum)./(high-low));

    fig(end+1) = MyFigure(); MyImShow(jmg); % ==Auto-Contrast
    
    
    % 2. Histogram Equalization:
    % the edge intensities value pixel will be enhenced
    % (the bright one get brighter, dark one get darker)
%     hsv = rgb2hsv(jmg);
%     gry = hsv(:,:,3);    
    gry = jmg(:,:,3);
%     fig(end+1) = MyFigure(); MyImShow(gry); % ==original
    [hx,ix] = hist(gry(:),256);
    figure; plot(ix,hx);
    
    % most intensitier level are used, especially edges
    gryeq = histeq(gry);
    fig(end+1) = MyFigure(); MyImShow(gryeq);% == modified
    [hx,ix] = hist(gryeq(:),256);
    figure; plot(ix,hx);

end

%% ======== Basic plot management ========
function [fig] = MyFigure()
    global fig;
    fig = figure('units','normalized');
    set(gcf,'position',[rand*0.1+0.2 rand*0.1+0.2 0.3 0.3]);
end

function [] = MyImShow(jmg)
    global fig img;
    figure(fig(end)); imshow(jmg); drawnow;
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
    
    % Avoid intensities > 255 or < 0
%     for i = 1:img.V
%         for j = 1:img.U
%            for k = 1:3
%               if jmg(i,j,k) > 255
%                   jmg(i,j,k) = 255;
%               elseif jmg(i,j,k) < 0
%                   jmg(i,j,k) = 0;
%               end
%            end
%         end
%     end
    jmg(jmg>255) =255;
    jmg(jmg<0) =0;
    
    % scale 0 ~ 255 to 0 ~ 1 (0: darkness / 255: brightness)
    if max(jmg(:)) > 1
        jmg = double(jmg)/img.L; 
    end;
    
    global fig;
    fig(end+1) = MyFigure(); MyImShow(jmg);
end