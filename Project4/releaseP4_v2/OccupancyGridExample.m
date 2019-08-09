% Short example about how to implement a deterministic OG
% Jose Guivant. For MTRN4100 students.
% ....................................................................
function main( )
% define the area of coverage and the number of cells we want to spend
MyContext.x1 = -20 ;MyContext.x2 = 20 ;
MyContext.y1 = 0 ;MyContext.y2 = 20 ;
MyContext.Nx = 100 ;
MyContext.Ny = 50 ;
% The area of coverage is: X values between -20m to 20m (suppose I am working
% in meters) and Y values are between 0m to 20m
% I also want to represent the OG by using a discrete grid of 50 x 100 (Ny
% x Nx ) cells.
% Create OG (I do it, inside this function) which I am calling.
DefineOG() ;
% .....................
% Some figure, to get points from user (because in this example I use points
% specified by the user)
figure(1) ; clf ; plot(0,0) ;
axis([MyContext.x1,MyContext.x2,MyContext.y1,MyContext.y2]) ;
% Get some points, provided by the user
disp('input (mouse click) 10 points in figure 1') ;
pp =ginput(10) ;
% .....................
% Set the cells that own the points (see the code of the function "PopulateOG")
PopulateOG(pp(:,1),pp(:,2)) ;
% show OG, as an image
figure(2); clf ; imagesc(MyContext.M) ; colormap gray ;
% in the image: the cells =1 will look white, and the rest (=0) will be black
set(gca,'ydir','normal','xdir','normal') ;
% just to tell Matlab how to show the image
return ; % done
% ...........................................................
function DefineOG( )
    % Define a structure, with parameters useful for processing an Occupancy
    % Grid.
    % here I define a Matrix for storing the values of the OG (I call it "M")
    MyContext.M = zeros(MyContext.Ny,MyContext.Nx,'uint8') ;
    % These constants are useful for scaling (x,y) points to cells' indexes.
    MyContext.Cx = MyContext.Nx/(MyContext.x2-MyContext.x1) ;
    MyContext.Cy = MyContext.Ny/(MyContext.y2-MyContext.y1) ;
    return;
end
% ...........................................................
% This function transforms points (x,y), for obtaining their equivalent cells' indexes.
% and then use them to set the related cells
function PopulateOG(x,y)
    % Firstly, we filter out those points that are outside our ROI (Region of Interest).
    % We keep the valid ones.
    ii = find((x>=MyContext.x1)&(x<MyContext.x2)&(y>=MyContext.y1)&(y<MyContext.y2));
    x=x(ii) ; y=y(ii) ;
    % just consider the points that are inside the OG's coverage
    % convert to indexes
    ix = floor((x-MyContext.x1)*MyContext.Cx)+1 ;
    iy = floor((y-MyContext.y1)*MyContext.Cy)+1 ;
    % Convert 2D indexes to linear indexes
    ixy = sub2ind(size(MyContext.M),iy,ix) ;
    % Set the associated OG's cells, to contain value =1
    MyContext.M(ixy) =1 ;
    return;
end
% ...........................................................
end % end of this program