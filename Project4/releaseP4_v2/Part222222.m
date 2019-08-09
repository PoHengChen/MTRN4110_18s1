% Short example about how to implement a deterministic OG
% Jose Guivant. For MTRN4100 students.
% ....................................................................
function main( )
g = load('NavMap.mat');
% define the area of coverage and the number of cells we want to spend
MyContext.x1 = 0 ;MyContext.x2 = 200 ;
MyContext.y1 = 0 ;MyContext.y2 = 200 ;
MyContext.Nx = 100 ;
MyContext.Ny = 50 ;

% The area of coverage is: X values between -20m to 20m (suppose I am working
% in meters) and Y values are between 0m to 20m
% I also want to represent the OG by using a discrete grid of 50 x 100 (Ny x Nx ) cells.
% Create OG (I do it, inside this function) which I am calling.
DefineOG() ;
% .....................
% Set the cells that own the points (see the code of the function "PopulateOG")
PopulateOG(g.Landmarks.xy(1,:),g.Landmarks.xy(2,:)) ;
% show OG, as an image
figure(1); clf ; 
imagesc(MyContext.M) ; 
colormap gray ;
% in the image: the cells =1 will look white, and the rest (=0) will be black
set(gca,'ydir','normal','xdir','normal') ;
% just to tell Matlab how to show the image
return ; % done
% ...........................................................
function DefineOG( )
    MyContext.M = zeros(MyContext.Ny,MyContext.Nx,'uint8') ;
    MyContext.Cx = MyContext.Nx/(MyContext.x2-MyContext.x1) ;
    MyContext.Cy = MyContext.Ny/(MyContext.y2-MyContext.y1) ;
    return;
end
% ...........................................................
% This function transforms points (x,y), for obtaining their equivalent cells' indexes.
% and then use them to set the related cells
function PopulateOG(x,y)
    ii = find((x>=MyContext.x1)&(x<MyContext.x2)&(y>=MyContext.y1)&(y<MyContext.y2));
    x=x(ii) ; y=y(ii) ;
    ix = floor((x-MyContext.x1)*MyContext.Cx)+1 ;
    iy = floor((y-MyContext.y1)*MyContext.Cy)+1 ;
    ixy = sub2ind(size(MyContext.M),iy,ix) ;
    MyContext.M(ixy) =1 ;
    return;
end
% ...........................................................
end % end of this program