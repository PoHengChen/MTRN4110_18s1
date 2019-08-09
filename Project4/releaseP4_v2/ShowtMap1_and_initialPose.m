% this is the MAP used for localization


load('NavMap.mat');
figure(1) ; clf() ; plot(Landmarks.xy(1,:),Landmarks.xy(2,:),'*');
ax=axis();
ax=ax+[-10,10,-10,10] ; axis(ax);
xlabel('X (cm)');
ylabel('Y (cm)');
title('Landmarks and initial pose of robot');



xy0 = [ 25, 105]; Heading0=89 * pi/180;

d=[cos(Heading0),sin(Heading0)]*40;

hold on;
quiver(xy0(1),xy0(2),d(1),d(2),'r');
plot(xy0(1),xy0(2),'*r');