%Author : Adnan Munawar
%Email  : amunawar@wpi.edu ; adnan.munawar@live.com
%MS Robotics, Worcester Polytechnic Institute



function[] = IK()

close all;
clear all;
clc;

%These Parameters define the Position of the Figures in the Figure Plot:
%These two paramteres define the number of the Rows (p_r) and the number of
%the columns (p_c) in the output figure
p_r = 3;
p_c = 2;
%This defines the position of the Figure 1, which is used to get mouse
%position input to drive the Robotic Arm
f_1 = 1;
%This defines the postion of the Figure where the output Robotic Arms is
%shown
f_2 = 2;
%This defines the position of the 3rd figure which shows the path generated
%by the end effector of the Robotic Arm.
f_3 = [3 4 5 6];


%This parameter defines the radius of the plane the Arm's effector is
%allowed to operate while the mouse is moved on a 2 dimensional flat
%surface. Basically you can think that the mouse or trackpad is the inside
%of a cylindrical surface characterized by the radius r where the movements
%of the mouse occur and the Robots end effector follows these movements.

radius = 450;

%These are the DH parameters of the Fanuc Robot. a is link length, ah is
%link twist, d is link offset.

a = [150;250;75;0;0;0];
ah = [90;0;90;-90;90;0];
d = [0;0;0;290;0;80];

% Link 3 is a peculiar case, having a L shaped link. In the DH parameters,
%this is compensated using 'a' and 'd'. However for Inverse Kinematics, the effective
% length is required. Which is the distance from one Joint Node to the other,
% actual length of link 3 = al3
al3 = sqrt(75^2+290^2);


figure(1);
subplot(p_r,p_c,f_1);
axis([-radius radius -600 800]);
set (gcf, 'WindowButtonMotionFcn', @mouseMove);
T = zeros(4,4,6);
pause(1.5);

%Running a loop to generate IK for a certain number of points and store the state space
% in an array.

for i=1:1000
subplot(p_r,p_c,f_1)
C = mouseMove;

px = C(1,1);
pz = C(1,2);


if  px > radius
    px = radius;
    
elseif px < -radius
       px = -radius;
end


if px >= 0 && px <= radius
    
    py = (sqrt(radius^2-px^2));
    
elseif px < 0 && px >= -radius
    
    py = (sqrt(radius^2-px^2));
    
end

        
pause(0.001); 
alp = 0;
gam = 0;
phi = 0;
 
    
Ra = [cosd(alp) -sind(alp) 0 ; sind(alp) cosd(alp) 0 ; 0 0 1];
Rb = [cosd(phi) 0 sind(phi) ; 0 1 0 ; -sind(phi) 0 cosd(phi)];
Rc = [cosd(gam) -sind(gam) 0 ; sind(gam) cosd(gam) 0 ; 0 0 1];
%Orientation of frame 6 in frame 0
R60 = Ra*Rb*Rc;
%Centroid of the wrist, we are computing them from the formula given in
%lecture nores
pcx = px - d(6)*R60(1,3);
pcy = py - d(6)*R60(2,3);
pcz = pz - d(6)*R60(3,3);
    
r=(sqrt(pcx^2+pcy^2))-150;
%length of point from base frame to end effector
%P06 = sqrt(pcx(1,i)^2+pcy(1,i)^2+pcz(1,i)^2);
%length of point from link 1's origin to wrist center
P61 = sqrt(pcz^2+r^2);
%theta 3 intermediate
t3i=acosd(( (a(2)^2) + (al3^2) - (P61^2) )/(2*a(2)*al3));
%actual theta 3
t3(1,i) = t3i - 104.5002;
%theta 1
t1(1,i)=(atan2(pcy,pcx)) * 180/pi;

gamma=acosd(( (P61^2) + (a(2)^2) - (al3^2) )/(2*a(2)*P61));
beta=atan2(pcz,r)*180/pi;
t2(1,i) = 90 - (beta + gamma);


th = [t1(1,i);-t2(1,i)+90;t3(1,i)];
th = real(th);


for j=1:3
T(:,:,j) = [cosd(th(j)) -sind(th(j))*cosd(ah(j)) sind(th(j))*sind(ah(j)) a(j)*cosd(th(j)); ...
sind(th(j)) cosd(th(j))*cosd(ah(j)) -cosd(th(j))*sind(ah(j)) a(j)*sind(th(j)); ...
0 sind(ah(j)) cosd(ah(j)) d(j) ; ...
0 0 0 1];
end

T30 = T(:,:,1)*T(:,:,2)*T(:,:,3);
R30 = T30(1:3,1:3);
R63 = R30.' * R60;

if R63(3,3) == 1
type = 1;  
t5(1,i) = 0;
t4(1,i) = 0;
t6(1,i) = atan2(R63(2,1),R63(1,1))*(180/pi);
elseif R63(3,3) == -1
type = 2;   
t5(1,i) = 180;
t4(1,i) = 0;
t6(1,i) = -atan2(-R63(2,1),-R63(1,1))*(180/pi);
else
type = 3;
t5(1,i) = atan2(sqrt(1-(R63(3,3)^2)),R63(3,3))*(180/pi);
t4(1,i) = atan2(R63(2,3),R63(1,3))*(180/pi);
t6(1,i) = atan2(R63(3,2),-R63(3,1))*(180/pi);              
end

th = [t1(1,i);-t2(1,i)+90;t3(1,i);-t4(1,i);t5(1,i);-t6(1,i)];

for j=4:6
T(:,:,j) = [cosd(th(j)) -sind(th(j))*cosd(ah(j)) sind(th(j))*sind(ah(j)) a(j)*cosd(th(j)); ...
sind(th(j)) cosd(th(j))*cosd(ah(j)) -cosd(th(j))*sind(ah(j)) a(j)*sind(th(j)); ...
0 sind(ah(j)) cosd(ah(j)) d(j) ; ...
0 0 0 1];
end

%P_org is a temporary vector for assigning the origins of successive frames
%by successive multiplication with the compounded transformations 1 to 6.
P_org = [0 0 0 1]';
P_temp = zeros(4,6);
T60 = eye(4);

for j=1:6
T60 = T60 * T(:,:,j);
P_temp(:,j) = T60 * P_org;
end

P = zeros(3,7);
P(:,2:7) = P_temp(1:3,:);
x = P(1,:);
y = P(2,:);
z = P(3,:);



subplot(p_r,p_c,f_2);
plot3([0;0],[0;0],[0;-350],'-b','LineWidth',6);
hold on;
plot3(x,y,z,'-ro','LineWidth',4,...
'MarkerEdgeColor','k',...
'MarkerFaceColor','g',...
'MarkerSize',10);

xlabel('x')
ylabel('y')
zlabel('z')
title('Arm Kinematics')
axis([-800 800 -800 800 -800 800]);
hold off;

subplot(p_r,p_c,f_3)
axis([-800 800 -800 800 -800 800]);
plot3(P(1,7),P(2,7),P(3,7),'LineWidth',4);
hold on;
%pause(0.01);

end
end


function C = mouseMove (object, eventdata)
C = get (gca, 'CurrentPoint');
end