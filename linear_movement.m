%% *** Robot (kinematic) model parameters *** 
clear all; 
close all; 
clc;

l0 = 3.0;  %% in cm 
l1 = 2.0;  
l2 = 3.0;
l3 = 2.0;
h = 6;

%% *** sampling period *** 
dt = 0.001; %dt = 0.001; i.e. 1 msec)   

%% *** DESIRED MOTION PROFILE - TASK SPACE *** 
T=10.0; 	% 10sec duration of motion 
t=0:dt:T;  
DT = length(t);

% Starting and Ending point A,B
x_A = 3.0;	
y_A = 3.0; 
z_A = h;

x_B =  4.0; 
y_B = 2.0;  
z_B = h;

disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...'); %% 
disp(' ');   

a_x = [x_A, 0, 3*(x_B-x_A)/T^2, - 2*(x_B-x_A)/T^3];

x = a_x(1)+a_x(2)*t+a_x(3)*t.^2+a_x(4)*t.^3;
y = y_A + (x - x_A)*(y_B - y_A)/(x_B - x_A);
z = ones(1,DT)*h;

v_x = [0 diff(x)];
v_y = [0 diff(y)];
v_z = zeros(1,DT);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% ****** KINEMATIC SIMULATION - Main loop ****** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %%  

%% ***** INVESRE KINEMATICS  -->  DESIRED MOTION - JOINT SPACE ***** 

q3  = -acos( (x.^2+ y.^2 + (z-l0).^2 - l1.^2 -l2.^2 -l3.^2)/(2*l2*l3)  );
c_3 = cos(q3);
s_3 = sin(q3);

q2 = 2*atan((l2 + c_3.*l3 - (- y.^2 + c_3.^2.*l3^2 + 2.*c_3.*l2*l3 + l2^2 + l3^2.*s_3.^2).^(1/2))./(y + l3.*s_3 + 0.00000001));
c_2 = cos(q2);
s_2 = sin(q2);
c_23 = cos(q2+q3);
s_23 = sin(q2+q3);

q1 = 2*atan( -(l1 - (- x.^2 + c_2.^2.*l2^2 + 2.*c_2.*c_23.*l2.*l3 + c_23.^2.*l3^2 + l1^2).^(1/2))./(x + c_2.*l2 + c_23.*l3 +0.00000001) );
c_1 =cos(q1);
s_1 = sin(q1);

if sum(abs(x.^2+ y.^2 + (z-l0).^2 - l1.^2 -l2.^2 -l3.^2)/(2*l2*l3)>1) ~= 0
    disp("Trajectory out of workspace");
    return;
end
if sum(l3.*sin(q2+q3) + l2.*sin(q2) ==0) >1
    disp("Special Case 1")
end
if sum( sin(q2+q3).*c_2 - c_23.*s_2 == 0) >1
    disp("Special Case 2")
end

v_q1 = [0 diff(q1)];
v_q2 = [0 diff(q2)];
v_q3 = [0 diff(q3)];

%% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS ***** 

xd1 = zeros(1,DT);
yd1 = zeros(1,DT);
zd1 = ones(1,DT)*l0;

xd2 = -l1.*sin(q1);
yd2 = zeros(1,DT);
zd2 = l0+l2.*cos(q1);

xd3 = 1.0.*l2.*cos(q1).*cos(q2) - 1.0.*l1.*sin(q1);
yd3 = 1.0.*l2.*sin(q2);
zd3 = 1.0.*l0 + 1.0.*l1.*cos(q1) + 1.0.*l2.*cos(q2).*sin(q1);

xd = 1.0.*l2.*cos(q1).*cos(q2) - 1.0.*l1.*sin(q1) + 1.0.*l3.*cos(q1).*cos(q2).*cos(q3) - 1.0.*l3.*cos(q1).*sin(q2).*sin(q3);
yd =  1.0.*l2.*sin(q2) + 1.0.*l3.*cos(q2).*sin(q3) + 1.0.*l3.*cos(q3).*sin(q2);
zd =  1.0.*l0 + 1.0.*l1.*cos(q1) + 1.0.*l2.*cos(q2).*sin(q1) + 1.0.*l3.*cos(q2).*cos(q3).*sin(q1) - 1.0.*l3.*sin(q1).*sin(q2).*sin(q3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

%% *** SAVE and PLOT output data *** %%** use functions plot(...)  
save;  %% --> save data to 'matlab.mat' file   

figure(1);  
subplot(2,3,1); 
plot(t,x); 
ylabel('x(t) (cm)'); 
xlabel('time t (sec)');  

subplot(2,3,2); 
plot(t,y); 
ylabel('y(t) (cm)'); 
xlabel('time t (sec)');  

subplot(2,3,3); 
plot(t,z); 
ylabel('z(t) (cm)'); 
xlabel('time t (sec)');  

subplot(2,3,4); 
plot(t,v_x); 
ylabel('v_x(t) (cm/s)'); 
xlabel('time t (sec)');  
 
subplot(2,3,5); 
plot(t,v_y); 
ylabel('v_y(t) (cm/s)'); 
xlabel('time t (sec)');  

subplot(2,3,6); 
plot(t,v_z); 
ylabel('v_z(t) (cm/s)'); 
xlabel('time t (sec)');

saveas(gcf,['p.png']);


figure(2);  

subplot(2,3,1); 
plot(t,q1 .*180/pi); 
ylabel('q1 (rad)'); 
xlabel('time t (sec)');  

subplot(2,3,2); 
plot(t,q2.*180/pi); 
ylabel('q2 (rad)'); 
xlabel('time t (sec)');  

subplot(2,3,3); 
plot(t,q3.*180/pi); 
ylabel('q3 (rad)'); 
xlabel('time t (sec)');  

subplot(2,3,4); 
plot(t,v_q1.*180/pi); 
ylabel('v_q1 (rad/sec)'); 
xlabel('time t (sec)');  

subplot(2,3,5); 
plot(t,v_q2.*180/pi); 
ylabel('v_q2 (rad/sec)'); 
xlabel('time t (sec)');  

subplot(2,3,6); 
plot(t,v_q3); 
ylabel('v_q3 (rad/sec)'); 
xlabel('time t (sec)');  

saveas(gcf,['q.png']);



% KINEMATIC SIMULATION
figure(4);

plot3(x,y,z, 'gs'); % trajectory
hold on
dt_a = 1000; 
plot([0], [0], 'o')    % axis origin

%% animation
for i = 1:dt_a:DT
    pause(0.05);     % pause motion

    % link from O to joint 1
    plot3([0,xd1(i)],[0,yd1(i)],[0,zd1(i)],'k');
    
    % joint 1
    plot3([xd1(i)],[yd1(i)],[zd1(i)],'co'); % as a cyan 'o'
    
    % link from joint 1 to joint 2
    plot3([xd1(i),xd2(i)],[yd1(i),yd2(i)],[zd1(i),zd2(i)], 'k');
    
    % joint 2
    plot3([xd2(i)],[yd2(i)],[zd2(i)],'md'); % as a diamond
    
    % link from joint 2 to joint 3
    plot3([xd2(i),xd3(i)],[yd2(i),yd3(i)],[zd2(i),zd3(i)], 'k');
    
    % joint 3
    plot3([xd3(i)],[yd3(i)],[zd3(i)],'b*'); % as a blue '*'
    
    % link from joint 3 to end-effector
    plot3([xd3(i),xd(i)],[yd3(i),yd(i)],[zd3(i),zd(i)], 'k');
    
    % end-effector
    plot3([xd(i)],[yd(i)],[zd(i)],'g+');
end
xlabel('x (cm)'); 
ylabel('y (cm)'); 
zlabel('z (cm)');
