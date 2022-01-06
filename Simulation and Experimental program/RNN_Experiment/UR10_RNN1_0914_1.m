% non-Kalman
clear
clc
close all

s_a=[0.108,-612.871,-571.731,0,0,0];
s_d=[128.267,0,0,163.694,115.804,-92.231];
s_alpha=[1.571107431496264,0,0,pi/2,pi/2,0];
space(1) = Revolute('d',s_d(1), 'a', s_a(1), 'alpha', s_alpha(1));
space(2) = Revolute('d',s_d(2), 'a', s_a(2), 'alpha', s_alpha(2));
space(3) = Revolute('d',s_d(3), 'a', s_a(3), 'alpha', s_alpha(3));
space(4) = Revolute('d',s_d(4), 'a', s_a(4), 'alpha', s_alpha(4));
space(5) = Revolute('d',s_d(5), 'a', s_a(5), 'alpha', s_alpha(5));
space(6) = Revolute('d',s_d(6), 'a', s_a(6), 'alpha', s_alpha(6));
space_arm=SerialLink(space, 'name', 'S');
% 
% MyTheta = [-3.0588+180  -24.3841 49.1065 ...
%     -114.7224 -90 180-86.9412]'*pi/180;
MyTheta=[3.0883676+0.0 -0.42615588 0.85806514 ...
    -2.0027056 -1.5704852 1.6240214]';
MyTheta=[-3.11 -98.68 -123.98...
    -50.79 88.44 2.97]'*pi/180;
MyTheta =[8.59  -88.70  -121.20 ...
    -59.14  90   111.41]'*pi/180;
space_arm.plot(MyTheta'); 
Angle_lowlimit=[-360,-360,-155,-360,-360,-360]*pi/180;
Angle_uplimit=[360,360, 155,360,360,360]*pi/180;
Ang_vel_lowlimit=[-120,-120,-120,-180,-180,-180]*pi/180;
Ang_vel_uplimit=[120,120, 120,180,180,180]*pi/180;

%%%
anglepub=rospublisher('/scaled_pos_joint_traj_controller/command',rostype.trajectory_msgs_JointTrajectory);
pause(0.1)
joint_state=rosmessage(anglepub);
tar_point=rosmessage(rostype.trajectory_msgs_JointTrajectoryPoint);
joint_state.JointNames = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"];
tar_point.TimeFromStart.Nsec = 5000;

global pos1 joint_angle joint_vel joint_effort
tar_pos=rossubscriber('/target',@callback1);
% tar_pos_point=rosmessage(tar_pos);
joint=rossubscriber('/joint_states',@callback2);
joint_pos=rosmessage(joint);
joint_pos=receive(joint);
MyTheta=[joint_pos.Position(3) joint_pos.Position(2) joint_pos.Position(1) joint_pos.Position(4) joint_pos.Position(5) -joint_pos.Position(6)]';
% joint_record=[];
%%%

T1=[cos(MyTheta(1)) -cos(s_alpha(1))*sin(MyTheta(1)) sin(s_alpha(1))*sin(MyTheta(1)) s_a(1)*cos(MyTheta(1));sin(MyTheta(1)) cos(s_alpha(1))*cos(MyTheta(1)) -sin(s_alpha(1))*cos(MyTheta(1)) s_a(1)*sin(MyTheta(1));0 sin(s_alpha(1)) cos(s_alpha(1)) s_d(1);0 0 0 1];
T2=[cos(MyTheta(2)) -cos(s_alpha(2))*sin(MyTheta(2)) sin(s_alpha(2))*sin(MyTheta(2)) s_a(2)*cos(MyTheta(2));sin(MyTheta(2)) cos(s_alpha(2))*cos(MyTheta(2)) -sin(s_alpha(2))*cos(MyTheta(2)) s_a(2)*sin(MyTheta(2));0 sin(s_alpha(2)) cos(s_alpha(2)) s_d(2);0 0 0 1];
T3=[cos(MyTheta(3)) -cos(s_alpha(3))*sin(MyTheta(3)) sin(s_alpha(3))*sin(MyTheta(3)) s_a(3)*cos(MyTheta(3));sin(MyTheta(3)) cos(s_alpha(3))*cos(MyTheta(3)) -sin(s_alpha(3))*cos(MyTheta(3)) s_a(3)*sin(MyTheta(3));0 sin(s_alpha(3)) cos(s_alpha(3)) s_d(3);0 0 0 1];
T4=[cos(MyTheta(4)) -cos(s_alpha(4))*sin(MyTheta(4)) sin(s_alpha(4))*sin(MyTheta(4)) s_a(4)*cos(MyTheta(4));sin(MyTheta(4)) cos(s_alpha(4))*cos(MyTheta(4)) -sin(s_alpha(4))*cos(MyTheta(4)) s_a(4)*sin(MyTheta(4));0 sin(s_alpha(4)) cos(s_alpha(4)) s_d(4);0 0 0 1];
T5=[cos(MyTheta(5)) -cos(s_alpha(5))*sin(MyTheta(5)) sin(s_alpha(5))*sin(MyTheta(5)) s_a(5)*cos(MyTheta(5));sin(MyTheta(5)) cos(s_alpha(5))*cos(MyTheta(5)) -sin(s_alpha(5))*cos(MyTheta(5)) s_a(5)*sin(MyTheta(5));0 sin(s_alpha(5)) cos(s_alpha(5)) s_d(5);0 0 0 1];
T6=[cos(MyTheta(6)) -cos(s_alpha(6))*sin(MyTheta(6)) sin(s_alpha(6))*sin(MyTheta(6)) s_a(6)*cos(MyTheta(6));sin(MyTheta(6)) cos(s_alpha(6))*cos(MyTheta(6)) -sin(s_alpha(6))*cos(MyTheta(6)) s_a(6)*sin(MyTheta(6));0 sin(s_alpha(6)) cos(s_alpha(6)) s_d(6);0 0 0 1];
T7=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 78 1];
Twr1y=[1,0,0,0;0,1,0,2.058;0,0,1,0;0,0,0,1];
T12=T1*T2;T13=T12*T3;T14=T13*Twr1y*T4;
T15=T14*T5;T16=T15*T6;T17=T16*T7;

EndEffector = T16*[0;0;0;1];
EndEffectorR = [T16(1,1);T16(2,1);T16(3,1);T16(1,3);T16(2,3);T16(3,3)];
Point_s1=EndEffector(1:3);
EE_initial=EndEffector(3);
%%%
QS = [];
Myerror = [];
My_norme=[];
MyattTheta=[];
MyattTheta1=[];
MyThetaPlot=[MyTheta'];
MyThetaVelocityPlot=[0,0,0,0,0,0];
MyThetaAccPlot = [0,0,0,0,0,0];
Myud=[];
Myd_u=[];

Target=[];
position_record=[];
joint_record=[];

Lamuda = 0.1;
I = eye(6);
Ts = 0.01;
T = 0.008;

TargetP=EndEffector(1:3);
%tar_pos=load('position2.txt');
% TargetP = tar_pos(1,:)'/1000;
TargetP(3)=EE_initial;
TargetR = [0;-1;0;0;0;1];
LastTargetP = TargetP;
LastTargetR = [0;-1;0;0;0;1];
dz=90;%Point_s2=[tar_pos(1,1);tar_pos(1,2);tar_pos(1,3)+dz];
% Vx=(Point_s2(1)-Point_s1(1))/0.8;Vy=(Point_s2(2)-Point_s1(2))/0.8;
%Vz=(Point_s2(3)-Point_s1(3))/1;
MyTotalStep=8000;

%
epsilon1=30;
epsilon2=30;
epsilon3=8000;
u=[0;0;0;0;0;0];
d_u=[0;0;0;0;0;0];
d_w=[0;0;0;0;0;0;0;0;0];
w=[0;0;0;0;0;0;0;0;0];
lamda=[0;0;0;0;0];
lamda0=[0;0;0;0;0;0;0;0;0];
d_lamda=[0;0;0;0;0;0;0;0;0];
rd=[0;0;0;0;0;0];
% rd0=[tar_pos(1,:)';0;-1;0;0;0;1];
r=[0;0;0;0;0;0;0;0;0];
r0=[EndEffector(1:3);0;0;0];

%%%
pause(0.1)
tic;
t1=toc;
%%%

for i=1:MyTotalStep

    X11=pos1(1)/1;Y11=pos1(2)/1;Z11=pos1(3)/1;
    if i<100
        TargetP(1,1)=Point_s1(1)+(X11(1)-Point_s1(1))/100*i;
        TargetP(2,1)=Point_s1(2)+(Y11(1)-Point_s1(2))/100*i;
        TargetP(3,1)=Point_s1(3);
    elseif i>=100&&i<=125
        TargetP(1,1)=X11(1)/1+0.000;
        TargetP(2,1)=Y11(1)/1-0.000;
        TargetP(3,1)=Point_s1(3);
    elseif i>125&&i<250
        TargetP(1,1)=X11(1)/1+0.000;
        TargetP(2,1)=Y11(1)/1-0.000;
        TargetP(3,1)=Point_s1(3)+(Z11(1)/1+dz-Point_s1(3))/125*(i-125);
    else
        TargetP(1,1)=X11(1)/1+0.000;
        TargetP(2,1)=Y11(1)/1-0.000;
        TargetP(3,1)=Z11(1)/1+dz;
    end
    
    rd=[TargetP;100000*TargetR];
%     Target_vel=[arc_vel(i,:)';0;0;0;0;0;0];
    %%%
    T1=[cos(MyTheta(1)) -cos(s_alpha(1))*sin(MyTheta(1)) sin(s_alpha(1))*sin(MyTheta(1)) s_a(1)*cos(MyTheta(1));sin(MyTheta(1)) cos(s_alpha(1))*cos(MyTheta(1)) -sin(s_alpha(1))*cos(MyTheta(1)) s_a(1)*sin(MyTheta(1));0 sin(s_alpha(1)) cos(s_alpha(1)) s_d(1);0 0 0 1];
    T2=[cos(MyTheta(2)) -cos(s_alpha(2))*sin(MyTheta(2)) sin(s_alpha(2))*sin(MyTheta(2)) s_a(2)*cos(MyTheta(2));sin(MyTheta(2)) cos(s_alpha(2))*cos(MyTheta(2)) -sin(s_alpha(2))*cos(MyTheta(2)) s_a(2)*sin(MyTheta(2));0 sin(s_alpha(2)) cos(s_alpha(2)) s_d(2);0 0 0 1];
    T3=[cos(MyTheta(3)) -cos(s_alpha(3))*sin(MyTheta(3)) sin(s_alpha(3))*sin(MyTheta(3)) s_a(3)*cos(MyTheta(3));sin(MyTheta(3)) cos(s_alpha(3))*cos(MyTheta(3)) -sin(s_alpha(3))*cos(MyTheta(3)) s_a(3)*sin(MyTheta(3));0 sin(s_alpha(3)) cos(s_alpha(3)) s_d(3);0 0 0 1];
    T4=[cos(MyTheta(4)) -cos(s_alpha(4))*sin(MyTheta(4)) sin(s_alpha(4))*sin(MyTheta(4)) s_a(4)*cos(MyTheta(4));sin(MyTheta(4)) cos(s_alpha(4))*cos(MyTheta(4)) -sin(s_alpha(4))*cos(MyTheta(4)) s_a(4)*sin(MyTheta(4));0 sin(s_alpha(4)) cos(s_alpha(4)) s_d(4);0 0 0 1];
    T5=[cos(MyTheta(5)) -cos(s_alpha(5))*sin(MyTheta(5)) sin(s_alpha(5))*sin(MyTheta(5)) s_a(5)*cos(MyTheta(5));sin(MyTheta(5)) cos(s_alpha(5))*cos(MyTheta(5)) -sin(s_alpha(5))*cos(MyTheta(5)) s_a(5)*sin(MyTheta(5));0 sin(s_alpha(5)) cos(s_alpha(5)) s_d(5);0 0 0 1];
    T6=[cos(MyTheta(6)) -cos(s_alpha(6))*sin(MyTheta(6)) sin(s_alpha(6))*sin(MyTheta(6)) s_a(6)*cos(MyTheta(6));sin(MyTheta(6)) cos(s_alpha(6))*cos(MyTheta(6)) -sin(s_alpha(6))*cos(MyTheta(6)) s_a(6)*sin(MyTheta(6));0 sin(s_alpha(6)) cos(s_alpha(6)) s_d(6);0 0 0 1];
    Twr1y=[1,0,0,0;0,1,0,2.058;0,0,1,0;0,0,0,1];
    T12=T1*T2;T13=T12*T3;T14=T13*Twr1y*T4;
    T15=T14*T5;T16=T15*T6;
    EndEffector = T16*[0;0;0;1];
    %%%
    EndEffectorR = [T16(1,1);T16(2,1);T16(3,1);T16(1,3);T16(2,3);T16(3,3)];
    r=[EndEffector(1:3);100000*EndEffectorR];
    %%%
    Error = TargetP - EndEffector(1:3);
    %%%
    ErrorR = TargetR - EndEffectorR;
    norm_e=norm(Error);
    norm_er=norm(ErrorR);
    %%%
    [J,diff_J1,diff_J2,diff_J3,diff_J4,diff_J5,diff_J6]=Jacobian_Cal(MyTheta);
    
%     ud=J'*(lamda0+(rd-rd0-r+r0)/epsilon1);
%     d_w=(Target_vel-J*J'*w)/epsilon3;
%     w=w+d_w*T;
    ud=J'*((rd-r)/epsilon1)+0*J'*w;
    d_u=(-u+ud)/epsilon2;
    u=u+d_u*T;
    [u1]=Joint_limit_RCNN(MyTheta,u+0.015*d_u,Angle_lowlimit,Angle_uplimit,Ang_vel_lowlimit,Ang_vel_uplimit);
    MyTheta=MyTheta+u1*T+0*0.015*d_u*T;
    
%     MyTheta=MyTheta+u*T;
    
    Myerror = [Myerror;Error',ErrorR'];
    My_norme=[My_norme;norm_e,norm_er];
    Myud=[Myud;ud'];
    Myd_u=[Myd_u;d_u'];
    MyThetaVelocityPlot=[MyThetaVelocityPlot;u1'];
    MyThetaPlot=[MyThetaPlot;MyTheta'];
    MyThetaAccPlot = [MyThetaAccPlot;d_u'];
    QS = [QS MyTheta];
    Target=[Target;TargetP'];

    position_record=[position_record;TargetP'];
    joint_record=[joint_record;joint_angle,joint_vel, joint_effort];
    tar_point.Positions=[MyTheta(1:5)',-MyTheta(6)];
    joint_state.Points=tar_point;
    
    t2=toc;
    pause(0.008-t2+t1)
    toc-t1
    t1=toc;
    send(anglepub,joint_state)
end

dlmwrite('position_nonkal1.txt', position_record);
dlmwrite('robot_states_nonkal1.txt', joint_record);

dlmwrite('plan_jointacc_nonkal1.txt', MyThetaAccPlot);
dlmwrite('plan_jointvel_nonkal1.txt', MyThetaVelocityPlot);
dlmwrite('plan_jointang_nonkal1.txt', MyThetaPlot);
dlmwrite('plan_error_nonkal1.txt', Myerror);

figure(1)
% space_arm.plot(QS','delay',0.001,'floorlevel',-0.4,'perspective')  %Create figure to simulate the robot motion in real time

t= (0:MyTotalStep)*T;
figure(2)
plot(t,MyThetaAccPlot(:,1));
hold on
plot(t,MyThetaAccPlot(:,2));
hold on
plot(t,MyThetaAccPlot(:,3));
hold on
plot(t,MyThetaAccPlot(:,4));
hold on
plot(t,MyThetaAccPlot(:,5));
hold on
plot(t,MyThetaAccPlot(:,6));
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',12);
ylabel('角加速度','FontSize',12);

figure(3)
plot(t,MyThetaVelocityPlot(:,1))
hold on
plot(t,MyThetaVelocityPlot(:,2))
hold on
plot(t,MyThetaVelocityPlot(:,3))
hold on
plot(t,MyThetaVelocityPlot(:,4))
hold on
plot(t,MyThetaVelocityPlot(:,5))
hold on
plot(t,MyThetaVelocityPlot(:,6))
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',30);
ylabel('角速度/ °/s','FontSize',30);

figure(4)
MyThetaPlot1=MyThetaPlot*180/pi;
plot(t,MyThetaPlot1(:,1),'r')
hold on
plot(t,MyThetaPlot1(:,2),'g')
hold on
plot(t,MyThetaPlot1(:,3),'c')
hold on
plot(t,MyThetaPlot1(:,4),'b')
hold on
plot(t,MyThetaPlot1(:,5),'m')
hold on
plot(t,MyThetaPlot1(:,6),'k')
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',30);
ylabel('角度/°','FontSize',30);

t= (1:MyTotalStep)*T;
figure(5)
subplot(2,1,1);
plot(t,Myerror(:,1),'r')
hold on
plot(t,Myerror(:,2),'y')
hold on
plot(t,Myerror(:,3),'b')
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',30);
ylabel('末端误差/mm','FontSize',30);
subplot(2,1,2);
plot(t,Myerror(:,4),'r')
hold on
plot(t,Myerror(:,5),'y')
hold on
plot(t,Myerror(:,6),'b')
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',30);
ylabel('姿态误差/','FontSize',30);

figure(8)
plot(t,tar_pos(:,1)*1,'-r','LineWidth',1)
hold on
plot(t,tar_pos(:,2)*1,'-g','LineWidth',1)
hold on
plot(t,tar_pos(:,3)*1,'-b','LineWidth',1)

figure(9)
plot3(Target(:,1),Target(:,2),Target(:,3),'-r');
hold on
plot3(tar_pos(:,1),tar_pos(:,2),tar_pos(:,3),'-b');

%%%
function callback1(~,msg)
    global pos1
    pos1=[msg.Positions(1) msg.Positions(2) msg.Positions(3)];
end
    
function callback2(~,msg)
    global joint_angle joint_vel joint_effort
    joint_angle=[msg.Position(3),msg.Position(2),msg.Position(1),msg.Position(4),msg.Position(5),msg.Position(6)];
    joint_vel=[msg.Velocity(3),msg.Velocity(2),msg.Velocity(1),msg.Velocity(4),msg.Velocity(5),msg.Velocity(6)];
    joint_effort=[msg.Effort(3),msg.Effort(2),msg.Effort(1),msg.Effort(4),msg.Effort(5),msg.Effort(6)];
end