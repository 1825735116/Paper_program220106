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

MyTheta=[-3.11 -98.68 -123.98...
    -50.79 88.44 2.97]'*pi/180;
space_arm.plot(MyTheta'); 
Angle_lowlimit=[-360,-360,-155,-360,-360,-360]*pi/180;
Angle_uplimit=[360,360, 155,360,360,360]*pi/180;
Ang_vel_lowlimit=[-120,-120,-120,-180,-180,-180]*pi/180*1;
Ang_vel_uplimit=[120,120, 120,180,180,180]*pi/180*1;

T1=[cos(MyTheta(1)) -cos(s_alpha(1))*sin(MyTheta(1)) sin(s_alpha(1))*sin(MyTheta(1)) s_a(1)*cos(MyTheta(1));sin(MyTheta(1)) cos(s_alpha(1))*cos(MyTheta(1)) -sin(s_alpha(1))*cos(MyTheta(1)) s_a(1)*sin(MyTheta(1));0 sin(s_alpha(1)) cos(s_alpha(1)) s_d(1);0 0 0 1];
T2=[cos(MyTheta(2)) -cos(s_alpha(2))*sin(MyTheta(2)) sin(s_alpha(2))*sin(MyTheta(2)) s_a(2)*cos(MyTheta(2));sin(MyTheta(2)) cos(s_alpha(2))*cos(MyTheta(2)) -sin(s_alpha(2))*cos(MyTheta(2)) s_a(2)*sin(MyTheta(2));0 sin(s_alpha(2)) cos(s_alpha(2)) s_d(2);0 0 0 1];
T3=[cos(MyTheta(3)) -cos(s_alpha(3))*sin(MyTheta(3)) sin(s_alpha(3))*sin(MyTheta(3)) s_a(3)*cos(MyTheta(3));sin(MyTheta(3)) cos(s_alpha(3))*cos(MyTheta(3)) -sin(s_alpha(3))*cos(MyTheta(3)) s_a(3)*sin(MyTheta(3));0 sin(s_alpha(3)) cos(s_alpha(3)) s_d(3);0 0 0 1];
T4=[cos(MyTheta(4)) -cos(s_alpha(4))*sin(MyTheta(4)) sin(s_alpha(4))*sin(MyTheta(4)) s_a(4)*cos(MyTheta(4));sin(MyTheta(4)) cos(s_alpha(4))*cos(MyTheta(4)) -sin(s_alpha(4))*cos(MyTheta(4)) s_a(4)*sin(MyTheta(4));0 sin(s_alpha(4)) cos(s_alpha(4)) s_d(4);0 0 0 1];
T5=[cos(MyTheta(5)) -cos(s_alpha(5))*sin(MyTheta(5)) sin(s_alpha(5))*sin(MyTheta(5)) s_a(5)*cos(MyTheta(5));sin(MyTheta(5)) cos(s_alpha(5))*cos(MyTheta(5)) -sin(s_alpha(5))*cos(MyTheta(5)) s_a(5)*sin(MyTheta(5));0 sin(s_alpha(5)) cos(s_alpha(5)) s_d(5);0 0 0 1];
T6=[cos(MyTheta(6)) -cos(s_alpha(6))*sin(MyTheta(6)) sin(s_alpha(6))*sin(MyTheta(6)) s_a(6)*cos(MyTheta(6));sin(MyTheta(6)) cos(s_alpha(6))*cos(MyTheta(6)) -sin(s_alpha(6))*cos(MyTheta(6)) s_a(6)*sin(MyTheta(6));0 sin(s_alpha(6)) cos(s_alpha(6)) s_d(6);0 0 0 1];
T12=T1*T2;T13=T12*T3;T14=T13*T4;
T15=T14*T5;T16=T15*T6;
EndEffector = T16*[0;0;0;1];
EndEffectorR = [T16(1,1);T16(2,1);T16(3,1);T16(1,3);T16(2,3);T16(3,3)];

%%%
Q = [];
Myerror = [];
MyattTheta=[];
MyattTheta1=[];
MyThetaPlot=[MyTheta'];
MyThetaVelocityPlot=[0,0,0,0,0,0];
MyThetaAccPlot = [0,0,0,0,0,0];
MyEEPosition=[];
Myud=[];
Myd_u=[];
My_norme=[];
Last_u1=0;
Lamuda = 0.1;
I = eye(6);
Ts = 0.01;
T = 0.008;

TargetP = [1200;100;50];
TargetR = [1;0;0;0;0;1];

angle=720;
vel_ang_lim=60;
acc_ang_lim=100;
tol_time=14;
angle_seq=Accelerate_Plan(angle,vel_ang_lim,acc_ang_lim,tol_time,T);
[ang_vel_seq,ang_acc_seq]=angular_vel_acc(angle,vel_ang_lim,acc_ang_lim,tol_time,T);
MyTotalStep = size(angle_seq,2);

r=330;
c1=[680,100,50];
norm1=[0,0,-1];
[position_sep,c_a,c_b]=Generate_circle(c1,r,norm1,90,angle_seq);
[arc_vel,arc_acc]=Circle_vel_acc(r,c_a,c_b,angle_seq,ang_vel_seq,ang_acc_seq);

epsilon1=18;
epsilon2=18;
epsilon3=8000;
u=[0;0;0;0;0;0];
d_u=[0;0;0;0;0;0];
d_w=[0;0;0;0;0;0;0;0;0];
w=[0;0;0;0;0;0;0;0;0];
lamda=[0;0;0;0;0];
lamda0=[0;0;0;0;0;0;0;0;0];
d_lamda=[0;0;0;0;0;0;0;0;0];
rd=[0;0;0;0;0;0];
rd0=[position_sep(1,:)';1;0;0;0;0;1];
r=[0;0;0;0;0;0;0;0;0];
r0=[EndEffector(1:3);0;0;0];
for i=1:MyTotalStep
    TargetP0=position_sep(i,:)';
    TargetP=TargetP0;
    TargetR=[1;0;0;0;0;1];
    rd=[TargetP;100000*TargetR];
    Target_vel=[arc_vel(i,:)';0;0;0;0;0;0];
    %%%
    T1=[cos(MyTheta(1)) -cos(s_alpha(1))*sin(MyTheta(1)) sin(s_alpha(1))*sin(MyTheta(1)) s_a(1)*cos(MyTheta(1));sin(MyTheta(1)) cos(s_alpha(1))*cos(MyTheta(1)) -sin(s_alpha(1))*cos(MyTheta(1)) s_a(1)*sin(MyTheta(1));0 sin(s_alpha(1)) cos(s_alpha(1)) s_d(1);0 0 0 1];
    T2=[cos(MyTheta(2)) -cos(s_alpha(2))*sin(MyTheta(2)) sin(s_alpha(2))*sin(MyTheta(2)) s_a(2)*cos(MyTheta(2));sin(MyTheta(2)) cos(s_alpha(2))*cos(MyTheta(2)) -sin(s_alpha(2))*cos(MyTheta(2)) s_a(2)*sin(MyTheta(2));0 sin(s_alpha(2)) cos(s_alpha(2)) s_d(2);0 0 0 1];
    T3=[cos(MyTheta(3)) -cos(s_alpha(3))*sin(MyTheta(3)) sin(s_alpha(3))*sin(MyTheta(3)) s_a(3)*cos(MyTheta(3));sin(MyTheta(3)) cos(s_alpha(3))*cos(MyTheta(3)) -sin(s_alpha(3))*cos(MyTheta(3)) s_a(3)*sin(MyTheta(3));0 sin(s_alpha(3)) cos(s_alpha(3)) s_d(3);0 0 0 1];
    T4=[cos(MyTheta(4)) -cos(s_alpha(4))*sin(MyTheta(4)) sin(s_alpha(4))*sin(MyTheta(4)) s_a(4)*cos(MyTheta(4));sin(MyTheta(4)) cos(s_alpha(4))*cos(MyTheta(4)) -sin(s_alpha(4))*cos(MyTheta(4)) s_a(4)*sin(MyTheta(4));0 sin(s_alpha(4)) cos(s_alpha(4)) s_d(4);0 0 0 1];
    T5=[cos(MyTheta(5)) -cos(s_alpha(5))*sin(MyTheta(5)) sin(s_alpha(5))*sin(MyTheta(5)) s_a(5)*cos(MyTheta(5));sin(MyTheta(5)) cos(s_alpha(5))*cos(MyTheta(5)) -sin(s_alpha(5))*cos(MyTheta(5)) s_a(5)*sin(MyTheta(5));0 sin(s_alpha(5)) cos(s_alpha(5)) s_d(5);0 0 0 1];
    T6=[cos(MyTheta(6)) -cos(s_alpha(6))*sin(MyTheta(6)) sin(s_alpha(6))*sin(MyTheta(6)) s_a(6)*cos(MyTheta(6));sin(MyTheta(6)) cos(s_alpha(6))*cos(MyTheta(6)) -sin(s_alpha(6))*cos(MyTheta(6)) s_a(6)*sin(MyTheta(6));0 sin(s_alpha(6)) cos(s_alpha(6)) s_d(6);0 0 0 1];
    T12=T1*T2;T13=T12*T3;T14=T13*T4;
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
    [u1]=Joint_limit_RCNN(MyTheta,1*u+0.040*d_u,Angle_lowlimit,Angle_uplimit,Ang_vel_lowlimit,Ang_vel_uplimit);
    MyTheta=MyTheta+1*u1*T+0*0.015*d_u*T;

    Myerror = [Myerror;Error',ErrorR'];
    Myud=[Myud;ud'];
    Myd_u=[Myd_u;d_u'];
    MyEEPosition=[MyEEPosition;EndEffector'];
    MyThetaVelocityPlot=[MyThetaVelocityPlot;u1'];
    MyThetaPlot=[MyThetaPlot;MyTheta'];
    MyThetaAccPlot = [MyThetaAccPlot;(u1-Last_u1)'/T];
    Last_u1=u1;
    Q = [Q MyTheta];
    My_norme=[My_norme;norm_e,norm_er];
end
figure(1)
space_arm.plot(Q','delay',0.001,'floorlevel',-0.4,'perspective')  %Create figure to simulate the robot motion in real time

t= (0:MyTotalStep)*0.008;
figure(2)
plot(t,MyThetaAccPlot(:,1),'-r');
hold on
plot(t,MyThetaAccPlot(:,2),'--g');
hold on
plot(t,MyThetaAccPlot(:,3),':b');
hold on
plot(t,MyThetaAccPlot(:,4),'-.m');
hold on
plot(t,MyThetaAccPlot(:,5),'--','Color',[1 0.5 0.1]);
hold on
plot(t,-MyThetaAccPlot(:,6),':k');


figure(3)
plot(t,MyThetaVelocityPlot(:,1),'-r')
hold on
plot(t,MyThetaVelocityPlot(:,2),'--g')
hold on
plot(t,MyThetaVelocityPlot(:,3),':b')
hold on
plot(t,MyThetaVelocityPlot(:,4),'-.m')
hold on
plot(t,MyThetaVelocityPlot(:,5),'--','Color',[1 0.5 0.1])
hold on
plot(t,-MyThetaVelocityPlot(:,6),':k')


figure(4)
MyThetaPlot1=MyThetaPlot*180/pi;
plot(t,MyThetaPlot1(:,1),'-r')
hold on
plot(t,MyThetaPlot1(:,2),'--g')
hold on
plot(t,MyThetaPlot1(:,3),':b')
hold on
plot(t,MyThetaPlot1(:,4),'-.m')
hold on
plot(t,MyThetaPlot1(:,5),'--','Color',[1 0.5 0.1])
hold on
plot(t,-MyThetaPlot1(:,6),':k')


t= (1:MyTotalStep)*0.008;
figure(5)
% subplot(2,1,1);
plot(t,Myerror(:,1)*1,'--r')
hold on
plot(t,Myerror(:,2)*1,':g')
hold on
plot(t,Myerror(:,3)*1,'-.b')
hold on
plot(t,My_norme(:,1)*1,'-k','LineWidth',0.5)


% subplot(2,1,2);
figure(6)
plot(t,My_norme(:,2),'b')

