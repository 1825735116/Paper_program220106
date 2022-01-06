clear
clc
close all
clf

s_a=[0.000108,-0.612871,-0.571731,0,0,0];
s_d=[0.128267,0,0,0.163694,0.115804,-0.092231];
s_alpha=[1.571107431496264,0,0,pi/2,pi/2,0];

space(1) = Revolute('d',s_d(1), 'a', s_a(1), 'alpha', s_alpha(1));
space(2) = Revolute('d',s_d(2), 'a', s_a(2), 'alpha', s_alpha(2));
space(3) = Revolute('d',s_d(3), 'a', s_a(3), 'alpha', s_alpha(3));
space(4) = Revolute('d',s_d(4), 'a', s_a(4), 'alpha', s_alpha(4));
space(5) = Revolute('d',s_d(5), 'a', s_a(5), 'alpha', s_alpha(5));
space(6) = Revolute('d',s_d(6), 'a', s_a(6), 'alpha', s_alpha(6));
space_arm=SerialLink(space, 'name', 'S');
% 
MyTheta=[-3.11 -98.68 -123.98...
    -50.79 88.44 2.97]'*pi/180;
figure(1)
space_arm.plot(MyTheta');
Angle_lowlimit=[-360,-360,-155,-360,-360,-360]*pi/180;
Angle_uplimit=[360,360, 155,360,360,360]*pi/180;
Median_ang=(Angle_lowlimit+Angle_uplimit)/2;
Angle_range=(Angle_uplimit-Angle_lowlimit);
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
LastEndEffector = EndEffector;
LastEndEffectorR = EndEffectorR;
LastPosture=[EndEffector(1:3);EndEffectorR];
Last_MyThetaVelocity=[0;0;0;0;0;0];
%%%
Myerror = [];
MyThetaAccPlot=[0,0,0,0,0,0];
MyThetaPlot=[MyTheta'];
MyThetaVelocityPlot=[0,0,0,0,0,0];
MyEEPosition=[];
Adj=[];
My_norme=[];

Lamuda = 0.001;
I = eye(6);
Ts = 0.0;
T = 0.008;
d_Costfunction=[];
TargetP = [1200;100;50];
TargetR = [1;0;0;0;0;1];
LastTargetP = TargetP;
LastTargetR = [1;0;0;0;0;1];

angle=720;
vel_ang_lim=60;
acc_ang_lim=100;
tol_time=14;
angle_seq=Accelerate_Plan(angle,vel_ang_lim,acc_ang_lim,tol_time,T);
[ang_vel_seq,ang_acc_seq]=angular_vel_acc(angle,vel_ang_lim,acc_ang_lim,tol_time,T);
MyTotalStep = size(angle_seq,2);

r=0.330;
c1=[0.680,0.100,0.050];
norm1=[0,0,-1];
[position_sep,c_a,c_b]=Generate_ellipsecircle(c1,r,norm1,90,angle_seq,1); % 生成位置
[arc_vel,arc_acc]=EllipseCircle_vel_acc(r,c_a,c_b,angle_seq,ang_vel_seq,ang_acc_seq,1);
m=1; init_c=3*diag([3,3,3,3,3,3]); init_k=diag([0,0,0,0,0,0]);
inte_u = 0;inte_e = 0;
LastU=0;U=0;
for i=1:MyTotalStep
    TargetP0=position_sep(i,:)';
    TargetP(1,1)=TargetP0(1)+0.0*randn(1);
    TargetP(2,1)=TargetP0(2)+0.0*randn(1);
    TargetP(3,1)=TargetP0(3)+0.0*randn(1);
    Target_acc=[arc_acc(i,:)';0;0;0;0;0;0];
    Target_vel=[arc_vel(i,:)';0;0;0;0;0;0];
    
    %
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
    Posture=[EndEffector(1:3);EndEffectorR];
    %%%
    Error = TargetP - EndEffector(1:3);
    ErrorR = TargetR - EndEffectorR;
    e=[Error;1*ErrorR];
    norm_e=norm(Error);
    norm_er=norm(ErrorR);
    %%%
    [J,diff_J1,diff_J2,diff_J3,diff_J4,diff_J5,diff_J6]=Jacobian_Cal(MyTheta);
    J(1:3,:)=0.001*J(1:3,:);diff_J1(1:3,:)=0.001*diff_J1(1:3,:);diff_J2(1:3,:)=0.001*diff_J2(1:3,:);diff_J3(1:3,:)=0.001*diff_J3(1:3,:);
    diff_J4(1:3,:)=0.001*diff_J4(1:3,:);diff_J5(1:3,:)=0.001*diff_J5(1:3,:);diff_J6(1:3,:)=0.001*diff_J6(1:3,:);
    PinvJ = inv( J'*J + Lamuda*I )*J';

    [y1]=Joint_limit_Null(MyTheta,Angle_lowlimit,Angle_uplimit);
    d_theta=0.35*PinvJ*e+0.15*y1;

    MyTheta=MyTheta+1*d_theta;
    MyThetaVelocity=1*d_theta/T;
    MyThetaAcc=(MyThetaVelocity-Last_MyThetaVelocity)/T;
    Last_MyThetaVelocity=MyThetaVelocity;
    
    Adj=[Adj;y1];
    MyEEPosition=[MyEEPosition;EndEffector'];
    Myerror=[Myerror;Error',ErrorR'];
    MyThetaAccPlot=[MyThetaAccPlot;MyThetaAcc'];
    MyThetaVelocityPlot = [MyThetaVelocityPlot;2*d_theta'/T];
    MyThetaPlot=[MyThetaPlot;MyTheta'];
    My_norme=[My_norme;norm_e,norm_er];
end
figure(1)
space_arm.plot(MyThetaPlot,'delay',0.001,'floorlevel',-0.4,'perspective')  %Create figure to simulate the robot motion in real time

t= (0:MyTotalStep)*1*0.008;
figure(2)
% subplot(2,1,1);
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

% subplot(2,1,2);
figure(3)
plot(t,MyThetaVelocityPlot(:,1),'-r');
hold on
plot(t,MyThetaVelocityPlot(:,2),'--g');
hold on
plot(t,MyThetaVelocityPlot(:,3),':b');
hold on
plot(t,MyThetaVelocityPlot(:,4),'-.m');
hold on
plot(t,MyThetaVelocityPlot(:,5),'--','Color',[1 0.5 0.1]);
hold on
plot(t,-MyThetaVelocityPlot(:,6),':k');

MyThetaPlot=MyThetaPlot*180/pi;
figure(4)
plot(t,MyThetaPlot(:,1),'-r');
hold on
plot(t,MyThetaPlot(:,2),'--g');
hold on
plot(t,MyThetaPlot(:,3),':b');
hold on
plot(t,MyThetaPlot(:,4),'-.m');
hold on
plot(t,MyThetaPlot(:,5),'--','Color',[1 0.5 0.1]);
hold on
plot(t,-MyThetaPlot(:,6),':k');

t= (1:MyTotalStep)*1*0.008;
figure(5)
% subplot(2,1,1);
plot(t,Myerror(:,1)*1000,'--r')
hold on
plot(t,Myerror(:,2)*1000,':g')
hold on
plot(t,Myerror(:,3)*1000,'-.b')
hold on
plot(t,My_norme(:,1)*1000,'-k','LineWidth',0.5)

% subplot(2,1,2);
figure(6)
% plot(t,Myerror(:,4),'r')
% hold on
% plot(t,Myerror(:,5),'g')
% hold on
% plot(t,Myerror(:,6),'b')
% hold on
plot(t,My_norme(:,2),'b')

