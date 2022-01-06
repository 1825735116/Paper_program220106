% Kalman
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
MyTheta =[8.59  -88.70  -121.20 ...
    -59.14  90   111.41]'*pi/180;

Angle_lowlimit=[-360,-360,10,-360,-360,-360]*pi/180;
Angle_uplimit=[360,360, 150,360,360,360]*pi/180;
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

figure(1)
space_arm.plot(MyTheta');

T1=[cos(MyTheta(1)) -cos(s_alpha(1))*sin(MyTheta(1)) sin(s_alpha(1))*sin(MyTheta(1)) s_a(1)*cos(MyTheta(1));sin(MyTheta(1)) cos(s_alpha(1))*cos(MyTheta(1)) -sin(s_alpha(1))*cos(MyTheta(1)) s_a(1)*sin(MyTheta(1));0 sin(s_alpha(1)) cos(s_alpha(1)) s_d(1);0 0 0 1];
T2=[cos(MyTheta(2)) -cos(s_alpha(2))*sin(MyTheta(2)) sin(s_alpha(2))*sin(MyTheta(2)) s_a(2)*cos(MyTheta(2));sin(MyTheta(2)) cos(s_alpha(2))*cos(MyTheta(2)) -sin(s_alpha(2))*cos(MyTheta(2)) s_a(2)*sin(MyTheta(2));0 sin(s_alpha(2)) cos(s_alpha(2)) s_d(2);0 0 0 1];
T3=[cos(MyTheta(3)) -cos(s_alpha(3))*sin(MyTheta(3)) sin(s_alpha(3))*sin(MyTheta(3)) s_a(3)*cos(MyTheta(3));sin(MyTheta(3)) cos(s_alpha(3))*cos(MyTheta(3)) -sin(s_alpha(3))*cos(MyTheta(3)) s_a(3)*sin(MyTheta(3));0 sin(s_alpha(3)) cos(s_alpha(3)) s_d(3);0 0 0 1];
T4=[cos(MyTheta(4)) -cos(s_alpha(4))*sin(MyTheta(4)) sin(s_alpha(4))*sin(MyTheta(4)) s_a(4)*cos(MyTheta(4));sin(MyTheta(4)) cos(s_alpha(4))*cos(MyTheta(4)) -sin(s_alpha(4))*cos(MyTheta(4)) s_a(4)*sin(MyTheta(4));0 sin(s_alpha(4)) cos(s_alpha(4)) s_d(4);0 0 0 1];
T5=[cos(MyTheta(5)) -cos(s_alpha(5))*sin(MyTheta(5)) sin(s_alpha(5))*sin(MyTheta(5)) s_a(5)*cos(MyTheta(5));sin(MyTheta(5)) cos(s_alpha(5))*cos(MyTheta(5)) -sin(s_alpha(5))*cos(MyTheta(5)) s_a(5)*sin(MyTheta(5));0 sin(s_alpha(5)) cos(s_alpha(5)) s_d(5);0 0 0 1];
T6=[cos(MyTheta(6)) -cos(s_alpha(6))*sin(MyTheta(6)) sin(s_alpha(6))*sin(MyTheta(6)) s_a(6)*cos(MyTheta(6));sin(MyTheta(6)) cos(s_alpha(6))*cos(MyTheta(6)) -sin(s_alpha(6))*cos(MyTheta(6)) s_a(6)*sin(MyTheta(6));0 sin(s_alpha(6)) cos(s_alpha(6)) s_d(6);0 0 0 1];
Twr1y=[1,0,0,0;0,1,0,0.002058;0,0,1,0;0,0,0,1];
T12=T1*T2;T13=T12*T3;T14=T13*Twr1y*T4;
T15=T14*T5;T16=T15*T6;

EndEffector = T16*[0;0;0;1];
EndEffectorR = [T16(1,1);T16(2,1);T16(3,1);T16(1,3);T16(2,3);T16(3,3)];
Point_s1=EndEffector(1:3);
LastEndEffector = EndEffector;
LastEndEffectorR = EndEffectorR;
LastPosture=[EndEffector(1:3);EndEffectorR];
EE_initial=EndEffector(3);
%%%
Myerror = [];
Myinte_e=[];
MyattTheta=[];
MyattTheta1=[];
MyThetaPlot=[MyTheta];
MyControlInput=[];
MyL_k=[];
MyL_c=[];
MyCon=[];
My_f=[];
My_adj=[];
MyCommandThetaVelocityPlot = [0;0;0;0;0;0];
MyCommandThetaAccPlot = [0;0;0;0;0;0];
position_record_kal=[];
position_record=[];
joint_record=[];

MyCommandThetaVelocity = [0;0;0;0;0;0];
OldMyCommandThetaVelocity = 0;
X2=MyCommandThetaVelocity;
diff_X2=[0;0;0;0;0;0];
X1=MyTheta;
X1_0=(Angle_lowlimit+Angle_uplimit)'/2;
diff_X1=[0;0;0;0;0;0];

Lamuda = 0.001;
I = eye(6);
Ts = 0.0;
T = 0.008;

TargetP=EndEffector(1:3);
TargetP(3)=EE_initial;
TargetR = [0;-1;0;0;0;1];
LastTargetP = TargetP;
LastTargetR = [0;-1;0;0;0;1];
Last_dq=[0;0;0;0;0;0];
dz=0.100;
MyTotalStep=8000;

%%%
I_kalman=eye(3);H=[1 0 0];
A=[1 T T^2/2;0 1 T;0 0 1];
Q=[0 0 0;0 T 0;0 0 T^2]*2;
R=[1,0.1,0.1];
Px00=[1 0 0;0 1 0;0 0 1];Py00=[1 0 0;0 1 0;0 0 1];Pz00=[1 0 0;0 1 0;0 0 1];
X00=[pos1(1)/1000;0;0];Y00=[pos1(2)/1000;0;0];Z00=[pos1(3)/1000;0;0];
Tar_X=[X00'];Tar_Y=[X00'];Tar_Z=[X00'];

%%%
pause(0.1)
tic;
t1=toc;
%%%

for i=1:MyTotalStep
    TargetP_Last=TargetP;
    %%%
    X10=A*X00;Y10=A*Y00;Z10=A*Z00;
    Px10=A*Px00*A'+Q;Py10=A*Py00*A'+Q;Pz10=A*Pz00*A'+Q;
    Kx1=Px10*H'/(H*Px10*H'+R(1));Ky1=Py10*H'/(H*Py10*H'+R(2));Kz1=Pz10*H'/(H*Pz10*H'+R(3));
    yx1=[pos1(1)/1000];yy1=[pos1(2)/1000];yz1=[pos1(3)/1000];
    X11=X10+Kx1*(yx1-H*X10);Y11=Y10+Ky1*(yy1-H*Y10);Z11=Z10+Kz1*(yz1-H*Z10);
    Px11=(I_kalman-Kx1*H)*Px10;Py11=(I_kalman-Ky1*H)*Py10;Pz11=(I_kalman-Kz1*H)*Pz10;
    
    X00=X11;Y00=Y11;Z00=Z11;
    Px00=Px11;Py00=Py11;Pz00=Pz11;
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
    Target_vel=[X11(2);Y11(2);Z11(2);0;0;0;0;0;0];
    %%%
    T1=[cos(MyTheta(1)) -cos(s_alpha(1))*sin(MyTheta(1)) sin(s_alpha(1))*sin(MyTheta(1)) s_a(1)*cos(MyTheta(1));sin(MyTheta(1)) cos(s_alpha(1))*cos(MyTheta(1)) -sin(s_alpha(1))*cos(MyTheta(1)) s_a(1)*sin(MyTheta(1));0 sin(s_alpha(1)) cos(s_alpha(1)) s_d(1);0 0 0 1];
    T2=[cos(MyTheta(2)) -cos(s_alpha(2))*sin(MyTheta(2)) sin(s_alpha(2))*sin(MyTheta(2)) s_a(2)*cos(MyTheta(2));sin(MyTheta(2)) cos(s_alpha(2))*cos(MyTheta(2)) -sin(s_alpha(2))*cos(MyTheta(2)) s_a(2)*sin(MyTheta(2));0 sin(s_alpha(2)) cos(s_alpha(2)) s_d(2);0 0 0 1];
    T3=[cos(MyTheta(3)) -cos(s_alpha(3))*sin(MyTheta(3)) sin(s_alpha(3))*sin(MyTheta(3)) s_a(3)*cos(MyTheta(3));sin(MyTheta(3)) cos(s_alpha(3))*cos(MyTheta(3)) -sin(s_alpha(3))*cos(MyTheta(3)) s_a(3)*sin(MyTheta(3));0 sin(s_alpha(3)) cos(s_alpha(3)) s_d(3);0 0 0 1];
    T4=[cos(MyTheta(4)) -cos(s_alpha(4))*sin(MyTheta(4)) sin(s_alpha(4))*sin(MyTheta(4)) s_a(4)*cos(MyTheta(4));sin(MyTheta(4)) cos(s_alpha(4))*cos(MyTheta(4)) -sin(s_alpha(4))*cos(MyTheta(4)) s_a(4)*sin(MyTheta(4));0 sin(s_alpha(4)) cos(s_alpha(4)) s_d(4);0 0 0 1];
    T5=[cos(MyTheta(5)) -cos(s_alpha(5))*sin(MyTheta(5)) sin(s_alpha(5))*sin(MyTheta(5)) s_a(5)*cos(MyTheta(5));sin(MyTheta(5)) cos(s_alpha(5))*cos(MyTheta(5)) -sin(s_alpha(5))*cos(MyTheta(5)) s_a(5)*sin(MyTheta(5));0 sin(s_alpha(5)) cos(s_alpha(5)) s_d(5);0 0 0 1];
    T6=[cos(MyTheta(6)) -cos(s_alpha(6))*sin(MyTheta(6)) sin(s_alpha(6))*sin(MyTheta(6)) s_a(6)*cos(MyTheta(6));sin(MyTheta(6)) cos(s_alpha(6))*cos(MyTheta(6)) -sin(s_alpha(6))*cos(MyTheta(6)) s_a(6)*sin(MyTheta(6));0 sin(s_alpha(6)) cos(s_alpha(6)) s_d(6);0 0 0 1];
    Twr1y=[1,0,0,0;0,1,0,0.002058;0,0,1,0;0,0,0,1];
    T12=T1*T2;T13=T12*T3;T14=T13*Twr1y*T4;
    T15=T14*T5;T16=T15*T6;
    
    EndEffector = T16*[0;0;0;1];
    %%%
    EndEffectorR = [T16(1,1);T16(2,1);T16(3,1);T16(1,3);T16(2,3);T16(3,3)];
    Posture=[EndEffector(1:3);EndEffectorR];
    %%%
    Error = TargetP - EndEffector(1:3);
    ErrorR = TargetR - EndEffectorR;
    e1=[TargetP - EndEffector(1:3);1*ErrorR];
    e=e1;
    max_e=max(abs(e1));
    %%%
    [J,diff_J1,diff_J2,diff_J3,diff_J4,diff_J5,diff_J6]=Jacobian_Cal(MyTheta);
    J(1:3,:)=0.001*J(1:3,:);diff_J1(1:3,:)=0.001*diff_J1(1:3,:);diff_J2(1:3,:)=0.001*diff_J2(1:3,:);diff_J3(1:3,:)=0.001*diff_J3(1:3,:);
    diff_J4(1:3,:)=0.001*diff_J4(1:3,:);diff_J5(1:3,:)=0.001*diff_J5(1:3,:);diff_J6(1:3,:)=0.001*diff_J6(1:3,:);
    PinvJ = inv( J'*J + Lamuda*I )*J';
    dq=0.1*PinvJ*e;
    MyTheta=MyTheta+dq;
    MyAcceleration=(dq/T-Last_dq/T)/T;
    Last_dq=dq;
    MyThetaPlot = [MyThetaPlot MyTheta];
    MyCommandThetaVelocityPlot=[MyCommandThetaVelocityPlot dq/T];
    MyCommandThetaAccPlot=[MyCommandThetaAccPlot MyAcceleration];
    Myerror=[Myerror;Error',ErrorR'];

    position_record_kal=[position_record_kal;TargetP'];
    position_record=[position_record;pos1(1)/1000,pos1(2)/1000,pos1(3)/1000];
    joint_record=[joint_record;joint_angle,joint_vel, joint_effort];
    tar_point.Positions=[MyTheta(1:5)',-MyTheta(6)];
    joint_state.Points=tar_point;

    t2=toc;
    pause(0.008-t2+t1)
    toc-t1
    t1=toc;
    send(anglepub,joint_state)
end

dlmwrite('position_kal1.txt', position_record_kal);
dlmwrite('position_cam1.txt', position_record);
dlmwrite('robot_states_kal1.txt', joint_record);

dlmwrite('plan_jointacc_kal1.txt', MyCommandThetaAccPlot);
dlmwrite('plan_jointvel_kal1.txt', MyCommandThetaVelocityPlot);
dlmwrite('plan_jointang_kal1.txt', MyThetaPlot);
dlmwrite('plan_error_kal1.txt', Myerror);

figure(1)
space_arm.plot(MyThetaPlot','delay',0.001,'floorlevel',-0.4,'perspective')  %Create figure to simulate the robot motion in real time

t= (0:MyTotalStep)*1;
figure(2)
plot(t,MyCommandThetaAccPlot(1,:),'r');
hold on
plot(t,MyCommandThetaAccPlot(2,:),'g');
hold on
plot(t,MyCommandThetaAccPlot(3,:),'c');
hold on
plot(t,MyCommandThetaAccPlot(4,:),'b');
hold on
plot(t,MyCommandThetaAccPlot(5,:),'m');
hold on
plot(t,MyCommandThetaAccPlot(6,:),'k');
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',12);
ylabel('角加速度','FontSize',12);

figure(3)
% subplot(2,1,1);
% plot(t,MyCommandThetaAccPlot(1,:),'r');
% hold on
% plot(t,MyCommandThetaAccPlot(2,:),'g');
% hold on
% plot(t,MyCommandThetaAccPlot(3,:),'c');
% hold on
% plot(t,MyCommandThetaAccPlot(4,:),'b');
% hold on
% plot(t,MyCommandThetaAccPlot(5,:),'m');
% hold on
% plot(t,MyCommandThetaAccPlot(6,:),'k');
% set(gca,'fontsize',15);
% xlabel('迭代次数','FontSize',12);
% ylabel('角加速度','FontSize',12);
% subplot(2,1,2);
plot(t,MyCommandThetaVelocityPlot(1,:),'r')
hold on
plot(t,MyCommandThetaVelocityPlot(2,:),'g')
hold on
plot(t,MyCommandThetaVelocityPlot(3,:),'c')
hold on
plot(t,MyCommandThetaVelocityPlot(4,:),'b')
hold on
plot(t,MyCommandThetaVelocityPlot(5,:),'m')
hold on
plot(t,MyCommandThetaVelocityPlot(6,:),'k')
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',30);
ylabel('角速度/ °/s','FontSize',30);

figure(4)
MyThetaPlot=MyThetaPlot*180/pi;
plot(t,MyThetaPlot(1,:),'r')
hold on
plot(t,MyThetaPlot(2,:),'g')
hold on
plot(t,MyThetaPlot(3,:),'c')
hold on
plot(t,MyThetaPlot(4,:),'b')
hold on
plot(t,MyThetaPlot(5,:),'m')
hold on
plot(t,MyThetaPlot(6,:),'k')
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',30);
ylabel('角度/rad','FontSize',30);

t= (1:MyTotalStep)*1;
figure(5)
subplot(2,1,1);
plot(t,Myerror(:,1)*1000,'r')
hold on
plot(t,Myerror(:,2)*1000,'g')
hold on
plot(t,Myerror(:,3)*1000,'b')
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',30);
ylabel('末端误差/mm','FontSize',30);
subplot(2,1,2);
plot(t,Myerror(:,4),'r')
hold on
plot(t,Myerror(:,5),'g')
hold on
plot(t,Myerror(:,6),'b')
set(gca,'fontsize',15);
xlabel('迭代次数','FontSize',30);
ylabel('姿态误差/','FontSize',30);

% figure(6)
% plot(t,target_position(:,1),'r');
% hold on
% plot(t,target_position(:,2),'g');
% hold on
% plot(t,target_position(:,3),'b');

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