% non-Kalman
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
% MyTheta=[34.77 -90 -135 ...
%     -45 88.44 2.97]'*pi/180;

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

figure(1)
space_arm.plot(MyTheta');

T1=[cos(MyTheta(1)) -cos(s_alpha(1))*sin(MyTheta(1)) sin(s_alpha(1))*sin(MyTheta(1)) s_a(1)*cos(MyTheta(1));sin(MyTheta(1)) cos(s_alpha(1))*cos(MyTheta(1)) -sin(s_alpha(1))*cos(MyTheta(1)) s_a(1)*sin(MyTheta(1));0 sin(s_alpha(1)) cos(s_alpha(1)) s_d(1);0 0 0 1];
T2=[cos(MyTheta(2)) -cos(s_alpha(2))*sin(MyTheta(2)) sin(s_alpha(2))*sin(MyTheta(2)) s_a(2)*cos(MyTheta(2));sin(MyTheta(2)) cos(s_alpha(2))*cos(MyTheta(2)) -sin(s_alpha(2))*cos(MyTheta(2)) s_a(2)*sin(MyTheta(2));0 sin(s_alpha(2)) cos(s_alpha(2)) s_d(2);0 0 0 1];
T3=[cos(MyTheta(3)) -cos(s_alpha(3))*sin(MyTheta(3)) sin(s_alpha(3))*sin(MyTheta(3)) s_a(3)*cos(MyTheta(3));sin(MyTheta(3)) cos(s_alpha(3))*cos(MyTheta(3)) -sin(s_alpha(3))*cos(MyTheta(3)) s_a(3)*sin(MyTheta(3));0 sin(s_alpha(3)) cos(s_alpha(3)) s_d(3);0 0 0 1];
T4=[cos(MyTheta(4)) -cos(s_alpha(4))*sin(MyTheta(4)) sin(s_alpha(4))*sin(MyTheta(4)) s_a(4)*cos(MyTheta(4));sin(MyTheta(4)) cos(s_alpha(4))*cos(MyTheta(4)) -sin(s_alpha(4))*cos(MyTheta(4)) s_a(4)*sin(MyTheta(4));0 sin(s_alpha(4)) cos(s_alpha(4)) s_d(4);0 0 0 1];
T5=[cos(MyTheta(5)) -cos(s_alpha(5))*sin(MyTheta(5)) sin(s_alpha(5))*sin(MyTheta(5)) s_a(5)*cos(MyTheta(5));sin(MyTheta(5)) cos(s_alpha(5))*cos(MyTheta(5)) -sin(s_alpha(5))*cos(MyTheta(5)) s_a(5)*sin(MyTheta(5));0 sin(s_alpha(5)) cos(s_alpha(5)) s_d(5);0 0 0 1];
T6=[cos(MyTheta(6)) -cos(s_alpha(6))*sin(MyTheta(6)) sin(s_alpha(6))*sin(MyTheta(6)) s_a(6)*cos(MyTheta(6));sin(MyTheta(6)) cos(s_alpha(6))*cos(MyTheta(6)) -sin(s_alpha(6))*cos(MyTheta(6)) s_a(6)*sin(MyTheta(6));0 sin(s_alpha(6)) cos(s_alpha(6)) s_d(6);0 0 0 1];
T7=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 -0.078 1];
Twr1y=[1,0,0,0;0,1,0,0.002058;0,0,1,0;0,0,0,1];
T12=T1*T2;T13=T12*T3;T14=T13*Twr1y*T4;
T15=T14*T5;T16=T15*T6;T17=T16*T7;

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
My_norme=[];
MyCommandThetaVelocityPlot = [0;0;0;0;0;0];
MyCommandThetaAccPlot = [0;0;0;0;0;0];

position_record=[];
joint_record=[];
Target=[];
MyCommandThetaVelocity = [0;0;0;0;0;0];
OldMyCommandThetaVelocity = 0;
X2=MyCommandThetaVelocity;
diff_X2=[0;0;0;0;0;0];
X1=MyTheta;
X1_0=(Angle_lowlimit+Angle_uplimit)'/2;
diff_X1=[0;0;0;0;0;0];

Lamuda = 0.0001;
I = eye(6);
Ts = 0.0;
T = 0.008;

% TargetP = [810;150;0];
TargetP=EndEffector(1:3);
%tar_pos=load('position2.txt');
% TargetP = tar_pos(1,:)'/1000;
TargetP(3)=EE_initial;
TargetR = [0;-1;0;0;0;1];
LastTargetP = TargetP;
LastTargetR = [0;-1;0;0;0;1];
dz=0.090;%Point_s2=[tar_pos(1,1);tar_pos(1,2);tar_pos(1,3)+dz];
MyTotalStep=8000;
%Vx=(Point_s2(1)-Point_s1(1))/0.8;Vy=(Point_s2(2)-Point_s1(2))/0.8;
%Vz=(Point_s2(3)-Point_s1(3))/1;
%%%
m=1; init_c=3*diag([1,1,1,1,1,1]); init_k=diag([0,0,0,0,0,0]);
inte_u = 0;inte_e = 0;
LastU=0;U=0;

%%%
pause(0.1)
tic;
t1=toc;
%%%

for i=1:MyTotalStep
    TargetP_Last=TargetP;
    X11=pos1(1)/1000;Y11=pos1(2)/1000;Z11=pos1(3)/1000;
    Target_vel=[(TargetP-TargetP_Last)/T;0;0;0;0;0;0];
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
        %TargetP(3,1)=Point_s1(3)+Vz*(i-125)*T;
    else
        TargetP(1,1)=X11(1)/1+0.000;
        TargetP(2,1)=Y11(1)/1-0.000;
        TargetP(3,1)=Z11(1)/1+dz;
    end
%     Target_vel=[X11(2);Y11(2);Z11(2);0;0;0;0;0;0];
%     Target_acc=[arc_acc(i,:)';0;0;0;0;0;0];
%     Target_vel=[arc_vel(i,:)';0;0;0;0;0;0];
    %%%
    T1=[cos(MyTheta(1)) -cos(s_alpha(1))*sin(MyTheta(1)) sin(s_alpha(1))*sin(MyTheta(1)) s_a(1)*cos(MyTheta(1));sin(MyTheta(1)) cos(s_alpha(1))*cos(MyTheta(1)) -sin(s_alpha(1))*cos(MyTheta(1)) s_a(1)*sin(MyTheta(1));0 sin(s_alpha(1)) cos(s_alpha(1)) s_d(1);0 0 0 1];
    T2=[cos(MyTheta(2)) -cos(s_alpha(2))*sin(MyTheta(2)) sin(s_alpha(2))*sin(MyTheta(2)) s_a(2)*cos(MyTheta(2));sin(MyTheta(2)) cos(s_alpha(2))*cos(MyTheta(2)) -sin(s_alpha(2))*cos(MyTheta(2)) s_a(2)*sin(MyTheta(2));0 sin(s_alpha(2)) cos(s_alpha(2)) s_d(2);0 0 0 1];
    T3=[cos(MyTheta(3)) -cos(s_alpha(3))*sin(MyTheta(3)) sin(s_alpha(3))*sin(MyTheta(3)) s_a(3)*cos(MyTheta(3));sin(MyTheta(3)) cos(s_alpha(3))*cos(MyTheta(3)) -sin(s_alpha(3))*cos(MyTheta(3)) s_a(3)*sin(MyTheta(3));0 sin(s_alpha(3)) cos(s_alpha(3)) s_d(3);0 0 0 1];
    T4=[cos(MyTheta(4)) -cos(s_alpha(4))*sin(MyTheta(4)) sin(s_alpha(4))*sin(MyTheta(4)) s_a(4)*cos(MyTheta(4));sin(MyTheta(4)) cos(s_alpha(4))*cos(MyTheta(4)) -sin(s_alpha(4))*cos(MyTheta(4)) s_a(4)*sin(MyTheta(4));0 sin(s_alpha(4)) cos(s_alpha(4)) s_d(4);0 0 0 1];
    T5=[cos(MyTheta(5)) -cos(s_alpha(5))*sin(MyTheta(5)) sin(s_alpha(5))*sin(MyTheta(5)) s_a(5)*cos(MyTheta(5));sin(MyTheta(5)) cos(s_alpha(5))*cos(MyTheta(5)) -sin(s_alpha(5))*cos(MyTheta(5)) s_a(5)*sin(MyTheta(5));0 sin(s_alpha(5)) cos(s_alpha(5)) s_d(5);0 0 0 1];
    T6=[cos(MyTheta(6)) -cos(s_alpha(6))*sin(MyTheta(6)) sin(s_alpha(6))*sin(MyTheta(6)) s_a(6)*cos(MyTheta(6));sin(MyTheta(6)) cos(s_alpha(6))*cos(MyTheta(6)) -sin(s_alpha(6))*cos(MyTheta(6)) s_a(6)*sin(MyTheta(6));0 sin(s_alpha(6)) cos(s_alpha(6)) s_d(6);0 0 0 1];
    Twr1y=[1,0,0,0;0,1,0,0.002058;0,0,1,0;0,0,0,1];
    T7=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0.078 1];
    T12=T1*T2;T13=T12*T3;T14=T13*Twr1y*T4;%T14=T13*T4;
    T15=T14*T5;T16=T15*T6;T17=T16*T6;
    
    EndEffector = T16*[0;0;0;1];
    %%%
    EndEffectorR = [T16(1,1);T16(2,1);T16(3,1);T16(1,3);T16(2,3);T16(3,3)];
    Posture=[EndEffector(1:3);EndEffectorR];
    %%%
    Error = TargetP - EndEffector(1:3);
    ErrorR = TargetR - EndEffectorR;
    norm_e=norm(Error);
    norm_er=norm(ErrorR);
    e1=[TargetP - EndEffector(1:3);1*ErrorR];
    e=e1;
    max_e=max(abs(e1));

    %%%
    [J,diff_J1,diff_J2,diff_J3,diff_J4,diff_J5,diff_J6]=Jacobian_Cal(MyTheta);
    J(1:3,:)=0.001*J(1:3,:);diff_J1(1:3,:)=0.001*diff_J1(1:3,:);diff_J2(1:3,:)=0.001*diff_J2(1:3,:);diff_J3(1:3,:)=0.001*diff_J3(1:3,:);
    diff_J4(1:3,:)=0.001*diff_J4(1:3,:);diff_J5(1:3,:)=0.001*diff_J5(1:3,:);diff_J6(1:3,:)=0.001*diff_J6(1:3,:);
    PinvJ = inv( J'*J + Lamuda*I )*J';
    diff_J=diff_J1*X2(1)+diff_J2*X2(2)+diff_J3*X2(3)+diff_J4*X2(4)+diff_J5*X2(5)+diff_J6*X2(6);
    [y1,y2]=Joint_limit(MyTheta,Angle_lowlimit,Angle_uplimit,X2,Ang_vel_lowlimit,Ang_vel_uplimit);
    %
    [adj1,adj2,adj3,adj4]=Gain_coefficient(MyTheta,Angle_lowlimit,Angle_uplimit,X2,e);
    e_vel=0*Target_vel-J*X2;
    k=init_k+1*diag(y1);
    c=init_c+1*diag(y2);
%     MyCommandThetaAcc=PinvJ*(Target_acc+(c*J-diff_J)*PinvJ*Target_vel+1*k*(0*e+1*J*X2)+25000*e+10000*inte_e+250*e_vel);
%     con1=1*PinvJ*(0+1*(J*c-diff_J)*PinvJ*(Posture-LastPosture)/T+1*(1*J*k*X2));
    con1=1*PinvJ*(0+1*(J*c-diff_J)*PinvJ*J*X2+1*(1*J*k*X2));
%     ((Posture-LastPosture)/T-J*X2)'
%     con2=1*PinvJ*(1500*adj2*e+(100)*adj2*adj3*e_vel);
    con2=1*PinvJ*(550*adj2*e+(15)*adj2*adj3*e_vel);
    MyCommandThetaAcc=0*adj1*con1+1*con2;
%     MyCommandThetaAcc=1*PinvJ*(1*Target_acc+1*(J*c-diff_J)*PinvJ*(Posture-LastPosture)/T+1*(1*J*k*X2))+1*PinvJ*(200*e+300*inte_e+50*e_vel);
    %%%%%%
    
    U=1*MyCommandThetaAcc+0.0*inte_u;
    max_U=max(abs(con1+con2));

    diff_X1=X2;
    diff_X2=-c/m*X2-k/m*(X1-X1_0)+U/m;
    max2=max(abs(diff_X2));
    
    X1=X1+X2*T;
    X2=X2+diff_X2*T;
    
    MyTheta=X1;
    MyCommandThetaVelocity=X2;
    %%%%%%
    MyCommandThetaAccPlot = [MyCommandThetaAccPlot diff_X2];
    MyCommandThetaVelocityPlot = [MyCommandThetaVelocityPlot MyCommandThetaVelocity];
    
    LastTargetP = TargetP;
    LastEndEffector = EndEffector;
    LastPosture=Posture;

    MyThetaPlot = [MyThetaPlot MyTheta];
    Myerror=[Myerror;Error',ErrorR'];
    MyControlInput=[MyControlInput;U',con1'+con2'];
    MyL_k=[MyL_k;diag(k)'];
    MyL_c=[MyL_c;diag(c)'];
    MyCon=[MyCon;con1',con2'];
    My_f=[My_f;k(3,3)*(MyTheta(3)-X1_0(3)),U(3)];
    My_adj=[My_adj;k(3,3),adj1,adj2,adj3];
    My_norme=[My_norme;norm_e,norm_er];
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

dlmwrite('plan_jointacc_nonkal1.txt', MyCommandThetaAccPlot);
dlmwrite('plan_jointvel_nonkal1.txt', MyCommandThetaVelocityPlot);
dlmwrite('plan_jointang_nonkal1.txt', MyThetaPlot);
dlmwrite('plan_error_nonkal1.txt', Myerror);

figure(1)
% space_arm.plot(MyThetaPlot','delay',0.001,'floorlevel',-0.4,'perspective')  %Create figure to simulate the robot motion in real time

t= (0:MyTotalStep)*1*0.008;
figure(2)
% subplot(2,1,1);
plot(t,MyCommandThetaAccPlot(1,:),'-r','LineWidth',1);
hold on
plot(t,MyCommandThetaAccPlot(2,:),'--g','LineWidth',1);
hold on
plot(t,MyCommandThetaAccPlot(3,:),':b','LineWidth',1);
hold on
plot(t,MyCommandThetaAccPlot(4,:),'-.m','LineWidth',1);
hold on
plot(t,MyCommandThetaAccPlot(5,:),'--','LineWidth',1,'Color',[1 0.5 0.1]);
hold on
plot(t,MyCommandThetaAccPlot(6,:),':k','LineWidth',1);
set(gca,'fontsize',10,'Box','off');set (gcf,'Position', [0 0 400 300]);
% xlim([-0.2 6.2])
xlabel('t(s)','FontSize',10);
ylabel('Joint accelerations(rad/s^2)','FontSize',10);
leg=legend('$$\ddot{\theta}_1$$','$\ddot{\theta}_2$','$\ddot{\theta}_3$','$\ddot{\theta}_4$','$\ddot{\theta}_5$','$\ddot{\theta}_6$','interpreter','latex','NumColumns',3,'FontSize',10);
leg.ItemTokenSize = [20,18];
legend('boxoff')
% subplot(2,1,2);
figure(3)
plot(t,MyCommandThetaVelocityPlot(1,:),'-r','LineWidth',1)
hold on
plot(t,MyCommandThetaVelocityPlot(2,:),'--g','LineWidth',1)
hold on
plot(t,MyCommandThetaVelocityPlot(3,:),':b','LineWidth',1)
hold on
plot(t,MyCommandThetaVelocityPlot(4,:),'-.m','LineWidth',1)
hold on
plot(t,MyCommandThetaVelocityPlot(5,:),'--','LineWidth',1,'Color',[1 0.5 0.1])
hold on
plot(t,MyCommandThetaVelocityPlot(6,:),':k','LineWidth',1)
set(gca,'fontsize',10,'Box','off');set (gcf,'Position', [10 10 400 300]);
% xlim([-0.2 6.2]);ylim([-3 3])
xlabel('t(s)','FontSize',10);
ylabel('Joint velocities(rad/s)','FontSize',10);
legend('$$\dot{\theta}_1$$','$\dot{\theta}_2$','$\dot{\theta}_3$','$\dot{\theta}_4$','$\dot{\theta}_5$','$\dot{\theta}_6$','interpreter','latex','NumColumns',3,'FontSize',10);
legend('boxoff')

figure(4)
MyThetaPlot1=MyThetaPlot*180/pi;
plot(t,MyThetaPlot1(1,:),'-r','LineWidth',1)
hold on
plot(t,MyThetaPlot1(2,:),'--g','LineWidth',1)
hold on
plot(t,MyThetaPlot1(3,:),':b','LineWidth',1)
hold on
plot(t,MyThetaPlot1(4,:),'-.m','LineWidth',1)
hold on
plot(t,MyThetaPlot1(5,:),'--','LineWidth',1,'Color',[1 0.5 0.1])
hold on
plot(t,MyThetaPlot1(6,:),':k','LineWidth',1)
set(gca,'fontsize',10,'Box','off');set (gcf,'Position', [10 10 400 300]);
% xlim([-0.2 6.2]);ylim([-200 240])
xlabel('t(s)','FontSize',10);
ylabel('Joint angles(°)','FontSize',10);
legend('${\theta}_1$','${\theta}_2$','${\theta}_3$','${\theta}_4$','${\theta}_5$','${\theta}_6$','interpreter','latex','NumColumns',3,'FontSize',10);
legend('boxoff')

t= (1:MyTotalStep)*1*0.008;
figure(5)
% subplot(2,1,1);
plot(t,Myerror(:,1)*1000,'--r','LineWidth',1)
hold on
plot(t,Myerror(:,2)*1000,':g','LineWidth',1)
hold on
plot(t,Myerror(:,3)*1000,'-.b','LineWidth',1)
hold on
plot(t,My_norme(:,1)*1000,'-k','LineWidth',1)
set(gca,'fontsize',10,'Box','off');set (gcf,'Position', [10 10 400 300]);
xlabel('t(s)','FontSize',10);
ylabel('Position error(mm)','FontSize',10);
legend('x direction error','y direction error','z direction error','EE error','interpreter','latex','NumColumns',1,'FontSize',10);
legend('boxoff')

figure(6)
plot(t,My_norme(:,2),'b')
set(gca,'fontsize',10,'Box','off');set (gcf,'Position', [10 10 400 300]);
xlabel('t(s)','FontSize',10);
ylabel('Attitude error(rad)','FontSize',10);
legend('Attitude error','interpreter','latex','NumColumns',1,'FontSize',10);
legend('boxoff')

figure(7)
plot(t,MyControlInput(:,1),'-r','LineWidth',1)
hold on
plot(t,MyControlInput(:,2),'--g','LineWidth',1)
hold on
plot(t,MyControlInput(:,3),':b','LineWidth',1)
hold on
plot(t,MyControlInput(:,4),'-.m','LineWidth',1)
hold on
plot(t,MyControlInput(:,5),'--','LineWidth',1,'Color',[1 0.5 0.1])
hold on
plot(t,MyControlInput(:,6),':k','LineWidth',1)
set(gca,'fontsize',10,'Box','off');set (gcf,'Position', [10 10 400 300]);
xlabel('t(s)','FontSize',10);
ylabel('Control input','FontSize',10);
legend('${u}_1$','${u}_2$','${u}_3$','${u}_4$','${u}_5$','${u}_6$','interpreter','latex','NumColumns',3,'FontSize',10);
legend('boxoff')

% figure(8)
% plot(t,tar_pos(:,1)*1,'-r','LineWidth',1)
% hold on
% plot(t,tar_pos(:,2)*1,'-g','LineWidth',1)
% hold on
% plot(t,tar_pos(:,3)*1,'-b','LineWidth',1)

figure(9)
plot3(Target(:,1),Target(:,2),Target(:,3),'-');
hold on
% plot3(tar_pos(:,1),tar_pos(:,2),tar_pos(:,3),'-');

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