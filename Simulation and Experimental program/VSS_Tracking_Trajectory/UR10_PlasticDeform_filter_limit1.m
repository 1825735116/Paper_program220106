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
MyTheta =[2.704986333979333  -0.831878581129427   1.707396091332526...
  -2.446313835610565  -1.570485222135656   2.007402646409449]';
MyTheta=[-3.11 -98.68 -123.98...
    -50.79 88.44 2.97]'*pi/180;
figure(1)
space_arm.plot(MyTheta');
Angle_lowlimit=[-360,-360,-155,-360,-360,-360]*pi/180;
Angle_uplimit=[360,360, 155,360,360,360]*pi/180;
Ang_vel_lowlimit=[-120,-120,-120,-180,-180,-180]*pi/180;
Ang_vel_uplimit=[120,120, 120,180,180,180]*pi/180;
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
%%%
Myerror = [];
Myinte_e=[];
MyattTheta=[];
MyattTheta1=[];
MyThetaPlot=[MyTheta];
MyEEPosition=[];
MyControlInput=[];
MyL_k=[];
MyL_c=[];
MyCon=[];
My_f=[];
My_adj=[];
My_norme=[];
MyCommandThetaVelocityPlot = [0;0;0;0;0;0];
MyCommandThetaAccPlot = [0;0;0;0;0;0];

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

TargetP = [1200;100;50];
TargetR = [0;-1;0;0;0;1];
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
[position_sep,c_a,c_b]=Generate_ellipsecircle(c1,r,norm1,90,angle_seq,1);
[arc_vel,arc_acc]=EllipseCircle_vel_acc(r,c_a,c_b,angle_seq,ang_vel_seq,ang_acc_seq,1);
m=1; init_c=3*diag([1,1,1,1,1,1]); init_k=diag([0,0,0,0,0,0]);
inte_u = 0;inte_e = 0;
LastU=0;U=0;
for i=1:MyTotalStep
    TargetP=position_sep(i,:)';
    Target_acc=[arc_acc(i,:)';0;0;0;0;0;0];
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
    e_vel=Target_vel-J*X2;
    [adj1,adj2,adj3,adj4]=Gain_coefficient(MyTheta,Angle_lowlimit,Angle_uplimit,PinvJ*e_vel,e);
    
    k=init_k+1*diag(y1);
    c=init_c+1*diag(y2);
%     MyCommandThetaAcc=PinvJ*(Target_acc+(c*J-diff_J)*PinvJ*Target_vel+1*k*(0*e+1*J*X2)+25000*e+10000*inte_e+250*e_vel);
%     con1=1*PinvJ*(1*Target_acc+1*(J*c-diff_J)*PinvJ*(Posture-LastPosture)/T+1*(1*J*k*X2));
    con1=1*PinvJ*(Target_acc+1*(J*c-diff_J)*PinvJ*J*X2+1*(1*J*k*X2));
%     ((Posture-LastPosture)/T-J*X2)'
    con2=1*PinvJ*(500*adj2*e+(40)*adj2*adj3*e_vel);

    MyCommandThetaAcc=1*adj1*adj2*con1+1*con2;
%     MyCommandThetaAcc=1*PinvJ*(1*Target_acc+1*(J*c-diff_J)*PinvJ*(Posture-LastPosture)/T+1*(1*J*k*X2))+1*PinvJ*(200*e+300*inte_e+50*e_vel);
    %%%%%%
    
%     inte_u=inte_u+MyCommandThetaAcc*T;
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
    MyCommandThetaVelocityPlot = [MyCommandThetaVelocityPlot MyCommandThetaVelocity]; % ???????????????????????????????????????????????????
    
    LastTargetP = TargetP;
    LastEndEffector = EndEffector;
    LastPosture=Posture;

    MyThetaPlot = [MyThetaPlot MyTheta]; % ???????????????????????????
    Myerror=[Myerror;Error',ErrorR',e_vel'];
    MyControlInput=[MyControlInput;U',con1'+con2'];
    MyEEPosition=[MyEEPosition;EndEffector'];
    MyL_k=[MyL_k;diag(k)'];
    MyL_c=[MyL_c;diag(c)'];
    MyCon=[MyCon;con1',con2'];
    My_f=[My_f;k(3,3)*(MyTheta(3)-X1_0(3)),U(3)];
    My_adj=[My_adj;k(3,3),adj1,adj2,adj3];
    My_norme=[My_norme;norm_e,norm_er];
end
figure(1)
space_arm.plot(MyThetaPlot','delay',0.001,'floorlevel',-0.4,'perspective')  %Create figure to simulate the robot motion in real time

t= (0:MyTotalStep)*1*0.008;
figure(2)
% subplot(2,1,1);
plot(t,MyCommandThetaAccPlot(1,:),'-r');
hold on
plot(t,MyCommandThetaAccPlot(2,:),'--g');
hold on
plot(t,MyCommandThetaAccPlot(3,:),':b');
hold on
plot(t,MyCommandThetaAccPlot(4,:),'-.m');
hold on
plot(t,MyCommandThetaAccPlot(5,:),'--','Color',[1 0.5 0.1]);
hold on
plot(t,-MyCommandThetaAccPlot(6,:),':k');

% subplot(2,1,2);
figure(3)
plot(t,MyCommandThetaVelocityPlot(1,:),'-r')
hold on
plot(t,MyCommandThetaVelocityPlot(2,:),'--g')
hold on
plot(t,MyCommandThetaVelocityPlot(3,:),':b')
hold on
plot(t,MyCommandThetaVelocityPlot(4,:),'-.m')
hold on
plot(t,MyCommandThetaVelocityPlot(5,:),'--','Color',[1 0.5 0.1])
hold on
plot(t,-MyCommandThetaVelocityPlot(6,:),':k')


figure(4)
MyThetaPlot1=MyThetaPlot*180/pi;
plot(t,MyThetaPlot1(1,:),'-r')
hold on
plot(t,MyThetaPlot1(2,:),'--g')
hold on
plot(t,MyThetaPlot1(3,:),':b')
hold on
plot(t,MyThetaPlot1(4,:),'-.m')
hold on
plot(t,MyThetaPlot1(5,:),'--','Color',[1 0.5 0.1])
hold on
plot(t,-MyThetaPlot1(6,:),':k')


t= (1:MyTotalStep)*1*0.008;
figure(5)
% subplot(2,1,1);
plot(t,Myerror(:,1)*1000,'r')
hold on
plot(t,Myerror(:,2)*1000,'g')
hold on
plot(t,Myerror(:,3)*1000,'b')
hold on
plot(t,My_norme(:,1)*1000,'-k','LineWidth',0.5)

% subplot(2,1,2);
figure(6)
% plot(t,Myerror(:,4),'r')
% hold on
% plot(t,Myerror(:,5),'g')
% hold on
% plot(t,Myerror(:,6),'b')
plot(t,My_norme(:,2),'b')


% figure(5)
% subplot(2,2,1);
% plot(t,MyCon(:,1),'r')
% hold on
% plot(t,MyCon(:,2),'g')
% hold on
% plot(t,MyCon(:,3),'c')
% hold on
% plot(t,MyCon(:,4),'b')
% hold on
% plot(t,MyCon(:,5),'m')
% hold on
% plot(t,MyCon(:,6),'k')
% subplot(2,2,2);
% plot(t,MyCon(:,7),'r')
% hold on
% plot(t,MyCon(:,8),'g')
% hold on
% plot(t,MyCon(:,9),'c')
% hold on
% plot(t,MyCon(:,10),'b')
% hold on
% plot(t,MyCon(:,11),'m')
% hold on
% plot(t,MyCon(:,12),'k')
% subplot(2,2,3);
% plot(t,MyControlInput(:,1),'r')
% hold on
% plot(t,MyControlInput(:,2),'g')
% hold on
% plot(t,MyControlInput(:,3),'c')
% hold on
% plot(t,MyControlInput(:,4),'b')
% hold on
% plot(t,MyControlInput(:,5),'m')
% hold on
% plot(t,MyControlInput(:,6),'k')
% subplot(2,2,4);
% plot(t,MyControlInput(:,7),'r')
% hold on
% plot(t,MyControlInput(:,8),'g')
% hold on
% plot(t,MyControlInput(:,9),'c')
% hold on
% plot(t,MyControlInput(:,10),'b')
% hold on
% plot(t,MyControlInput(:,11),'m')
% hold on
% plot(t,MyControlInput(:,12),'k')

% figure(6)
% subplot(2,2,1);
% plot(t,My_adj(:,1),'r')
% subplot(2,2,2);
% plot(t,My_adj(:,2),'r')
% subplot(2,2,3);
% plot(t,My_adj(:,3),'r')
% subplot(2,2,4);
% plot(t,My_adj(:,4),'r')

% figure(7)
% plot(t,Myerror(:,10),'r')
% hold on
% plot(t,Myerror(:,11),'g')
% hold on
% plot(t,Myerror(:,12),'g')
% hold on
% plot(t,Myerror(:,13),'g')
% hold on
% plot(t,Myerror(:,14),'g')
% hold on
% plot(t,Myerror(:,15),'g')
% hold on
% plot(t,Myerror(:,16),'g')
% hold on
% plot(t,Myerror(:,17),'g')
% hold on
% plot(t,Myerror(:,18),'g')
% figure(7)
% plot(t,My_f(:,1),'r')
% hold on
% plot(t,My_f(:,2),'g')
% figure(7)
% plot(t,target_position(:,1),'r');
% hold on
% plot(t,target_position(:,2),'g');
% hold on
% plot(t,target_position(:,3),'b');