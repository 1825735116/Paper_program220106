function [y1,y2]=Joint_limit(theta,ang_lowlimit,ang_uplimit,ang_vel,vel_lowlimit,vel_uplimit)
x=theta;
x1=ang_lowlimit;
x2=ang_uplimit;
d_x=20*pi/180;
range=2*pi./(ang_uplimit-ang_lowlimit);
% d_ang=(ang_uplimit-ang_lowlimit)/2;
% 
y1=zeros(1,size(theta,1));
v=ang_vel;
v1=vel_lowlimit;
v2=vel_uplimit;
d_v=25*pi/180;
% d_angvel=(vel_uplimit-vel_lowlimit)/2;
range_vel=2*pi./(vel_uplimit-vel_lowlimit);
y2=zeros(1,size(ang_vel,1));
dim=size(ang_vel,1);
% 
for i=1:6
%     d_x=d_ang(i);
    if x(i)<x1(i)+d_x
        y1(i)=(1-exp(-(x(i)-x1(i)-d_x)^2/1))*100*range(i);
    elseif x(i)>=x1(i)+d_x && x(i)<=x2(i)-d_x
        y1(i)=0;
    elseif x(i)>x2(i)-d_x
        y1(i)=(1-exp(-(x(i)-x2(i)+d_x)^2/1))*100*range(i);
    end
end
for i=1:6
%     d_v=d_angvel(i);
    if v(i)<v1(i)+d_v
        y2(i)=(1-exp(-(v(i)-v1(i)-d_v)^2/10))*200*range_vel(i);
    elseif v(i)>=v1(i)+d_v && v(i)<=v2(i)-d_v
        y2(i)=0;
    elseif v(i)>v2(i)-d_v
        y2(i)=(1-exp(-(v(i)-v2(i)+d_v)^2/10))*200*range_vel(i);
    end
end
end