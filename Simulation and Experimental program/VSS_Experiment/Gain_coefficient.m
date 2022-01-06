function [adj1,adj2,adj3,adj4]=Gain_coefficient(theta,ang_lowlimit,ang_uplimit,ang_vel,e)

theta1=ang_lowlimit;
theta2=ang_uplimit;
d_theta=20*pi/180;
dim1=size(theta,1);
% dim2=size(e,1);
adj4=ones(dim1,1);
for i=1:dim1
    if theta(i)<theta1(i)+d_theta
        coe(i)=exp(-100*(theta(i)-theta1(i)-d_theta)^2/1);
    elseif theta(i)>=theta1(i)+d_theta&&theta(i)<=theta2(i)-d_theta
        coe(i)=1;
    else
        coe(i)=exp(-100*(theta(i)-theta2(i)+d_theta)^2/1);
    end
end
adj1=min(abs(coe));

max_e=max(abs(e));
if max_e<=0.005
    adj2=1;
else
    adj2=0.005/max_e;
end

% for i=1:dim1
%     limit=abs(ang_vel(i));
%     if limit<=1
%         adj3(i)=1;
%     else
%         adj3(i)=1/limit;
%     end
% end
limit=max(abs(ang_vel));
if limit<=0.5
    adj3=1;
else
    adj3=0.5/limit;
end

for i=1:dim1
    limit=abs(ang_vel(i));
    if limit<=0.5
        adj4(i)=0.5;
    else
        adj4(i)=0.5/limit;
    end
end
end