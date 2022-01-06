function [y1]=Joint_limit_Null(theta,ang_lowlimit,ang_uplimit)

d_x=20*pi/180;
w=1;
rho=1/12;kr=0.2;j=2;
delta_q=ang_uplimit-ang_lowlimit;
ang_upact=ang_uplimit-rho*delta_q;
ang_lowact=ang_lowlimit+rho*delta_q;
alpha=pi./(2*rho*delta_q);

num=size(theta,1);
adj=zeros(num,1);
for i=1:num
    if theta(i)>ang_upact(i)
        adj(i)=-j*alpha(i)*(tan(alpha(i)*(theta(i)-ang_upact(i))))^(j-1)/(cos(alpha(i)*(theta(i)-ang_upact(i))))^2;
    elseif theta(i)<ang_lowact(i)
        adj(i)=-j*alpha(i)*(tan(alpha(i)*(theta(i)-ang_lowact(i))))^(j-1)/(cos(alpha(i)*(theta(i)-ang_lowact(i))))^2;
    else
        adj(i)=0;
    end
end
y1=kr*adj;
% for i=1:6
%     m=ang_lowlimit(i);M=ang_uplimit(i);
%     c=ang_lowlimit(i)+d_x;C=ang_uplimit(i)-d_x;
%     if theta(i)>c && theta(i)<C
%         y1(i)=0;
%     elseif theta(i)>C && theta(i)<M
%         y1(i)=w/4*(1+cos(pi*(M-theta(i))/C));
%     elseif theta(i)>m && theta(i)<c
%         y1(i)=-w/4*(1+cos(pi*(theta(i)-m)/c));
%     else
%         y1(i)=2*w;
%     end
% end

% for i=1:6
%     m=ang_lowlimit(i);M=ang_uplimit(i);
%     c=ang_lowlimit(i)+d_x;C=ang_uplimit(i)-d_x;
%     if theta(i)>c && theta(i)<C
%         y1(i)=0;
%     elseif theta(i)>C && theta(i)<M
%         y1(i)=w/4*pi*sin(pi*(M-theta(i))/C);
%     elseif theta(i)>m && theta(i)<c
%         y1(i)=-w/4*pi*sin(pi*(theta(i)-m)/c);
%     else
%         y1(i)=0;
%     end
% end
end