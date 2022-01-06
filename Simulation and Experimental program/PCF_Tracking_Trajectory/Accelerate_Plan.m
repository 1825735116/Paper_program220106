function [ang_sequence] = Accelerate_Plan(angle,vel_ang_lim,acc_ang_lim,tol_time,period)

ka=90;
seq_num=tol_time/period+1;
ang_sequence=zeros(1,seq_num);
t1=sqrt(vel_ang_lim/ka);
t2=2*sqrt(vel_ang_lim/ka);
t3=(angle-2*vel_ang_lim*t1)/vel_ang_lim+t2;
t4=t3+t1;
t5=t3+t2;

for i=1:seq_num
    t=(i-1)*period;
    if t<=t1
        displace=1/6*ka*t^3;
    elseif t>t1&&t<t2
        displace=-1/6*ka*t^3+ka*t1*t^2-ka*t1^2*t+1/3*ka*t1^3;
    elseif t>=t2&&t<=t3
        displace=ka*t1^2*t-ka*t1^3;
    elseif t>t3&&t<=t4
        displace=-1/6*ka*t^3+0.5*ka*t3*t^2+(ka*t1^2-0.5*ka*t3^2)*t+1/6*ka*t3^3-ka*t1^3;
    elseif t>t4&&t<=t5
        displace=1/6*ka*t^3-0.5*ka*t5*t^2+0.5*ka*t5^2*t+angle-1/6*ka*t5^3;
    elseif t>t5
        displace=angle;
    end
    ang_sequence(i)=displace;
end

end

