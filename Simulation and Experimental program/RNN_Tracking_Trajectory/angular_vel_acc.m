function [ang_vel_seq,ang_acc_seq]=angular_vel_acc(angle,vel_ang_lim,acc_ang_lim,tol_time,period)
ka=90;
seq_num=tol_time/period+1;
ang_vel_seq=zeros(1,seq_num);
ang_acc_seq=zeros(1,seq_num);
t1=sqrt(vel_ang_lim/ka);
t2=2*sqrt(vel_ang_lim/ka);
t3=(angle-2*vel_ang_lim*t1)/vel_ang_lim+t2;
t4=t3+t1;
t5=t3+t2;

for i=1:seq_num
    t=(i-1)*period;
    if t<=t1
        velocity=0.5*ka*t^2;
    elseif t>t1&&t<t2 
        velocity=-0.5*ka*t^2+2*ka*t1*t-ka*t1^2;
    elseif t>=t2&&t<=t3
        velocity=ka*t1^2;
    elseif t>t3&&t<=t4
        velocity=-0.5*ka*t^2+ka*t3*t+ka*t1^2-0.5*ka*t3^2;
    elseif t>t4&&t<=t5
        velocity=0.5*ka*t^2-ka*t5*t+0.5*ka*t5^2;
    elseif t>t5
        velocity=0;  
    end
    ang_vel_seq(i)=velocity;
end

for i=1:seq_num
    t=(i-1)*period;
    if t<=t1
        acceleration=ka*t;
    elseif t>t1&&t<t2 
        acceleration=-ka*t+2*ka*t1;
    elseif t>=t2&&t<=t3
        acceleration=0;
    elseif t>t3&&t<=t4
        acceleration=-ka*t+ka*t3;
    elseif t>t4&&t<=t5
        acceleration=ka*t-ka*t5;
    elseif t>t5
        acceleration=0;  
    end
    ang_acc_seq(i)=acceleration;
end
end