function [speed]=Joint_limit_RCNN(theta,ang_vel,ang_low,ang_up,Ang_vel_low,Ang_vel_up)
speed=zeros(6,1);
k=10;
for i=1:6
    a=-k*(theta(i)-ang_low(i));
    b=-k*(theta(i)-ang_up(i));
    if Ang_vel_low(i)>a
        y1(i)=Ang_vel_low(i);
    else
        y1(i)=a;
    end
    
    if Ang_vel_up(i)<b
        y2(i)=Ang_vel_up(i);
    else
        y2(i)=b;
    end   
end

for i=1:6
    if ang_vel(i)<y1(i)
        speed(i)=y1(i);
    elseif ang_vel(i)>y2(i)
        speed(i)=y2(i);
    else
        speed(i)=ang_vel(i);
    end
end
end