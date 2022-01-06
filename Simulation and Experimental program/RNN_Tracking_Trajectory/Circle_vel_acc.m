function [arc_vel,arc_acc]=Circle_vel_acc(radius,a,b,angle_seq,ang_vel_seq,ang_acc_seq)
angle_seq=angle_seq*pi/180;
ang_vel_seq=ang_vel_seq*pi/180;
ang_acc_seq=ang_acc_seq*pi/180;
seq_length=size(angle_seq);
arc_vel=zeros(seq_length(2),3);
arc_acc=zeros(seq_length(2),3);

for i=1:seq_length(2)
    arc_vel(i,1)=-radius*a(1)*sin(angle_seq(i))*ang_vel_seq(i)+radius*b(1)*cos(angle_seq(i))*ang_vel_seq(i);
    arc_vel(i,2)=-radius*a(2)*sin(angle_seq(i))*ang_vel_seq(i)+radius*b(2)*cos(angle_seq(i))*ang_vel_seq(i);
    arc_vel(i,3)=-radius*a(3)*sin(angle_seq(i))*ang_vel_seq(i)+radius*b(3)*cos(angle_seq(i))*ang_vel_seq(i);
end

for i=1:seq_length(2)
    arc_acc(i,1)=(-radius*a(1)*cos(angle_seq(i))-radius*b(1)*sin(angle_seq(i)))*ang_vel_seq(i)^2+(-radius*a(1)*sin(angle_seq(i))+radius*b(1)*cos(angle_seq(i)))*ang_acc_seq(i);
    arc_acc(i,2)=(-radius*a(2)*cos(angle_seq(i))-radius*b(2)*sin(angle_seq(i)))*ang_vel_seq(i)^2+(-radius*a(2)*sin(angle_seq(i))+radius*b(2)*cos(angle_seq(i)))*ang_acc_seq(i);
    arc_acc(i,3)=(-radius*a(3)*cos(angle_seq(i))-radius*b(3)*sin(angle_seq(i)))*ang_vel_seq(i)^2+(-radius*a(3)*sin(angle_seq(i))+radius*b(3)*cos(angle_seq(i)))*ang_acc_seq(i);
end
end