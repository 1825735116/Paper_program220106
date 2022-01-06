function [arc,a,b] = Generate_ellipsecircle(center,radius,nor_vector,init_angle,angle_seq,ratio)

init_angle=init_angle*pi/180;
angle_seq=angle_seq*pi/180;
seq_length=size(angle_seq);
arc=zeros(seq_length(2),3);
plane_norm=[cos(init_angle+pi/2),sin(init_angle+pi/2),0];
pre_a=cross(plane_norm,nor_vector);
pre_b=cross(nor_vector,pre_a);
a=pre_a/norm(pre_a);
b=pre_b/norm(pre_b);
b=b/ratio;
for i=1:seq_length(2)
%     point(1)=center(1)+radius*cos(angle_seq(i))*a(1)+radius*sin(angle_seq(i))*b(1);
%     point(2)=center(2)+radius*cos(angle_seq(i))*a(2)+radius*sin(angle_seq(i))*b(2);
%     point(3)=center(3)+radius*cos(angle_seq(i))*a(3)+radius*sin(angle_seq(i))*b(3);
    arc(i,1)=center(1)+radius*cos(angle_seq(i))*a(1)+radius*sin(angle_seq(i))*b(1);
    arc(i,2)=center(2)+radius*cos(angle_seq(i))*a(2)+radius*sin(angle_seq(i))*b(2);
    arc(i,3)=center(3)+radius*cos(angle_seq(i))*a(3)+radius*sin(angle_seq(i))*b(3);
%     acr(i,:)=point;
end

end

