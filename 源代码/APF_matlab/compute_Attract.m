% %��������
% function [Yatx,Yaty]=compute_Attract(X,Xsum,k,angle,b,Po,n)%�������Ϊ��ǰ��꣬Ŀ����꣬���泣��,���������ĽǶ�
% %��·���ϵ���ʱ����Ϊÿ��ʱ�̵�Xgoal
% R=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%·�����Ŀ��ľ���ƽ��
% r=sqrt(R);%·�����Ŀ��ľ���
% %deltax=Xgoal(1)-X(1);
% %deltay=Xgoal(2)-X(2);
% Yatx=k*r*cos(angle);%angle=Y(1)
% Yaty=k*r*sin(angle);
% end
function [Yatx,Yaty]=compute_Attract(X,Xsum,k,angle)%输入参数为当前坐标，目标坐标，增益常数,分量和力的角度
%把路径上的临时点作为每个时刻的Xgoal
R=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%路径点和目标的距离平方
r=sqrt(R);%路径点和目标的距离
Yatx=k*r*cos(angle);%angle=Y(1).引力随距离减少而减弱
Yaty=k*r*sin(angle);
