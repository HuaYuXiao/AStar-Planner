% function Y=compute_angle(X,Xsum,n)%Y��������������x��ĽǶ�����,X�������꣬Xsum��Ŀ����ϰ����������,��(n+1)*2����
%   for i=1:n+1%n���ϰ���Ŀ
%       deltaXi=Xsum(i,1)-X(1)
%       deltaYi=Xsum(i,2)-X(2)
%       ri=sqrt(deltaXi^2+deltaYi^2)
%       if deltaYi>0
%           theta=acos(deltaXi/ri)
%       else
%           theta=-acos(deltaXi/ri)
%       end
% %       if i==1%��ʾ��Ŀ��
% %           angle=theta
% %       else
% %           angle=pi+theta
% %       end     
%       Y(i)=theta%����ÿ���Ƕ���Y�������棬��һ��Ԫ������Ŀ��ĽǶȣ����涼�����ϰ��ĽǶ�
%   end

function Y=compute_angle(X,Xsum,n)%Y是引力，斥力与x轴的角度向量,X是起点坐标，Xsum是目标和障碍的坐标向量,是(n+1)*2矩阵
  for i=1:n+1%n是障碍数目
      deltaX(i)=Xsum(i,1)-X(1);
      deltaY(i)=Xsum(i,2)-X(2);
      r(i)=sqrt(deltaX(i)^2+deltaY(i)^2);
      theta=sign(deltaY(i))*acos(deltaX(i)/r(i));
      Y(i)=theta;   %angle;%保存每个角度在Y向量里面，第一个元素是与目标的角度，后面都是与障碍的角度
  end