% %��������
% function [Yrerxx,Yreryy,Yataxx,Yatayy]=compute_repulsion(X,Xsum,m,angle_at,angle_re,n,Po)%�������Ϊ��ǰ��꣬Xsum��Ŀ����ϰ���������������泣��,�ϰ���Ŀ�귽��ĽǶ�
%  Rat=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%·�����Ŀ��ľ���ƽ��
%  rat=sqrt(Rat);%·�����Ŀ��ľ���
%  for i=1:n
%     Rrei(i)=(X(1)-Xsum(i+1,1))^2+(X(2)-Xsum(i+1,2))^2;%·������ϰ��ľ���ƽ��
%     rre(i)=sqrt(Rrei(i));%·������ϰ��ľ��뱣��������rrei��
%     if rre(i)>Po%���ÿ���ϰ���·���ľ�������ϰ�Ӱ����룬������Ϊ0
%           Yrerx(i)=0
%           Yrery(i)=0
%           Yatax(i)=0
%           Yatay(i)=0
%     else
%        Yrer(i)=m*(1/rre(i)-1/Po)*1/Rrei(i)*Rat%�ֽ��Fre1����
%        Yata(i)=m*((1/rre(i)-1/Po)^2)*rat%�ֽ��Fre2����
%        Yrerx(i)=Yrer(i)*cos(angle_re(i))%angle_re(i)=Y(i+1)
%        Yrery(i)=Yrer(i)*sin(angle_re(i))
%        Yatax(i)=Yata(i)*cos(angle_at)%angle_at=Y(1)
%        Yatay(i)=Yata(i)*sin(angle_at)
%    end%�жϾ����Ƿ����ϰ�Ӱ�췶Χ��
% end
%    Yrerxx=sum(Yrerx)%���ӳ����ķ���
%    Yreryy=sum(Yrery)
%    Yataxx=sum(Yatax)
%    Yatayy=sum(Yatay)
%%斥力计算
function [Yrerxx,Yreryy,Yataxx,Yatayy]=compute_repulsion(X,Xsum,m,angle_at,angle_re,n,Po,a)%输入参数为当前坐标，Xsum是目标和障碍的坐标向量，增益常数,障碍，目标方向的角度
Rat=(X(1)-Xsum(1,1))^2+(X(2)-Xsum(1,2))^2;%路径点和目标的距离平方
rat=sqrt(Rat);%路径点和目标的距离
for i=1:n
    Rrei(i)=(X(1)-Xsum(i+1,1))^2+(X(2)-Xsum(i+1,2))^2;%路径点和障碍的距离平方
    rre(i)=sqrt(Rrei(i));%路径点和障碍的距离保存在数组rrei中
    R0=(Xsum(1,1)-Xsum(i+1,1))^2+(Xsum(1,2)-Xsum(i+1,2))^2; %障碍和目标间的距离
    r0=sqrt(R0);
    if rre(i)>Po%如果每个障碍和路径的距离大于障碍影响距离，斥力令为0
          Yrerx(i)=0;
          Yrery(i)=0;
          Yatax(i)=0;
          Yatay(i)=0;
    else
       if rre(i)<Po/2    %近距离的
%        if r0<Po/2    %近距离的
       Yrer(i)=m*(1/rre(i)-1/Po)*(1/Rrei(i))*(rat^a);%分解的Fre1向量
       Yata(i)=a*m*((1/rre(i)-1/Po)^2)*(rat^(1-a))/2;%分解的Fre2向量       Yata(i)=0;
       Yrerx(i)=Yrer(i)*cos(angle_re(i)+pi);%angle_re(i)=Y(i+1)
       Yrery(i)=Yrer(i)*sin(angle_re(i)+pi);
       Yatax(i)=Yata(i)*cos(angle_at);%angle_at=Y(1)
       Yatay(i)=Yata(i)*sin(angle_at);
        else
           Yrer(i)=m*(1/rre(i)-1/Po)*1/Rrei(i)*Rat;%分解的Fre1向量
           Yata(i)=m*((1/rre(i)-1/Po)^2)*rat;%分解的Fre2向量       Yata(i)=0;
           Yrerx(i)=Yrer(i)*cos(angle_re(i)+pi);%angle_re(i)=Y(i+1)
           Yrery(i)=Yrer(i)*sin(angle_re(i)+pi);%斥力+180度，方向相反
           Yatax(i)=Yata(i)*cos(angle_at);%angle_at=Y(1)
           Yatay(i)=Yata(i)*sin(angle_at);
        end
   end%判断距离是否在障碍影响范围内
end
   Yrerxx=sum(Yrerx);%叠加斥力的分量
   Yreryy=sum(Yrery);
   Yataxx=sum(Yatax);
   Yatayy=sum(Yatay);