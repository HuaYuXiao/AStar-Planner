function path = APF2D(start,goal,obstacle)
% Artificial Potential Field（APF）avoiding obstacle path
%%APF参数初始化
%如果不能实现预期目标，可能也与初始的增益系数，Po设置的不合适有关。
att = 35;%引力增益系数
req = 10;%斥力增益系数
p0 = 5;%障碍物产生影响的最大距离，当障碍与移动目标之间距离大于Po时，斥力为0。
step = 2;%步长
maxIter = 200;%最大循环迭代次数
n = length(obstacle(:,1));%障碍物个数
 
path = start;%路径初始化
newNode = start;
for i = 1:maxIter
    %% 引力计算
    V_att = goal - newNode;%路径点到目标点的向量
    r_att = sqrt(V_att(1)^2 + V_att(2)^2);%路径点到目标点的欧氏距离
    P_att = att * V_att;%引力
    
    %% 斥力计算
    %改进的人工势场法，将斥力分散一部分到引力方向。通过添加随机扰动r_att^n实现，r_att为路径点到目标点的欧氏距离，本文n取2。
    V_req = zeros(n,2);
    for j =1:n
        V_req(j,:) = [obstacle(j,1) - newNode(1), obstacle(j,2) - newNode(2)];%路径点到各个障碍物的向量
        r_req(j) = sqrt(V_req(j,1)* V_req(j,1) + V_req(j,2)* V_req(j,2));%路径点到各个障碍物的欧氏距离
    end   
    P_req = 0;
    for k = 1:n
        if r_req(k) <= p0
            P_req1 = req * (1 / r_req(k) - 1 / p0) * r_att^2 / r_req(k)^2;%斥力分量1：障碍物指向路径点的斥力
            P_req2 = req * (1 / r_req(k) - 1 / p0)^2 * r_att;%斥力分量2：路径点指向目标点的分引力
            P_reqk = P_req1 / r_req(k) * V_req(k,:) + P_req2 / r_att * V_att;%合力分散到x,y方向
            P_req = P_req + P_reqk;%斥力
        end     
    end
    %% 合力计算
    P = P_att + P_req;
    newNode = newNode + step * P / norm(P);
    path = [path; newNode];   
end
 
end
 