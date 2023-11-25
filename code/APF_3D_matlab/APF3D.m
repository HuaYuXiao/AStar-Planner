% Artificial Potential Field（APF）avoiding obstacle path
function path = APF3D(start,goal,obstacle)
    %%APF参数初始化
    %引力增益系数
    att = 30;
    %斥力增益系数
    req = -15;
    %障碍物产生影响的最大距离，当障碍与移动目标之间距离大于Po时，斥力为0。
    p0 = 5.5;
    %步长
    step = 0.2;
    %最大循环迭代次数
    maxIter = 200;
    %障碍物个数
    n = length(obstacle(:,1));
    
    %路径初始化
    path = start;
    newNode = start;
    for i = 1:maxIter
        %% 引力计算
        %路径点到目标点的向量
        V_att = goal - newNode;
        %路径点到目标点的欧氏距离
        r_att = sqrt(V_att(1)^2 + V_att(2)^2 + V_att(3)^2);
        %引力
        P_att = att * V_att;
        
        %% 斥力计算
        %改进的人工势场法，将斥力分散一部分到引力方向。通过添加随机扰动r_att^n实现，r_att为路径点到目标点的欧氏距离，本文n取2。
        V_req = zeros(n,3);
        for j =1:n
            %路径点到各个障碍物的向量
            V_req(j,:) = [obstacle(j,1) - newNode(1), obstacle(j,2) - newNode(2), obstacle(j,3) - newNode(3)];
            %路径点到各个障碍物的欧氏距离
            r_req(j) = sqrt(V_req(j,1)* V_req(j,1) + V_req(j,2)* V_req(j,2) + V_req(j,3)* V_req(j,3));
        end   
        P_req = 0;
        for k = 1:n
            if r_req(k) <= p0
                %斥力分量1：障碍物指向路径点的斥力
                P_req1 = req * (1 / r_req(k) - 1 / p0) * r_att^2 / r_req(k)^2;
                %斥力分量2：路径点指向目标点的分引力
                P_req2 = req * (1 / r_req(k) - 1 / p0)^2 * r_att;
                %合力分散到x,y方向
                P_reqk = P_req1 / r_req(k) * V_req(k,:) + P_req2 / r_att * V_att;
                %斥力
                P_req = P_req + P_reqk;
            end     
        end
        %% 合力计算
        P = P_att + P_req;
        newNode = newNode + step * P / norm(P);
        path = [path; newNode];   
    end
end
 