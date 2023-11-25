function drawCube(position)
    % 绘制方块障碍物
    x = [position(1), position(1), position(1) + position(4), position(1) + position(4), position(1)];
    y = [position(2), position(2) + position(5), position(2) + position(5), position(2), position(2)];
    z = [position(3), position(3), position(3), position(3), position(3)];
    fill3(x, y, z, 'b');
end
