function axis = drawAxis(T_n0)

    R = T_n0(1:3,1:3);
    P = T_n0(1:3,4);
    S = 50;
    
    axis(1) = plot3([0 S*R(1,1)]+P(1), [0 S*R(2,1)]+P(2), [0 S*R(3,1)]+P(3), 'red', 'LineWidth', 2);
    axis(2) = plot3([0 S*R(1,2)]+P(1), [0 S*R(2,2)]+P(2), [0 S*R(3,2)]+P(3), 'green', 'LineWidth', 2);
    axis(3) = plot3([0 S*R(1,3)]+P(1), [0 S*R(2,3)]+P(2), [0 S*R(3,3)]+P(3), 'blue', 'LineWidth', 2);
end