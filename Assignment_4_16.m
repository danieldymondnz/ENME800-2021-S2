
syms T [1 7]
theta = [0 0 0 0 0 0 0];

DH = [  0       0       0       T1
        0       1       0       T2
        pi/4    0       sqrt(2) T3
        0       sqrt(2) 0       T4 ];

% Dh = subs(DH, T, theta);

% Generate the T_subs Cells
T_10 = dhToTMatrix(DH, 1, 0)
T_21 = dhToTMatrix(DH, 2, 1)
T_32 = dhToTMatrix(DH, 3, 2)
T_43 = dhToTMatrix(DH, 4, 3)
T_30 = simplify(dhToTMatrix(DH, 3, 0))
T_40 = dhToTMatrix(DH, 4, 0)
T_subs = {T_10, T_21, T_32, T_43};

outputPlot = plot(1);
view(3);
hold on;

F_0 = eye(4);
outputPlot = drawAxis(F_0);

T_10 = subsSymToT(DH, 1, 0, theta, a, d);
outputPlot = plot3([F_0(1,4) T_10(1,4)], [F_0(2,4) T_10(2,4)], [F_0(3,4) T_10(3,4)], 'black');
outputPlot = drawAxis(T_10);

T_20 = subsSymToT(DH, 2, 0, theta, a, d);
outputPlot = drawAxis(T_20);
outputPlot = plot3([T_10(1,4) T_20(1,4)], [T_10(2,4) T_20(2,4)], [T_10(3,4) T_20(3,4)], 'black');

T_30 = subsSymToT(DH, 3, 0, theta, a, d);
outputPlot = drawAxis(T_30);
outputPlot = plot3([T_20(1,4) T_30(1,4)], [T_20(2,4) T_30(2,4)], [T_20(3,4) T_30(3,4)], 'black');

T_40 = subsSymToT(DH, 4, 0, theta, a, d);
outputPlot = drawAxis(T_40);
outputPlot = plot3([T_30(1,4) T_40(1,4)], [T_30(2,4) T_40(2,4)], [T_30(3,4) T_40(3,4)], 'black');
