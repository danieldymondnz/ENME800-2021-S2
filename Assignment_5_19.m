
syms T [1 7]
syms A [1 7]
syms D [1 7]
theta = [0 0 0 0 0 0 0];
a = [0 0 0 0 0 0 0];
d = [0 0 0 0 0 0 0];

DH = [   0      0       0       T1
        -pi/2       A1       D2     0 ];

DH = subs(DH, T, theta);
DH = subs(DH, A, a);
DH = subs(DH, D, d);

% Generate the T_subs Cells
T_10 = dhToTMatrix(DH, 1, 0)
T_21 = dhToTMatrix(DH, 2, 1)
T_20 = dhToTMatrix(DH, 2, 0)
T_subs = {T_10, T_21};

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