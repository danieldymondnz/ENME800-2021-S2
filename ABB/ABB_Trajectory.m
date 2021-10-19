syms D7

% The tool at the Starting Position can be described as
T_70_PO = [ 0   0   1   1000
            0   -1  0   0050
            1   0   0   1000 
            0   0   0   1       ];

T_70_PA = [ 1   0   0   1125
            0   1   0   0025
            0   0   -1  0308
            0   0   0   1       ];

% Determine T_60
[T_subs, DH] = ABB_Config();
T_76 = cell2sym(T_subs(7));
T_76 = subs(T_76, D7, 165);
T_76_inv = inverseTransMatrix(T_76);

T_60_PO = multiplyTransMatrix(T_70_PO, T_76_inv);
T_60_PO = double(T_60_PO);

T_60_PA = multiplyTransMatrix(T_70_PA, T_76_inv);
T_60_PA = double(T_60_PA);

%% Solve for Position O

[thetas] = ABB_IK_solveTheta(T_60_PO)

%% Solve for Position A
%[T_subs, DH] = ABB_Config();
[thetas] = ABB_IK_solveTheta(T_60_PA)

