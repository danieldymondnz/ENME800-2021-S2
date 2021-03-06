%% Forward Kinematics for ABB
% Load Config from DH
[T_subs, ~, ~, DH] = ABB_Config();
T_10 = cell2sym(T_subs(1)); T_21 = cell2sym(T_subs(2));
T_32 = cell2sym(T_subs(3)); T_43 = cell2sym(T_subs(4));
T_54 = cell2sym(T_subs(5)); T_65 = cell2sym(T_subs(6));
T_76 = cell2sym(T_subs(7));

% Generate Simplifications, then assemble
% T_63
T_63 = dhToTMatrix(DH, 6, 3);
T_63 = thetaToTrigSyms(T_63)

% T_31
T_31 = dhToTMatrix(DH, 3, 1);
T_31 = simplify(T_31); T_31 = subs23Syms(T_31);

% T_30
T_30 = multiplyTransMatrix(T_10, T_31);

% T_60
T_60 = multiplyTransMatrix(T_30, T_63);
T_60 = thetaToTrigSyms(T_60)

% T_70
T_70 = multiplyTransMatrix(T_60, T_76);
T_70 = thetaToTrigSyms(T_70)