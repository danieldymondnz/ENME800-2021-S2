
% Create Syms for Theta
syms T [1 7];
syms A [1 7];
syms D [1 7];
syms S [1 7];
syms C [1 7];
syms R [3 3];
syms P [3 1];

T_70_Dummy = [R, P];
T_70_Dummy = [T_70_Dummy; 0 0 0 1];

% Load Config
[T_subs, DH] = ABB_Config();

T_10 = cell2sym(T_subs(1));
T_21 = cell2sym(T_subs(2));
T_32 = cell2sym(T_subs(3));
T_43 = cell2sym(T_subs(4));
T_54 = cell2sym(T_subs(5));
T_65 = cell2sym(T_subs(6));
T_76 = cell2sym(T_subs(7));

T_70 = dhToTMatrix(DH, 7, 0);
T_70 = simplify(T_70);
T_70 = thetaToTrigSyms(T_70)

% Perform Inverse by re-arranging
[LH_Inv, RHS] = generateInverse(DH, 2);
LHS = thetaToTrigSyms(simplify(multiplyTransMatrix(LH_Inv, T_70_Dummy)))
RHS = thetaToTrigSyms(simplify(RHS))


