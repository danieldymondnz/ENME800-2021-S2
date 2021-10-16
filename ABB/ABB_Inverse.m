
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

T_73 = dhToTMatrix(DH, 7, 3)

%% Split at 1
[LH_Inv, RHS] = generateInverse(DH,1);
LHS = multiplyTransMatrix(LH_Inv, T_70_Dummy);
RHS = RHS;

% Use (2, 4) to find equation for T1
eqn_t1 = simplify(LHS(2,4)) == simplify(RHS(2,4))
t1 = [atan2(P2, P1); atan2(-P2, -P1)]

% % Use (1, 4) and (3, 4)
% eqn_x = simplify(LHS(1,4)) == simplify(RHS(1,4))
% eqn_z = LHS(3,4) == simplify(RHS(3,4))
% sol_l = simplify(expand(LHS(1,4)^2 + LHS(3,4)^2))
% sol_r = simplify(expand(RHS(1,4)^2 + RHS(3,4)^2))
% sol_alg = simplify(sol_l == sol_r)
% 
% LHS = thetaToTrigSyms(LHS)
% RHS = thetaToTrigSyms(RHS)



%% Split at 2
% T_1 = atan2(P2, P1); % alt atan2(-P2, -P1)
[LH_Inv, RHS] = generateInverse(DH,2);
LH_Inv = simplify(LH_Inv); %subs(, T1, T_1);
RHS = simplify(RHS); %subs(, T1, T_1);
LHS = thetaToTrigSyms(multiplyTransMatrix(LH_Inv, T_70_Dummy))
RHS = thetaToTrigSyms(RHS)

%% Split at 3
[LH_Inv, RHS] = generateInverse(DH,3);
LHS = thetaToTrigSyms(simplify(multiplyTransMatrix(LH_Inv, T_70_Dummy)))
RHS = thetaToTrigSyms(simplify(RHS))

% eqn1 = LHS(1,4) == RHS(1,4)
% eqn2 = LHS(2,4) == RHS(2,4)
% 
% % [A, B] = equationsToMatrix([eqn1 eqn2], [sin(T2 + T3) cos(T2 + T3)])
% [solt2, solt3] = solve(eqn1, eqn2)
% solt3 = simplify(solt3)
