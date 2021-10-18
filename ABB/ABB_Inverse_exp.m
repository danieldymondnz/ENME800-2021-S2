
% Create Syms for Theta
syms T [1 7]; syms A [1 7]; syms D [1 7];
syms S [1 7]; syms C [1 7];
syms R [3 3]; syms P [3 1];
syms C23; syms S23;


T_60_Dummy = [R, P];
T_60_Dummy = [T_60_Dummy; 0 0 0 1];

% Load Config
[T_subs, DH] = ABB_Config();

T_10 = cell2sym(T_subs(1));
T_21 = cell2sym(T_subs(2));
T_32 = cell2sym(T_subs(3));
T_43 = cell2sym(T_subs(4));
T_54 = cell2sym(T_subs(5));
T_65 = cell2sym(T_subs(6));
T_76 = cell2sym(T_subs(7));

%% Generate Full Transformation Matrix w/ Simplifications

% T_76
T_76 = thetaToTrigSyms(T_76);

% T_64
T_64 = dhToTMatrix(DH, 6, 4);
T_64 = thetaToTrigSyms(T_64);

% T_36
T_43 = dhToTMatrix(DH, 4, 3);
T_43 = thetaToTrigSyms(T_43);

% T_63
T_63 = multiplyTransMatrix(T_43, T_64)

% T_31
T_31 = dhToTMatrix(DH, 3, 1);
T_31 = simplify(T_31);
T_31 = subs23Syms(T_31);
T_31 = thetaToTrigSyms(T_31)

% T_30
T_30 = multiplyTransMatrix(T_10, T_31);
T_30 = thetaToTrigSyms(T_30)

% T_60
T_60 = multiplyTransMatrix(T_30, T_63)

%% Split at 1
T_10_inv = inverseTransMatrix(T_10);
LHS = multiplyTransMatrix(T_10_inv, T_60_Dummy)
RHS = multiplyTransMatrix(T_31, T_63);

% RHS = T_63
% [LH_Inv, RHS] = generateInverse(DH,1);
% LHS = multiplyTransMatrix(LH_Inv, T_70_Dummy);
% RHS = RHS;

% Use (2, 4) to find equation for T1
eqn_t1 = simplify(LHS(2,4)) == simplify(RHS(2,4))
t1 = [atan2(P2, P1); atan2(-P2, -P1)]

% Use (1, 4) and (3, 4)
eqn_x = simplify(LHS(1,4)) == simplify(RHS(1,4))
eqn_z = LHS(3,4) == simplify(RHS(3,4))

% Square and Add
eqnadd = simplify(expand(LHS(1,4)^2) + expand(LHS(3,4)^2) == expand(RHS(1,4)^2) + expand(RHS(3,4)^2))

eqnx_2 = expand( LHS(1,4)^2 ) == expand( RHS(1,4)^2 )
eqnz_2 = expand( LHS(3,4)^2 ) == expand( RHS(3,4)^2 )

% sol_l = simplify(expand(LHS(1,4)^2 + LHS(3,4)^2))
% sol_r = simplify(expand(RHS(1,4)^2 + RHS(3,4)^2))
% sol_alg = simplify(sol_l == sol_r)
% 
% LHS = thetaToTrigSyms(LHS)
% RHS = thetaToTrigSyms(RHS)


%% Split at 2
T_20 = multiplyTransMatrix(T_10, T_21);
T_20_inv = inverseTransMatrix(T_20);
LHS = multiplyTransMatrix(T_20_inv, T_60_Dummy)
RHS = multiplyTransMatrix(T_32, T_63)

% Use (1, 4) and (3, 4)
eqn_x = thetaToTrigSyms(simplify(LHS(1,4)) == simplify(RHS(1,4)))
eqn_z = thetaToTrigSyms(LHS(3,4) == simplify(RHS(3,4)))



%% Split at 2
% T_1 = atan2(P2, P1); % alt atan2(-P2, -P1)
% [LH_Inv, RHS] = generateInverse(DH,2);
% LH_Inv = simplify(LH_Inv); %subs(, T1, T_1);
% RHS = simplify(RHS); %subs(, T1, T_1);
% LHS = thetaToTrigSyms(multiplyTransMatrix(LH_Inv, T_70_Dummy))
% RHS = thetaToTrigSyms(RHS)

%% Split at 3
T_30_inv = inverseTransMatrix(T_30)
LHS = multiplyTransMatrix(T_30_inv, T_60_Dummy)
RHS = T_63

eq_x = collect(LHS(1,4), [C23 S23]) == RHS(1,4)
eq_y = collect(LHS(2,4), [C23 S23]) == RHS(2,4)
eq_z = collect(LHS(3,4), [C23 S23]) == RHS(3,4)

% Two Factorised Equations - Solve for T_23
% -(A1 + A2*C2 - P1*C1 - P2*S1)*C23 - (D1 + A2*S2 - P3)*S23 = A3
% -(A1 + A2*C2 - P1*C1 - P2*S1)*S23 + (D1 + A2*S2 - P3)*C23 = D4

% Solution for T_23 is found as:
% admbc = -D4 * (A1 + A2*C2 - P1*C1 - P2*S1) - A3 * (D1 + A2*S2 - P3)
% acpbd = -A3 * (A1 + A2*C2 - P1*C1 - P2*S1) + D4 * (D1 + A2*S2 - P3)
% T_23 = atan2(admbc, acpbd)
% T_2 = T_23 - T_3
