%% Symbols
syms T [1 7]; syms A [1 7]; syms D [1 7];
syms S [1 7]; syms C [1 7];
syms R [3 3]; syms P [3 1];
syms C23; syms S23;

%% Generate T_nm Matrices
% Dummy for T_60
T_60_Dummy = [R, P];
T_60_Dummy = [T_60_Dummy; 0 0 0 1];

% Load Config from DH
[T_subs, DH] = ABB_Config();
T_10 = cell2sym(T_subs(1));
T_21 = cell2sym(T_subs(2));
T_32 = cell2sym(T_subs(3));
T_43 = cell2sym(T_subs(4));
T_54 = cell2sym(T_subs(5));
T_65 = cell2sym(T_subs(6));
T_76 = cell2sym(T_subs(7));

%% Generate Simplifications
% T_76
%T_76 = thetaToTrigSyms(T_76);

% T_64
T_64 = dhToTMatrix(DH, 6, 4);
%T_64 = thetaToTrigSyms(T_64);

% T_36
T_43 = dhToTMatrix(DH, 4, 3);
%T_43 = thetaToTrigSyms(T_43);

% T_63
T_63 = multiplyTransMatrix(T_43, T_64);

% T_31
T_31 = dhToTMatrix(DH, 3, 1);
T_31 = simplify(T_31);
T_31 = subs23Syms(T_31);
%T_31 = thetaToTrigSyms(T_31);

% T_30
T_30 = multiplyTransMatrix(T_10, T_31);
%T_30 = thetaToTrigSyms(T_30);

% T_60
T_60 = multiplyTransMatrix(T_30, T_63);

%% Perform Split at Frame 1 to find Theta 1 (T_1)
T_10_inv = inverseTransMatrix(T_10);
LHS = multiplyTransMatrix(T_10_inv, T_60_Dummy)
RHS = multiplyTransMatrix(T_31, T_63)

% Use (2, 4) to find equation for T_1
eqn1_y = thetaToTrigSyms(simplify(LHS(2,4)) == simplify(RHS(2,4)))
t1 = [atan2(P2, P1); atan2(-P2, -P1)]

% Use (1, 4) and (3, 4)
eqn1_x = thetaToTrigSyms(simplify(LHS(1,4)) == simplify(RHS(1,4)))
eqn1_z = thetaToTrigSyms(LHS(3,4) == simplify(RHS(3,4)))


%% Perform Split at Frame 2 to find Theta 3 (T_3)
T_20 = multiplyTransMatrix(T_10, T_21);
T_20_inv = inverseTransMatrix(T_20);
LHS = multiplyTransMatrix(T_20_inv, T_60_Dummy)
RHS = multiplyTransMatrix(T_32, T_63)

eqn1_x = thetaToTrigSyms(simplify(LHS(1,4)) == simplify(RHS(1,4)))
eqn1_y = thetaToTrigSyms(simplify(LHS(2,4)) == simplify(RHS(2,4)))
eqn1_z = thetaToTrigSyms(LHS(3,4) == simplify(RHS(3,4)))

%% Perform Split at Frame 3 to find Theta 2 (T_2)
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