%% Inverse Kinematics Script for ABB-2600-12/1.85

%% Symbols
syms T [1 6]; syms A [1 7]; syms D [1 7];
syms S [1 7]; syms C [1 7]; syms C23; syms S23;
syms R [3 3]; syms P [3 1];

%% Load Config from DH
[T_subs, ~, ~, DH] = ABB_Config();
T_10 = cell2sym(T_subs(1)); T_21 = cell2sym(T_subs(2));
T_32 = cell2sym(T_subs(3)); T_43 = cell2sym(T_subs(4));
T_54 = cell2sym(T_subs(5)); T_65 = cell2sym(T_subs(6));
T_76 = cell2sym(T_subs(6));

%% Get Euler
% Generate syms for cos, sin for alpha, beta, gamma
syms c_a; syms c_b; syms c_g; syms s_a; syms s_b; syms s_g;

T = sym([(c_a*c_b)    (c_a*s_b*s_g - s_a*c_g) (c_a*s_b*c_g + s_a*s_g)     P1
         (s_a*s_b)    (s_a*s_b*s_g + c_a*c_g) (s_a*s_b*c_g - c_a*s_g)     P2
         -s_b         (c_b*s_g)               (c_b*c_g)                   P3
         0            0                       0                           1  ]);

% Generate the Transformation Matrix for T_60 for {W} calculation
T_76_inv = inverseTransMatrix(T_76);
T_final = multiplyTransMatrix(T, T_76)

% Create dummy for T_60, which represents T_final for simplification
T_60_Dummy = [R, P];
T_60_Dummy = [T_60_Dummy; 0 0 0 1];

%% Generate Simplifications
% Generate T_63
T_64 = dhToTMatrix(DH, 6, 4);
T_63 = multiplyTransMatrix(T_43, T_64);

% T_31
T_31 = dhToTMatrix(DH, 3, 1);
T_31 = simplify(T_31); T_31 = subs23Syms(T_31);

% T_30
T_30 = multiplyTransMatrix(T_10, T_31);

% T_60
T_60 = multiplyTransMatrix(T_30, T_63);

%% Perform Split at Frame 1 to find Theta 1 (T_1)
T_10_inv = inverseTransMatrix(T_10);
LHS_1 = multiplyTransMatrix(T_10_inv, T_60_Dummy)
RHS_1 = multiplyTransMatrix(T_31, T_63)

% Use (2, 4) to find equation for T_1
eqn1_y = thetaToTrigSyms(simplify(LHS_1(2,4)) == simplify(RHS_1(2,4)))
T_1 = [atan2(P2, P1); atan2(-P2, -P1)]

%% Perform Split at Frame 2 to find Theta 3 (T_3)
T_20 = multiplyTransMatrix(T_10, T_21);
T_20_inv = inverseTransMatrix(T_20);
LHS = multiplyTransMatrix(T_20_inv, T_60_Dummy)
RHS = multiplyTransMatrix(T_32, T_63)

% Use (1:3,4) to find equation for T_3
eqn2_x = thetaToTrigSyms(simplify(LHS(1,4)) == simplify(RHS(1,4)))
eqn2_y = thetaToTrigSyms(simplify(LHS(2,4)) == simplify(RHS(2,4)))
eqn2_z = thetaToTrigSyms(LHS(3,4) == simplify(RHS(3,4)))
[exp, K] = ABB_IK_T3(eqn2_x, eqn2_y, eqn2_z)
T_3 = atan2(A3, -D4) - [atan2(K, sqrt(A3^2 + D4^2 - K^2)), atan2(K, -sqrt(A3^2 + D4^2 - K^2))]

%% Perform Split at Frame 3 to find Theta 2 (T_2) and Theta 4 (T_4)
T_30_inv = inverseTransMatrix(T_30);
LHS = multiplyTransMatrix(T_30_inv, T_60_Dummy)
RHS = T_63

% Use (1:2, 4) to find equation for T_2
eqn3_x = collect(thetaToTrigSyms(LHS(1,4)), [C23 S23]) == RHS(1,4)
eqn3_y = collect(thetaToTrigSyms(LHS(2,4)), [C23 S23]) == RHS(2,4)
[eqn3] = ABB_IK_T2(eqn3_x, eqn3_y)
a = (D1 - P3);
b = (P1*cos(T1) - A1 + P2*sin(T1));
c = -(P1^2*cos(T1)^2 - P2^2*cos(T1)^2 + A1^2 + A2^2 - A3^2 + D1^2 - D4^2 + P2^2 + P3^2 - 2*D1*P3 + P1*P2*sin(2*T1) - 2*A1*P1*cos(T1) - 2*A1*P2*sin(T1))/(2*A2);
t_2 = atan2(b,a) + [ atan2(sqrt(a^2 + b^2 - c^2), c) , -atan2(sqrt(a^2 + b^2 - c^2), c) ]

% Use (1, 3) and (3, 3) to find equation for T_4
% Assuming that sin(T5) != 0
eqn4_x = collect(LHS(1,3), [C23 S23]) == RHS(1,3)
eqn4_z = collect(LHS(3,3), [C23 S23]) == RHS(3,3)
eqn4_x = thetaToTrigSyms(lhs(eqn4_x) /-sin(T5)) == thetaToTrigSyms(rhs(eqn4_x) /-sin(T5));
eqn4_z = thetaToTrigSyms(lhs(eqn4_z) /-sin(T5)) == thetaToTrigSyms(rhs(eqn4_z) /-sin(T5));
S = solve([eqn4_x, eqn4_z], [C4 S4])
T_4 = atan2(S.S4/S5, S.C4/S5)

%% Perform Split at Frame 4 to get T_5
T_40 = multiplyTransMatrix(T_30, T_43);
T_40_inv = inverseTransMatrix(T_40);
LHS = multiplyTransMatrix(T_40_inv, T_60_Dummy)
RHS = T_64
eqn5_x = thetaToTrigSyms(collect(LHS(1,3), [C23 S23]) == RHS(1,3));
eqn5_z = thetaToTrigSyms(collect(LHS(3,3), [C23 S23]) == RHS(3,3));
S = solve([eqn5_x, eqn5_z], [S5 C5])
T_5 = atan2(S.S5, S.C5)

%% Perform Split at Frame 5 to get T_6
T_50 = multiplyTransMatrix(T_40, T_54);
T_50_inv = inverseTransMatrix(T_50);
LHS = multiplyTransMatrix(T_50_inv, T_60_Dummy)
RHS = T_65
eqn6_x = thetaToTrigSyms(collect(LHS(1,1), [C23 S23]) == RHS(1,1));
eqn6_z = thetaToTrigSyms(collect(LHS(3,1), [C23 S23]) == RHS(3,1));
S = solve([eqn6_x, eqn6_z], [S6 C6])
T_6 = atan2(S.S6, S.C6)