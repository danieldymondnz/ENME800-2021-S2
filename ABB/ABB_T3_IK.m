% Test
syms T [1 6];
syms A [1 6];
syms D [1 6];
syms S [1 6];
syms S23; syms C23;
syms C [1 6];
syms R [3 3];
syms P [3 1];
syms A2A3; syms A2D4;

%% Equations
eqn_x = P3*S2 - A1*C2 - D1*S2 + C1*C2*P1 + C2*P2*S1 == A2 + A3*C3 + D4*S3
eqn_y = C2*P3 - C2*D1 + A1*S2 - C1*P1*S2 - P2*S1*S2 == A3*S3 - C3*D4
eqn_z = P1*S1 - C1*P2 == 0

%% Create Equations
LHS = expand( (lhs(eqn_x))^2 + (lhs(eqn_y))^2 + (lhs(eqn_z))^2 )
RHS = expand( (rhs(eqn_x))^2 + (rhs(eqn_y))^2 + (rhs(eqn_z))^2 )

%% Perform Cleanup of LHS

LHS = collect(LHS, [A1^2 A2^2 A3^2]);
LHS = subs(LHS, (C2^2 + S2^2), 1)
LHS = collect(LHS, [D1^2 D4^2]);
LHS = subs(LHS, (C2^2 + S2^2), 1) 
LHS = collect(LHS, [P1^2 P2^2 P3^2]);
LHS = subs(LHS, (C2^2 + S2^2), 1) 

LHS = subs(LHS, [C1 C2 C3 S1 S2 S3], [cos(T1) cos(T2) cos(T3) sin(T1) sin(T2) sin(T3)])

LHS = simplify(LHS)


RHS = collect(RHS, A3^2)
RHS = subs(RHS, (C3^2 + S3^2), 1) 
RHS = collect(RHS, D4^2)
RHS = subs(RHS, (C3^2 + S3^2), 1)

LHS = LHS - (A2^2 + A3^2 + D4^2)
RHS = RHS - (A2^2 + A3^2 + D4^2)

LHS = simplify(LHS / (2*A2))
RHS = simplify(RHS / (2*A2))

eqn_f = LHS == RHS
