function [output] = ABB_IK_T23(eqn_14, eqn_24)
%   ABB_IK_T3 Generate the solution for T3 in a separate script
%   Takes (1:3, 4) from the matrix split at Frame 2 to find an
%   expression for theta 3 (T_3)

    % Syms
    syms T [1 6]; syms A [1 6]; syms D [1 6]; syms A2C2; syms A2D4; syms A2C3; syms T23;
    syms S [1 6]; syms C [1 6]; syms S23; syms C23;
    syms R [3 3]; syms P [3 1];

    % By Squaring and Adding, a solution can be derrived
    % Square LHS and RHS Components for each equation and add equations
    LHS = expand( (lhs(eqn_14))^2 + (lhs(eqn_24))^2 )
    RHS = expand( (rhs(eqn_14))^2 + (rhs(eqn_24))^2 )

    % RHS is Good

    % Deal to LHS
    LHS = subs(LHS, [S1 C1 S2 C2 S23 C23], [sin(T1) cos(T1) sin(T2) cos(T2) sin(T23) cos(T23)])
    LHS = simplify(LHS)
    LHS = collect(LHS, [P3 D1])
    LHS = subs(LHS, (C23^2 + S23^2), 1)

    LHS = LHS - (A1^2 + A2^2 + D1^2 + P2^2 + P3^2)
    RHS = RHS - (A1^2 + A2^2 + D1^2 + P2^2 + P3^2)

    LHS = subs(LHS, [sin(T2) cos(T2)], [S2 C2])
    LHS = collect(LHS, [S2 C2])

    LHS = LHS - (P1^2*cos(T1)^2 + sin(2*T1)*P1*P2 - 2*A1*P1*cos(T1) - P2^2*cos(T1)^2 - 2*A1*sin(T1)*P2 - 2*D1*P3)
    RHS = RHS - (P1^2*cos(T1)^2 + sin(2*T1)*P1*P2 - 2*A1*P1*cos(T1) - P2^2*cos(T1)^2 - 2*A1*sin(T1)*P2 - 2*D1*P3)

    LHS = collect(LHS, [A2])
    LHS = simplify(LHS / (A2))
    RHS = RHS / (A2)

    LHS = collect(LHS, [2])
    LHS = simplify(LHS / (2))
    RHS = RHS / (2)

    LHS = collect(LHS, [S2 C2])

    output = LHS == RHS



end

