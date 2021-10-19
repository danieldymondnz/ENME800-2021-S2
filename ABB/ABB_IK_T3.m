function [solution, K] = ABB_IK_T3(eqn_14, eqn_24, eqn_34)
%   ABB_IK_T3 Generate the solution for T3 in a separate script
%   Takes (1:3, 4) from the matrix split at Frame 2 to find an
%   expression for theta 3 (T_3)

    % Syms
    syms T [1 6]; syms A [1 6]; syms D [1 6]; syms A2A3; syms A2D4;
    syms S [1 6]; syms C [1 6]; syms S23; syms C23;
    syms R [3 3]; syms P [3 1];
    
    % By Squaring and Adding, a solution can be derrived
    % Square LHS and RHS Components for each equation and add equations
    LHS = expand( (lhs(eqn_14))^2 + (lhs(eqn_24))^2 + (lhs(eqn_34))^2 )
    RHS = expand( (rhs(eqn_14))^2 + (rhs(eqn_24))^2 + (rhs(eqn_34))^2 )
    
    % Perform Cleanup of LHS
    LHS = collect(LHS, [A1^2 A2^2 A3^2]);
    LHS = subs(LHS, (C2^2 + S2^2), 1)
    LHS = collect(LHS, [D1^2 D4^2]);
    LHS = subs(LHS, (C2^2 + S2^2), 1) 
    LHS = collect(LHS, [P1^2 P2^2 P3^2]);
    LHS = subs(LHS, (C2^2 + S2^2), 1) 
    LHS = subs(LHS, [C1 C2 C3 S1 S2 S3], [cos(T1) cos(T2) cos(T3) sin(T1) sin(T2) sin(T3)])
    LHS = simplify(LHS)
    
    % Perform Cleanup of RHS
    RHS = collect(RHS, A3^2)
    RHS = subs(RHS, (C3^2 + S3^2), 1) 
    RHS = collect(RHS, D4^2)
    RHS = subs(RHS, (C3^2 + S3^2), 1)
    
    % Shuffle Variables between RHS and LHS
    LHS = LHS - (A2^2 + A3^2 + D4^2)
    RHS = RHS - (A2^2 + A3^2 + D4^2)
 
    % Shuffle Common Factors between RHS and LHS
    LHS = simplify(LHS / (2*A2))
    RHS = simplify(RHS / (2*A2))
    
    % Generate final expression
    syms K
    solution = RHS == K
    K = LHS

end

