function [eqn_14, eqn_24] = ABB_IK_T23(eqn_14, eqn_24)
%   ABB_IK_T3 Generate the solution for T3 in a separate script
%   Takes (1:3, 4) from the matrix split at Frame 2 to find an
%   expression for theta 3 (T_3)

    % Syms
    syms T [1 6]; syms A [1 6]; syms D [1 6]; syms A2C2; syms A2D4; syms A2C3;
    syms S [1 6]; syms C [1 6]; syms S23; syms C23;
    syms R [3 3]; syms P [3 1];

    eqn_14 = ((- A1 - A2*C2)*C1^2 + P1*C1 + (- A1 - A2*C2)*S1^2 + P2*S1)*C23 + (P3 - D1 - A2*S2)*S23 == A3
    eqn_24 = (P3 - D1 - A2*S2)*C23 + ((A1 + A2*C2)*C1^2 - P1*C1 + (A1 + A2*C2)*S1^2 - P2*S1)*S23 == -D4

    eqn_14 = ((- A1 - A2*C2) + P1*C1 + P2*S1)*C23 + (P3 - D1 - A2*S2)*S23 == A3
    
    %eqn_14 = (P1*C1 + P2*S1 - A1)*C23 - (D1 - P3)*S23 == A3 + A2*(C2*C23 + S2*S23)
    %eqn_24 = (P1*C1 + P2*S1 - A1)*S23 + (D1 - P3)*C23 == D4 + A2*(C2*S23 - S2*C23)

    a = (P1*C1 + P2*S1 - A1);
    b = (D1 - P3);
    c = A3 + A2*(C2*C23 + S2*S23);
    d = D4 + A2*(C2*S23 - S2*C23);
    admbc = a*d - b*c
    acpbd = a*c - b*d
    admbc = expand(admbc)
    admbc = collect(admbc, [S23 C23])


    % Factorise eqn_14
    eqn_14 = subs(eqn_14, C23, (C2*C3 - S2*S3))
    eqn_14 = subs(eqn_14, S23, (C2*S3 - S2*C3))
    eqn_14 = collect(eqn_14, [A2])
    eqn_14 = subs(eqn_14, (C1^2 + S1^2), 1)
    eqn_14 = collect(eqn_14, [C2])
    eqn_14 = subs(eqn_14, (C1^2 + S1^2), 1)
    eqn_14 = collect(eqn_14, [A1])
    eqn_14 = subs(eqn_14, (C1^2 + S1^2), 1)
    eqn_14 = collect(eqn_14, [A2])
    eqn_14 = subs(eqn_14, (C2^2 + S2^2), 1)
    eqn_14 = collect(eqn_14, [S3 C3])

    eqn_14 = collect(eqn_14, A1)
    eqn_14 = subs(eqn_14, (C1^2 + S1^2), 1)
    eqn_14 = subs(eqn_14, (A2*C2), A2C2)
    eqn_14 = collect(eqn_14, A2C2)
    eqn_14 = subs(eqn_14, (C1^2 + S1^2), 1)
    eqn_14 = subs(eqn_14, A2C2, (A2*C2))
    eqn_14 = collect(eqn_14, [C23 S23])

    % Factorise eqn_24
    eqn_24 = collect(eqn_24, A1)
    eqn_24 = subs(eqn_24, (C1^2 + S1^2), 1)
    eqn_24 = subs(eqn_24, (A2*C2), A2C2)
    eqn_24 = collect(eqn_24, A2C2)
    eqn_24 = subs(eqn_24, (C1^2 + S1^2), 1)
    eqn_24 = subs(eqn_24, A2C2, (A2*C2))
    eqn_24 = collect(eqn_24, [C23 S23])

end

