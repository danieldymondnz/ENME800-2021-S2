function T_out = subsSymToT(DH, n, m, t, a, d)
%SUBSSYMTOT Summary of this function goes here
%   Detailed explanation goes here

    syms T [1 7];
    syms A [1 7];
    syms D [1 7];

    % Get Symbolic Equation
    T_syms = dhToTMatrix(DH, n, m);

    % Subs Syms
    T_out = subs(T_syms, [T1 T2 T3 T4 T5 T6 T7], t);
    T_out = subs(T_out, [A1 A2 A3 A4 A5 A6 A7], a);
    T_out = subs(T_out, [D1 D2 D3 D4 D5 D6 D7], d);

    T_out = double(T_out);

end

