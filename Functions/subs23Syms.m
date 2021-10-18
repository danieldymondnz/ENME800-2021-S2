function T_out = subs23Syms(T_in)
    
    syms T [1 7];
    syms C23; syms S23;

    % Simplify for Factors C23 and S23
    T_out = subs(T_in, cos(T2 + T3), C23)
    T_out = subs(T_out, sin(T2 + T3), S23)

end

