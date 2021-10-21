function symsOut = thetaToTrigSyms(symsIn)
    
    syms T [1 6]; syms A [1 6]; syms D [1 6];
    syms S [1 6]; syms C [1 6];

    symsOut = symsIn;
    for i = 1:length(T)
        symsOut = subs(symsOut, [sin(T(i)) cos(T(i))], [S(i) C(i)]);
    end

end