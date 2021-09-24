function symsOut = thetaToTrigSymsConversion(symsIn, thetaSyms, trigSyms)
    
    symsOut = symsIn;
    for i = 1:length(thetaSyms)
        symsOut = subs(symsOut, [sin(thetaSyms(i)) cos(thetaSyms(i))], [trigSyms(2*i - 1) trigSyms(2*i)]);
    end

end

