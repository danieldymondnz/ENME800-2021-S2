function [outputExpression] = symsSimplification(inputExpression)
%SYMSSIMPLIFICATION Cleanup Symbolic Expressions using cos and sine of common trig identities
%   Detailed explanation goes here

    syms s1 c1 s2 c2 s3 c3 s4 c4 s5 c5 s6 c6 s7 c7
    syms t1 t2 t3 t4 t5 t6 t7
    outputExpression = inputExpression;
    
    % Cleanup sin^2 + cos^2 = 1
    outputExpression = subs(outputExpression, s1^2 + c1^2, 1);
    outputExpression = subs(outputExpression, c1^2 + s1^2, 1);
    outputExpression = subs(outputExpression, s2^2 + c2^2, 1);
    outputExpression = subs(outputExpression, c2^2 + s2^2, 1);
    outputExpression = subs(outputExpression, s3^2 + c3^2, 1);
    outputExpression = subs(outputExpression, c3^2 + s3^2, 1);
    outputExpression = subs(outputExpression, s4^2 + c4^2, 1);
    outputExpression = subs(outputExpression, c4^2 + s4^2, 1);
    outputExpression = subs(outputExpression, s5^2 + c5^2, 1);
    outputExpression = subs(outputExpression, c5^2 + s5^2, 1);
    outputExpression = subs(outputExpression, s6^2 + c6^2, 1);
    outputExpression = subs(outputExpression, c6^2 + s6^2, 1);
    outputExpression = subs(outputExpression, s7^2 + c7^2, 1);
    outputExpression = subs(outputExpression, c7^2 + s7^2, 1);
    
end

