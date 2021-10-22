function [Tinv, T_mMax] = generateInverse(DH,m)
%   m: Start of T for Right Side

    max = height(DH);

    % Generate Tinv
    Tinv = dhToTMatrix(DH, m, 0);
    Tinv = inv(Tinv);

    % Generate T_mn 
    T_mMax = dhToTMatrix(DH, max - 1, m);

end