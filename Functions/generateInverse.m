function [Tinv, T_mMax] = generateInverse(DH,m)
%GENERATEINVERSE Summary of this function goes here
%   Input Parameters

%   Outputs three symbols
%   Tinv * T_0n = T_mn

%   T: Transformation Matrices (as symbols)
%      [T_10, T_12 ... T_n,n-1]
%   m: Start of T for Right Side

    max = height(DH);

    % Generate Tinv
    Tinv = dhToTMatrix(DH, m, 0);
    Tinv = inv(Tinv);

    % Generate T_mn 
    T_mMax = dhToTMatrix(DH, max, m);

end

