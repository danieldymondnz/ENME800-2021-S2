function [Tinv, T_mn] = generateInverse(T,m)
%GENERATEINVERSE Summary of this function goes here
%   Input Parameters

%   Outputs three symbols
%   Tinv * T_0n = T_mn

%   T: Transformation Matrices (as symbols)
%      [T_10, T_12 ... T_n,n-1]
%   m: Start of T for Right Side

    % Generate Tinv
    Tinv = eye(4);
    for i = 1:m
        Tinv = multiplyTransMatrice(Tinv, cell2sym(T(i)));
    end
    Tinv = inv(Tinv);

    % Generate T_mn
    T_mn = eye(4);
    for i = m:length(T)
        T_mn = multiplyTransMatrice(T_mn,cell2sym(T(i)));
    end

end

