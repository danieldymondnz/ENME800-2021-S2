function T_CA = multiTransMatrix(T_BA, T_CB)

    % Extract Parameters
    R_BA = T_BA(1:3, 1:3);  R_CB = T_CB(1:3, 1:3);
    P_BA = T_BA(1:3,4);     P_CB = T_CB(1:3,4);

    % Perform Matrice Calculation
    R_CA = R_BA * R_CB;
    P_CA = R_BA * P_CB + P_BA;
    
    % Assemble Matrice
    T_CA = [R_CA, P_CA];
    T_CA = [T_CA; 0, 0, 0, 1];

end