function T_AC = multiplyTransMatrice(T_AB, T_BC)

    % Extract Parameters
    R_AB = T_AB(1:3, 1:3);  R_BC = T_BC(1:3, 1:3);
    P_AB = T_AB(1:3,4);     P_BC = T_BC(1:3,4);
    
    % Perform Matrice Calculation
    R_AC = R_AB * R_BC;
    P_AC = R_AB * P_BC + P_AB;
    
    % Assemble Matrice
    T_AC = [R_AC, P_AC];
    T_AC = [T_AC; 0, 0, 0, 1];

end

