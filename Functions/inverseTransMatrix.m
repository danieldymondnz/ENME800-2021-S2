function [inversedMatrix] = inverseTransMatrix(transformationMatrix)
    
    % Extract Rotation Matrix and Position Vector
    P_BA = transformationMatrix(1:3, 4);
    R_BA = transformationMatrix(1:3, 1:3);
    
    % Inverse the Rotation Matrice
    R_AB_T = transpose(R_BA);

    % Inverse the Position Vector
    P_AB = -R_AB_T * P_BA;

    inversedMatrix = [R_AB_T, P_AB];
    inversedMatrix = [inversedMatrix; 0, 0, 0, 1];

end

