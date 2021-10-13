function T_nm = dhToTMatrix(DH, n, m)
    
    %dhToTMatrix Generate the Transformation matrix from DH.
    % dH - The DH Parameters for this Robot 
    % n  - Frame to be described
    % m  - Frame in which 'n' is described
    
    T_nm = [];

    for subset = m+1:n

        if subset == 0
            T_nm = eye(4);
        else
            i = subset;
            a = DH(i, 2);
            alpha = DH(i, 1);
            theta = DH(i, 4);
            d = DH(i, 3);
            
            T = sym([   cos(theta)                -sin(theta)            0               a
                        sin(theta)*cos(alpha)    cos(theta)*cos(alpha) -sin(alpha)	-sin(alpha)*d
                        sin(theta)*sin(alpha)    cos(theta)*sin(alpha) cos(alpha)     cos(alpha)*d
                        0                          0                       0               1               ]);
            
            if isempty(T_nm)
                T_nm = T;
            else
                T_nm = multiplyTransMatrix(T_nm, T);
            end

        end

    end

%     a, alpha, theta, d
% 
%     % Create the Matrice
%     T = sym([cos(theta)                -sin(theta)            0               a
%          sin(theta)*cos(alpha)    cos(theta)*cos(alpha) -sin(alpha)	-sin(alpha)*d
%          sin(theta)*sin(alpha)    cos(theta)*sin(alpha) cos(alpha)     cos(alpha)*d
%          0                          0                       0               1               ]);
%      
end

