function [thetas] = ABB_IK_solveTheta(T_70)
%ABB_IK_SOLVETHETA Summary of this function goes here
%   Detailed explanation goes here

    % Syms
    syms T [1 6]; syms A [1 6]; syms D [1 6];
    syms S [1 6]; syms C [1 6];
    syms R [3 3]; syms P [3 1];

    % Extract Parameters
    R1_1 = T_70(1,1); R1_2 = T_70(1,2); R1_3 = T_70(1,3);
    R2_1 = T_70(2,1); R2_2 = T_70(2,2); R2_3 = T_70(2,3); 
    R3_1 = T_70(3,1); R3_2 = T_70(3,2); R3_3 = T_70(3,3);
    P1 = T_70(1,4);   P2 = T_70(2,4);   P3 = T_70(3,4);

    D1 = 445; D4 = 795; D7 = 165;
    A1 = 150; A2 = 900; A3 = 115;
    D = [445 0 0 795 0 0 165];
    A = [150 900 115 0 0 0 0];
    

    % Solve for T_1
    T_1 = double([atan2(P2, P1); atan2(-P2, -P1)]);

    % Solve for T_3
    for i=1:length(T_1)

        % Determine T_3 Values
        k_3 = -(- A1^2 + 2*cos(T_1(i))*A1*P1 + 2*sin(T_1(i))*A1*P2 + A2^2 + A3^2 - D1^2 + 2*D1*P3 + D4^2 - P1^2 - P2^2 - P3^2)/(2*A2);
        T_3 = double([atan2(A3, -D4) - atan2(k_3, sqrt(A3^2 + D4^2 - k_3^2)), atan2(A3, -D4) - atan2(k_3, -sqrt(A3^2 + D4^2 - k_3^2))]);

        % Append to Matrix
        thetas(2*i-1, :) = [T_1(i) 0 T_3(1) 0 0 0];
        thetas(2*i, :) = [T_1(i) 0 T_3(2) 0 0 0];

    end

    temp_t3 = zeros(8,6);

    % Solve for T_2
    for i=1:height(thetas)

        % Solve for T_2
        % Subs values from T_1, T_3
        T1 = thetas(i,1);
        T3 = thetas(i,3);

        % (P1*cos(T1)* + P2*sin(T1) - A1)*cos(T2) + (P3 - D1)*sin(T2) == (A2 + A3*cos(T3) + D4*sin(T3))
        a = P1*cos(T1)* + P2*sin(T1) - A1;
        b = (P3 - D1);
        c = (A2 + A3*cos(T3) + D4*sin(T3));

        % Solve
        r = a^2 + b^2 - c^2;
        if r < 1e-3
            r = 0;
        end
        T_21 = atan2(b, a) - atan2(sqrt(r), c);
        T_22 = atan2(b, a) + atan2(sqrt(r), c);

        % Append to Matrix
        temp_t3(2*i - 1, :) = [T1 T_21 T3 0 0 0];
        temp_t3(2*i, :) =     [T1 T_22 T3 0 0 0];

    end

    thetas = temp_t3;
    temp_t4 = zeros(16,6);

    % Solve for T_4
    for i=1:height(thetas)

        % Solve for T_2
        % Subs values from T_1, T_3
        T1 = thetas(i,1);
        T2 = thetas(i,2);
        T3 = thetas(i,3);

        S23 = cos(T2)*sin(T3) + sin(T2)*cos(T3)
        C23 = cos(T2)*cos(T3) - sin(T2)*sin(T3)

        C4 = -(R3_3*S23 + C23*(R2_3*sin(T1) + cos(T1)*R1_3))
        S4 = -(R1_3*sin(T1) - cos(T1)*R2_3)

        % Solve
        T_41 = atan2(S4, C4)
        T_42 = -T_41

        % Append to Matrix
        temp_t4(2*i - 1, :) = [T1 T2 T3 T_41 0 0];
        temp_t4(2*i, :) =     [T1 T2 T3 T_42 0 0];

    end

    thetas = temp_t4;
    temp_t5 = zeros(32,6);
    
    % Solve for T_5
    for i=1:height(thetas)

        % Subs values from T_1, T_3
        T1 = thetas(i,1);
        T2 = thetas(i,2);
        T3 = thetas(i,3);
        T4 = thetas(i,4);

        S23 = cos(T2)*sin(T3) + sin(T2)*cos(T3);
        C23 = cos(T2)*cos(T3) - sin(T2)*sin(T3);
        C1 = cos(T1); S1 = sin(T1);
        C4 = cos(T4); S4 = sin(T4);

        S5 = C1*R2_3*S4 - C23*(C1*C4*R1_3 + C4*R2_3*S1) - C4*R3_3*S23 - R1_3*S1*S4;
        C5 = S23*(R2_3*S1 + C1*R1_3) - C23*R3_3;

        % Solve
        T_51 = atan2(S5, C5);
        T_52 = -T_51;

        % Append to Matrix
        temp_t5(2*i - 1, :) = [T1 T2 T3 T4 T_51 0];
        temp_t5(2*i, :) =     [T1 T2 T3 T4 T_52 0];

    end

    thetas = temp_t5;
    temp_t6 = zeros(64,6);


    % Solve for T_6
    for i=1:height(thetas)

        % Subs values from T_1, T_3
        T1 = thetas(i,1);
        T2 = thetas(i,2);
        T3 = thetas(i,3);
        T4 = thetas(i,4);
        T5 = thetas(i,5);

        S23 = cos(T2)*sin(T3) + sin(T2)*cos(T3);
        C23 = cos(T2)*cos(T3) - sin(T2)*sin(T3);
        C1 = cos(T1); S1 = sin(T1);
        C4 = cos(T4); S4 = sin(T4);
        C5 = cos(T5); S5 = sin(T5);

        S6 = C4*R1_1*S1 - C1*C4*R2_1 - C23*(C1*R1_1*S4 + R2_1*S1*S4) - R3_1*S4*S23;
        C6 = C23*(C1*C4*C5*R1_1 - R3_1*S5 + C4*C5*R2_1*S1) + S23*(C4*C5*R3_1 + C1*R1_1*S5 + R2_1*S1*S5) - C1*C5*R2_1*S4 + C5*R1_1*S1*S4;

        % Solve
        T_61 = atan2(S6, C6);
        T_62 = -T_61;

        % Append to Matrix
        temp_t6(2*i - 1, :) = [T1 T2 T3 T4 T5 T_61];
        temp_t6(2*i, :) =     [T1 T2 T3 T4 T5 T_62];

    end

    thetas = temp_t6;

    % Clean-up for solutions that satisfy
    [T_subs, DH] = ABB_Config();
    % T_60 = dhToTMatrix(DH, 6, 0);


    for i=1:height(thetas)

        

        T1 = thetas(i,1); T2 = thetas(i,2); T3 = thetas(i,3);
        T4 = thetas(i,4); T5 = thetas(i,5); T6 = thetas(i,6); T7 = 0;
        theta = [T1 T2 T3 T4 T5 T6 T7];
        
        result = subsSymToT(DH, 6, 0, theta, A, D);
        
        badData = 0;
        for j=1:height(result)
            for k=1:width(result)

                
                % If not about the same, drop this 
                if abs(result(j, k) - T_70(j, k)) > 100
                    badData = 1;
                    continue
                end

                

            end
            if badData == 1
                continue
            end
        end
        
        if (badData)
            thetas(i, :) = 0;
        end    

    end

    % Cleanup
    thetas( all(~thetas,2), : ) = [];

end

