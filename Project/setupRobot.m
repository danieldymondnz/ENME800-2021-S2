function [T_brk, T_sub] = setupRobot()

    syms s1 c1 s2 c2 s3 c3 s4 c4 s5 c5 s6 c6 s7 c7
    syms t1 t2 t3 t4 t5 t6 t7

    %       alpha   a       d       theta
    dH = [  0       0       0.445   t1
            pi/2    0.150   0       (t2 + pi/2)
            pi      0.900   0       t3
            -pi/2   0.115   0.795   t4
            -pi/2   0       0       t5
            pi/2    0       0       t6
            0       0       0.165   t7];

    % Generate the DH Transformation Matrices between links using method
    T_10 = dhToTMatrix(dH(1,2), dH(1,1), dH(1,4), dH(1,3));
    T_21 = dhToTMatrix(dH(2,2), dH(2,1), dH(2,4), dH(2,3));
    T_32 = dhToTMatrix(dH(3,2), dH(3,1), dH(3,4), dH(3,3));
    T_43 = dhToTMatrix(dH(4,2), dH(4,1), dH(4,4), dH(4,3));
    T_54 = dhToTMatrix(dH(5,2), dH(5,1), dH(5,4), dH(5,3));
    T_65 = dhToTMatrix(dH(6,2), dH(6,1), dH(6,4), dH(6,3));
    T_76 = dhToTMatrix(dH(7,2), dH(7,1), dH(7,4), dH(7,3));

    % Return as cell array for use
    T_sub = {T_10 T_21 T_32 T_43 T_54 T_65 T_76};

    % Generate the overall T_70 matrice
    T_20 = multiplyTransMatrice(T_10, T_21);
    T_30 = multiplyTransMatrice(T_20, T_32);
    T_40 = multiplyTransMatrice(T_30, T_43);
    T_50 = multiplyTransMatrice(T_40, T_54);
    T_60 = multiplyTransMatrice(T_50, T_65);
    T_70 = multiplyTransMatrice(T_60, T_76);
    T_brk = {T_10 T_20 T_30 T_40 T_50 T_60 T_70};

end

