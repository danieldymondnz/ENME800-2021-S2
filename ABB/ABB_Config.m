function [T_subs, a, d, DH] = ABB_Config()
%ABB_Config Generates T_nm matrices and DH data for the ABB IRB-2600-12/1.85 Robot
% Outputs:
% T_subs:   Cell Matrix of symbolic expressions for Tnm
%           {T_10, T_21, ... , T_76}
% a     :   Values for A (rads)
% d     :   Values for D (mm)
% DH    :   The DH Matrix
    
    % Create Syms for Theta
    syms T [1 6]; syms A [1 7]; syms D [1 7];
    
    % Create DH Matrix
    %       alpha   a       d       theta
    DH = [  0       0       D1      T1
            pi/2    A1      0       (T2 + pi/2)
            0       A2      0       T3
            pi/2    A3      D4      T4
            pi/2    0       0       T5
            -pi/2   0       0       T6
            0       0       D7      0];

    % Create vectors for A/D values
    a = [150 900 115 0 0 0 0];
    d = [445 0 0 795 0 0 165];

    % Generate the T_subs Cells
    T_10 = dhToTMatrix(DH, 1, 0);
    T_21 = dhToTMatrix(DH, 2, 1);
    T_32 = dhToTMatrix(DH, 3, 2);
    T_43 = dhToTMatrix(DH, 4, 3);
    T_54 = dhToTMatrix(DH, 5, 4);
    T_65 = dhToTMatrix(DH, 6, 5);
    T_76 = dhToTMatrix(DH, 7, 6);
    T_subs = {T_10, T_21, T_32, T_43, T_54, T_65, T_76};
    
end