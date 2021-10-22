%% Jacobian

% Syms
syms dT [6 1]; syms dW [1 6]; syms dV [1 6];

%% Get the Velocities relative to {0} by working up through {1} -> {6}

[T_subs, ~, ~, DH] = ABB_Config();

%W_ii = R_i(i+1) w_ii
R_nm1N = {}; RZn = {}; P_nNm1 = {};
% Generate R_mn & RZn for each frame
for n = 1:6
    T_nNm1 = dhToTMatrix(DH, n, n-1);
    R_nm1N{n} = sym2cell(transpose(T_nNm1(1:3,1:3)));
    P_nNm1{n} = sym2cell(T_nNm1(1:3,4));
    RZn{n} = T_nNm1(1:3,3);
end

% Velocities for Frame 0



for i=0:5

    if i == 0
        Wii = [0; 0; 0];
        Vii = [0; 0; 0];
    else
        Wii = W_ii{i};
        Vii = V_ii{i};
    end

    W_ii{i+1} = cell2sym(R_nm1N{i+1}) * Wii + dT(i+1) * RZn{i+1}
    V_ii{i+1} = cell2sym(R_nm1N{i+1}) * (Vii + cross(Wii, cell2sym(P_nNm1{i+1})))

end

% Determine Jacobian for Linear Velocity
V_3 = V_ii{6}

V_3 = collect(V_3, transpose(dT(:)))

J_3 = {};
% Generate Jacobian Matrix
for i=1:3
    [coefficients,~] = equationsToMatrix(V_3(1,:),transpose(dT(:)))
    J_3{i, 1} = simplify(coefficients(1));
    J_3{i, 2} = simplify(coefficients(2));
    J_3{i, 3} = simplify(coefficients(3));
    J_3{i, 4} = simplify(coefficients(4));
    J_3{i, 5} = simplify(coefficients(5));
    J_3{i, 6} = simplify(coefficients(6));
end



%  test
W_11 = [0; 0; dT1];
