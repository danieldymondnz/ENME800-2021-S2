%% Jacobian Calculation

% Syms
syms dT [6 1]; syms dW [1 6]; syms dV [1 6]; syms C23; syms S23;
syms T [1 7]; syms A [1 7]; syms D [1 7];

%% Generate Simplifications
[T_subs, a, d, DH] = ABB_Config();

% T_63
T_43 = dhToTMatrix(DH, 4, 3);
T_64 = dhToTMatrix(DH, 6, 4);
T_63 = multiplyTransMatrix(T_43, T_64);

% T_31
T_31 = dhToTMatrix(DH, 3, 1);
T_31 = simplify(T_31); T_31 = subs23Syms(T_31);

% T_30
T_10 = dhToTMatrix(DH, 1, 0);
T_30 = multiplyTransMatrix(T_10, T_31);

% T_60
T_60 = multiplyTransMatrix(T_30, T_63);

% T_70
T_76 = dhToTMatrix(DH, 7, 6);
T_70 = multiplyTransMatrix(T_60, T_76);

%% Get the Velocities relative to {0} by working up through {1} -> {6}

% Generate R_mn & RZn for each frame
R_nm1N = {}; RZn = {}; P_nNm1 = {};
for n = 1:7

    % Frame 7 has not rotational velocity relative to 6
    if n == 7
        theta = 0;
    else
        theta = T(1:6);
    end

    % Extract Transformation Matrix
    T_nNm1 = dhToTMatrix(DH, n, n-1);
    T_nNm1 = subs(T_nNm1, [D1 D2 D3 D4 D5 D6 D7], d);
    T_nNm1 = subs(T_nNm1, [A1 A2 A3 A4 A5 A6 A7], a);

    % Append components to table
    R_nm1N{n} = transpose(T_nNm1(1:3,1:3));
    P_nNm1{n} = T_nNm1(1:3,4);
    RZn{n} = T_nNm1(1:3,3);

end

% Determine the velocity propagation to {7}
for i=0:6

    % Fetch velocities from table, or if frame 1, use 0
    if i == 0
        Wii = [0; 0; 0];
        Vii = [0; 0; 0];
    else
        Wii = W_ii{i};
        Vii = V_ii{i};
    end

    % Fetch syms for velocity from table, or use 0 if {7}
    if i == 6
        dt = 0;
    else
        dt = dT(i+1);
    end

    % Calcualte and append to table
    W_ii{i+1} = R_nm1N{i+1} * Wii + dt * RZn{i+1};
    V_ii{i+1} = R_nm1N{i+1} * (Vii + cross(Wii, P_nNm1{i+1}));

end

%% Assemble Jacobian
% Get Linear and Rotational Components, then collect for dT Terms
V_3 = V_ii{7};
V_3 = collect(V_3, transpose(dT(:)));
W_3 = W_ii{7};
W_3 = collect(W_3, transpose(dT(:)));

% Generate Jacobian Matrix
J_7 = {};
for i=1:3
    [coefficients,~] = equationsToMatrix(V_3(i,:),transpose(dT(:)));
    J_7{i, 1} = simplify(coefficients(1));
    J_7{i, 2} = simplify(coefficients(2));
    J_7{i, 3} = simplify(coefficients(3));
    J_7{i, 4} = simplify(coefficients(4));
    J_7{i, 5} = simplify(coefficients(5));
    J_7{i, 6} = simplify(coefficients(6));
    [coefficients,~] = equationsToMatrix(W_3(i,:),transpose(dT(:)));
    J_7{3+i, 1} = simplify(coefficients(1));
    J_7{3+i, 2} = simplify(coefficients(2));
    J_7{3+i, 3} = simplify(coefficients(3));
    J_7{3+i, 4} = simplify(coefficients(4));
    J_7{3+i, 5} = simplify(coefficients(5));
    J_7{3+i, 6} = simplify(coefficients(6));
end

% Translate Jacobian into {B}
R_70 = T_70(1:3, 1:3);
J_0(1:3, :) = R_70 * J_7(1:3, :);
J_0(4:6, :) = R_70 * J_7(4:6, :);

%% Find singularity
J_0 = subs(J_0, C23, cos(T2 + T3))
J_0 = subs(J_0, S23, sin(T2 + T3))
determinant = det(J_0(4:6, 4:6))

solution = solve(determinant == 0, T(1:6))