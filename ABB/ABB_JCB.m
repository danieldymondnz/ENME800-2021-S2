%% Jacobian

syms T [1 6]; syms A [1 7]; syms D [1 7];
syms S [1 7]; syms C [1 7]; syms C23; syms S23;
syms R [3 3]; syms P [3 1];

%% Generate T_i0 matrices
[T_subs, ~, ~, DH] = ABB_Config();

% T_10
T_10 = dhToTMatrix(DH, 1, 0);

% T_20
T_20 = dhToTMatrix(DH, 2, 0);

% T_31
T_31 = dhToTMatrix(DH, 3, 1);
T_31 = simplify(T_31); T_31 = subs23Syms(T_31);

% T_30
T_30 = multiplyTransMatrix(T_10, T_31);

% T_40
T_43 = dhToTMatrix(DH, 4, 3);
T_40 = multiplyTransMatrix(T_30, T_43);

% T_50
T_54 = dhToTMatrix(DH, 5, 4);
T_50 = multiplyTransMatrix(T_40, T_54);

% Generate T_63
T_64 = dhToTMatrix(DH, 6, 4);
T_63 = multiplyTransMatrix(T_43, T_64);

% T_60
T_60 = multiplyTransMatrix(T_30, T_63);

% T_70
T_76 = dhToTMatrix(DH, 7 ,6);
T_70 = multiplyTransMatrix(T_60, T_76);

T_i0 = {T_10, T_20, T_30, T_40, T_50, T_60, T_70};

%% Generate the Jacobian for the linear velocity of the tool tip
O_n = T_70(1:3, 4);

% For each Jacobian column, generate and append to jPos matrix
for joint = 1:6
    
    % If first joint, O(i-1) is the identity matrix
    if joint == 1
        T_0i = eye(4);

    % Otherwise, assign transformation matrix
    else
        T_0i = cell2sym(T_i0(joint - 1));
    end

    % Extract parameters
    O_im1 = T_0i(1:3, 4);
    Z_im1 = T_0i(1:3, 3);

    % Cross product and append to table
    J_col = cross(Z_im1, (O_n - O_im1));
    jPos(:, joint) = J_col;
    J_col;

end