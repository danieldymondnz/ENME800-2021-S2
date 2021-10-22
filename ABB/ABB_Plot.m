function [outputPlot, x, y, z] = ABB_Plot(theta, showPathPreview, outputPlot)
%ABB_PLOT Generate an Output Plot of the Robot Arm
%   theta:              row vector of thetas 1-7
%   showPathPreview:    Set to 1 to show path preview, or 0 otherwise.
    
    % Define Syms
    syms T [1 6]; syms S [1 7]; syms C [1 7];
    syms A [1 7]; syms D [1 7];

    % Load the ABB Config
    [~, a, d, DH] = ABB_Config();

    % If the Path Preview is desired
    if not(showPathPreview == 0)
        outputPlot = drawDesiredPath();
    end

    % Plot {0}
    F_0 = eye(4);
    outputPlot = drawAxis(F_0);
    T_nm10 = F_0;

    % Iterate through all frames
    for n = 1:height(DH)
        T_n0 = subsSymToT(DH, n, 0, theta, a, d);
        outputPlot = plot3([T_nm10(1,4) T_n0(1,4)], [T_nm10(2,4) T_n0(2,4)], [T_nm10(3,4) T_n0(3,4)], 'black', 'LineWidth', 2);
        outputPlot = drawAxis(T_n0);
        T_nm10 = T_n0;
    end
    
    x = T_n0(1,4);
    y = T_n0(2,4);
    z = T_n0(3,4);

    axis equal;

end