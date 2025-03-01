function [A_estimated_q] = QUEST(b_i, r_i, w_i)
    % QUEST: Quaternion Estimator algorithm for satellite attitude determination.
    %
    % INPUTS:
    %   - b_i: Measured vectors in the body frame (2x3 matrix)
    %   - r_i: Corresponding reference vectors in the inertial frame (2x3 matrix)
    %   - w_i: Weights associated with each vector (2x1 vector)
    %
    % OUTPUTS:
    %   - A_estimated_q: Estimated attitude as a Direction Cosine Matrix (DCM)

    % Validate input dimensions
    if size(b_i, 2) ~= 3 || size(r_i, 2) ~= 3
        error('b_i y r_i deben ser matrices de 2x3 (dos vectores 3D).');
    end
    if length(w_i) ~= size(b_i, 1)
        error('w_i debe tener la misma longitud que el número de filas en b_i (2).');
    end

    % Normalize the measured body-frame vectors (b_i) and reference vectors (r_i)
    for i = 1:size(b_i, 1)
        b_i(i, :) = b_i(i, :) / norm(b_i(i, :)); % Normalize each row vector
        r_i(i, :) = r_i(i, :) / norm(r_i(i, :)); % Normalize reference vectors as well
    end

    % Compute the observation matrix B
    B = zeros(3, 3); % Initialize B
    for i = 1:size(b_i, 1)
        B = B + w_i(i) * (b_i(i, :)' * r_i(i, :)); % B = Σ(w_i * b_i^T * r_i)
    end

    % Compute sigma (trace of B), symmetric matrix S, and vector Z
    sigma = trace(B);              % Sum of diagonal elements (trace of B)
    S = B + B';                    % Symmetric part of B
    Z = [B(2, 3) - B(3, 2);        % Vector of anti-symmetric elements
         B(3, 1) - B(1, 3);
         B(1, 2) - B(2, 1)];

    % Construct the extended K matrix (4x4)
    K = [sigma, Z';
         Z, S - sigma * eye(3)];

    % Compute eigenvalues and eigenvectors of K
    [eigvec, eigval] = eig(K);     % Compute eigen decomposition
    [~, max_idx] = max(diag(eigval)); % Find the index of the largest eigenvalue

    % Extract the quaternion corresponding to the largest eigenvalue
    q_max = eigvec(:, max_idx);

    % Ensure quaternion follows a consistent convention (positive scalar part)
    if q_max(1) < 0
        q_max = -q_max; % Flip sign to maintain positive quaternion convention
    end

    % Convert quaternion to Direction Cosine Matrix (DCM)
    A_estimated_q = quat2dcm(q_max');
end

