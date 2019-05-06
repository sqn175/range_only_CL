function [Cor, iter] = MDS_Adam(M, Converge, iterth)
%{
    Compute node location by MDS initialization and Adam gradient descent
    in 2D space.
    
    Inputs:

    - M: Adjacent matrix of nodes in the network, of shape (N, N), where 
         M(i, j) is the raw distance between node i and j, measured by 
         sensors.

    - Converge: Converge parameter.
    
    Outputs:

    - Cor: Node locations, of shape (N, 2), where Cor(i, :) is node i's
           coordinate (x_i, y_i).

    - iter: Number of iterations when converge.

    Author: Hongyuan Du

%}

[n, ~] = size(M);
lr = 0.05;%Ñ§Ï°ÂÊ
beta1 = 0.9;
beta2 = 0.999;
m = zeros(n, 2);
v = zeros(n, 2);

% MDS initialization    
Cor = MDS(M, '2D');

% figure test
figure; hold on; title('MDS');axis equal
ids = [1 4 0 2];
for i = 1:4
    plot(Cor(i,1),Cor(i,2),'.','DisplayName',num2str(ids(i)));
end
legend show
% figure test end
Cor = Cor - Cor(1, :);
mo = sqrt(sum(Cor(2, :) .^ 2));
c = Cor(2, 1) / mo;
s = Cor(2, 2) / mo;
Cor = Cor * [c, -s; s, c];

% Start iteration
iter = 0;
while 1
    
    iter = iter + 1;
    
    % Check iter
    if iter > iterth
        
        break;
        
    end
    
    % Compute new adjacent matrix
    Cor_square = sum(Cor .^ 2, 2);
    M_t = (-2 .* (Cor * Cor.') + Cor_square) + Cor_square.';
    M_t(M_t < 0) = 0;
    M_t = sqrt(M_t);
    
    % Compute gradients
    M_t = M_t + eye(n); % Avoid divided by zero.
    
    dM = M_t - M;
    dx = Cor(:, 1) - Cor(:, 1).';
    dy = Cor(:, 2) - Cor(:, 2).';
    g = zeros(n, 2);
    g(:, 1) = sum((dM .* dx) ./ M_t, 2);
    g(:, 2) = sum((dM .* dy) ./ M_t, 2);
    
    % Adam update
    m = beta1 .* m + (1 - beta1) .* g;
    v = beta2 .* v + (1 - beta2) .* g .* g;
    m_unbias = m ./ (1 - beta1 .^ iter);
    v_unbias = v ./ (1 - beta2 .^ iter);
    update = lr .* m_unbias ./ (sqrt(v_unbias) + 1e-8);
    Cor = Cor - update;
    
    % Constraint
    Cor(1, :) = 0;
    Cor(2, 2) = 0;
    
    % If converge
    if max(max(abs(update))) < Converge
        break;
    end
    
end

end
