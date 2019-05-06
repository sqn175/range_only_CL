function Cor_ini = MDS(M, space)
%{
    MDS initialization.

    Input:
    
    - M: The adjacent matrix of network, of shape (N, N).

    - space: In 2D or 3D space.

    Output:

    - Cor_ini: Coordinates of nodes in the network, of shape (N, 2) if 2D 
               or (N, 3) if 3D.

    Author: Hongyuan Du
%}

P = M .^ 2;
P = -0.5 * (((P - mean(P, 1)) - mean(P, 2)) + mean(mean(P)));
[U, V, ~] = svd(P);

if strcmp(space, '2D')
    
    Cor_ini = U(:, 1:2) * sqrt(V(1:2, 1:2));

else
    
    Cor_ini = U(:, 1:3) * sqrt(V(1:3, 1:3));
    
end

end
