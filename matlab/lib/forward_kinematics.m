function [ T ] = forward_kinematics(DH, q, base)
% forward_kinematics calcula la solución de cinemática directa para cualquier
% robot serialmente acoplado de n-grados de libertad.
%
% Entradas:
%   DH:   Matriz de parámetros Denavit-Hartenberg de tamaño n x 5.
%   q:    Vector de posiciones articulares (1 x n) o matriz de trayectoria (N x n).
%   base: (opcional) transformación homogénea {mundo} -> {0}. Si se omite se
%         usa la identidad (resultados en el marco de la base {0}).
%
% Salidas:
%   T:  Matriz de transformación homogénea 4x4 (configuración única)
%       o arreglo 4x4xN (trayectoria N configuraciones).

    n = size(DH, 1);

    if nargin < 3 || isempty(base)
        base = eye(4);
    end

    if isvector(q)
        q = q(:)';
        num_configs = 1;
        single = true;
    else
        num_configs = size(q, 1);
        single = false;
    end

    T = zeros(4, 4, num_configs);

    for k = 1:num_configs
        T_k = base;
        for i = 1:n
            theta_offset = DH(i, 1);
            d_offset     = DH(i, 2);
            a            = DH(i, 3);
            alpha        = DH(i, 4);
            sigma        = DH(i, 5);

            if sigma == 0
                theta = q(k, i) + theta_offset;
                d = d_offset;
            else
                theta = theta_offset;
                d = q(k, i) + d_offset;
            end

            T_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                   sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                   0,           sin(alpha),             cos(alpha),            d;
                   0,           0,                      0,                     1];

            T_k = T_k * T_i;
        end
        T(:, :, k) = T_k;
    end

    if single
        T = T(:, :, 1);
    end
end
