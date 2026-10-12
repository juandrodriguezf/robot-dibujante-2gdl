function J = Jacobian(robot, DH, q)
%Jacobian  Jacobiano geometrico de un robot serialmente acoplado.
%
%   J = Jacobian(robot, DH, q)
%
%   Entradas:
%       robot : estructura del robot obtenida con puma_param(). Se usa
%               unicamente para validar la coherencia con DH.
%       DH    : matriz de parametros Denavit-Hartenberg de tamano n x 5.
%               Columnas: [theta_offset, d_offset, a, alpha, sigma].
%       q     : posiciones articulares 1 x n.
%
%   Salida:
%       J     : matriz 6 x n. Las tres primeras filas forman la parte lineal
%               (velocidad lineal del efector final) y las tres ultimas la
%               parte angular (velocidad angular), ambas con respecto a la
%               base. Para una articulacion i:
%
%                   z_i = T_0_{i-1}(1:3, 3)   eje de la articulacion
%                   o_i = T_0_{i-1}(1:3, 4)   origen de la articulacion
%                   p_e = T_0_{n}(1:3, 4)     posicion del efector final
%
%               rotacional  (sigma = 0):  J_v = z_i x (p_e - o_i)
%                                         J_w = z_i
%               prismatica  (sigma = 1):  J_v = z_i
%                                         J_w = 0
%
%   En la convencion Denavit-Hartenberg estandar la articulacion i gira
%   alrededor del eje z de la articulacion anterior, z_{i-1}, y no alrededor
%   de z_i. El marco de la base (i = 0) se toma como la identidad.
%
%   NOTA. Para robots con muneca esferica las columnas de J_v correspondientes
%   a las ultima(s) articulacion(es) son identicamente cero: esas
%   articulaciones no desplazan el efector final y solo afectan su
%   orientacion.

    if nargin < 3
        error('Jacobian:args', 'Se requieren las entradas robot, DH y q.');
    end

    n = size(DH, 1);
    q = q(:)';

    if length(q) ~= n
        error('Jacobian:badq', ...
            'q debe contener %d valores (uno por articulacion).', n);
    end

    if isstruct(robot) && isfield(robot, 'DH')
        if size(robot.DH, 1) ~= n
            error('Jacobian:badrobot', ...
                'robot y DH describen robots con distinto numero de GDL.');
        end
    end

    % Marcos T_0_{i-1} para i = 1..n, obtained de los marcos T_0_i (i=1..n)
    % de homogenous_matrix() anteponiendo el marco de la base.
    [r, p] = homogenous_matrix(DH, q);
    r = cat(3, eye(3), r);
    p = [zeros(3, 1), p];

    p_e = p(:, n + 1);

    J = zeros(6, n);

    for i = 1:n
        z_i = r(1:3, 3, i);
        o_i = p(:, i);

        if DH(i, 5) == 0        % rotacional
            J(1:3, i) = cross(z_i, p_e - o_i);
            J(4:6, i) = z_i;
        else                    % prismatica
            J(1:3, i) = z_i;
            J(4:6, i) = zeros(3, 1);
        end
    end
end