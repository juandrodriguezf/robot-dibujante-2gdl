function [x, y] = trayectoria_reuleaux(w, C, npts)
%trayectoria_reuleaux  Contorno de un triangulo de Reuleaux.
%
%   [x, y] = trayectoria_reuleaux(w, C, npts)
%
%   Entradas:
%       w    : ancho constante del triangulo [m].
%       C    : centro [xc yc] [m].
%       npts : numero aproximado de puntos del contorno (por defecto 240).
%
%   El triangulo de Reuleaux se construye con tres arcos de radio w, cada uno
%   centrado en el vertice opuesto. El radio de la circunferencia circunscrita
%   es Rc = w/sqrt(3). Devuelve el contorno cerrado (el ultimo punto repite al
%   primero).

    if nargin < 3 || isempty(npts); npts = 240; end

    Rc  = w / sqrt(3);
    ang = deg2rad([90 210 330]);
    V   = C(:) + Rc * [cos(ang); sin(ang)];     % 2 x 3 vertices

    nseg = ceil(npts / 3);
    x = []; y = [];
    for k = 1:3
        c  = V(:, k);
        a1 = V(:, mod(k, 3) + 1);
        a2 = V(:, mod(k + 1, 3) + 1);
        th1 = atan2(a1(2) - c(2), a1(1) - c(1));
        th2 = atan2(a2(2) - c(2), a2(1) - c(1));
        dth = mod(th2 - th1 + pi, 2*pi) - pi;   % arco menor -> lado abombado
        th  = th1 + linspace(0, dth, nseg + 1);
        x = [x, c(1) + w*cos(th)]; %#ok<AGROW>
        y = [y, c(2) + w*sin(th)]; %#ok<AGROW>
    end

    % Eliminar puntos duplicados (vertices compartidos) conservando el cierre
    keep = [true, hypot(diff(x), diff(y)) > 1e-9];
    x = x(keep); y = y(keep);
    x = [x, x(1)]; y = [y, y(1)];
end
