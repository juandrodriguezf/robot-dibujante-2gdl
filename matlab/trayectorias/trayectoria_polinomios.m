function [tv, r, rd, rdd, pp] = trayectoria_polinomios(W, T, dt, nv)
%trayectoria_polinomios  Interpolacion cartesiana con splines cubicos.
%
%   [tv, r, rd, rdd, pp] = trayectoria_polinomios(W, T, dt, nv)
%
%   Interpola una secuencia de puntos de paso (via points) con splines cubicos
%   y entrega posicion, velocidad y aceleracion cartesianas por tramos.
%
%   La trayectoria se recorre en nv segmentos de igual duracion; en las fronteras
%   entre segmentos (los nv "vertices" de la figura) la velocidad cartesiana se
%   anula mediante un reescalado temporal suave (smoothstep), de modo que la
%   velocidad es continua (C1) y no hay discontinuidades de curvatura en los
%   vertices.  Con nv = 1 se recorre a velocidad aproximadamente constante
%   (parametro por longitud de arco).
%
%   Entradas:
%       W  : puntos de paso, 2 x N (o N x 2) [m].
%       T  : duracion total [s].
%       dt : paso de muestreo [s].
%       nv : numero de segmentos con reposo en las fronteras (por defecto 1).
%
%   Salidas: tv (1xM) [s], r / rd / rdd (2xM) [m], [m/s], [m/s^2], pp.

    if nargin < 4 || isempty(nv); nv = 1; end

    if size(W, 1) == 2
        P = W;
    else
        P = W';
    end

    % Parametro por longitud de arco, normalizado a [0,1] (splines en u)
    seg = hypot(diff(P(1,:)), diff(P(2,:)));
    u   = [0, cumsum(seg)];
    if u(end) == 0; u(end) = 1; end
    uk  = u / u(end);

    pp.x  = spline(uk, P(1,:));
    pp.y  = spline(uk, P(2,:));
    pp.dx = ppdiff(pp.x);  pp.ddx = ppdiff(pp.dx);
    pp.dy = ppdiff(pp.y);  pp.ddy = ppdiff(pp.dy);

    tv = 0:dt:T;
    [u, du, du2] = tiempo_a_parametro(tv, T, nv);

    r   = [ppval(pp.x, u);                ppval(pp.y, u)];
    rd  = [ppval(pp.dx, u).*du;           ppval(pp.dy, u).*du];
    rdd = [ppval(pp.ddx, u).*du.^2 + ppval(pp.dx, u).*du2; ...
           ppval(pp.ddy, u).*du.^2 + ppval(pp.dy, u).*du2];
end


function [u, du, du2] = tiempo_a_parametro(tv, T, nv)
%tiempo_a_parametro  Mapa t -> u con reposo (du=0) en las nv fronteras.
%   u en [0,1]; en cada segmento se usa un smoothstep s(l)=3l^2-2l^3.
    s   = min(max(tv / T, 0), 1);
    p   = nv * s;
    j   = min(floor(p), nv - 1);      % indice de segmento (0..nv-1)
    l   = p - j;                      % parametro local 0..1
    l   = min(max(l, 0), 1);

    ss  = 3*l.^2 - 2*l.^3;            % smoothstep
    u   = (j + ss) / nv;

    du  = (6*l.*(1 - l)) / T;                 % du/dt
    du2 = (nv * (6 - 12*l)) / (T^2);          % d2u/dt2
end


function d = ppdiff(p)
%ppdiff  Derivada de una pieza polinomica (estructura pp) sin toolbox extra.
    d.form   = 'pp';
    d.breaks = p.breaks;
    d.pieces = p.pieces;
    d.order  = p.order - 1;
    d.dim    = p.dim;
    d.coefs  = p.coefs(:, 1:end-1) .* ((p.order-1):-1:1);
end
