function Tau = ID(DH, PARAM, q, qd, qdd, grav, F_ext, Icm)
%ID  Dinamica inversa por Newton-Euler recursivo (formulacion 3D).
%
%   Tau = ID(DH, PARAM, q, qd, qdd)
%   Tau = ID(DH, PARAM, q, qd, qdd, grav)
%   Tau = ID(DH, PARAM, q, qd, qdd, grav, F_ext)
%   Tau = ID(DH, PARAM, q, qd, qdd, grav, F_ext, Icm)
%
%   Resuelve tau = M(q)qdd + C(q,qd)qd + g(q) para un robot serial de
%   articulaciones rotacionales, con la formulacion recursiva de Newton-Euler.
%
%   Entradas:
%       DH    : n x 5, parametros Denavit-Hartenberg [theta d a alpha sigma].
%               Se asume sigma = 0 (articulaciones rotacionales) y una base
%               alineada con el mundo (solo traslacion).
%       PARAM : n x m, parametros dinamicos. Columnas 1..7:
%               [m Sx Sy Sz Ixx Iyy Izz] (masa, primer momento e inercias
%               principales respecto al centro de masa).
%       q     : pt x n (o 1 x n), posiciones articulares [rad].
%       qd    : pt x n (o 1 x n), velocidades articulares [rad/s].
%       qdd   : pt x n (o 1 x n), aceleraciones articulares [rad/s^2].
%       grav  : 3 x 1 (o 1 x 3), gravedad en el marco mundo [m/s^2].
%               Por defecto [0; 0; -9.81].
%       F_ext : 6 x 1, wrench externo en el efector final [f; n], expresado
%               en el marco del efector. Por defecto ceros.
%       Icm   : 3 x 3 x n, tensor de inercia completo de cada eslabon
%               respecto a su centro de masa y en su marco DH. Si se omite,
%               se usa la parte diagonal de PARAM (columnas 5..7).
%
%   Salida:
%       Tau   : pt x n, par articular requerido en cada punto [N*m].
%
%   La pasada directa calcula velocidades y aceleraciones de cada eslabon y
%   la pasada inversa las fuerzas y momentos; el par se obtiene proyectando
%   el momento sobre el eje z de cada articulacion.

    if nargin < 6 || isempty(grav)
        grav = [0; 0; -9.81];
    end
    if nargin < 7 || isempty(F_ext)
        F_ext = zeros(6, 1);
    end

    n = size(DH, 1);

    q   = reshape(q,   [], n);
    qd  = reshape(qd,  [], n);
    qdd = reshape(qdd, [], n);

    if size(q, 1) ~= size(qd, 1) || size(q, 1) ~= size(qdd, 1)
        error('ID:dimensiones', 'q, qd y qdd deben tener las mismas filas.');
    end
    if size(PARAM, 1) < n || size(PARAM, 2) < 7
        error('ID:parametros', 'PARAM debe ser al menos n x 7.');
    end
    if any(DH(:, 5) ~= 0)
        error('ID:articulacion', 'Solo se soportan articulaciones rotacionales.');
    end

    grav  = grav(:);
    F_ext = F_ext(:);

    m = PARAM(:, 1);
    r = PARAM(:, 2:4);

    if nargin >= 8 && ~isempty(Icm)
        I = Icm;
        if ~isequal(size(I), [3 3 n])
            error('ID:inercia', 'Icm debe ser 3 x 3 x n.');
        end
    else
        I = zeros(3, 3, n);
        for j = 1:n
            I(:, :, j) = diag([PARAM(j,5) PARAM(j,6) PARAM(j,7)]);
        end
    end

    z0  = [0; 0; 1];
    pt  = size(q, 1);
    Tau = zeros(pt, n);
    Rm  = cell(1, n);
    ps  = zeros(3, n);

    for k = 1:pt
        % ------------------------------------------------ transformaciones
        for j = 1:n
            th = q(k, j) + DH(j, 1);
            al = DH(j, 4);
            a  = DH(j, 3);
            d  = DH(j, 2);
            ct = cos(th); st = sin(th);
            ca = cos(al); sa = sin(al);

            Rm{j} = [ct  -st*ca   st*sa;
                     st   ct*ca  -st*sa;
                      0    sa      ca];
            ps(:, j) = [a; d*sa; d*ca];
        end

        % ------------------------------------------------ pasada directa
        Fm = zeros(3, n);
        Nm = zeros(3, n);
        w  = zeros(3, 1);
        wd = zeros(3, 1);
        vd = -grav;
        for j = 1:n
            Rt = Rm{j}.';
            rj = r(j, :).';

            wd = Rt * (wd + z0*qdd(k, j) + cross(w, z0*qd(k, j)));
            w  = Rt * (w + z0*qd(k, j));
            vd = cross(wd, ps(:, j)) + cross(w, cross(w, ps(:, j))) + Rt*vd;

            vhat = cross(wd, rj) + cross(w, cross(w, rj)) + vd;
            Fm(:, j) = m(j) * vhat;
            Nm(:, j) = I(:, :, j)*wd + cross(w, I(:, :, j)*w);
        end

        % ------------------------------------------------ pasada inversa
        f  = F_ext(1:3);
        nn = F_ext(4:6);
        for j = n:-1:1
            if j == n
                R = eye(3);
            else
                R = Rm{j+1};
            end

            nn = R*(nn + cross(R.'*ps(:, j), f)) + ...
                 cross(ps(:, j) + r(j, :).', Fm(:, j)) + Nm(:, j);
            f  = R*f + Fm(:, j);

            Tau(k, j) = nn.' * (Rm{j}.' * z0);
        end
    end
end
