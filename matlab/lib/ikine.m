function [q, info] = ikine(robot, T, q_inicial, opts)
%ikine  Cinematica inversa por metodo del Jacobiano.
%
%   [q, info] = ikine(robot, T, q_inicial)
%   [q, info] = ikine(robot, T, q_inicial, opts)
%
%   Entradas:
%       robot     : estructura del robot obtenida con puma_param().
%       T         : transformacion homogenea 4x4 del efector final con
%                   respecto a la base. Si se entrega un arreglo 4x4xN se
%                   resuelven los N puntos de una trayectoria cartesiana.
%       q_inicial : 1 x n (pose articular inicial). Si T es una trayectoria y
%                   q_inicial es 1 x n, la solucion de cada punto se usa
%                   como semilla del siguiente. Si q_inicial es N x n, cada
%                   fila es la semilla de su punto.
%       opts      : (opcional) estructura con los campos
%                       tol      : tolerancia de posicion [m]      (1e-10)
%                       max_iter : maximo de iteraciones           (200)
%                       lambda   : amortiguamiento de la pseudoinversa,
%                                  0 = sin amortiguamiento          (0)
%                       k_post   : ganancia del termino de espacio
%                                  nulo, 0 = deshabilitado          (1)
%                       max_step : paso maximo por iteracion [rad] (0.5)
%                       q_lim    : limites articulares 2 x n, o []
%                                                                    ([])
%
%   Salidas:
%       q     : 1 x n para un unico punto, o N x n para una trayectoria.
%       info  : estructura de diagnostico con los campos
%                       info.iterations : iteraciones usadas en cada punto
%                       info.error      : norma del error de posicion
%                                        final [m]
%                       info.converged  : logico de convergencia
%                       info.sigma      : valores singulares de J_v
%                       info.null_dim   : dimension del espacio nulo de J_v
%
%   Metodo
%   ------
%   Se resuelve J_v * dq = e, con e = p_objetivo - p_e(q), mediante
%
%       dq = J_v^+ e + (I - J_v^+ J_v) k_post (q_ref - q)
%
%   El primer termino es la solucion de norma minima de la pseudoinversa de
%   Moore-Penrose. El segundo es una proyeccion sobre el espacio nulo de J_v
%   y no altera el movimiento del efector final: es la componente de
%   auto-movimiento. Como q_ref se toma igual a la semilla q_inicial,
%   cambiar la semilla permite alcanzar configuraciones distintas del mismo
%   efector final, es decir, todas las soluciones de la cinematica inversa de
%   posicion que el robot admite.
%
%   Solo se utiliza la parte lineal del Jacobiano, por lo que la orientacion
%   del efector final queda libre dentro de ese espacio nulo.
%
%   Ante singularidad, lambda > 0 sustituye la pseudoinversa por la solucion
%   amortiguada (Levenberg-Marquardt)
%   J_v^+ = (J_v J_v^T + lambda^2 I)^-1 J_v^T.
%
%   Nota sobre la forma de los vectores: dq se calcula como columna y se
%   reordena con reshape antes de sumarlo a q. Sumarlo directamente produciria
%   una expansion implicita de 1 x n + n x 1 a n x n.
%
%   IMPORTANTE: 'robot' debe ser la estructura devuelta por puma_param(), no un
%   objeto SerialLink. Si se entrega un SerialLink como primer argumento, MATLAB
%   despacha la llamada al metodo SerialLink.ikine() del Robotics Toolbox en
%   lugar de entrar a esta funcion. Ese metodo resuelve la pose completa, exige
%   un numero de argumentos distinto e ignora por completo el campo 'opts', de
%   modo que la llamada no produce ningun error y devuelve una solucion
%   distinta a la pretendida. Para comprobar que se esta usando esta
%   implementacion:
%
%       which('ikine')
%       assert(strcmp(class(robot), 'struct'))

    if nargin < 3
        error('ikine:args', 'Se requieren las entradas robot, T y q_inicial.');
    end

    if nargin < 4 || isempty(opts)
        opts = struct();
    end
    opts = local_defaults(opts);

    if ~isstruct(robot) || ~isfield(robot, 'DH')
        error('ikine:badrobot', ...
            'robot debe ser la estructura devuelta por puma_param().');
    end

    DH = robot.DH;
    n = size(DH, 1);

    base = eye(4);
    if isfield(robot, 'base') && ~isempty(robot.base)
        base = robot.base;
    end

    % Numero de puntos de la trayectoria 
    if ndims(T) <= 2
        N = 1;
    else
        N = size(T, 3);
    end

    % Semillas 
    if isvector(q_inicial)
        q_in = q_inicial(:)';
        if length(q_in) ~= n
            error('ikine:badq', ...
                'q_inicial debe contener %d valores (uno por articulacion).', n);
        end
        % Una sola semilla, encadenada de un punto al siguiente.
        semillas = zeros(n, N);
        semillas(:, 1) = q_in;
        for k = 2:N
            semillas(:, k) = semillas(:, k - 1);
        end
    else
        if size(q_inicial, 1) ~= N || size(q_inicial, 2) ~= n
            error('ikine:badq', ...
                ['q_inicial debe ser 1 x %d o %d x %d (una semilla por ' ...
                 'punto de la trayectoria).'], n, n, N);
        end
        semillas = q_inicial';
    end

    q_out          = zeros(N, n);
    info.iterations = zeros(N, 1);
    info.error      = zeros(N, 1);
    info.converged  = false(N, 1);
    info.sigma      = zeros(N, min(3, n));
    info.null_dim   = zeros(N, 1);

    for k = 1:N
        p_objetivo = reshape(T(1:3, 4, k), 3, 1);

        q_act     = semillas(:, k)';
        q_ref     = q_act;               % postura objetivo = semilla
        iter      = 0;
        converged = false;
        err       = Inf;

        for it = 1:opts.max_iter
            iter = it;

            [~, p] = homogeneous_matrix(DH, q_act, base);
            e   = p_objetivo - p(:, n);
            err = norm(e);

            if err <= opts.tol
                converged = true;
                break
            end

            J  = geometric_jacobian(robot, DH, q_act);
            Jv = J(1:3, :);

            if opts.lambda > 0
                Jp = (Jv * Jv' + opts.lambda^2 * eye(3)) \ Jv';
            else
                Jp = pinv(Jv);
            end

            % Componente de norma minima: desplaza el efector final.
            dq = Jp * e;

            % Componente de auto-movimiento en el espacio nulo de J_v.
            if opts.k_post > 0
                Nn = eye(n) - Jp * Jv;
                dq = dq + opts.k_post * (Nn * (q_ref(:) - q_act(:)));
            end

            % Limite de paso para evitar divergencias.
            paso = norm(dq);
            if opts.max_step > 0 && paso > opts.max_step
                dq = dq * (opts.max_step / paso);
            end

            q_act = q_act + reshape(dq, 1, []);

            if ~isempty(opts.q_lim)
                q_act = min(max(q_act, opts.q_lim(1, :)), opts.q_lim(2, :));
            end
        end

        if ~converged
            [~, p] = homogeneous_matrix(DH, q_act, base);
            err = norm(p_objetivo - p(:, n));
        end

        Jv = geometric_jacobian(robot, DH, q_act);
        Jv = Jv(1:3, :);
        s  = svd(Jv);

        q_out(k, :)        = q_act;
        info.iterations(k) = iter;
        info.error(k)      = err;
        info.converged(k)  = converged;
        info.sigma(k, :)   = s(:)';
        info.null_dim(k)   = n - rank(Jv);
    end

    if N == 1
        q = reshape(q_out, 1, []);
    else
        q = q_out;
    end
end

% A

function o = local_defaults(o)
    def.tol      = 1e-10;
    def.max_iter = 200;
    def.lambda   = 0;
    def.k_post   = 1;
    def.max_step = 0.5;
    def.q_lim    = [];

    campos = fieldnames(def);
    for i = 1:numel(campos)
        if ~isfield(o, campos{i}) || isempty(o.(campos{i}))
            o.(campos{i}) = def.(campos{i});
        end
    end
end