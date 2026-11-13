function [tv, q, qd, qdd] = trayectoria_articular()
%trayectoria_articular  Trayectoria articular de P7 (referencia del control P11).
%
%   [tv, q, qd, qdd] = trayectoria_articular()
%
%   Reproduce la trayectoria Reuleaux de P7 (10 cm, centro (0, 0.20) m,
%   T = 20 s, dt = 0.01 s) y entrega posicion, velocidad y aceleracion
%   articulares. Es la misma primera parte de carga_mecanica.m.
%
%   Salidas: tv (M x 1) [s]; q, qd, qdd (M x 2) [rad], [rad/s], [rad/s^2].

    aqui = fileparts(mfilename('fullpath'));
    addpath(fullfile(aqui, '..', 'lib'));
    addpath(fullfile(aqui, '..', 'params'));
    add_rvc_path();

    [robot, ~] = plotter_param();
    DH = robot.DH;

    [wx, wy] = trayectoria_reuleaux(0.10, [0 0.20], 48);
    W = [wx; wy];
    [tv, r, rd, rdd] = trayectoria_polinomios(W, 20, 0.01, 3);
    M = numel(tv);

    Tarr = repmat(eye(4), 1, 1, M);
    Tarr(1,4,:) = r(1,:);
    Tarr(2,4,:) = r(2,:);
    [q, info] = ikine(robot, Tarr, [0.7 1.6]);
    if sum(info.converged) ~= M
        error('trayectoria_articular:ikine', ...
            'La cinematica inversa no convergio en %d de %d puntos.', ...
            M - sum(info.converged), M);
    end

    Jv = zeros(2, 2, M);
    for k = 1:M
        J = geometric_jacobian(robot, DH, q(k,:));
        Jv(:,:,k) = J(1:2,:);
    end

    qd  = zeros(2, M);
    qdd = zeros(2, M);
    for k = 1:M
        qd(:,k) = Jv(:,:,k) \ rd(:,k);
    end
    for k = 1:M
        if k == 1
            Jd = (Jv(:,:,2) - Jv(:,:,1)) / 0.01;
        elseif k == M
            Jd = (Jv(:,:,M) - Jv(:,:,M-1)) / 0.01;
        else
            Jd = (Jv(:,:,k+1) - Jv(:,:,k-1)) / 0.02;
        end
        qdd(:,k) = Jv(:,:,k) \ (rdd(:,k) - Jd*qd(:,k));
    end

    tv  = tv(:);
    qd  = qd';
    qdd = qdd';
end
