function [tau1, tau2] = carga_mecanica()
%carga_mecanica  Carga mecanica de P8 para el modelo de P10.
%
%   [tau1, tau2] = carga_mecanica()
%
%   Reproduce la trayectoria de P7 y calcula, con la dinamica inversa de P8
%   (ID), el par que debe entregar cada articulacion. Devuelve dos series de
%   tiempo listas para las entradas tau1 y tau2 de motor_2gdl.slx.
%
%   Se usa la misma trayectoria, el mismo modelo dinamico y la misma fuerza
%   externa del lapiz que en P8. Si P7 cambia, este archivo debe actualizarse
%   con la nueva trayectoria.

    aqui = fileparts(mfilename('fullpath'));
    addpath(aqui);
    addpath(fullfile(aqui, '..', 'dinamica'));
    addpath(fullfile(aqui, '..', 'trayectorias'));
    addpath(fullfile(aqui, '..', 'lib'));
    addpath(fullfile(aqui, '..', 'cinematica'));
    addpath(fullfile(aqui, '..', 'params'));
    add_rvc_path();

    [robot, PARAM] = plotter_param();
    DH = robot.DH;

    % Trayectoria de P7 (Reuleaux, 10 cm, centro (0, 0.20) m)
    [wx, wy] = trayectoria_reuleaux(0.10, [0 0.20], 48);
    W = [wx; wy];
    [tv, r, rd, rdd] = trayectoria_polinomios(W, 20, 0.01, 3);
    M = numel(tv);

    % Cinematica inversa y derivadas articulares
    Tarr = repmat(eye(4), 1, 1, M);
    Tarr(1,4,:) = r(1,:);
    Tarr(2,4,:) = r(2,:);
    [q, info] = ikine(robot, Tarr, [0.7 1.6]);
    if sum(info.converged) ~= M
        error('carga_mecanica:ikine', ...
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

    % Dinamica inversa de P8
    grav  = [0 0 -9.81];
    F_ext = [0 0 -0.294 0 0 0]';   % 30 g del lapiz, fuerza normal al papel
    tau = ID(DH, PARAM, q, qd', qdd', grav, F_ext, robot.dyn.I);

    tau1 = timeseries(tau(:,1), tv);
    tau2 = timeseries(tau(:,2), tv);
    tau1.Name = 'tau1';
    tau2.Name = 'tau2';

    fprintf('Carga de P8 para P10:\n');
    fprintf('  tau1: max = %.4f mN.m, RMS = %.4f mN.m\n', ...
        1e3*max(abs(tau(:,1))), 1e3*rms(tau(:,1)));
    fprintf('  tau2: max = %.4f mN.m, RMS = %.4f mN.m\n', ...
        1e3*max(abs(tau(:,2))), 1e3*rms(tau(:,2)));
end
