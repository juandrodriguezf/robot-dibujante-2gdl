% trayectoria_test.m
% Punto 7: planeacion de trayectorias del plotter de 2 GdL.
%
% Contenido:
%   1. Trayectoria cartesiana del triangulo de Reuleaux.
%   2. Interpolacion cartesiana con splines cubicos -> r(t), r_dot(t), r_ddot(t).
%   3. Trayectorias articulares por cinematica inversa -> q(t), q_dot(t), q_ddot(t)
%      (q_dot y q_ddot via el Jacobiano: q_dot = Jv^-1 r_dot,
%       q_ddot = Jv^-1 (r_ddot - Jdot q_dot)).
%   4. Validacion (FK recompuesta y toolbox) y velocidades/aceleraciones maximas
%      frente al limite de los servos.
%   5. Figuras vectoriales (fuente >= 26 pt).
%
% Requiere: plotter_param.m, forward_kinematics.m, homogenous_matrix.m,
% Jacobian.m, ikine.m, trayectoria_reuleaux.m, trayectoria_polinomios.m,
% add_rvc_path.m.

clear;
clc;
close all;

aqui = fileparts(mfilename('fullpath'));
addpath(fullfile(aqui, '..', 'lib'));
addpath(fullfile(aqui, '..', 'cinematica'));
add_rvc_path();

figdir = fullfile(aqui, '..', '..', 'docs', 'informe-1', 'figuras');
if ~exist(figdir, 'dir'); mkdir(figdir); end

fprintf('=========================================================\n');
fprintf(' PUNTO 7 - PLANEACION DE TRAYECTORIAS (Plotter 2 GdL)\n');
fprintf('=========================================================\n\n');

[robot, PARAM] = plotter_param();
DH = robot.DH;
n  = robot.n;
L1 = DH(1,3);
L2 = DH(2,3);
Rmin = abs(L1 - L2);
Rmax = L1 + L2;

% ------------------------------------------------ 1-2. TRAYECTORIA CARTESIANA
fprintf('--- 1-2. Trayectoria cartesiana e interpolacion ----------------\n');
[wx, wy] = trayectoria_reuleaux(0.10, [0 0.20], 48);   % ancho 10 cm, centro (0,20) cm
W  = [wx; wy];
T  = 20;                                         % duracion total [s]
dt = 0.01;                                       % paso de muestreo [s] (100 Hz)

[tv, r, rd, rdd] = trayectoria_polinomios(W, T, dt, 3);   % 3 lados: reposo en los vertices
M = numel(tv);
fprintf('Puntos de paso          : %d\n', size(W,2));
fprintf('Duracion / muestreo     : %.0f s  /  dt = %.3f s  (%d muestras)\n', T, dt, M);
fprintf('Velocidad cartesiana max: %.4f m/s (v -> 0 en los 3 vertices)\n', max(hypot(rd(1,:), rd(2,:))));
fprintf('Aceleracion cart. max   : %.4f m/s^2\n', max(hypot(rdd(1,:), rdd(2,:))));

% ------------------------------------------------ 3. CINEMATICA INVERSA
fprintf('\n--- 3. Trayectorias articulares por cinematica inversa --------\n');
Tarr = repmat(eye(4), 1, 1, M);
Tarr(1,4,:) = r(1,:);
Tarr(2,4,:) = r(2,:);
[q, info] = ikine(robot, Tarr, [0.7 1.6]);        % semilla rama A, encadenada
fprintf('Convergencia            : %d de %d puntos\n', sum(info.converged), M);
fprintf('Error de posicion max   : %.3e m\n', max(info.error));
fprintf('Iteraciones media/max   : %.1f / %d\n', mean(info.iterations), max(info.iterations));

% Velocidades y aceleraciones articulares via el Jacobiano
Jv = zeros(2,2,M);
for k = 1:M
    J = Jacobian(robot, DH, q(k,:));
    Jv(:,:,k) = J(1:2,:);
end
qd  = zeros(2,M);
qdd = zeros(2,M);
for k = 1:M
    qd(:,k) = Jv(:,:,k) \ rd(:,k);
end
for k = 1:M
    if k == 1
        Jd = (Jv(:,:,2)   - Jv(:,:,1)  ) / dt;
    elseif k == M
        Jd = (Jv(:,:,M)   - Jv(:,:,M-1)) / dt;
    else
        Jd = (Jv(:,:,k+1) - Jv(:,:,k-1)) / (2*dt);
    end
    qdd(:,k) = Jv(:,:,k) \ (rdd(:,k) - Jd*qd(:,k));
end

% ------------------------------------------------ 4. VALIDACION
fprintf('\n--- 4. Validacion ---------------------------------------------\n');
Tfk = forward_kinematics(DH, q);
pk  = reshape(Tfk(1:2,4,:), 2, M);
err = max(vecnorm(pk - r, 2, 1));
fprintf('max |FK(q) - r(t)| (propia)      : %.3e m\n', err);
if ~isempty(robot.link)
    Tk = robot.link.fkine(q(round(M/2),:));
    fprintf('SerialLink.fkine  (muestra media): [%.6f %.6f] m (objetivo [%.6f %.6f])\n', ...
        Tk.t(1:2), r(:,round(M/2)));
end

max_q   = max(abs(q),   [], 1);     % q  es M x 2
max_qd  = max(abs(qd),  [], 2)';    % qd es 2 x M
max_qdd = max(abs(qdd), [], 2)';    % qdd es 2 x M
w_servo = (60/0.19) * pi/180;    % Futaba S3003: 0.19 s / 60 deg -> rad/s
fprintf('\nLimite angular de los servos    : %.2f rad/s (0.19 s / 60 deg)\n', w_servo);
fprintf('  max |q1|, |q2|      = %.3f, %.3f rad\n',   max_q);
fprintf('  max |q1d|, |q2d|    = %.3f, %.3f rad/s\n', max_qd);
fprintf('  max |q1dd|, |q2dd|  = %.3f, %.3f rad/s^2\n', max_qdd);
if max(max_qd) <= w_servo
    fprintf('  -> las velocidades articulares estan dentro del limite del servo.\n');
else
    fprintf('  -> ATENCION: se supera la velocidad del servo.\n');
end

% ------------------------------------------------ 5. FIGURAS
fprintf('\n--- 5. Figuras -------------------------------------------------\n');
col = [0.20 0.45 0.95; 0.90 0.45 0.10];

% Figura 1: trayectoria cartesiana
f1 = figure('Color', 'w', 'Position', [80 80 1200 640]);
tiledlayout(f1, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile;
plot(tv, r(1,:), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(tv, r(2,:), '-', 'Color', col(2,:), 'LineWidth', 2.5);
grid on; box on;
xlabel('t [s]'); ylabel('posicion [m]');
title('Posicion cartesiana');
legend({'x(t)', 'y(t)'}, 'Location', 'best');

ax = nexttile;
hold on; grid on; box on;
ang = linspace(0, 2*pi, 400);
hExt = plot(Rmax*cos(ang), Rmax*sin(ang), '--', 'Color', [0.8 0.8 0.8]);
hInt = plot(Rmin*cos(ang), Rmin*sin(ang), '--', 'Color', [0.8 0.8 0.8]);
hVia = plot(W(1,:), W(2,:), ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
hTray = plot(r(1,:), r(2,:), '-', 'Color', [0.20 0.60 0.90], 'LineWidth', 2.5);
hIni = plot(r(1,1), r(2,1), 'o', 'MarkerSize', 10, 'MarkerFaceColor', [0.1 0.1 0.1], 'MarkerEdgeColor', 'k');
axis equal;
xlabel('x [m]'); ylabel('y [m]');
title('Trayectoria en el plano');
legend([hExt hInt hVia hTray hIni], ...
       {'Limite exterior', 'Limite interior', 'Via points', 'Interpolada', 'Inicio'}, ...
       'Location', 'south', 'Orientation', 'horizontal', 'NumColumns', 2);
estilizar(f1);
print(f1, '-depsc2', '-r300', fullfile(figdir, 'trayectoria_cartesiana.eps'));
fprintf('Figura guardada: trayectoria_cartesiana.eps\n');

% Figura 2: trayectorias articulares (posicion, velocidad, aceleracion)
f2 = figure('Color', 'w', 'Position', [80 80 900 1050]);
tiledlayout(f2, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile;
plot(tv, q(:,1), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(tv, q(:,2), '-', 'Color', col(2,:), 'LineWidth', 2.5);
grid on; box on; ylabel('q [rad]'); title('Posicion articular');
legend({'q_1', 'q_2'}, 'Location', 'best');

ax = nexttile;
plot(tv, qd(1,:), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(tv, qd(2,:), '-', 'Color', col(2,:), 'LineWidth', 2.5);
grid on; box on; ylabel('dq/dt [rad/s]'); title('Velocidad articular');
legend({'q_1', 'q_2'}, 'Location', 'best');

ax = nexttile;
plot(tv, qdd(1,:), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(tv, qdd(2,:), '-', 'Color', col(2,:), 'LineWidth', 2.5);
grid on; box on; ylabel('d^2q/dt^2 [rad/s^2]'); xlabel('t [s]');
title('Aceleracion articular');
legend({'q_1', 'q_2'}, 'Location', 'best');

estilizar(f2);
print(f2, '-depsc2', '-r300', fullfile(figdir, 'trayectoria_articular.eps'));
fprintf('Figura guardada: trayectoria_articular.eps\n');

fprintf('\ntrayectoria_test.m: completado.\n');


% --------------------------------------------------------------- FUNCIONES
function estilizar(fig)
%estilizar  Fondo claro y fuente >= 26 pt (independiente del tema de MATLAB).
    ax = findall(fig, 'Type', 'axes');
    set(ax, 'Color', 'w', 'XColor', [0 0 0], 'YColor', [0 0 0], ...
            'GridColor', [0.75 0.75 0.75], 'FontSize', 26);
    lg = findobj(fig, 'Type', 'Legend');
    if ~isempty(lg)
        set(lg, 'Color', 'w', 'TextColor', [0 0 0], 'EdgeColor', [0.7 0.7 0.7], ...
            'FontSize', 26);
    end
    tt = findall(fig, 'Type', 'axes');
    for a = tt'
        a.Title.FontSize = 26;
    end
end
