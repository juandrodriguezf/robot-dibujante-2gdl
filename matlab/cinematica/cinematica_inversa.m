% cinematica_inversa.m
% Punto 6: cinematica inversa del plotter de 2 GdL por el metodo del Jacobiano.
%
% Contenido:
%   1. Jacobiano geometrico: verificacion analitica y por diferencias finitas.
%   2. Cinematica inversa en forma cerrada (las dos ramas: codo arriba/abajo).
%   3. Cinematica inversa iterativa con ikine() (metodo del Jacobiano) desde
%      distintas semillas -> recupera todas las configuraciones del mismo punto.
%   4. Validacion (error de posicion, reconfiguracion por FK y comparacion con
%      SerialLink.fkine/ikine).
%   5. Analisis de singularidad (det J y numero de condicion) en el espacio de
%      trabajo.
%   6. Figuras vectoriales.
%
% Requiere: plotter_param.m, forward_kinematics.m, homogenous_matrix.m,
% Jacobian.m, ikine.m y add_rvc_path.m (Robotics Toolbox opcional).

clear;
clc;
close all;

aqui = fileparts(mfilename('fullpath'));
addpath(fullfile(aqui, '..', 'lib'));
add_rvc_path();

figdir = fullfile(aqui, '..', '..', 'docs', 'informe-1', 'figuras');
if ~exist(figdir, 'dir'); mkdir(figdir); end

fprintf('=========================================================\n');
fprintf(' PUNTO 6 - CINEMATICA INVERSA POR JACOBIANO (Plotter 2 GdL)\n');
fprintf('=========================================================\n\n');

[robot, PARAM] = plotter_param();
DH = robot.DH;
n  = robot.n;
L1 = DH(1,3);
L2 = DH(2,3);
Rmin = abs(L1 - L2);
Rmax = L1 + L2;

% ------------------------------------------------ 1. JACOBIANO
fprintf('--- 1. Jacobiano geometrico ------------------------------------\n');
q_v = deg2rad([30 -70]);
J   = Jacobian(robot, DH, q_v);
Jv  = J(1:2, :);          % parte lineal planar (la fila z es nula)

q1 = q_v(1); q2 = q_v(2);
Jv_an = [-L1*sin(q1) - L2*sin(q1+q2), -L2*sin(q1+q2);
          L1*cos(q1) + L2*cos(q1+q2),  L2*cos(q1+q2)];

h = 1e-7; Jv_fd = zeros(2, n);
for k = 1:n
    dk = zeros(1, n); dk(k) = h;
    pa = forward_kinematics(DH, q_v + dk);
    pb = forward_kinematics(DH, q_v - dk);
    Jv_fd(:, k) = (pa(1:2,4) - pb(1:2,4)) / (2*h);
end

fprintf('q_v = [%.1f %.1f] deg\n', q_v*180/pi);
fprintf('max |Jv_propio - Jv_analitico|      = %.3e\n', max(abs(Jv - Jv_an), [], 'all'));
fprintf('max |Jv_propio - Jv_dif_finitas|    = %.3e\n', max(abs(Jv - Jv_fd), [], 'all'));
fprintf('det(Jv) = %.6f   (L1*L2*sin(q2) = %.6f)\n', det(Jv), L1*L2*sin(q2));
fprintf('Los ceros de det(Jv) ocurren en q2 = 0 y q2 = 180 deg (singularidades).\n');

% ------------------------------------------------ 2. IK FORMA CERRADA
fprintf('\n--- 2. Cinematica inversa (forma cerrada, 2R plano) ------------\n');
p_obj = [0.08; 0.20];
Qcf = ik_2r(p_obj, L1, L2);
fprintf('p_objetivo = [%.4f %.4f] m   |p| = %.4f m\n', p_obj, norm(p_obj));
fprintf('  rama A (codo arriba): q = [%.4f %.4f] rad  (%.1f, %.1f deg)\n', Qcf(1,:), Qcf(1,:)*180/pi);
fprintf('  rama B (codo abajo) : q = [%.4f %.4f] rad  (%.1f, %.1f deg)\n', Qcf(2,:), Qcf(2,:)*180/pi);

% ------------------------------------------------ 3. IK ITERATIVA (JACOBIANO)
fprintf('\n--- 3. Cinematica inversa iterativa ikine() ---------------------\n');
Tobj = eye(4); Tobj(1:2,4) = p_obj;

% Semillas que conducen a cada rama (signo de q2 distinto)
semillas = [deg2rad([10  10]);    % -> rama A
            deg2rad([10 -10]);    % -> rama B
            deg2rad([0    0]);    % caso de partida simple
            deg2rad([90  90])];   % otra semilla
Qik = zeros(size(semillas));
info_iter = zeros(size(semillas,1),1);
info_err  = zeros(size(semillas,1),1);

fprintf('  semilla [deg]            q resultante [deg]        iter    error[m]\n');
for i = 1:size(semillas,1)
    [qi, infi] = ikine(robot, Tobj, semillas(i,:));
    Qik(i,:) = qi;
    info_iter(i) = infi.iterations;
    info_err(i)  = infi.error;
    fprintf('  [%6.1f %6.1f]  ->  [%7.2f %7.2f]   %5d   %.2e\n', ...
        semillas(i,:)*180/pi, qi*180/pi, infi.iterations, infi.error);
end

% Reconfiguracion por cinematica directa (validacion de posicion)
fprintf('\n  Validacion por cinematica directa (posicion alcanzada):\n');
for i = 1:size(semillas,1)
    pk = forward_kinematics(DH, Qik(i,:));
    fprintf('    semilla %d -> p = [%.8f %.8f], error = %.2e m\n', ...
        i, pk(1:2,4), norm(pk(1:2,4) - p_obj));
end

% ------------------------------------------------ 4. VALIDACION CON TOOLBOX
fprintf('\n--- 4. Validacion con el Robotics Toolbox ----------------------\n');
if ~isempty(robot.link)
    Tk = robot.link.fkine(Qik(1,:));
    fprintf('  max |fkine(q_ik) - p_objetivo| = %.3e m\n', max(abs(Tk.t(1:2) - p_obj)));
    try
        q_tb = robot.link.ikine(Tobj, 'q0', semillas(1,:), 'mask', [1 1 0 0 0 0]);
        fprintf('  SerialLink.ikine() -> q = [%.4f %.4f] deg\n', q_tb*180/pi);
        fprintf('  max |q_ikine - q_Toolbox| = %.3e rad\n', max(abs(Qik(1,:) - q_tb)));
    catch err
        fprintf('  (SerialLink.ikine no aplicable: %s)\n', err.message);
    end
else
    fprintf('  (Robotics Toolbox no disponible: se omite)\n');
end

% Error frente a la forma cerrada
err_cf = min([max(abs(Qik - Qcf(1,:)), [], 2), max(abs(Qik - Qcf(2,:)), [], 2)], [], 2);
fprintf('  max |q_ikine - q_forma_cerrada| = %.3e rad\n', max(err_cf));

% ------------------------------------------------ 5. SINGULARIDAD
fprintf('\n--- 5. Singularidad en el espacio de trabajo -------------------\n');
% w(r) = |det Jv| = L1*L2*|sin q2|; q2 depende del radio r.
r = linspace(Rmin, Rmax, 300);
c2 = (r.^2 - L1^2 - L2^2) / (2*L1*L2);
c2 = min(max(c2, -1), 1);
q2r = acos(c2);
w = L1*L2*sin(q2r);
fprintf('  w(r) = |det Jv| es maximo en r = sqrt(L1^2+L2^2) = %.4f m\n', sqrt(L1^2+L2^2));
fprintf('  w -> 0 en los limites r = %.2f y r = %.2f m (singularidades)\n', Rmin, Rmax);

% ------------------------------------------------ 6. FIGURAS
fprintf('\n--- 6. Figuras -------------------------------------------------\n');

% Figura 1: las dos configuraciones para el mismo punto
f1 = figure('Color', 'w', 'Position', [100 100 760 620]);
hold on; axis equal; grid on; box on;
ang = linspace(0, 2*pi, 400);
hMax = plot(Rmax*cos(ang), Rmax*sin(ang), '--', 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
hMin = plot(Rmin*cos(ang), Rmin*sin(ang), '--', 'Color', [0.8 0.8 0.8], 'LineWidth', 1);
col = [0.20 0.45 0.95; 0.95 0.45 0.15];
hRama = gobjects(2,1);
for b = 1:2
    qb = Qcf(b,:);
    codo = [L1*cos(qb(1)), L1*sin(qb(1))];
    tip  = codo + [L2*cos(sum(qb)), L2*sin(sum(qb))];
    hRama(b) = plot([0 codo(1)], [0 codo(2)], '-', 'Color', col(b,:), 'LineWidth', 3);
    plot([codo(1) tip(1)], [codo(2) tip(2)], '-', 'Color', col(b,:), 'LineWidth', 3);
    plot([0 codo(1) tip(1)], [0 codo(2) tip(2)], 'o', 'MarkerSize', 7, ...
         'MarkerFaceColor', col(b,:), 'MarkerEdgeColor', 'k');
end
hStar = plot(p_obj(1), p_obj(2), 'p', 'MarkerSize', 16, 'MarkerFaceColor', [0.1 0.1 0.1], 'MarkerEdgeColor', 'k');
xlabel('x [m]'); ylabel('y [m]');
title('Cinematica inversa: dos configuraciones para el mismo punto');
legend([hMax hMin hRama(1) hRama(2) hStar], ...
       {'Limite exterior', 'Limite interior', 'Rama A (codo arriba)', ...
        'Rama B (codo abajo)', 'Objetivo'}, 'Location', 'southoutside', ...
        'Orientation', 'horizontal', 'NumColumns', 3);
xlim([-Rmax*1.05 Rmax*1.05]);
estilizar(f1);
print(f1, '-depsc2', '-r300', fullfile(figdir, 'cinematica_inversa_ramas.eps'));
fprintf('Figura guardada: cinematica_inversa_ramas.eps\n');

% Figura 2: manipulabilidad frente al radio
f2 = figure('Color', 'w', 'Position', [100 100 720 460]);
hold on; grid on; box on;
plot(r, w, '-', 'Color', [0.20 0.45 0.95], 'LineWidth', 2.5);
r_re = 0.20 + [-1 1]*0.10/sqrt(3);
yl = [0 1.05*max(w)];
plot(r_re, [yl(2) yl(2)], '-', 'Color', [0.85 0.55 0.15], 'LineWidth', 2);
text(mean(r_re), yl(2)*0.93, 'Rango radial del triangulo', ...
     'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.6 0.4 0.1]);
xlabel('Radio del efector final r [m]'); ylabel('|det J_v| = L_1 L_2 |sin q_2|');
title('Manipulabilidad frente al radio (singularidad en los limites)');
xlim([Rmin Rmax]); ylim(yl);
estilizar(f2);
print(f2, '-depsc2', '-r300', fullfile(figdir, 'cinematica_inversa_singularidad.eps'));
fprintf('Figura guardada: cinematica_inversa_singularidad.eps\n');

fprintf('\ncinematica_inversa.m: completado.\n');


% --------------------------------------------------------------- FUNCIONES
function Q = ik_2r(p, L1, L2)
%ik_2r  Cinematica inversa analitica del robot 2R plano. Devuelve las dos ramas.
    x = p(1); y = p(2);
    c2 = (x^2 + y^2 - L1^2 - L2^2) / (2*L1*L2);
    c2 = min(max(c2, -1), 1);
    q2a = acos(c2);
    q2b = -acos(c2);
    q1a = atan2(y, x) - atan2(L2*sin(q2a), L1 + L2*cos(q2a));
    q1b = atan2(y, x) - atan2(L2*sin(q2b), L1 + L2*cos(q2b));
    Q = [q1a q2a; q1b q2b];
end

function estilizar(fig)
%estilizar  Fondo claro e independiente del tema de MATLAB.
    ax = findall(fig, 'Type', 'axes');
    set(ax, 'Color', 'w', 'XColor', [0 0 0], 'YColor', [0 0 0], ...
            'GridColor', [0.75 0.75 0.75], 'LineWidth', 1.0);
    lg = findobj(fig, 'Type', 'Legend');
    if ~isempty(lg)
        set(lg, 'Color', 'w', 'TextColor', [0 0 0], 'EdgeColor', [0.7 0.7 0.7]);
    end
end
