% cinematica_directa.m
% Punto 5: cinematica directa del plotter de 2 GdL.
%
% Contenido:
%   1. Derivacion de las matrices de transformacion homogenea y del producto
%      acumulativo T_0^n (convencion estandar de Denavit-Hartenberg).
%   2. Evaluacion numerica, verificacion del producto de matrices y validacion
%      contra el Robotics Toolbox de Peter Corke (SerialLink.fkine).
%   3. Figura vectorial del esquema (marcos DH, brazo y volumen de trabajo).
%
% Requiere: plotter_param.m, forward_kinematics.m, homogeneous_matrix.m y, para
% la validacion, add_rvc_path.m (Robotics Toolbox opcional).

clear;
clc;
close all;

aqui = fileparts(mfilename('fullpath'));
addpath(fullfile(aqui, '..', 'lib'));
addpath(fullfile(aqui, '..', 'params'));
add_rvc_path();

figdir = fullfile(aqui, '..', '..', 'docs', 'informe-1', 'figuras');
if ~exist(figdir, 'dir'); mkdir(figdir); end

fprintf('=========================================================\n');
fprintf(' PUNTO 5 - CINEMATICA DIRECTA (Plotter 2 GdL)\n');
fprintf('=========================================================\n\n');

[robot, PARAM] = plotter_param();
DH = robot.DH;
n  = robot.n;
L1 = DH(1,3);
L2 = DH(2,3);
d1 = DH(1,2);
d2 = DH(2,2);
base = robot.base;              % {W} -> {0}

q_test = deg2rad([60 -50]);

fprintf('Parametros DH [theta_offset d a alpha sigma]:\n');
disp(DH);

% ---------------------------------------------------------------- 1. DERIVACION
fprintf('--- 1. Transformaciones homogeneas ------------------------------\n');
fprintf(['Matriz DH generica (convencion estandar):\n' ...
         '  A_i = [ c_i  -s_i*cA   s_i*sA   a_i*c_i ;\n' ...
         '          s_i   c_i*cA  -c_i*sA   a_i*s_i ;\n' ...
         '           0     sA       cA       d_i   ;\n' ...
         '           0     0        0        1      ]\n']);
fprintf('con c_i=cos(q_i+off), s_i=sin(q_i+off), sA=sin(alpha_i), cA=cos(alpha_i).\n');
fprintf('\nCon alpha_1 = alpha_2 = 0 (robot plano):\n');
fprintf('  A_1 = [ c1 -s1 0 L1*c1 ; s1 c1 0 L1*s1 ; 0 0 1 d1 ; 0 0 0 1 ]\n');
fprintf('  A_2 = [ c2 -s2 0 L2*c2 ; s2 c2 0 L2*s2 ; 0 0 1 d2 ; 0 0 0 1 ]\n');
fprintf('\nProducto acumulativo T_0_2 = A_1*A_2 (marco {0}):\n');
fprintf('  T_0_2 = [ c12 -s12 0 L1*c1 + L2*c12 ;\n');
fprintf('            s12  c12 0 L1*s1 + L2*s12 ;\n');
fprintf('              0    0 1         d1 + d2 ;\n');
fprintf('              0    0 0               1 ]\n');
fprintf('\nMarco mundo {W} -> {0} = trasl(0, 0, %.4f)\n', base(3,4));
fprintf('Cinematica directa (posicion del efector final en {W}):\n');
fprintf('  x = L1*cos(q1) + L2*cos(q1 + q2)\n');
fprintf('  y = L1*sin(q1) + L2*sin(q1 + q2)\n');
fprintf('  z = 0 m sobre el papel   (h0 + d1 + d2 = %.4f)\n', base(3,4) + d1 + d2);

% Verificacion del producto de matrices frente a forward_kinematics()
T_prod = dh_matrix(DH(1,:), q_test(1)) * dh_matrix(DH(2,:), q_test(2));
T_fk   = forward_kinematics(DH, q_test);
fprintf('\nmax |A_1*A_2 - forward_kinematics| = %.3e\n', max(abs(T_prod(:) - T_fk(:))));

% ------------------------------------------------------------------ 2. NUMERICO
fprintf('\n--- 2. Validacion numerica -------------------------------------\n');
p_num = base * T_fk;
p_num = p_num(1:3,4);
fprintf('q_test = [%.4f %.4f] rad (%.0f, %.0f deg)\n', q_test, q_test*180/pi);
fprintf('FK propia      p = [%.8f %.8f %.8f] m\n', p_num);

p_an = [L1*cos(q_test(1)) + L2*cos(sum(q_test));
        L1*sin(q_test(1)) + L2*sin(sum(q_test));
        0];
fprintf('FK analitica   p = [%.8f %.8f %.8f] m\n', p_an);
fprintf('max |num - analitica| = %.3e m\n', max(abs(p_num - p_an)));

if ~isempty(robot.link)
    Tk = robot.link.fkine(q_test);
    pk = Tk.t;
    fprintf('Toolbox fkine  p = [%.8f %.8f %.8f] m\n', pk);
    fprintf('max |propia - Toolbox| = %.3e m\n', max(abs(p_num - pk)));
else
    fprintf('(Robotics Toolbox no disponible: se omite la comparacion)\n');
end

fprintf('\nTabla de cinematica directa:\n');
fprintf('   q1[deg]  q2[deg]      x[m]      y[m]      z[m]\n');
Qtab = [0 0; 45 -60; 90 -90; 120 45; -30 120];
for i = 1:size(Qtab,1)
    qq = deg2rad(Qtab(i,:));
    p = base * forward_kinematics(DH, qq);
    p = p(1:3,4);
    fprintf('  %7.1f %8.1f  %8.4f  %8.4f  %8.4f\n', ...
            qq(1)*180/pi, qq(2)*180/pi, p(1), p(2), p(3));
end

% ---------------------------------------------------------------- 3. FIGURA
fprintf('\n--- 3. Figura --------------------------------------------------\n');

Rmin = L1 - L2;
Rmax = L1 + L2;

q_brazo = deg2rad([60 -50]);
[R, pj] = homogeneous_matrix(DH, q_brazo, base);
p_all = [zeros(3,1), pj];
r_all = cat(3, eye(3), R);

f = figure('Color', 'w', 'Position', [100 100 1000 900]);
hold on; axis equal; grid on; box on;

ang = linspace(0, 2*pi, 400);
hmax = plot(Rmax*cos(ang), Rmax*sin(ang), '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);
hmin = plot(Rmin*cos(ang), Rmin*sin(ang), '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1);

[tx, ty] = reuleaux(0.10, [0 0.20]);
hreul = plot(tx, ty, '-', 'Color', [0.20 0.60 0.90], 'LineWidth', 1.6);

base_pt = p_all(:,1);
codo  = p_all(:,2);
punta = p_all(:,3);
hlink = plot([base_pt(1) codo(1)], [base_pt(2) codo(2)], '-', 'Color', [0.10 0.10 0.10], 'LineWidth', 3);
plot([codo(1) punta(1)], [codo(2) punta(2)], '-', 'Color', [0.10 0.10 0.10], 'LineWidth', 3);

esc = 0.03;
for i = 1:(n+1)
    o = p_all(:,i);
    xa = r_all(:,1,i);
    ya = r_all(:,2,i);
    quiver(o(1), o(2), xa(1)*esc, xa(2)*esc, 0, 'Color', [0.85 0.15 0.15], 'LineWidth', 1.3, 'MaxHeadSize', 3);
    quiver(o(1), o(2), ya(1)*esc, ya(2)*esc, 0, 'Color', [0.15 0.55 0.15], 'LineWidth', 1.3, 'MaxHeadSize', 3);
end

plot(base_pt(1), base_pt(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.1 0.1 0.1], 'MarkerEdgeColor', 'k');
plot(codo(1), codo(2), 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.1 0.1 0.1], 'MarkerEdgeColor', 'k');
hpen = plot(punta(1), punta(2), 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.20 0.60 0.90], 'MarkerEdgeColor', 'k');

xlabel('x [m]'); ylabel('y [m]');
title({'Cinematica directa del plotter 2 GdL', ...
       'Marcos DH y espacio de trabajo'});
legend([hmax hreul hlink hpen], ...
    {'Limite de trabajo', 'Triangulo de Reuleaux', 'Eslabones', 'Efector final'}, ...
    'Location', 'southoutside', 'Orientation', 'horizontal', 'NumColumns', 2);
xlim([-Rmax*1.05 Rmax*1.05]);

% Fondo claro y fuente >= 24 pt (independiente del tema de MATLAB) para el informe
ax = gca;
set(ax, 'Color', 'w', 'XColor', [0 0 0], 'YColor', [0 0 0], 'GridColor', [0.75 0.75 0.75], 'FontSize', 26);
ax.Title.FontSize = 26;
lg = findobj(f, 'Type', 'Legend');
if ~isempty(lg)
    set(lg, 'Color', 'w', 'TextColor', [0 0 0], 'EdgeColor', [0.7 0.7 0.7], 'FontSize', 26);
end

print(f, '-depsc2', '-r300', fullfile(figdir, 'cinematica_directa_esquema.eps'));
fprintf('Figura guardada: cinematica_directa_esquema.eps\n');

fprintf('\ncinematica_directa.m: completado.\n');


% --------------------------------------------------------------- FUNCIONES
function A = dh_matrix(DHrow, q)
%dh_matrix  Matriz DH estandar de un eslabon para la posicion articular q.
    th = q + DHrow(1);
    d  = DHrow(2);
    a  = DHrow(3);
    al = DHrow(4);
    A = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al)  a*cos(th);
         sin(th)  cos(th)*cos(al) -cos(th)*sin(al)  a*sin(th);
         0        sin(al)          cos(al)          d;
         0        0                0                1];
end

function [x, y] = reuleaux(w, C)
%reuleaux  Triangulo de Reuleaux de ancho constante w, centro C.
%   Tres arcos de radio w, cada uno centrado en el vertice opuesto.
    Rc = w / sqrt(3);
    ang = deg2rad([90 210 330]);
    V = C(:) + Rc * [cos(ang); sin(ang)];
    x = []; y = [];
    for k = 1:3
        c  = V(:, k);
        a1 = V(:, mod(k, 3) + 1);
        a2 = V(:, mod(k + 1, 3) + 1);
        th1 = atan2(a1(2) - c(2), a1(1) - c(1));
        th2 = atan2(a2(2) - c(2), a2(1) - c(1));
        dth = mod(th2 - th1 + pi, 2*pi) - pi;
        th  = th1 + linspace(0, dth, 60);
        x = [x, c(1) + w*cos(th)];
        y = [y, c(2) + w*sin(th)];
    end
    x = [x, x(1)];
    y = [y, y(1)];
end
