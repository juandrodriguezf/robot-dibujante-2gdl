% simulacion_plotter.m
% Simulacion animada del robot dibujante de 2 GdL trazando el triangulo de
% Reuleaux sobre el papel, con el modelo CAD real (mallas de SolidWorks).
%
% Calcula la trayectoria cartesiana (Reuleaux + splines), resuelve la cinematica
% inversa por el metodo del Jacobiano y anima el robot con plotter_cad.
% Exporta a docs/media/:
%       simulacion_plotter.mp4    (animacion completa)
%       simulacion_plotter.gif    (version reducida para compartir)
%
% Requiere: plotter_param.m, trayectoria_reuleaux.m, trayectoria_polinomios.m,
% forward_kinematics.m, ikine.m, plotter_cad.m y add_rvc_path.m.

clear;
clc;
close all;

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
addpath(fullfile(aqui, '..', 'lib'));
addpath(fullfile(aqui, '..', 'cinematica'));
add_rvc_path();

outdir = fullfile(aqui, '..', '..', 'docs', 'media');
if ~exist(outdir, 'dir'); mkdir(outdir); end

fprintf('=========================================================\n');
fprintf(' SIMULACION DEL PLOTTER 2 GdL (triangulo de Reuleaux)\n');
fprintf('=========================================================\n\n');

[robot, ~] = plotter_param();
DH = robot.DH;
L1 = DH(1,3);
L2 = DH(2,3);

% ------------------------------------------------ 1. TRAYECTORIA
[wx, wy] = trayectoria_reuleaux(0.10, [0 0.20], 48);
W  = [wx; wy];
T  = 20;        % duracion total [s]
dt = 0.01;      % paso de muestreo [s]
[tv, r, ~, ~] = trayectoria_polinomios(W, T, dt, 3);
M = numel(tv);

% ------------------------------------------------ 2. CINEMATICA INVERSA
Tarr = repmat(eye(4), 1, 1, M);
Tarr(1,4,:) = r(1,:);
Tarr(2,4,:) = r(2,:);
[q, info] = ikine(robot, Tarr, [0.7 1.6]);
fprintf('IK: %d de %d puntos, error max = %.2e m\n', sum(info.converged), M, max(info.error));

% ------------------------------------------------ 3. FIGURA
fig = figure('Color', 'w', 'Position', [40 40 1360 760]);
tl  = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax = nexttile;
G  = plotter_cad('crear', ax, q(1,:));
plot3(ax, W(1,:), W(2,:), 0.002*ones(1,size(W,2)), ':', 'Color', [0.7 0.7 0.7], 'LineWidth', 1.2);
hTrail = plot3(ax, nan, nan, nan, '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 3);
titulo = title(ax, '');

axXY = nexttile;
plot(axXY, W(1,:)*1000, W(2,:)*1000, ':', 'Color', [0.7 0.7 0.7]); hold(axXY, 'on');
hXY = plot(axXY, nan, nan, '-', 'Color', [0.85 0.10 0.10], 'LineWidth', 2.5);
hP  = plot(axXY, nan, nan, 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.1 0.1 0.1], 'MarkerEdgeColor', 'k');
axis(axXY, 'equal'); grid(axXY, 'on'); box(axXY, 'on');
xlim(axXY, 1000*[min(W(1,:)) max(W(1,:))] + [-15 15]);
ylim(axXY, 1000*[min(W(2,:)) max(W(2,:))] + [-15 15]);
xlabel(axXY, 'x [mm]'); ylabel(axXY, 'y [mm]');
title(axXY, 'Trazo en el plano');

estilizar(fig);

% ------------------------------------------------ 4. VIDEO
fps    = 30;
stride = max(1, round(M / (T*fps)));
outmp4 = fullfile(outdir, 'simulacion_plotter.mp4');
vw = VideoWriter(outmp4, 'MPEG-4');
vw.FrameRate = fps;
open(vw);

hfrac = [0.05 0.30 0.55 0.80];
hit   = 1 + stride*round((hfrac*(M-1))/stride);
P   = zeros(M, 3);
idx = 0;
fprintf('Animando %d cuadros (stride = %d)...\n', numel(1:stride:M), stride);
for k = 1:stride:M
    qk = q(k,:);
    plotter_cad('mover', G, qk);
    pk = robot.base * forward_kinematics(DH, qk);
    P(k,:) = pk(1:3,4)';

    set(hTrail, 'XData', P(1:k,1), 'YData', P(1:k,2), 'ZData', P(1:k,3) + 0.0008);
    set(hXY,    'XData', 1000*P(1:k,1), 'YData', 1000*P(1:k,2));
    set(hP,     'XData', 1000*P(k,1),   'YData', 1000*P(k,2));
    titulo.String = sprintf('t = %5.2f s', tv(k));

    drawnow limitrate;
    writeVideo(vw, getframe(fig));

    if ismember(k, hit)
        idx = idx + 1;
        exportgraphics(fig, fullfile(outdir, sprintf('sim_cuadro_%d.png', idx)), 'Resolution', 110);
    end
end
close(vw);
fprintf('Video guardado: %s\n', outmp4);

% ------------------------------------------------ 5. GIF (reducido)
gifstep = max(1, round(M / 120));   % ~120 cuadros
outgif  = fullfile(outdir, 'simulacion_plotter.gif');
for k = 1:gifstep:M
    plotter_cad('mover', G, q(k,:));
    pk = robot.base * forward_kinematics(DH, q(k,:));
    set(hTrail, 'XData', P(1:k,1), 'YData', P(1:k,2), 'ZData', P(1:k,3) + 0.0008);
    set(hXY,    'XData', 1000*P(1:k,1), 'YData', 1000*P(1:k,2));
    set(hP,     'XData', 1000*P(k,1),   'YData', 1000*P(k,2));
    titulo.String = sprintf('t = %5.2f s', tv(k));
    drawnow;
    fr = getframe(fig);
    [A, map] = rgb2ind(frame2im(fr), 256);
    if k == 1
        imwrite(A, map, outgif, 'gif', 'LoopCount', Inf, 'DelayTime', gifstep*dt);
    else
        imwrite(A, map, outgif, 'gif', 'WriteMode', 'append', 'DelayTime', gifstep*dt);
    end
end
fprintf('GIF guardado  : %s\n', outgif);
fprintf('\nsimulacion_plotter.m: completado.\n');


% --------------------------------------------------------------- FUNCIONES
function estilizar(fig)
%estilizar  Fondo claro y fuente >= 24 pt (independiente del tema de MATLAB).
    ax = findall(fig, 'Type', 'axes');
    set(ax, 'Color', 'w', 'XColor', [0 0 0], 'YColor', [0 0 0], ...
            'GridColor', [0.75 0.75 0.75], 'FontSize', 20);
    for a = ax'
        a.Title.FontSize = 22;
    end
end
