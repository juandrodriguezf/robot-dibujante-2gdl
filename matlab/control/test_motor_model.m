% test_motor_model.m
% Punto 10: verificacion del modelo de motor DC de primer orden y sensores.
%
%   Pruebas:
%       1. Escalon de 6 V y 3 V: velocidad de regimen y constante de tiempo.
%       2. Bloqueo: par y corriente de bloqueo.
%       3. Carga de P8: comparacion con y sin carga.
%       4. Sensores: error de posicion, corriente y velocidad.
%       5. Mapa par-velocidad con los puntos de operacion de P8.
%
%   Genera figuras EPS en docs/informe-2/figuras.
%
%   Requiere: motor_2gdl.slx, param_motor.m, param_sensores.m,
%             carga_mecanica.m.

clear;
clc;
close all;

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);

figdir = fullfile(aqui, '..', '..', 'docs', 'informe-2', 'figuras');
if ~exist(figdir, 'dir')
    mkdir(figdir);
end

mdl = 'motor_2gdl';
if ~exist(fullfile(aqui, [mdl '.slx']), 'file')
    build_motor_model();
end
cargado = bdIsLoaded(mdl);
if ~cargado
    load_system(mdl);
end

mo = param_motor();
se = param_sensores();

fprintf('=========================================================\n');
fprintf(' PUNTO 10 - VERIFICACION DEL MODELO DE MOTOR Y SENSORES\n');
fprintf('=========================================================\n\n');

% ------------------------------------------------ 1. ESCALONES 6 V Y 3 V
fprintf('--- 1. Escalones de tension ------------------------------------\n');
tt = [0; 0.0999; 0.1; 0.2];
y6 = simular(mdl, timeseries([0;0;6;6], tt), timeseries([0;0;0;0], tt), ...
             timeseries([0;0;0;0], tt), timeseries([0;0;0;0], tt), 0.2);
y3 = simular(mdl, timeseries([0;0;3;3], tt), timeseries([0;0;0;0], tt), ...
             timeseries([0;0;0;0], tt), timeseries([0;0;0;0], tt), 0.2);

w6 = y6.y1(end, 2);
w3 = y3.y1(end, 2);

k0   = find(y6.t >= 0.1, 1);
w63  = 0.632 * mo.w_noload;
k63  = k0 - 1 + find(y6.y1(k0:end, 2) >= w63, 1);
tau_est = y6.t(k63) - 0.1;

i6_peak = max(y6.y1(:, 3));
i3_peak = max(y3.y1(:, 3));

fprintf('omega sin carga (6 V) = %.4f rad/s (esperado %.4f)\n', w6, mo.w_noload);
fprintf('omega sin carga (3 V) = %.4f rad/s (esperado %.4f)\n', w3, mo.w_noload/2);
fprintf('constante de tiempo  = %.4f s (esperado %.4f)\n', tau_est, mo.tau_m);
fprintf('corriente pico 6 V   = %.4f A (esperado %.4f)\n', i6_peak, mo.i_stall);
fprintf('corriente pico 3 V   = %.4f A (esperado %.4f)\n', i3_peak, mo.i_stall/2);

% ------------------------------------------------ 2. BLOQUEO
fprintf('\n--- 2. Bloqueo (V = 6 V, carga = par de bloqueo) ---------------\n');
tts = [0; 0.02];
ys = simular(mdl, timeseries([6;6], tts), timeseries([0;0], tts), ...
             timeseries([mo.tau_stall; mo.tau_stall], tts), ...
             timeseries([0;0], tts), 0.02);
w_stall = ys.y1(end, 2);
i_stall = ys.y1(end, 3);
tau_stall = mo.Kt * i_stall;

fprintf('omega en bloqueo = %.4f rad/s (esperado 0)\n', w_stall);
fprintf('corriente        = %.4f A (esperado %.4f)\n', i_stall, mo.i_stall);
fprintf('par              = %.4f N.m (esperado %.4f)\n', tau_stall, mo.tau_stall);

% ------------------------------------------------ 3. CARGA DE P8
fprintf('\n--- 3. Carga mecanica de P8 -----------------------------------\n');
[tau1, tau2] = carga_mecanica();
yp = simular(mdl, timeseries([0;0;6;6], tt), timeseries([0;0;0;0], tt), ...
             tau1, tau2, 0.2);
w_load = yp.y1(end, 2);
dw = w_load - w6;

fprintf('omega con carga P8 = %.4f rad/s\n', w_load);
fprintf('cambio de velocidad = %+.4f rad/s (%.3f %%)\n', dw, 100*dw/w6);

% ------------------------------------------------ 4. SENSORES
fprintf('\n--- 4. Sensores -------------------------------------------------\n');
e_pos = y6.y1(:, 4) - y6.y1(:, 1);
e_cur = y6.y1(:, 6) - y6.y1(:, 3);
e_vel = y6.y1(:, 5) - y6.y1(:, 2);

fprintf('posicion : max|e| = %.5f rad, RMS = %.5f rad (res = %.5f rad)\n', ...
    max(abs(e_pos)), rms(e_pos), se.pos_res);
fprintf('corriente: max|e| = %.5f A,   RMS = %.5f A\n', ...
    max(abs(e_cur)), rms(e_cur));
fprintf('velocidad: max|e| = %.5f rad/s, RMS = %.5f rad/s\n', ...
    max(abs(e_vel)), rms(e_vel));

% ------------------------------------------------ 5. FIGURAS
fprintf('\n--- 5. Figuras -------------------------------------------------\n');
col = [0.20 0.45 0.95; 0.90 0.45 0.10];

% Figura 1: escalones de tension
f1 = figure('Color', 'w', 'Position', [60 60 1200 900]);
tiledlayout(f1, 2, 2, 'TileSpacing', 'compact', 'Padding', 'loose');

nexttile;
plot(y6.t, y6.y1(:,2), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(y3.t, y3.y1(:,2), '-', 'Color', col(2,:), 'LineWidth', 2.5);
yline(mo.w_noload, '--', 'Color', col(1,:), 'LineWidth', 1.5);
yline(mo.w_noload/2, '--', 'Color', col(2,:), 'LineWidth', 1.5);
grid on; box on;
xlabel('t [s]'); ylabel('\omega [rad/s]');
title('Velocidad angular');
legend({'6 V','3 V'}, 'Location', 'best');

nexttile;
plot(y6.t, y6.y1(:,2), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
yline(w63, '--k', '\tau_m', 'FontSize', 20);
xline(0.1 + mo.tau_m, '--k');
grid on; box on;
xlim([0.099 0.125]); ylim([0 1.05*mo.w_noload]);
xlabel('t [s]'); ylabel('\omega [rad/s]');
title('Detalle del transitorio (6 V)');

nexttile;
plot(y6.t, y6.y1(:,3), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(y3.t, y3.y1(:,3), '-', 'Color', col(2,:), 'LineWidth', 2.5);
grid on; box on;
xlabel('t [s]'); ylabel('i [A]');
title('Corriente');
legend({'6 V','3 V'}, 'Location', 'best');

nexttile;
plot(y6.t, y6.y1(:,1), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(y3.t, y3.y1(:,1), '-', 'Color', col(2,:), 'LineWidth', 2.5);
grid on; box on;
xlabel('t [s]'); ylabel('\theta [rad]');
title('Posicion angular');
legend({'6 V','3 V'}, 'Location', 'best');

estilizar(f1);
print(f1, '-depsc2', '-r300', fullfile(figdir, 'motor_escalon.eps'));
fprintf('Figura guardada: motor_escalon.eps\n');

% Figura 2: sensores
f2 = figure('Color', 'w', 'Position', [80 60 1200 1100]);
tiledlayout(f2, 3, 1, 'TileSpacing', 'compact', 'Padding', 'loose');

nexttile;
plot(y6.t, y6.y1(:,1), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(y6.t, y6.y1(:,4), '.', 'Color', col(2,:), 'MarkerSize', 7);
grid on; box on;
xlim([0.1 0.15]);
xlabel('t [s]'); ylabel('pos [rad]');
title('Sensor de posicion');
legend({'real','medida'}, 'Location', 'best');

nexttile;
plot(y6.t, y6.y1(:,3), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(y6.t, y6.y1(:,6), '.', 'Color', col(2,:), 'MarkerSize', 7);
grid on; box on;
xlim([0.099 0.12]);
xlabel('t [s]'); ylabel('i [A]');
title('Sensor de corriente');
legend({'real','medida'}, 'Location', 'best');

nexttile;
plot(y6.t, y6.y1(:,2), '-', 'Color', col(1,:), 'LineWidth', 2.5); hold on;
plot(y6.t, y6.y1(:,5), '.', 'Color', col(2,:), 'MarkerSize', 7);
grid on; box on;
xlim([0.1 0.2]);
xlabel('t [s]'); ylabel('\omega [rad/s]');
title('Sensor de velocidad (derivada filtrada)');
legend({'real','medida'}, 'Location', 'best');

estilizar(f2);
print(f2, '-depsc2', '-r300', fullfile(figdir, 'motor_sensores.eps'));
fprintf('Figura guardada: motor_sensores.eps\n');

% Figura 3: mapa par-velocidad
f3 = figure('Color', 'w', 'Position', [80 80 900 650]);
ax = axes(f3);
plot([0 mo.tau_stall], [mo.w_noload 0], '-', 'Color', [0.2 0.2 0.2], ...
     'LineWidth', 2.5); hold on;
plot(0, mo.w_noload, 'o', 'MarkerSize', 10, 'MarkerFaceColor', col(1,:), ...
     'MarkerEdgeColor', 'k');
plot(mo.tau_stall, 0, 's', 'MarkerSize', 10, 'MarkerFaceColor', col(2,:), ...
     'MarkerEdgeColor', 'k');
tau1_max = max(abs(tau1.Data));
tau2_max = max(abs(tau2.Data));
w1_max = 0.1501;   % P7: max |q1_dot|
w2_max = 0.2057;   % P7: max |q2_dot|
plot(tau1_max, w1_max, 'd', 'MarkerSize', 10, 'MarkerFaceColor', [0.2 0.7 0.2], ...
     'MarkerEdgeColor', 'k');
plot(tau2_max, w2_max, 'd', 'MarkerSize', 10, 'MarkerFaceColor', [0.5 0.2 0.7], ...
     'MarkerEdgeColor', 'k');
grid on; box on;
xlim([-0.02 1.05*mo.tau_stall]); ylim([0 1.1*mo.w_noload]);
xlabel('\tau [N.m]'); ylabel('\omega [rad/s]');
title('Mapa par-velocidad del S3003');
legend({'limite','sin carga','bloqueo','P8 q_1','P8 q_2'}, ...
       'Location', 'best');
text(0.04, 0.55*mo.w_noload, ...
     sprintf('P8: %.2e N.m, %.2f rad/s', tau1_max, w1_max), ...
     'Color', [0 0 0], 'FontSize', 20);
estilizar(f3);
print(f3, '-depsc2', '-r300', fullfile(figdir, 'motor_par_velocidad.eps'));
fprintf('Figura guardada: motor_par_velocidad.eps\n');

% ------------------------------------------------ RESUMEN
fprintf('\n--- Resumen de verificacion -----------------------------------\n');
fprintf('%-30s %12s %12s %12s\n', 'Prueba', 'Esperado', 'Obtenido', 'Error');
fprintf('%-30s %12.4f %12.4f %12.4f\n', 'omega_nl 6 V [rad/s]', ...
    mo.w_noload, w6, w6 - mo.w_noload);
fprintf('%-30s %12.4f %12.4f %12.4f\n', 'omega_nl 3 V [rad/s]', ...
    mo.w_noload/2, w3, w3 - mo.w_noload/2);
fprintf('%-30s %12.4f %12.4f %12.4f\n', 'tau_m [s]', ...
    mo.tau_m, tau_est, tau_est - mo.tau_m);
fprintf('%-30s %12.4f %12.4f %12.4f\n', 'i_stall [A]', ...
    mo.i_stall, i_stall, i_stall - mo.i_stall);
fprintf('%-30s %12.4f %12.4f %12.4f\n', 'tau_stall [N.m]', ...
    mo.tau_stall, tau_stall, tau_stall - mo.tau_stall);
fprintf('%-30s %12.4f %12.4f %12.4f\n', 'Delta omega carga P8 [rad/s]', ...
    0, dw, dw);

% ------------------------------------------------ COMPROBACIONES
assert(abs(w6 - mo.w_noload) < 0.01*mo.w_noload, ...
    'Velocidad sin carga fuera de tolerancia.');
assert(abs(w3 - mo.w_noload/2) < 0.01*mo.w_noload, ...
    'Velocidad a 3 V fuera de tolerancia.');
assert(abs(i_stall - mo.i_stall) < 0.02*mo.i_stall, ...
    'Corriente de bloqueo fuera de tolerancia.');
assert(abs(tau_stall - mo.tau_stall) < 0.02*mo.tau_stall, ...
    'Par de bloqueo fuera de tolerancia.');
assert(all(isfinite(y6.y1(:))) && all(isfinite(y3.y1(:))) && ...
       all(isfinite(ys.y1(:))) && all(isfinite(yp.y1(:))), ...
    'Alguna simulacion produjo valores no finitos.');

fprintf('\nVerificacion P10 completada.\n');

if ~cargado
    close_system(mdl, 0);
end


% --------------------------------------------------------------- FUNCIONES
function r = simular(mdl, V1, V2, tau1, tau2, T)
%simular  Ejecuta motor_2gdl con las entradas dadas y devuelve las salidas.
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('V1', V1);
    in = in.setVariable('V2', V2);
    in = in.setVariable('tau1', tau1);
    in = in.setVariable('tau2', tau2);
    in = in.setModelParameter('StopTime', num2str(T));
    out = sim(in);
    r.t  = out.y1.Time;
    r.y1 = out.y1.Data;
    r.y2 = out.y2.Data;
end


function estilizar(fig)
%estilizar  Fondo claro y fuente >= 26 pt (independiente del tema).
    ax = findall(fig, 'Type', 'axes');
    set(ax, 'Color', 'w', 'XColor', [0 0 0], 'YColor', [0 0 0], ...
            'GridColor', [0.75 0.75 0.75], 'FontSize', 26);
    lg = findobj(fig, 'Type', 'Legend');
    if ~isempty(lg)
        set(lg, 'Color', 'w', 'TextColor', [0 0 0], ...
            'EdgeColor', [0.7 0.7 0.7], 'FontSize', 26);
    end
    tt = findall(fig, 'Type', 'axes');
    for a = tt'
        a.Title.FontSize = 26;
        a.Title.Color = [0 0 0];
        a.Title.FontWeight = 'bold';
    end
end
