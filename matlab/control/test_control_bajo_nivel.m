% test_control_bajo_nivel.m
% Punto 11: verificacion INDIVIDUAL del control de bajo nivel de cada articulacion.
%
%   Pruebas:
%       1. Escalon de 30 deg en cada articulacion (la otra en reposo).
%       2. Escalones de distinta amplitud (5, 15, 60 deg).
%       3. Seguimiento de la trayectoria real de P7 (Reuleaux) con la carga de P8,
%          con y sin prealimentacion de velocidad.
%       4. Robustez: inercia J entre 0.5 y 2 veces el valor estimado.
%
%   Genera figuras EPS en docs/informe-2/figuras.
%
%   Requiere: motor_2gdl_ctrl.slx (se crea con build_control_model.m),
%             param_motor.m, param_sensores.m, param_control.m,
%             carga_mecanica.m, trayectoria_articular.m.

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
addpath(fullfile(aqui, '..', 'trayectorias'));
addpath(fullfile(aqui, '..', 'lib'));
addpath(fullfile(aqui, '..', 'params'));
addpath(fullfile(aqui, '..', 'dinamica'));

figdir = fullfile(aqui, '..', '..', 'docs', 'informe-2', 'figuras');
if ~exist(figdir, 'dir')
    mkdir(figdir);
end

mdl = 'motor_2gdl_ctrl';
if ~exist(fullfile(aqui, [mdl '.slx']), 'file')
    build_control_model();
end
cargado = bdIsLoaded(mdl);
if ~cargado
    load_system(mdl);
end

mo = param_motor();
se = param_sensores();
ct = param_control();

fprintf('=========================================================\n');
fprintf(' PUNTO 11 - VERIFICACION DEL CONTROL DE BAJO NIVEL\n');
fprintf('=========================================================\n');
fprintf('Kp = %.2f V/rad | Kff = %.4f V.s/rad | zeta = %.2f | wn = %.1f rad/s\n\n', ...
    ct.Kp, ct.Kff, ct.zeta, ct.wn);

col = [0.20 0.45 0.95; 0.90 0.45 0.10];
Z4  = @(t) timeseries(zeros(size(t)), t);

% ------------------------------------------------ 1. ESCALON 30 deg
fprintf('--- 1. Escalon de 30 deg por articulacion ----------------------\n');
tt  = [0; 0.1999; 0.2; 1.5];
A   = deg2rad(30);
stp = timeseries(A*[0; 0; 1; 1], tt);
res1 = cell(1, 2);
for j = 1:2
    if j == 1
        r = simular(mdl, stp, Z4(tt), Z4(tt), Z4(tt), Z4(tt), Z4(tt), 1.5, {});
        th = r.y1(:, 1);  om = r.y1(:, 2);  ia = r.y1(:, 3);  Vc = r.y1(:, 7);  tm = r.y1(:, 4);
    else
        r = simular(mdl, Z4(tt), Z4(tt), Z4(tt), stp, Z4(tt), Z4(tt), 1.5, {});
        th = r.y2(:, 1);  om = r.y2(:, 2);  ia = r.y2(:, 3);  Vc = r.y2(:, 7);  tm = r.y2(:, 4);
    end
    m = metricas_escalon(r.t, th, A, 0.2);
    fprintf('Joint%d: sobreimpulso = %.3f %% | t_subida = %.3f s | t_estab(2%%) = %.3f s\n', ...
        j, m.OS, m.tr, m.ts);
    fprintf('        error final = %.4f deg | V max = %.2f V | i max = %.4f A (i_stall %.4f)\n', ...
        rad2deg(m.ess), max(abs(Vc)), max(abs(ia)), mo.i_stall);
    assert(m.OS < 5,                'Joint%d: sobreimpulso excesivo.', j);
    assert(m.ts < 0.3,              'Joint%d: establecimiento lento.', j);
    assert(abs(m.ess) <= se.pos_res,'Joint%d: error final mayor que la resolucion del sensor.', j);
    assert(max(abs(Vc)) <= mo.Vmax + 1e-9, 'Joint%d: V excede el limite.', j);
    res1{j} = struct('t', r.t, 'th', th, 'om', om, 'i', ia, 'V', Vc, 'tm', tm);
end

% ------------------------------------------------ 2. AMPLITUDES
fprintf('\n--- 2. Escalones de distinta amplitud (Joint1) -----------------\n');
for a = [5 15 60]
    Aa = deg2rad(a);
    r = simular(mdl, timeseries(Aa*[0;0;1;1], tt), Z4(tt), Z4(tt), Z4(tt), Z4(tt), Z4(tt), 1.5, {});
    m = metricas_escalon(r.t, r.y1(:,1), Aa, 0.2);
    fprintf('%3d deg: sobreimpulso = %.3f %% | t_estab = %.3f s | i max = %.4f A\n', ...
        a, m.OS, m.ts, max(abs(r.y1(:,3))));
    assert(m.OS < 5, 'Sobreimpulso excesivo en escalon de %d deg.', a);
end

% ------------------------------------------------ 3. TRAYECTORIA REAL P7
fprintf('\n--- 3. Trayectoria real de P7 (Reuleaux) con carga de P8 -------\n');
[tv, q, qd] = trayectoria_articular();
qs = q - ct.q_off';                         % coordenadas del servo
fprintf('Rango de q  [deg]: q1 %.1f a %.1f | q2 %.1f a %.1f\n', ...
    rad2deg(min(q(:,1))), rad2deg(max(q(:,1))), rad2deg(min(q(:,2))), rad2deg(max(q(:,2))));
fprintf('Rango servo [deg]: s1 %.1f a %.1f | s2 %.1f a %.1f (limite +-%.0f)\n', ...
    rad2deg(min(qs(:,1))), rad2deg(max(qs(:,1))), rad2deg(min(qs(:,2))), rad2deg(max(qs(:,2))), ...
    rad2deg(mo.qmax));
assert(all(abs(qs(:)) < mo.qmax), ...
    ['La trayectoria excede el recorrido del servo (+-90 deg). ' ...
     'Revisar q_off en param_control.m (offset de montaje).']);

[tau1, tau2] = carga_mecanica();
ic1 = {[mdl '/Joint1/int_th'], 'InitialCondition', num2str(qs(1,1), '%.12g')};
ic2 = {[mdl '/Joint2/int_th'], 'InitialCondition', num2str(qs(1,2), '%.12g')};
q1r = timeseries(qs(:,1), tv);  q2r = timeseries(qs(:,2), tv);
w1r = timeseries(qd(:,1), tv);  w2r = timeseries(qd(:,2), tv);

% (a) P + prealimentacion  /  (b) P puro (equivale al servo real)
rFF = simular(mdl, q1r, w1r, tau1, q2r, w2r, tau2, tv(end), [ic1 ic2]);
sinFF = {[mdl '/Joint1/Kff'], 'Gain', '0', [mdl '/Joint2/Kff'], 'Gain', '0'};
rP  = simular(mdl, q1r, w1r, tau1, q2r, w2r, tau2, tv(end), [ic1 ic2 sinFF]);

fprintf('%-8s %-14s %12s %12s %10s %10s\n', 'Joint', 'Control', 'RMS [deg]', 'max [deg]', 'V max', 'i max');
tab = zeros(2, 2, 4);
for j = 1:2
    for c = 1:2
        if c == 1, r = rP;  nom = 'P puro';
        else,      r = rFF; nom = 'P + prealim.';
        end
        yy = r.(sprintf('y%d', j));
        th = resamp(r.t, yy(:,1), tv);
        e  = rad2deg(qs(:,j) - th(:));
        tab(j,c,:) = [rms(e) max(abs(e)) max(abs(yy(:,7))) max(abs(yy(:,3)))];
        fprintf('Joint%-3d %-14s %12.4f %12.4f %10.3f %10.4f\n', j, nom, tab(j,c,1), tab(j,c,2), tab(j,c,3), tab(j,c,4));
        assert(max(abs(e)) < 1.0, 'Joint%d (%s): error de seguimiento > 1 deg.', j, nom);
    end
end

% ------------------------------------------------ 4. ROBUSTEZ J
fprintf('\n--- 4. Robustez ante error en la inercia J (escalon 30 deg) ----\n');
fJ = [0.5 1 1.5 2];
resJ = cell(1, numel(fJ));
for k = 1:numel(fJ)
    bp = {[mdl '/Joint1/invJ'], 'Gain', num2str(1/(mo.J*fJ(k)), '%.12g')};
    r = simular(mdl, stp, Z4(tt), Z4(tt), Z4(tt), Z4(tt), Z4(tt), 1.5, bp);
    m = metricas_escalon(r.t, r.y1(:,1), A, 0.2);
    fprintf('J x %.1f: sobreimpulso = %.3f %% | t_estab = %.3f s\n', fJ(k), m.OS, m.ts);
    assert(m.OS < 5, 'Sobreimpulso excesivo con J x %.1f.', fJ(k));
    resJ{k} = struct('t', r.t, 'th', r.y1(:,1));
end

% ------------------------------------------------ 5. FIGURAS
fprintf('\n--- 5. Figuras -------------------------------------------------\n');

% Figura 1: escalon de 30 deg
f1 = figure('Color', 'w', 'Position', [60 60 1200 900]);
tiledlayout(f1, 2, 2, 'TileSpacing', 'compact', 'Padding', 'loose');
R = res1{1};
nexttile; plot(R.t, rad2deg(A)*(R.t >= 0.2), '--k', 'LineWidth', 2); hold on;
plot(R.t, rad2deg(R.th), '-', 'Color', col(1,:), 'LineWidth', 2.5);
plot(R.t, rad2deg(R.tm), '.', 'Color', col(2,:), 'MarkerSize', 6);
grid on; box on; xlabel('t [s]'); ylabel('\theta [deg]'); title('Posicion');
legend({'ref', 'real', 'medida'}, 'Location', 'southeast');
nexttile; plot(R.t, R.om, '-', 'Color', col(1,:), 'LineWidth', 2.5);
grid on; box on; xlabel('t [s]'); ylabel('\omega [rad/s]'); title('Velocidad');
nexttile; plot(R.t, R.V, '-', 'Color', col(1,:), 'LineWidth', 2.5);
grid on; box on; xlabel('t [s]'); ylabel('V [V]'); title('Tension de control');
nexttile; plot(R.t, R.i, '-', 'Color', col(1,:), 'LineWidth', 2.5);
grid on; box on; xlabel('t [s]'); ylabel('i [A]'); title('Corriente');
estilizar(f1);
print(f1, '-depsc2', '-r300', fullfile(figdir, 'control_escalon.eps'));
fprintf('Figura guardada: control_escalon.eps\n');

% Figura 2: trayectoria real
f2 = figure('Color', 'w', 'Position', [80 60 1200 1100]);
tiledlayout(f2, 3, 2, 'TileSpacing', 'compact', 'Padding', 'loose');
for j = 1:2
    yP  = rP.(sprintf('y%d', j));
    yF  = rFF.(sprintf('y%d', j));
    nexttile(j);
    plot(tv, rad2deg(qs(:,j)), '--k', 'LineWidth', 2); hold on;
    plot(rFF.t, rad2deg(yF(:,1)), '-', 'Color', col(1,:), 'LineWidth', 2);
    grid on; box on; xlabel('t [s]'); ylabel('\theta_{servo} [deg]');
    title(sprintf('Joint%d: posicion', j));
    nexttile(2 + j);
    plot(tv, rad2deg(qs(:,j) - resamp(rP.t,  yP(:,1), tv)), '-', 'Color', col(2,:), 'LineWidth', 1.5); hold on;
    plot(tv, rad2deg(qs(:,j) - resamp(rFF.t, yF(:,1), tv)), '-', 'Color', col(1,:), 'LineWidth', 1.5);
    grid on; box on; xlabel('t [s]'); ylabel('error [deg]'); title('Error de seguimiento');
    legend({'P puro', 'P + prealim.'}, 'Location', 'best');
    nexttile(4 + j);
    plot(rFF.t, yF(:,7), '-', 'Color', col(1,:), 'LineWidth', 1.5);
    grid on; box on; xlabel('t [s]'); ylabel('V [V]'); title('Tension de control');
end
estilizar(f2);
print(f2, '-depsc2', '-r300', fullfile(figdir, 'control_trayectoria.eps'));
fprintf('Figura guardada: control_trayectoria.eps\n');

% Figura 3: robustez
f3 = figure('Color', 'w', 'Position', [80 80 900 650]);
ax = axes(f3); hold(ax, 'on');
for k = 1:numel(fJ)
    plot(resJ{k}.t, rad2deg(resJ{k}.th), 'LineWidth', 2.5);
end
plot(tt, rad2deg(stp.Data), '--k', 'LineWidth', 2);
grid on; box on; xlabel('t [s]'); ylabel('\theta [deg]');
title('Robustez ante error en J');
legend([arrayfun(@(f) sprintf('J x %.1f', f), fJ, 'UniformOutput', false), {'ref'}], ...
    'Location', 'southeast');
estilizar(f3);
print(f3, '-depsc2', '-r300', fullfile(figdir, 'control_robustez.eps'));
fprintf('Figura guardada: control_robustez.eps\n');

fprintf('\nVerificacion P11 completada.\n');

if ~cargado
    close_system(mdl, 0);
end


% --------------------------------------------------------------- FUNCIONES
function r = simular(mdl, q1, w1, tau1, q2, w2, tau2, T, bp)
%simular  Ejecuta motor_2gdl_ctrl con las entradas dadas.
%   bp: celda de parametros de bloque {blockpath, 'Param', 'valor', ...} o {}.
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('q1_ref', q1);
    in = in.setVariable('w1_ref', w1);
    in = in.setVariable('tau1',   tau1);
    in = in.setVariable('q2_ref', q2);
    in = in.setVariable('w2_ref', w2);
    in = in.setVariable('tau2',   tau2);
    in = in.setModelParameter('StopTime', num2str(T));
    for k = 1:3:numel(bp)
        in = in.setBlockParameter(bp{k}, bp{k+1}, bp{k+2});
    end
    out = sim(in);
    r.t  = out.y1.Time;
    r.y1 = out.y1.Data;
    r.y2 = out.y2.Data;
end


function yi = resamp(t, y, tq)
%resamp  Interpola una senal registrada (descarta tiempos repetidos del solver).
    [tu, iu] = unique(t);
    yi = interp1(tu, y(iu), tq(:));
end


function m = metricas_escalon(t, th, A, t0, pos_res)
%metricas_escalon  Sobreimpulso, tiempo de subida (10-90 %), establecimiento.
%   La banda de establecimiento es max(2 % de A, resolucion del sensor) para
%   que los escalones pequenos sean medibles con la cuantizacion de 0.5 deg.
    if nargin < 5
        pos_res = deg2rad(0.5);
    end

    m.OS  = max(0, (max(th)/A - 1)*100);
    k10 = find(th >= 0.1*A & t >= t0, 1);
    k90 = find(th >= 0.9*A & t >= t0, 1);
    if isempty(k10) || isempty(k90)
        m.tr = NaN;
    else
        m.tr = t(k90) - t(k10);
    end

    band = max(0.02*A, pos_res);
    kf = find(abs(th - A) > band, 1, 'last');
    if isempty(kf)
        m.ts = 0;
    else
        m.ts = t(min(kf + 1, numel(t))) - t0;
    end
    m.ess = th(end) - A;
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
