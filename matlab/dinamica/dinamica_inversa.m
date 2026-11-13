% dinamica_inversa.m
% Punto 8: resumen de torques maximos por articulacion.
%
%   Calcula la dinamica inversa de la trayectoria planeada y reporta, para
%   cada articulacion, el maximo absoluto, el pico positivo y el pico
%   negativo del par requerido.
%
%   Los pares resultan del orden de mN·m: las articulaciones giran sobre ejes
%   verticales, asi que la gravedad y la fuerza normal del lapiz (ambas en z)
%   no generan par; solo quedan los terminos inerciales y de Coriolis de
%   eslabones muy livianos.
%
%   Requiere que esten definidas en el workspace:
%       DH, PARAM   (plotter_param)
%       q, qd, qdd  (trayectoria_test)
%   Si existe robot (salida de plotter_param), se usa su tensor de inercia
%   completo (robot.dyn.I); si no, la parte diagonal de PARAM.
%
%   Uso:
%       trayectoria_test;
%       dinamica_inversa;

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
addpath(fullfile(aqui, '..', 'lib'));

vars = {'DH', 'PARAM', 'q', 'qd', 'qdd'};
ok = false(size(vars));
for k = 1:numel(vars)
    ok(k) = exist(vars{k}, 'var') ~= 0;
end
faltan = vars(~ok);
if ~isempty(faltan)
    error('dinamica_inversa:variables', ...
        'Faltan variables: %s. Ejecute trayectoria_test antes.', strjoin(faltan, ', '));
end

if exist('robot', 'var') && isfield(robot, 'dyn') && isfield(robot.dyn, 'I') ...
        && size(robot.dyn.I, 3) == size(DH, 1)
    Icm = robot.dyn.I;
else
    Icm = [];
end

grav  = [0 0 -9.81];
F_ext = [0 0 -0.294 0 0 0]';   % 30 g del lapiz, fuerza normal al papel

tau = ID(DH, PARAM, q, qd', qdd', grav, F_ext, Icm);

% 1. Maximos absolutos (esfuerzo pico en magnitud)
[max_abs_tau, idx_abs] = max(abs(tau), [], 1);

% 2. Maximos y minimos con signo
[max_pos_tau, idx_pos] = max(tau, [], 1);
[min_neg_tau, idx_neg] = min(tau, [], 1);

% 3. Resultados en consola
fprintf('=======================================================\n');
fprintf('        RESUMEN DE TORQUES MAXIMOS POR ARTICULACION    \n');
fprintf('=======================================================\n');

for col = 1:size(tau, 2)
    fprintf('Articulacion %d:\n', col);
    fprintf('  - Max. absoluto (|Torque|): %8.3f mN·m  (en el punto %d de %d)\n', ...
        1e3*max_abs_tau(col), idx_abs(col), size(tau, 1));
    fprintf('  - Pico positivo (+Torque):   %8.3f mN·m  (en el punto %d)\n', ...
        1e3*max_pos_tau(col), idx_pos(col));
    fprintf('  - Pico negativo (-Torque):   %8.3f mN·m  (en el punto %d)\n', ...
        1e3*min_neg_tau(col), idx_neg(col));
    fprintf('-------------------------------------------------------\n');
end
