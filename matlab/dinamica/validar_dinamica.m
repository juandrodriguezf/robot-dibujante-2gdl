% validar_dinamica.m
% Punto 8: validacion de ID contra el Robotics Toolbox.
%
%   Compara, sobre poses aleatorias dentro de los limites articulares:
%       1. ID con el tensor de inercia completo (robot.dyn.I)
%       2. ID con la parte diagonal de PARAM (comportamiento por defecto)
%       3. par debido a una fuerza externa en el plano vs Jv'*f
%
%   Requiere: plotter_param.m, ID.m, geometric_jacobian.m, add_rvc_path.m
%   y el Robotics Toolbox.

clear;
clc;

aqui = fileparts(mfilename('fullpath'));
addpath(aqui);
addpath(fullfile(aqui, '..', 'lib'));
addpath(fullfile(aqui, '..', 'params'));
add_rvc_path();

[robot, PARAM] = plotter_param();
DH = robot.DH;
n  = robot.n;

if isempty(robot.link)
    error('validar_dinamica:toolbox', ...
        'Se requiere el Robotics Toolbox para la validacion cruzada.');
end

% SerialLink equivalente con la inercia diagonal de PARAM
Ld = repmat(Revolute('d', DH(1,2), 'a', DH(1,3), 'alpha', DH(1,4), ...
    'qlim', robot.qlim(1,:), 'm', PARAM(1,1), 'r', PARAM(1,2:4), ...
    'I', diag(PARAM(1,5:7))), 1, n);
for i = 1:n
    Ld(i) = Revolute('d', DH(i,2), 'a', DH(i,3), 'alpha', DH(i,4), ...
        'qlim', robot.qlim(i,:), 'm', PARAM(i,1), 'r', PARAM(i,2:4), ...
        'I', diag(PARAM(i,5:7)));
end
rd = SerialLink(Ld, 'name', 'diag', 'base', robot.base, ...
    'offset', DH(:,1)');
rd.gravity = robot.link.gravity;

rng(42);
N = 200;
Q   = (rand(N, n) - 0.5) * pi;
Qd  = (rand(N, n) - 0.5) * 2;
Qdd = (rand(N, n) - 0.5) * 10;
grav = [0; 0; -9.81];

err_full  = 0;
err_diag  = 0;
err_force = 0;

for k = 1:N
    q   = Q(k, :);
    qd  = Qd(k, :);
    qdd = Qdd(k, :);

    % 1. Tensor de inercia completo
    t_full = ID(DH, PARAM, q, qd, qdd, grav, zeros(6,1), robot.dyn.I);
    t_rne  = robot.link.rne(q, qd, qdd, 'slow');
    err_full = max(err_full, max(abs(t_full(:) - t_rne(:))));

    % 2. Inercia diagonal (por defecto)
    t_diag = ID(DH, PARAM, q, qd, qdd, grav, zeros(6,1));
    t_rd   = rd.rne(q, qd, qdd, 'slow');
    err_diag = max(err_diag, max(abs(t_diag(:) - t_rd(:))));

    % 3. Fuerza externa en el plano
    T = robot.link.fkine(q);
    f_base = [0.5; -0.3; 0];
    f_ee   = (T.R.') * f_base;
    t_f    = ID(DH, PARAM, q, 0*q, 0*q, [0 0 0], [f_ee; 0; 0; 0], robot.dyn.I);
    J      = geometric_jacobian(robot, DH, q);
    t_ref  = (J(1:3,:).') * f_base;
    err_force = max(err_force, max(abs(t_f(:) - t_ref(:))));
end

fprintf('=========================================================\n');
fprintf(' VALIDACION DE LA DINAMICA INVERSA (Plotter 2 GdL)\n');
fprintf('=========================================================\n');
fprintf('Poses aleatorias: %d\n\n', N);
fprintf('max |ID - rne| (tensor completo) = %.3e N·m\n', err_full);
fprintf('max |ID - rne| (diagonal)        = %.3e N·m\n', err_diag);
fprintf('max |tau_ext - Jv''*f|           = %.3e N·m\n', err_force);

tol = 1e-9;
if max([err_full, err_diag, err_force]) < tol
    fprintf('\nOK: todas las comparaciones coinciden (< %.0e).\n', tol);
else
    fprintf('\nATENCION: alguna comparacion supera %.0e.\n', tol);
end
