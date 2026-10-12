function G = plotter_cad(accion, varargin)
%PLOTTER_CAD  Dibuja el plotter con la geometria real exportada de SolidWorks.
%
%   G = plotter_cad('crear', ax, q)      crea la escena en los ejes ax
%   plotter_cad('mover', G, q)           actualiza la pose (q en rad)
%
%   Las mallas (cad/export/plotter_mallas_CAD.mat) estan expresadas en el marco
%   del cuerpo al que pertenecen: {W} (base + servo 1), {1} (eslabon 1) y {2}
%   (eslabon 2). Cada cuerpo se mueve con la transformacion DH del robot
%   (plotter_param), de modo que lo que se ve ES el modelo cinematico.

switch lower(accion)
    case 'crear'
        G = crear(varargin{:});
    case 'mover'
        G = varargin{1};
        mover(G, varargin{2});
    otherwise
        error('Accion desconocida: %s', accion);
end
end

% -------------------------------------------------------------------------
function G = crear(ax, q, opciones)
if nargin < 2 || isempty(q), q = [0 0]; end
if nargin < 3, opciones = struct(); end
if ~isfield(opciones, 'marcos'), opciones.marcos = true; end
if ~isfield(opciones, 'papel'),  opciones.papel  = true; end

p = plotter_param();
G.p = p;
[~, ~, A] = local_fk(p);
G.A = A;

S = load(fullfile(fileparts(mfilename('fullpath')), '..', '..', 'cad', 'export', 'plotter_mallas_CAD.mat'));
M = S.mallas;

hold(ax, 'on');
G.ax = ax;
G.hW = hgtransform('Parent', ax);
G.h1 = hgtransform('Parent', ax);
G.h2 = hgtransform('Parent', ax);
padres = {G.hW, G.h1, G.h2};

nombres = fieldnames(M);
for k = 1:numel(nombres)
    m = M.(nombres{k});
    alfa = 1;
    if strcmp(nombres{k}, 'Base'), alfa = 0.45; end
    patch('Parent', padres{double(m.cuerpo) + 1}, 'Faces', m.F, 'Vertices', m.V, ...
          'FaceColor', m.color, 'EdgeColor', 'none', 'FaceAlpha', alfa, ...
          'FaceLighting', 'flat', 'AmbientStrength', 0.45, 'DiffuseStrength', 0.7, ...
          'SpecularStrength', 0.15);
end

if opciones.papel   % hoja carta sobre la mesa
    patch('Parent', ax, 'XData', [0.06 0.34 0.34 0.06], 'YData', [-0.108 -0.108 0.108 0.108], ...
          'ZData', -0.0005*[1 1 1 1], 'FaceColor', [1 1 1], 'EdgeColor', [0.6 0.6 0.6], ...
          'FaceAlpha', 0.9, 'LineStyle', '--');
end

G.marcos = gobjects(0);
if opciones.marcos
    G.marcos = [ triada(ax,    eye(4),  0.06, 'W'), ...
                 triada(G.hW,  p.base, 0.04, '0'), ...
                 triada(G.h1,  eye(4),  0.04, '1'), ...
                 triada(G.h2,  eye(4),  0.04, '2') ];
end

axis(ax, 'equal'); grid(ax, 'on');
xlabel(ax, 'x_W [m]'); ylabel(ax, 'y_W [m]'); zlabel(ax, 'z_W [m]');
xlim(ax, [-0.08 0.34]); ylim(ax, [-0.26 0.30]); zlim(ax, [0 0.15]);
view(ax, -40, 24);
if isempty(findobj(ax, 'Type', 'light'))
    camlight(ax, 'headlight'); light(ax, 'Position', [1 -1 2], 'Style', 'infinite');
end
mover(G, q);
end

% -------------------------------------------------------------------------
function mover(G, q)
p = G.p;
A = G.A;
T1 = p.base * A(q(1), 1);      % {W} -> {1}
T2 = T1 * A(q(2), 2);          % {W} -> {2}
set(G.h1, 'Matrix', T1);
set(G.h2, 'Matrix', T2);
end

% -------------------------------------------------------------------------
function [T, A1, A] = local_fk(p)
%local_fk  Handler de las matrices DH estandar de cada eslabon.
DH = p.DH;
A1 = [];
A  = @(q, i) dh_estandar(q, DH(i,1), DH(i,2), DH(i,3), DH(i,4));
T  = p.base * A(0,1) * A(0,2);
end

function A = dh_estandar(q, offset, d, a, al)
th = q + offset;
A = [cos(th) -sin(th)*cos(al)  sin(th)*sin(al)  a*cos(th);
     sin(th)  cos(th)*cos(al) -cos(th)*sin(al)  a*sin(th);
     0        sin(al)          cos(al)          d;
     0        0                0                1];
end

% -------------------------------------------------------------------------
function h = triada(padre, T, L, nombre)
o = T(1:3,4); R = T(1:3,1:3);
c = [0.85 0.1 0.1; 0.1 0.6 0.1; 0.1 0.25 0.85];
ejes = 'xyz';
h = gobjects(1, 7);
for i = 1:3
    e = o + L*R(:,i);
    h(i) = line('Parent', padre, 'XData', [o(1) e(1)], 'YData', [o(2) e(2)], ...
                'ZData', [o(3) e(3)], 'Color', c(i,:), 'LineWidth', 2.5);
    h(3+i) = text('Parent', padre, 'Position', o + 1.2*L*R(:,i), ...
                  'String', [ejes(i) '_{' nombre '}'], 'Color', c(i,:), ...
                  'FontWeight', 'bold', 'FontSize', 9);
end
h(7) = text('Parent', padre, 'Position', o + [0.006; 0.006; -0.012], ...
            'String', ['\{' nombre '\}'], 'FontWeight', 'bold', 'FontSize', 11, ...
            'BackgroundColor', [1 1 1], 'Margin', 1);
end
