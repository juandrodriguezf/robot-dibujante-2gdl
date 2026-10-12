function [robot, PARAM] = plotter_param()
%plotter_param  Modelo cinematico del robot dibujante de 2 GdL (plotter de Reuleaux).
%
%   [robot, PARAM] = plotter_param()
%
%   Cinematica: 2R plano. Las dos articulaciones son rotacionales sobre ejes z
%   paralelos (convencion estandar de Denavit-Hartenberg). El robot traza la
%   figura sobre un plano horizontal.
%
%   Medidas reales extraidas del ensamble CAD (Plotter_2GdL.SLDASM), tomadas de
%   centro de eje a centro de eje:
%       L1 = 0.200 m   eje del servo base  -> eje del servo 2
%       L2 = 0.100 m   eje del servo 2     -> punta del marcador
%       d1 = 0.000 m   altura del plano de dibujo (plano de trabajo z = d1)
%
%   Salidas:
%       robot : estructura del manipulador
%                 robot.DH     matriz n x 5 [theta_offset d a alpha sigma]
%                 robot.PARAM  parametros dinamicos (estimacion, ver nota)
%                 robot.name   nombre del robot
%                 robot.n      numero de grados de libertad
%                 robot.link   objeto SerialLink del Robotics Toolbox (o [])
%       PARAM : misma matriz que robot.PARAM.
%
%   Nota sobre PARAM (estimacion de primer orden, a refinar en el modelo
%   dinamico): masas tomadas del CAD; cuerpo 1 = Eslabon1 + servo2 + horn2 (+horn1);
%   cuerpo 2 = Eslabon2 + esfero. S = [Sx Sy Sz] posicion del centro de masa en el
%   marco del eslabon [m]. Inercias Ixx Iyy Izz [kg m^2]; columnas de motor
%   (Jm G B Tc) en cero por no estar definidas aun.

    % --- Geometria (medidas reales del CAD) ---
    L1 = 0.200;      % m
    L2 = 0.100;      % m
    d1 = 0.000;      % m

    % Matriz de parametros Denavit-Hartenberg
    %         theta   d     a      alpha  sigma
    DH = [     0     d1    L1      0      0;    % articulacion 1 (base)
               0      0    L2      0      0 ];  % articulacion 2 (codo)

    % Parametros dinamicos (estimacion)  [m Sx Sy Sz Ixx Iyy Izz Jm G B Tc+ Tc-]
    PARAM = [ 0.04390  0.1867  0  0  0  0  1.60e-3  0 0 0 0 0;   % cuerpo 1
              0.00947  0.0848  0  0  0  0  7.20e-5  0 0 0 0 0 ]; % cuerpo 2

    % Estructura del robot
    robot.DH    = DH;
    robot.PARAM = PARAM;
    robot.name  = 'Plotter2GdL';
    robot.n     = size(DH, 1);
    robot.link  = [];

    % Objeto SerialLink (solo si el Robotics Toolbox esta disponible)
    if exist('SerialLink', 'class') == 8
        try
            robot.link = local_serial_link(DH, robot.name);
        catch err
            fprintf('aviso: no se pudo construir SerialLink (%s).\n', err.message);
            robot.link = [];
        end
    end
end


function L = local_serial_link(DH, name)
%local_serial_link  Ensambla un SerialLink a partir de la matriz DH.
%   Se crea un arreglo de eslabones nuevo en cada llamada: SerialLink guarda una
%   referencia al arreglo recibido y dos objetos creados del mismo arreglo
%   compartirian los mismos eslabones.
    n = size(DH, 1);
    for i = 1:n
        L(i) = Revolute('d', DH(i,2), 'a', DH(i,3), 'alpha', DH(i,4));
    end
    qoff = DH(:,1)';
    L = SerialLink(L, 'name', name, 'offset', qoff);
end
