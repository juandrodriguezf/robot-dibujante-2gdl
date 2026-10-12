function [robot, PARAM] = plotter_param()
%plotter_param  Modelo cinematico y dinamico del robot dibujante de 2 GdL.
%
%   [robot, PARAM] = plotter_param()
%
%   Cinematica: 2R plano sobre ejes z paralelos. La punta del esfero dibuja
%   sobre el plano horizontal del papel (marco mundo {W}, z = 0).
%
%   Medidas reales tomadas del ensamblaje CAD (Plotter_2GdL.SLDASM):
%       L1 = 0.200 m   eje del servo base      -> eje del servo 2
%       L2 = 0.100 m   eje del servo 2          -> punta del marcador
%       d1 = 0.020 m   cara del horn {0}        -> eje del servo 2
%       d2 = -0.0685 m eje del servo 2          -> punta del marcador
%       {W} -> {0} = trasl(0, 0, 0.0485)   (placa base + horn del servo 1)
%
%   Salidas:
%       robot : estructura del manipulador
%                 robot.DH     matriz n x 5 [theta_offset d a alpha sigma]
%                 robot.PARAM  parametros dinamicos (m, CDM, inercias, motor)
%                 robot.base   transformacion {W} -> {0} (4x4)
%                 robot.qlim   limites articulares (2 x n)
%                 robot.dyn    m (n x 1), r (n x 3), I (3 x 3 x n)
%                 robot.name, robot.n, robot.link (SerialLink o [])
%       PARAM : misma matriz que robot.PARAM.
%
%   Parametros dinamicos tomados del CAD (SolidWorks): masas, centros de masa
%   e inercias de cada cuerpo respecto a su CDM y a los ejes de su marco DH.
%   Cuerpo 1 = Eslabon1 + Horn1 + Servo2; cuerpo 2 = Eslabon2 + Horn2 + Esfero.
%   Columnas de motor (Jm G B Tc) en cero (se definen en la Parte 2).

    % --- Geometria (medidas reales del ensamblaje CAD) ---
    esp_balso  = 0.006;    % espesor de la lamina de balso
    h_placa    = 0.006;    % placa inferior de la base
    h_horn     = 0.0425;   % base del servo -> cara superior del horn
    h_pestana  = 0.0285;   % base del servo -> cara superior de la pestana
    L1 = 0.200;            % m
    L2 = 0.100;            % m

    % --- Marcos ---
    % {W} mundo: sobre el papel (plano de dibujo), en el eje del servo 1.
    % {0} articulacion 1: cara del horn del servo 1 (z0 = eje del servo 1).
    % {1} articulacion 2: cara del horn del servo 2 (z1 = eje del servo 2).
    % {2} efector: punta del esfero sobre el papel.
    h0 = h_placa + h_horn;                              % 0.0485  {W}->{0}
    Tbase = [1 0 0 0; 0 1 0 0; 0 0 1 h0; 0 0 0 1];

    % Alturas de la cadena vertical (convencion DH estandar)
    d1 = esp_balso + (h_horn - h_pestana);             % 0.020
    d2 = -(h0 + d1);                                   % -0.0685 (hasta el papel)

    % Matriz de parametros Denavit-Hartenberg
    %         theta   d     a     alpha  sigma
    DH = [     0     d1    L1     0      0;             % articulacion 1
               0     d2    L2     0      0 ];           % articulacion 2

    % Limites articulares (Futaba S3003: 180 grados)
    qlim = [-pi/2 pi/2; -pi/2 pi/2];

    % --- Parametros dinamicos (del CAD) ---
    m1 = 0.043900;  r1 = [-0.026889  0  -0.023099];
    I1 = [ 6.051213e-06  0             3.727225e-06;
           0             1.073608e-04  0;
           3.727225e-06  0             1.046865e-04];
    m2 = 0.010336;  r2 = [-0.022266  0   0.074512];
    I2 = [ 8.508063e-06  0            -1.061245e-06;
           0             2.252327e-05  0;
          -1.061245e-06  0             1.444026e-05];

    % Filas [m Sx Sy Sz Ixx Iyy Izz Jm G B Tc+ Tc-] (columnas de motor en cero)
    PARAM = [ m1 r1 I1(1,1) I1(2,2) I1(3,3) 0 0 0 0 0;
              m2 r2 I2(1,1) I2(2,2) I2(3,3) 0 0 0 0 0 ];

    % --- Estructura del robot ---
    robot.DH    = DH;
    robot.PARAM = PARAM;
    robot.name  = 'Plotter2GdL';
    robot.n     = size(DH, 1);
    robot.base  = Tbase;
    robot.qlim  = qlim;
    robot.dyn.m = [m1; m2];
    robot.dyn.r = [r1; r2];
    robot.dyn.I = cat(3, I1, I2);
    robot.link  = [];

    % Objeto SerialLink (solo si el Robotics Toolbox esta disponible)
    if exist('SerialLink', 'class') == 8
        try
            robot.link = local_serial_link(robot);
        catch err
            fprintf('aviso: no se pudo construir SerialLink (%s).\n', err.message);
            robot.link = [];
        end
    end
end


function L = local_serial_link(robot)
%local_serial_link  Ensambla un SerialLink a partir del modelo del robot.
%   Se crea un arreglo de eslabones nuevo en cada llamada: SerialLink guarda una
%   referencia al arreglo recibido y dos objetos creados del mismo arreglo
%   compartirian los mismos eslabones.
    DH = robot.DH;
    n  = size(DH, 1);
    for i = 1:n
        L(i) = Revolute('d', DH(i,2), 'a', DH(i,3), 'alpha', DH(i,4), ...
                        'qlim', robot.qlim(i,:), ...
                        'm', robot.dyn.m(i), 'r', robot.dyn.r(i,:), ...
                        'I', robot.dyn.I(:,:,i));
    end
    qoff = DH(:,1)';
    L = SerialLink(L, 'name', robot.name, 'base', robot.base, 'offset', qoff);
end
