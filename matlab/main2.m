% main2.m
% Ejecucion automatica de la Parte 2 (Puntos 8-11) del proyecto.
%
%   Por ahora ejecuta:
%       Punto 7  trayectoria_test   (insumo: q, qd, qdd en el workspace)
%       Punto 8  dinamica_inversa   (RNE y resumen de torques por articulacion)
%       Punto 10 test_motor_model   (modelo de motor DC + sensores y figuras)
%
%   Puntos 9 y 11 quedan pendientes.
%
%   Nota: trayectoria_test regenera las figuras del Punto 7 en
%   docs/informe-1/figuras/; test_motor_model regenera las de P10 en
%   docs/informe-2/figuras/.
%
%   Uso:  >> main2
%
%   Requiere MATLAB + Robotics Toolbox for MATLAB (Peter Corke). La ruta del
%   toolbox se detecta automaticamente con add_rvc_path().

clear;
clc;
close all;

% Anadir al path la carpeta del proyecto y todas sus subcarpetas
aqui = fileparts(mfilename('fullpath'));
addpath(genpath(aqui));

fprintf('#########################################################\n');
fprintf('#  EJECUCION AUTOMATICA - PARTE 2 (P8 y P10)            #\n');
fprintf('#########################################################\n\n');

fprintf('\n========== PUNTO 7: TRAYECTORIAS (insumo) =========\n');
trayectoria_test;

fprintf('\n========== PUNTO 8: DINAMICA INVERSA ==============\n');
dinamica_inversa;

fprintf('\n========== PUNTO 10: MODELO DE MOTOR Y SENSORES =========\n');
test_motor_model;

fprintf('\n#########################################################\n');
fprintf('#  EJECUCION COMPLETADA\n');
fprintf('#########################################################\n');
