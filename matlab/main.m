% main.m
% Ejecucion automatica de la Parte 1 (Puntos 5, 6 y 7) del proyecto:
% robot dibujante de 2 GdL (triangulo de Reuleaux).
%
%   Ejecuta, en orden:
%       Punto 5  cinematica_directa   (DH, matrices homogeneas, validacion)
%       Punto 6  cinematica_inversa   (Jacobiano, IK, configuraciones)
%       Punto 7  trayectoria_test     (Reuleaux, splines, trayectorias articulares)
%       Simulacion simulacion_plotter (animacion con mallas CAD)
%
%   Los scripts imprimen sus resultados en consola y exportan sus figuras
%   vectoriales (.eps) a docs/informe-1/figuras/; la simulacion exporta el
%   video (.mp4) y el GIF (.gif) a docs/media/.
%
%   Uso:  >> main
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
fprintf('#  EJECUCION AUTOMATICA - PARTE 1 (Puntos 5-7 + simul.) #\n');
fprintf('#########################################################\n\n');

fprintf('\n========== PUNTO 5: CINEMATICA DIRECTA ==========\n');
cinematica_directa;

fprintf('\n========== PUNTO 6: CINEMATICA INVERSA ==========\n');
cinematica_inversa;

fprintf('\n========== PUNTO 7: TRAYECTORIAS ================\n');
trayectoria_test;

fprintf('\n========== SIMULACION ANIMADA (CAD) =============\n');
simulacion_plotter;

fprintf('\n#########################################################\n');
fprintf('#  EJECUCION COMPLETADA\n');
fprintf('#########################################################\n');
