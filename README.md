# Robot manipulador dibujante de 2 GdL

Robot manipulador de **2 grados de libertad** que traza de forma autónoma el
**triángulo de Reuleaux** sobre una superficie (papel). El proyecto cubre el
ciclo completo **modelado → simulación → experimento → fabricación**, incluida
la electrónica de control y el control de movimiento.

Asignatura *Herramientas de Robótica* — Pontificia Universidad Javeriana.

## Tarea

El efector final (un marcador) debe recorrer la trayectoria continua de un
triángulo de Reuleaux de ancho constante 10 cm. Para cada punto de la
trayectoria el sistema resuelve la cinemática inversa y genera los comandos de
las articulaciones:

```
Generación        Cinemática        Controlador        Robot            Nueva posición
de trayectoria →  inversa     →     de articulador →   (manipulador) →  del efector final
(Reuleaux)        (IK)              (PID)                                │
      ▲                                                                  │
      └──────────────── Cinemática directa (FK) ◄──── variables articulares
```

## Estructura del repositorio

```
├── docs/                    Informes (LaTeX) por entregable
│   ├── informe-1/           Parte 1: especificaciones y modelado cinemático (Puntos 1-7)
│   ├── informe-2/           Parte 2: modelado dinámico (Puntos 8-11)
│   └── informe-final/       Parte 3: fabricación y pruebas (Puntos 12-14)
├── cad/                     Modelo CAD (SolidWorks) + exportaciones STEP/STL
├── matlab/                  Código MATLAB
│   ├── lib/                 Funciones genéricas reutilizables (FK, Jacobiano, IK, ...)
│   ├── cinematica/          Cinemática directa e inversa (Puntos 5-6)
│   ├── trayectorias/        Planeación de trayectorias (Punto 7)
│   ├── dinamica/            Dinámica Newton-Euler (Punto 8)
│   └── control/             Modelos y control en Simulink (Puntos 10-11)
├── firmware/                Firmware Arduino (Puntos 13-14)
└── hardware/                Electrónica e impresión 3D (Puntos 12-13)
```

## Requisitos de software

- **MATLAB** (R2026a) + [Robotics Toolbox for MATLAB](https://petercorke.com/toolboxes/robotics-toolbox/)
  (Peter Corke, LGPL). La ruta del toolbox se detecta automáticamente o se fija con la
  variable de entorno `RVC_ROOT`.
- **SolidWorks** para el CAD (o cualquier visor de STEP/STL).
- **Arduino IDE** para el firmware.

## Cómo reproducir

```matlab
% En MATLAB, desde la raíz del repositorio:
addpath(genpath('matlab'));
run('matlab/cinematica/main.m')
```

`main.m` valida la cinemática contra el Robotics Toolbox y exporta las figuras
a `docs/informe-1/figuras/`.

## Estado

| Entregable | Puntos | Estado |
|---|---|---|
| Informe 1 (Semana 10) | 1-7 | En desarrollo |
| Informe 2 (Semana 13) | 8-11 | Pendiente |
| Informe final (Semana 16) | 12-14 | Pendiente |

## Autores

- José Elkin Estrada Morales — estradam_jose@javeriana.edu.co
- Juan David Rodríguez Franco — juandrodriguezf@javeriana.edu.co
- David Santiago Mojica Lugo — david-mojical@javeriana.edu.co

## Licencia

Código y hardware bajo licencia MIT (ver `LICENSE`). El *Robotics Toolbox for
MATLAB* de Peter Corke es de terceros (LGPL) y no se redistribuye en este
repositorio.
