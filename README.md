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
│   │   └── figuras/         Figuras del informe (vectoriales y capturas)
│   ├── informe-2/           Parte 2: modelado dinámico (Puntos 8-11)
│   ├── informe-final/       Parte 3: fabricación y pruebas (Puntos 12-14)
│   └── media/               Video/GIF de la simulación animada
├── cad/                     Modelo CAD (SolidWorks)
│   └── export/              Mallas STL, plotter_mallas_CAD.mat y propiedades de masa
├── matlab/                  Código MATLAB
│   ├── lib/                 Funciones genéricas reutilizables (FK, Jacobiano, IK, ...)
│   ├── params/              Parámetros del robot (modelo cinemático/dinámico)
│   ├── cinematica/          Cinemática directa e inversa (Puntos 5-6)
│   ├── trayectorias/        Planeación de trayectorias y simulación CAD (Punto 7)
│   ├── dinamica/            Dinámica Newton-Euler (Punto 8)
│   └── control/             Modelos y control en Simulink (Puntos 10-11)
├── firmware/                Firmware Arduino (Puntos 13-14)
└── hardware/                Electrónica e impresión 3D (Puntos 12-13)
```

## Requisitos de software

- **MATLAB** (R2026a) + [Robotics Toolbox for MATLAB](https://petercorke.com/toolboxes/robotics-toolbox/)
  (Peter Corke, LGPL). La ruta del toolbox se detecta automáticamente o se fija con la
  variable de entorno `RVC_ROOT`.
- **Simulink** (bloques base). El modelo de P10 no requiere Simscape Electrical.
- **SolidWorks** para el CAD (o cualquier visor de STEP/STL).
- **Arduino IDE** para el firmware.

## Cómo reproducir

```matlab
% En MATLAB, desde la raíz del repositorio:
addpath(genpath('matlab'));
main
```

`main.m` ejecuta la cinemática directa (Punto 5), la cinemática inversa
(Punto 6), la planeación de trayectorias (Punto 7) y la simulación animada con
las mallas CAD. Las figuras vectoriales se exportan a `docs/informe-1/figuras/`
y el video/GIF de la simulación a `docs/media/`.

### Parte 2 (P8 y P10)

```matlab
main2
```

`main2.m` ejecuta:

- **P7** – trayectorias (insumo).
- **P8** – dinámica inversa y torques por articulación.
- **P10** – modelo de motor DC de primer orden y sensores en Simulink, con la
  carga mecánica de P8.

El modelo de P10 está en `matlab/control/`:

- `param_motor.m` – parámetros del S3003 (`Ke = Kt`, `R`, `J`; `L = B = 0`).
- `param_sensores.m` – sensores de posición, corriente y velocidad.
- `build_motor_model.m` + `motor_2gdl.slx` – planta y sensores con bloques
  base (sin Simscape Electrical).
- `carga_mecanica.m` – par de carga de P8 como series de tiempo.
- `test_motor_model.m` – verificación contra el datasheet y figuras en
  `docs/informe-2/figuras/`.

Supuestos: el S3003 es un servo con lazo interno; para simulación se usa un
motor DC equivalente de primer orden calibrado con el datasheet (`L` y `B`
despreciadas, `J` estimada). En la parte experimental se usa el S3003 tal como
se compró (PWM). El detalle está en `docs/informe-2/p10_notas.md`.

La validación cruzada de P8 contra el Robotics Toolbox se ejecuta con
`matlab/dinamica/validar_dinamica.m`.

## Estado

| Entregable | Puntos | Estado |
|---|---|---|
| Informe 1 (Semana 10) | 1-7 | Completo |
| Informe 2 (Semana 13) | 8-11 | En progreso (P8, P10) |
| Informe final (Semana 16) | 12-14 | Pendiente |

## Autores

- José Elkin Estrada Morales — estradam_jose@javeriana.edu.co
- Juan David Rodríguez Franco — juandrodriguezf@javeriana.edu.co
- David Santiago Mojica Lugo — david-mojical@javeriana.edu.co

## Licencia

Código y hardware bajo licencia MIT (ver `LICENSE`). El *Robotics Toolbox for
MATLAB* de Peter Corke es de terceros (LGPL) y no se redistribuye en este
repositorio.
