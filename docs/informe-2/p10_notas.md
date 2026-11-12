# Punto 10 — Modelo de los motores y sensores

## Objetivo

Implementar en Simulink el modelo de cada motor y los sensores necesarios para
el control de bajo nivel de cada grado de libertad, ajustado con los parámetros
del actuador seleccionado. Para el proyecto se usa el servomotor **Futaba
S3003** (4,8–6 V, 4,1 kg·cm, 0,19 s/60°).

El S3003 es un servo integrado: incluye motor, reductora, potenciómetro y
controlador analógico. El lazo de posición ya viene cerrado internamente y el
servo solo recibe PWM. Por eso, para la simulación se adopta un **motor DC
equivalente de primer orden** referido al eje de salida, calibrado con el
datasheet; en la parte experimental se usa el S3003 tal como se compró. Esta
simplificación fue aprobada para el alcance de P10/P11.

## Modelo equivalente

Por cada articulación se resuelve:

```
i        = (V - Ke·ω) / R
J·dω/dt  = Kt·i - τ_load
dθ/dt    = ω
```

Se desprecian la inductancia `L` y el amortiguamiento viscoso `B` para obtener
el modelo de primer orden. La salida es el ángulo del eje de salida del servo.

| Parámetro | Valor | Fuente |
|---|---:|---|
| `Ke = Kt` | 1,0886 V·s/rad = N·m/A | Datasheet (6 V) |
| `R` | 16,245 Ω | Datasheet |
| `J` | 3e-4 kg·m² | Estimada |
| `L` | 0 | Despreciada |
| `B` | 0 | Despreciada |
| `Vmax` | 6 V | Datasheet |
| `ω_nl` | 5,5116 rad/s | Datasheet (0,19 s/60°) |
| `τ_stall` | 0,4021 N·m | Datasheet (4,1 kg·cm) |
| `i_stall` | 0,3693 A | Derivada |
| `K = 1/Ke` | 0,9186 rad/s/V | Derivada |
| `τm = J·R/(Kt·Ke)` | 4,11 ms | Derivada |
| Límites articulares | ±90° | Recorrido del servo |

La inercia no se publica en el datasheet; se adopta un valor típico de un servo
pequeño. La incertidumbre de `J` afecta la constante de tiempo, no la velocidad
de régimen ni el par de bloqueo.

## Sensores

El S3003 no expone el potenciómetro ni la corriente del motor. Los sensores se
modelan de forma genérica para el lazo de P11:

| Sensor | Modelo | Parámetros |
|---|---|---|
| Posición | cuantización + ruido + saturación | ±90°, resolución 0,5°, ruido 0,05°, offset 0 |
| Corriente | filtro pasa-bajos + ruido + saturación | ±1 A, ruido 2 mA, ancho de banda 1000 Hz |
| Velocidad | derivada filtrada de la posición | corte 20 Hz |

Periodo de muestreo: `Ts = 0,01 s`.

## Implementación en Simulink

Archivos en `matlab/control/`:

- `param_motor.m` – parámetros del motor.
- `param_sensores.m` – parámetros de los sensores.
- `build_motor_model.m` – construye el modelo.
- `motor_2gdl.slx` – modelo con dos instancias (`Joint1`, `Joint2`).
- `carga_mecanica.m` – par de carga de P8 como series de tiempo.
- `test_motor_model.m` – verificación y figuras.

Entradas del modelo: `V1`, `V2` (tensión) y `tau1`, `tau2` (par de carga).
Salidas: `y1`, `y2 = [θ_true, ω_true, i_true, θ_meas, ω_meas, i_meas]`.

El modelo usa solo bloques base de Simulink; no requiere Simscape Electrical.

## Carga mecánica de P8

`carga_mecanica.m` reproduce la trayectoria de P7 y calcula el par de P8 con
`ID`. Resultados: `τ1 = 0,1774 mN·m` (RMS 0,0689) y `τ2 = 0,0320 mN·m`
(RMS 0,0068). Al simular con esa carga, la velocidad cambia solo
`+0,0013 rad/s` (0,024 %): el actuador está muy sobredimensionado.

## Verificación

| Prueba | Esperado | Obtenido |
|---|---:|---:|
| `ω_nl` a 6 V | 5,5116 rad/s | 5,5116 rad/s |
| `ω_nl` a 3 V | 2,7558 rad/s | 2,7558 rad/s |
| `τm` | 4,11 ms | 4,3 ms |
| `i_stall` | 0,3693 A | 0,3693 A |
| `τ_stall` | 0,4021 N·m | 0,4021 N·m |
| Cambio por carga de P8 | 0 | +0,0013 rad/s (0,024 %) |

Sensores: posición con error máximo 0,00494 rad (0,28°) y RMS 0,00211 rad;
corriente con error máximo 0,271 A en el escalón (RMS 0,0168 A); velocidad con
error máximo 3,18 rad/s (RMS 0,754 rad/s), debido a la derivada de la posición
cuantizada. Para P11 conviene filtrar más la velocidad o usar un observador.

### Figuras

- `figuras/motor_escalon.eps` – escalones de 6 V y 3 V: velocidad, corriente,
  posición y detalle del transitorio con `τm`.
- `figuras/motor_sensores.eps` – señales reales vs. medidas de posición,
  corriente y velocidad.
- `figuras/motor_par_velocidad.eps` – mapa par-velocidad del S3003 con los
  puntos de operación de P8.

## Supuestos y limitaciones

- El S3003 tiene su propio lazo de control; el modelo DC es un equivalente para
  simulación. En la parte experimental se usa el servo tal como se compró, con
  entrada PWM.
- `L` y `B` se desprecian; `J` es estimada. No se modelan zona muerta ni
  holgura del servo en P10; son no idealidades que pueden incluirse en P11.
- Los sensores no están disponibles en el servo real; son modelos genéricos
  para el lazo de control.
- No se usa Simscape Electrical porque no está licenciado; se emplean bloques
  base de Simulink.
- El dato de 0,19 s/60° corresponde al servo en lazo cerrado; P10 verifica la
  planta (velocidad y par) y P11 verifica la respuesta en lazo cerrado.

## Referencias

- Futaba S3003 Standard Servo, hoja de datos:
  <https://futabausa.com/wp-content/uploads/2019/07/S3003.pdf>
- Enunciado del proyecto, Parte 2, puntos 10 y 11.
