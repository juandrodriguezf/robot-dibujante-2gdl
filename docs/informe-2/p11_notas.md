# P11 – Control de bajo nivel (explicación simple)

## 1. ¿Qué hace el servo Futaba S3003 por dentro?
Funciona como un **termostato**, pero con el ángulo:

1. Tú (el ESP32) mandas un pulso PWM. Su ancho dice **qué ángulo quieres**.
2. Un potenciómetro pegado al eje mide **qué ángulo tiene de verdad**.
3. Un comparador calcula **cuánto falta**: `falta = deseado − real`.
4. Más falta → más voltaje al motor (máximo 6 V). Poca falta → poco voltaje.
5. Cuando `falta = 0` el motor se detiene.

Eso, en control, se llama **control proporcional (P)**: `V = Kp · falta`.
Dibujo: `figuras/diag_servo_simple.png` (versión detallada: `diag_s3003_interno.png`).

## 2. ¿Qué se agregó en Simulink?
Su modelo P10 solo tenía el motor y los sensores. Dentro de cada `JointX` se agregó el
"termostato" (`matlab/control/build_control_model.m` → `motor_2gdl_ctrl.slx`):

```
V = límite_±6V [ Kp·(ángulo_deseado − ángulo_medido) + Kff·velocidad_deseada ]
```
* `Kp = 43,5 V/rad` → es lo que hace el servo real (se calculó para que no se pase ni oscile).
* `Kff·velocidad_deseada` → un "empujón" extra porque ya sabemos a qué velocidad va la trayectoria.
  El servo comercial **no** tiene este empujón; sirve para la simulación.
* Se evalúa cada 0,01 s y usa el ángulo **medido** (con ruido y pasos de 0,5°), no el real.

Dibujo: `figuras/diag_lazo_bajo_nivel.png`.

## 3. Resultados (MATLAB, trayectoria real de P7)
| Prueba | Resultado |
|---|---|
| Escalón 30° | sobreimpulso 0,18 % (Joint1) y 0 % (Joint2), establecimiento ≈0,11 s, error final ≈0,22° |
| Joint1 sigue la trayectoria | error RMS 0,168° (solo P) / 0,094° (con empujón) |
| Joint2 sigue la trayectoria | error RMS 0,234° (solo P) / 0,091° (con empujón) |
| Esfuerzo del motor | < 0,6 V y < 0,04 A (el límite es 6 V y 0,37 A) |
| Si J real es 0,5× a 2× lo estimado | sigue estable, sobreimpulso ≤ 2,2 % |

Casi todo el error restante viene del sensor (solo distingue pasos de 0,5°).
Se ve en la gráfica un temblor de ±0,25 V en la tensión: es esa cuantización; el servo real lo
evita con su banda muerta.

## 4. ⚠ Problema encontrado en la trayectoria de P7
El servo y su sensor solo giran **±90°**. La trayectoria de P7 pide:

* q1: de 43,7° a 76,4° → cabe.
* q2: de 65,8° a **128,9°** → **se pasa de 90° el 66 % del tiempo**.

Sin corregirlo, Joint2 se descontrola en la simulación (el sensor se queda pegado en 90°).
Solución usada: **montar el brazo con un desfase** para que el centro del servo coincida con
el centro de la trayectoria. Se definió `q_off = [60°, 95°]` en `param_control.m`
(`ángulo_servo = ángulo_articulación − q_off`). Así el servo trabaja en
[−16°, +16°] (Joint1) y [−29°, +34°] (Joint2).
**En el robot físico esto significa poner el horn de cada servo rotado ese ángulo al armarlo**
(o cambiar el tamaño/posición del dibujo). Dibujo: `figuras/fig_limite_servo.png`.

## 5. Cómo correrlo
En MATLAB, en la carpeta raíz: `addpath(genpath('matlab')); main`.
También se puede ejecutar solo P11 con `main_p11`.
**Nota:** el código MATLAB de P11 ya fue verificado y `main_p11` termina completo.
Los números de este documento son los que imprime MATLAB; las figuras de resultados
se guardan como EPS en `docs/informe-2/figuras/`.
