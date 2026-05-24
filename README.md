# **Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores en Webots**

Asignatura: Robótica y Sistemas Autónomos (ICI 4150)
Integrantes: Nicolás Fuentes, Eva Ponce, Esteban Schanze, Juan Geraldo

## 1. **Objetivo**
Implementar un sistema de navegación reactiva para un robot móvil diferencial
en Webots, usando sensores de distancia y encoders de rueda, aplicando filtrado
sobre las mediciones y un filtro de Kalman para estimar la distancia frontal a
obstáculos.

## 2. **Robot y sensores utilizados**
Se utiliza el robot e-puck de Webots (robot diferencial con dos ruedas
motrices independientes). Dispositivos empleados:

- **Sensores de proximidad IR** (`ps0`–`ps7`): Los frontales son `ps0` y `ps7`;
  los diagonales `ps1` (derecho) y `ps6` (izquierdo); los laterales `ps2`
  (derecho) y `ps5` (izquierdo); los traseros `ps3` y `ps4`. El valor crudo
  se convierte a distancia en metros mediante interpolación lineal por tramos
  sobre una tabla de lookup calibrada.
- **Encoders** (`left wheel sensor`, `right wheel sensor`): Entregan la
  posición angular acumulada de cada rueda en radianes.
- **Motores** (`left wheel motor`, `right wheel motor`): Configurados en modo
  velocidad (posición objetivo en infinito).
- **Sensores láser frontales** (`antena_izq`, `antena_der`): Dos sensores de
  distancia tipo láser montados al frente del robot con una apertura de
  ≈ 22.5° cada uno. Entregan directamente la distancia en metros al obstáculo
  más cercano (rango 0–1 m). Constituyen la fuente principal de medición para
  la etapa de corrección del filtro de Kalman.

Parámetros físicos del e-puck: radio de rueda *r* = 0.0205 m,
distancia entre ruedas *L* = 0.052 m.

## 3. **Frecuencia de muestreo**
El controlador se ejecuta al `basicTimeStep` del mundo, con un paso de 64 ms:

- Periodo de muestreo: *Tₛ* = 0.064 s
- Frecuencia de muestreo: *fₛ* = 1 / *Tₛ* ≈ 15.625 Hz
- Muestras registradas por experimento: `____`

## 4. **Análisis de las señales registradas**
El controlador genera automáticamente un archivo `registro.csv` en el
directorio del controlador con una fila por cada paso de simulación. Las
columnas registradas son:

| Grupo | Columnas |
|---|---|
| Tiempo | `tiempo_s` |
| IR crudos | `ps0_raw` … `ps7_raw` |
| Láser frontal | `laser_izq_m`, `laser_der_m` |
| Encoders | `enc_izq_rad`, `enc_der_rad` |
| Desplazamiento | `delta_d_m`, `total_d_m` |
| Señal cruda / filtrada | `z_raw_m`, `z_filtrada_m` |
| Filtro de Kalman | `kalman_x_m`, `kalman_P`, `kalman_K` |
| Estado | `modo_evasion` (0/1) |

Las columnas `z_raw_m`, `z_filtrada_m` y `kalman_K` quedan vacías en los
pasos donde ningún láser detecta obstáculo.

*(Gráficos por generar a partir de `registro.csv`:)*

- **Señales crudas** — lecturas directas del láser y los sensores IR a lo
  largo del tiempo, mostrando el nivel de ruido inherente a cada sensor.
- **Señal filtrada vs. cruda** — comparación de `z_raw_m` y `z_filtrada_m`
  para evaluar la reducción de ruido y el retardo introducido por la media
  móvil.
- **Estimación Kalman** — evolución de `kalman_x_m` superpuesta con la
  medición filtrada, evidenciando la fusión entre el modelo de predicción
  (encoders) y la corrección (láser).
- **Ganancia de Kalman** — comportamiento de *K(k)* a lo largo del tiempo;
  valores altos indican mayor confianza en la medición, valores bajos indican
  mayor confianza en la predicción.

## 5. **Estimación del avance mediante encoders**
Los encoders entregan la posición angular acumulada θ de cada rueda en
radianes. El arco recorrido por una rueda entre dos instantes consecutivos es:

```
sᵢ = r · Δθᵢ
```

donde *r* es el radio de la rueda y Δθᵢ = θᵢ(k) − θᵢ(k−1) es la diferencia
de lectura del encoder entre el paso actual y el anterior.

El desplazamiento lineal del centro del robot (promedio de ambas ruedas) se
calcula como:

```
Δd = (s_izq + s_der) / 2 = r · (Δθ_izq + Δθ_der) / 2
```

Este valor se acumula en cada paso para obtener el desplazamiento total
recorrido, y también alimenta la etapa de predicción del filtro de Kalman:
al avanzar Δd, la distancia estimada al obstáculo frontal disminuye en esa
misma magnitud.

**Nota:** Esta estimación es válida para trayectorias rectas o curvas suaves.
En giros pronunciados, el promedio de arcos subestima el desplazamiento real
del centro, pero para la navegación reactiva implementada resulta
suficientemente precisa.

## 6. **Filtro simple aplicado**
Se aplica un filtro de **media móvil** de *N* = 5 muestras sobre la medición
láser frontal antes de alimentarla al filtro de Kalman. En cada paso donde
hay detección, la medición cruda *z(k)* se agrega a un buffer circular y se
calcula:

```
z̄(k) = (1/N) · Σ z(k−i),   i = 0, …, N−1
```

Cuando el láser pierde detección, el buffer se vacía para evitar arrastrar
mediciones obsoletas.

**Efecto:** suaviza picos de ruido aislados a costa de introducir un retardo
de hasta *N*/2 pasos (≈ 160 ms con *N* = 5). *(Comparar la señal cruda y
la filtrada en los gráficos correspondientes.)*

## 7. **Filtro de Kalman**
Filtro de Kalman escalar para estimar la distancia frontal *d* al obstáculo.

**Estado:** *x* = distancia al obstáculo (metros).

**Modelo de transición** (predicción con encoders): el robot avanza Δ*d*, por
lo que el obstáculo se acerca en esa misma magnitud:

```
x̂⁻(k) = x̂(k−1) − Δd(k)
P⁻(k)  = P(k−1) + Q
```

**Modelo de observación** (corrección con láser frontal): la medición *z(k)*
es la menor distancia reportada por los dos sensores láser cuando al menos uno
detecta un obstáculo dentro de su rango:

```
K(k)  = P⁻(k) / (P⁻(k) + R)
x̂(k)  = x̂⁻(k) + K(k) · (z(k) − x̂⁻(k))
P(k)  = (1 − K(k)) · P⁻(k)
```

Cuando ningún láser detecta obstáculo (ambas lecturas ≥ 0.95 m), se resetea
la estimación al máximo (1.0 m) y se incrementa la varianza para que el filtro
reaccione rápidamente ante una nueva detección.

Parámetros usados: *Q* = 1×10⁻⁴, *R* = 1×10⁻³. *Q* bajo implica alta
confianza en el modelo cinemático de los encoders; *R* mayor que *Q* refleja
que la medición láser tiene más incertidumbre relativa que la predicción
basada en odometría.

## 8. **Lógica de navegación reactiva**
La decisión de movimiento utiliza **histéresis** (dos umbrales) sobre la
distancia estimada por el Kalman para evitar oscilaciones en la frontera de
decisión:

- Si el robot está en modo **libre** y *d̂* cae por debajo del **umbral bajo**
  (0.20 m): se **activa** el modo evasión.
- Si el robot está en modo **evasión** y *d̂* supera el **umbral alto**
  (0.30 m): se **desactiva** el modo evasión y vuelve a avanzar.
- Mientras *d̂* permanezca entre ambos umbrales, el modo actual se mantiene.

Dentro del modo evasión, la dirección del giro se decide comparando las
lecturas láser izquierda y derecha:

- Si la diferencia |*d_izq* − *d_der*| < margen de simetría (0.75 m): el
  peligro se considera frontal puro y el robot **rota en su lugar**.
- Si el obstáculo está más cerca por la izquierda: **gira a la derecha**.
- Si el obstáculo está más cerca por la derecha: **gira a la izquierda**.

Los sensores IR se leen y convierten a distancia como referencia de
monitoreo, pero no participan directamente en la decisión de navegación.

## 9. **Resultados en los escenarios de prueba**

### **Escenario 1 — entorno simple**
El primer escenario consiste en una arena rectangular cerrada (4 paredes) sin
obstáculos interiores. El robot avanza en línea recta hasta detectar una pared
frontal, momento en el que la estimación de Kalman cae bajo el umbral de
evasión y se activa el giro. Al completar la maniobra, el robot retoma la
marcha recta hacia otra pared, repitiendo el ciclo indefinidamente.

Se observa que:
- La transición entre modo libre y evasión es estable gracias a la
  histéresis; no se producen oscilaciones en la frontera del umbral.
- En las esquinas, donde ambos sensores láser detectan paredes
  simultáneamente con distancias similares, el robot interpreta la situación
  como peligro simétrico y rota en su lugar hasta despejar una dirección.
- El desplazamiento total acumulado crece de forma continua, confirmando que
  el robot se mantiene en movimiento sin quedar atrapado.

### **Escenario 2 — entorno complejo**
El segundo escenario presenta múltiples obstáculos (cajas de madera y una
maceta circular) distribuidos dentro de la arena, generando pasillos estrechos
y geometrías variadas.

![Escenario Complejo](/img/EC.png)

Se observa que:
- El robot esquiva exitosamente tanto las cajas rectangulares como la maceta
  de perfil circular, adaptando la dirección de giro según el lado por el que
  detecta mayor proximidad.
- A pesar del espacio libre entre los obstáculos, el robot tiende a
  establecer un recorrido cíclico aproximadamente circular, resultado de la
  lógica de evasión que favorece giros consistentes hacia el lado más
  despejado.
- La fusión Kalman mantiene una estimación suave y estable de la distancia
  frontal, evitando reacciones bruscas ante mediciones ruidosas puntuales.

## 10. **Conclusiones**


## 11. **Instrucciones de ejecución**

### Requisitos
- [Webots R2025a](https://cyberbotics.com/) (o compatible)
- Python 3.x (incluido con Webots)

### Pasos
1. Clonar el repositorio:
   ```bash
   git clone https://github.com/evaponce2javi/Lab2_Robotica.git
   cd Lab2_Robotica
   ```
2. Abrir el mundo en Webots:
   - Desde Webots: **File → Open World…** → seleccionar `webots/worlds/lab2.wbt`.
   - O bien desde la línea de comandos:
     ```bash
     webots webots/worlds/lab2.wbt
     ```
3. Al abrir el mundo, el controlador `e-puck_lab2_corregido` se carga
   automáticamente (está referenciado en el archivo `.wbt`).
4. Presionar el botón ▶ (Play) en Webots para iniciar la simulación.
5. La consola de Webots mostrará la telemetría del robot (lecturas láser,
   estado Kalman, lecturas IR) cada ~0.5 s.
