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

- Sensores de proximidad IR: ps0–ps7. Los frontales son ps0 y ps7;
los laterales izquierdos ps5/ps6 y los derechos ps1/ps2.
- Encoders: sensores de posición left wheel sensor y right wheel sensor,
que entregan el ángulo acumulado de cada rueda en radianes.
- Motores: left wheel motor y right wheel motor, en modo velocidad.

Parámetros físicos del e-puck: radio de rueda r = 0.0205 m,
distancia entre ruedas L = 0.052 m.

## 3. **Frecuencia de muestreo**
El controlador se ejecuta al basicTimeStep del mundo. Con un paso de
(completar, p. ej. 32) ms:

- `Ts = 0.064 s`
- `fs = 1/Ts = 15.6 Hz`
- Muestras registradas por experimento: `____`

## 4. **Análisis de las señales registradas**
El controlador registra en registro.csv las lecturas crudas de los sensores
IR, los encoders y todas las señales derivadas. (Describir el ruido observado,
las variaciones y tendencias a partir de fig_crudas.png.)

## 5. **Estimación del avance mediante encoders**
El avance lineal entre dos instantes se obtiene de s = r·θ. Para el robot
completo se promedia el avance de ambas ruedas:

```
Δd = r · (Δθ_izq + Δθ_der) / 2
```

donde `Δθ` es la diferencia de lectura del encoder entre dos pasos.

## 6. **Filtro simple aplicado**
Se aplica una media móvil de WINDOW = 5 muestras sobre la distancia
frontal. (Comparar la señal cruda y la filtrada con fig_comparacion.png:
reducción de ruido vs. retardo introducido.)

## 7. **Filtro de Kalman**
Filtro de Kalman escalar para estimar la distancia frontal d.

**Etapa de predicción** (con encoders):

```
d_pred = d̂ + Δd      (Δd = −avance, porque avanzar reduce la distancia)
P_pred = P + Q
```

**Etapa de corrección** (con sensores IR frontales):

```
K      = P_pred / (P_pred + R)
d̂      = d_pred + K·(z − d_pred)
P      = (1 − K)·P_pred
```

Parámetros usados: `Q = ____`, `R = ____`. *(Justificar la elección y comentar
el efecto sobre la ganancia `K` observada en `fig_ganancia.png`.)*

## 8. **Lógica de navegación reactiva**
- Si la distancia estimada `d̂ > SAFE_HIGH` → el robot avanza.
- Si `d̂ < SAFE_LOW` → el robot gira.
- Se usan dos umbrales (histéresis) para evitar giros innecesarios cerca del
límite.
- La dirección del giro se decide con los sensores laterales: si el obstáculo
está más cerca por la izquierda, gira a la derecha, y viceversa.

## 9. **Resultados en los escenarios de prueba**

### **Escenario 1 — entorno simple**


### **Escenario 2 — entorno complejo**
El segundo escenario presenta múltiples obstáculos y pasillos estrechos para evaluar la robustez de la fusión sensorial.

![Escenario Complejo](/img/EC.png)

## 10. **Conclusiones**


## 11. **Instrucciones de ejecucion**
