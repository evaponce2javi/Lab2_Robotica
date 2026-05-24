# Controlador para el robot E-puck (Webots)
# Laboratorio 2 de ICI4150-2 - Robotica y Sistemas Autónomos

from controller import Robot
from typing import Tuple


# ─── constantes del robot e-puck ───────────────────────────────────────────────

WHEEL_RADIUS_M   = 0.0205
AXLE_DISTANCE_M  = 0.052
TIMESTEP_MS      = 64
SAMPLE_TIME_S    = TIMESTEP_MS / 1000.0
SAMPLE_FREQ_HZ   = 1.0 / SAMPLE_TIME_S

# sensores laser frontales (antena_izq, antena_der): retornan distancia directa en metros.
# por encima del threshold se interpreta como "sin obstaculo en rango"
LASER_MAX_DIST_M          = 1.0
LASER_DETECTION_THRESHOLD = 0.95

# tabla de lookup del IR del e-puck en webots.
# el sensor no sigue una curva limpiamente exponencial, por eso interpolamos
# por tramos en lugar de usar una formula global, evitando errores fuera del rango
# calibrado.
SENSOR_LOOKUP_TABLE = (
    (0.000, 4095.00),
    (0.005, 2133.33),
    (0.010, 1465.73),
    (0.015,  601.46),
    (0.020,  383.84),
    (0.030,  234.93),
    (0.040,  158.03),
    (0.050,  120.00),
    (0.060,  104.09),
    (0.070,   67.19),
)

SENSOR_MAX_DIST_M          = SENSOR_LOOKUP_TABLE[-1][0]
SENSOR_MIN_DIST_M          = SENSOR_LOOKUP_TABLE[0][0]
SENSOR_DETECTION_THRESHOLD = 80.0   # raw por debajo de esto => sin obstaculo en rango

# velocidades y umbrales de navegacion
VEL_CRUCERO     = 5.0
VEL_EVASION     = 2.0
UMBRAL_ALERTA   = 0.25   # distancia kalman desde la que se activa la evasion
MARGEN_SIMETRIA = 0.75   # diferencia laser izq/der bajo la cual el peligro es frontal puro

# filtro de kalman:
#   Q bajo => confiamos bastante en el modelo cinematico de los encoders
#   R mas alto que Q => el laser tiene mas ruido que el modelo
KALMAN_Q = 1e-4
KALMAN_R = 1e-3


# ─── helpers ───────────────────────────────────────────────────────────────────

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def raw_to_distance(raw_value: float) -> Tuple[float, bool]:
    """Convierte raw IR a distancia en metros con interpolacion lineal por tramos.

    Retorna:
        distance_m: distancia estimada, saturada al rango confiable del sensor
        detected:   True si raw supera el umbral => hay obstaculo en rango
    """
    if raw_value <= SENSOR_DETECTION_THRESHOLD:
        return SENSOR_MAX_DIST_M, False

    first_distance, first_raw = SENSOR_LOOKUP_TABLE[0]
    if raw_value >= first_raw:
        return first_distance, True

    for i in range(len(SENSOR_LOOKUP_TABLE) - 1):
        d1, raw1 = SENSOR_LOOKUP_TABLE[i]
        d2, raw2 = SENSOR_LOOKUP_TABLE[i + 1]
        # raw disminuye cuando aumenta la distancia, por eso raw1 >= raw2
        if raw1 >= raw_value >= raw2:
            alpha = (raw_value - raw1) / (raw2 - raw1)
            return clamp(d1 + alpha * (d2 - d1), SENSOR_MIN_DIST_M, SENSOR_MAX_DIST_M), True

    return SENSOR_MAX_DIST_M, False


def encoder_delta_to_displacement(delta_left_rad: float, delta_right_rad: float) -> float:
    """Estima desplazamiento lineal del centro del robot desde deltas angulares.

    Webots entrega posicion angular en radianes (no pulsos), por eso la
    conversion es directamente s = r * delta_phi. Valido para movimiento recto
    o curvas suaves; no considera el componente de giro.
    """
    arc_left  = WHEEL_RADIUS_M * delta_left_rad
    arc_right = WHEEL_RADIUS_M * delta_right_rad
    return (arc_left + arc_right) / 2.0


# ─── filtro de kalman 1D ────────────────────────────────────────────────────────

class KalmanFilter1D:
    """Filtro de kalman escalar para estimar la distancia frontal al obstaculo.

    estado  x  = distancia al obstaculo (metros)
    modelo    : x_k = x_{k-1} - delta_s_k  (robot avanza, obstaculo se acerca)
    medicion  : z_k = min(laser_izq, laser_der)  cuando hay deteccion
    """

    def __init__(self, initial_estimate: float, initial_variance: float) -> None:
        self.x = initial_estimate
        self.p = initial_variance

    def predict(self, delta_displacement: float) -> None:
        """El obstaculo se acerca segun lo que avanzo el robot este step."""
        self.x = clamp(self.x - delta_displacement, 0.0, LASER_MAX_DIST_M)
        self.p = self.p + KALMAN_Q

    def update(self, measurement: float) -> None:
        """Fusion de prediccion y medicion laser; ganancia K balancea ambas fuentes."""
        kalman_gain = self.p / (self.p + KALMAN_R)
        self.x = clamp(self.x + kalman_gain * (measurement - self.x), 0.0, LASER_MAX_DIST_M)
        self.p = (1.0 - kalman_gain) * self.p

    def reset_to_no_detection(self) -> None:
        """Sin z_k disponible (laser fuera de rango): resetear al maximo y subir varianza.

        Subir la varianza asegura que kalman reaccione rapido cuando vuelva
        a aparecer una medicion real, en vez de quedarse pegado en el maximo.
        """
        self.x = LASER_MAX_DIST_M
        self.p = max(self.p + KALMAN_Q, KALMAN_R)

    @property
    def estimate(self) -> float:
        return self.x


# ─── inicializacion del robot ───────────────────────────────────────────────────

robot = Robot()

motor_right = robot.getDevice('right wheel motor')
motor_left  = robot.getDevice('left wheel motor')
motor_right.setPosition(float('inf'))
motor_left.setPosition(float('inf'))
motor_right.setVelocity(0.0)
motor_left.setVelocity(0.0)

antena_izq = robot.getDevice('antena_izq')
antena_der = robot.getDevice('antena_der')
antena_izq.enable(TIMESTEP_MS)
antena_der.enable(TIMESTEP_MS)

# mapeo de sensores IR por posicion:
#   ps0 => frontal derecho    ps7 => frontal izquierdo
#   ps1 => diagonal derecho   ps6 => diagonal izquierdo
#   ps2 => lateral derecho    ps5 => lateral izquierdo
#   ps3, ps4 => traseros
# se habilitan los 8 para monitorear si una pared entra por diagonal o lateral
# antes de que el laser frontal la vea
SENSOR_NAMES = ('ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7')
proximity_sensors = {}
for name in SENSOR_NAMES:
    sensor = robot.getDevice(name)
    sensor.enable(TIMESTEP_MS)
    proximity_sensors[name] = sensor

encoder_left  = robot.getDevice('left wheel sensor')
encoder_right = robot.getDevice('right wheel sensor')
encoder_left.enable(TIMESTEP_MS)
encoder_right.enable(TIMESTEP_MS)


# ─── estado inicial ─────────────────────────────────────────────────────────────

prev_encoder_left  = 0.0
prev_encoder_right = 0.0
encoders_initialized = False

total_displacement = 0.0

kalman = KalmanFilter1D(initial_estimate=LASER_MAX_DIST_M, initial_variance=1.0)

print(f"[*] controlador inicializado => t_s={SAMPLE_TIME_S:.4f}s, f_s={SAMPLE_FREQ_HZ:.2f}hz")
print(f"[*] IR => rango confiable ~{SENSOR_MAX_DIST_M:.3f}m, umbral raw={SENSOR_DETECTION_THRESHOLD:.1f}")
print(f"[*] kalman => Q={KALMAN_Q}, R={KALMAN_R}")


# ─── loop principal ─────────────────────────────────────────────────────────────

while robot.step(TIMESTEP_MS) != -1:

    # 1. encoders: delta angular -> desplazamiento lineal -> prediccion kalman
    current_encoder_left  = encoder_left.getValue()
    current_encoder_right = encoder_right.getValue()

    if not encoders_initialized:
        prev_encoder_left  = current_encoder_left
        prev_encoder_right = current_encoder_right
        encoders_initialized = True
        delta_displacement = 0.0
    else:
        delta_left_rad  = current_encoder_left  - prev_encoder_left
        delta_right_rad = current_encoder_right - prev_encoder_right
        delta_displacement = encoder_delta_to_displacement(delta_left_rad, delta_right_rad)
        prev_encoder_left  = current_encoder_left
        prev_encoder_right = current_encoder_right

    total_displacement += abs(delta_displacement)

    # 2. IR: solo referencia, el laser es la fuente principal del kalman
    raw_values  = {name: proximity_sensors[name].getValue() for name in SENSOR_NAMES}
    ir_readings = {name: raw_to_distance(raw_values[name]) for name in SENSOR_NAMES}

    # 3. laser frontal: z_k es el sensor mas cercano (peligro mas inminente)
    dist_izq = antena_izq.getValue()
    dist_der = antena_der.getValue()

    laser_measurements = [d for d in (dist_izq, dist_der) if d < LASER_DETECTION_THRESHOLD]
    front_detected = len(laser_measurements) > 0

    if front_detected:
        kalman.predict(delta_displacement)
        kalman.update(min(laser_measurements))
    else:
        kalman.reset_to_no_detection()

    distancia_estimada_frontal = kalman.estimate

    # 4. navegacion reactiva
    if distancia_estimada_frontal < UMBRAL_ALERTA:
        diferencia_laser = abs(dist_izq - dist_der)

        if diferencia_laser < MARGEN_SIMETRIA:
            # peligro simetrico: rotar en sitio
            v_izq, v_der = VEL_CRUCERO, -VEL_CRUCERO
        elif dist_izq <= dist_der:
            # peligro mas cerca por izquierda => girar derecha
            v_izq, v_der = VEL_CRUCERO, -VEL_EVASION
        else:
            # peligro mas cerca por derecha => girar izquierda
            v_izq, v_der = -VEL_EVASION, VEL_CRUCERO
    else:
        v_izq, v_der = VEL_CRUCERO, VEL_CRUCERO

    motor_left.setVelocity(v_izq)
    motor_right.setVelocity(v_der)

    # debug periodico cada ~0.5s (8 steps * 64ms)
    step_count = int(robot.getTime() / SAMPLE_TIME_S)
    if step_count % 8 == 0:
        estado = "peligro" if distancia_estimada_frontal < UMBRAL_ALERTA else "libre"

        # estado del laser y kalman
        print(
            f"[*] laser => izq={dist_izq:.3f}m  der={dist_der:.3f}m  "
            f"kalman={distancia_estimada_frontal:.3f}m  estado={estado}  "
            f"total={total_displacement:.4f}m"
        )

        # raw IR de los sensores relevantes (frontales, diagonales, laterales)
        # util para ver si una pared se aproxima antes de que el laser la detecte
        print(
            f"[*] IR raw => "
            f"ps7={raw_values['ps7']:7.1f}  ps0={raw_values['ps0']:7.1f}  "
            f"ps6={raw_values['ps6']:7.1f}  ps1={raw_values['ps1']:7.1f}  "
            f"ps5={raw_values['ps5']:7.1f}  ps2={raw_values['ps2']:7.1f}"
        )

        # IR convertido a distancia solo si algun sensor frontal o diagonal detecto algo
        ir_fl_dist, ir_fl_det = ir_readings['ps7']
        ir_fr_dist, ir_fr_det = ir_readings['ps0']
        ir_dl_dist, ir_dl_det = ir_readings['ps6']
        ir_dr_dist, ir_dr_det = ir_readings['ps1']
        if any((ir_fl_det, ir_fr_det, ir_dl_det, ir_dr_det)):
            print(
                f"[!] IR deteccion => "
                f"front_izq={ir_fl_dist:.4f}m({'ok' if ir_fl_det else '--'})  "
                f"front_der={ir_fr_dist:.4f}m({'ok' if ir_fr_det else '--'})  "
                f"diag_izq={ir_dl_dist:.4f}m({'ok' if ir_dl_det else '--'})  "
                f"diag_der={ir_dr_dist:.4f}m({'ok' if ir_dr_det else '--'})"
            )