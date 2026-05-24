from controller import Robot
import matplotlib.pyplot as plt

robot = Robot()

# ==========================
# Configuración general
# ==========================

TIME_STEP = 50
MAX_SPEED = 6.28
MAX_SAMPLES = 500

# Kalman
Q = 0.15
R = 2
P = 1
estimate = 66

# Navegación
SAFE_DISTANCE = 100
SIDE_LIMIT = 200

# Detectar bucles
rotationCounter = 0
ROTATION_LIMIT = 15
# Memoria de decisión
decisionMode = False
decisionCounter = 0
DECISION_STEPS = 12

decisionDirection = "left"

# Robot
WHEEL_RADIUS = 0.0205

# ==========================
# Motores
# ==========================

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# ==========================
# Sensores
# ==========================

sensorNames = ['ps0','ps7','ps5','ps2']

sensors=[]

for name in sensorNames:

    sensor=robot.getDevice(name)
    sensor.enable(TIME_STEP)

    sensors.append(sensor)

# ==========================
# Encoders
# ==========================

leftEncoder=robot.getDevice('left wheel sensor')
rightEncoder=robot.getDevice('right wheel sensor')

leftEncoder.enable(TIME_STEP)
rightEncoder.enable(TIME_STEP)

prevLeft=leftEncoder.getValue()
prevRight=rightEncoder.getValue()

# ==========================
# Datos
# ==========================

front_raw=[]
front_filtered=[]
kalman_data=[]

distance_data=[]

window=[]

sample_count=0


# ==========================
# Promedio móvil
# ==========================

def moving_average(value):

    window.append(value)

    if len(window)>5:
        window.pop(0)

    return sum(window)/len(window)


# ==========================
# Loop principal
# ==========================

while robot.step(TIME_STEP)!=-1:

    sample_count +=1

    # ==========================
    # Sensores
    # ==========================

    frontRight=sensors[0].getValue()
    frontLeft=sensors[1].getValue()

    leftSide=sensors[2].getValue()
    rightSide=sensors[3].getValue()

    front=(frontRight+frontLeft)/2

    filtered=moving_average(front)

    # ==========================
    # Encoders
    # ==========================

    currentLeft=leftEncoder.getValue()
    currentRight=rightEncoder.getValue()

    deltaLeft=currentLeft-prevLeft
    deltaRight=currentRight-prevRight

    leftDistance=WHEEL_RADIUS*deltaLeft
    rightDistance=WHEEL_RADIUS*deltaRight

    avance=(leftDistance+rightDistance)/2

    prevLeft=currentLeft
    prevRight=currentRight


    # ==========================
    # Kalman
    # ==========================

   
    
    # Evitar valores inválidos
    if filtered != filtered:
        filtered = 67
    
    if avance != avance:
        avance = 0
    
    
    # Predicción
    prediction = estimate + avance
    
    P = P + Q
    
    
    # Evitar división inválida
    if (P + R) == 0:
    
        K = 0
    
    else:
    
        K = P / (P + R)
    
    
    # Corrección
    estimate = prediction + K * (filtered - prediction)
    
    
    # Actualizar incertidumbre
    P = (1 - K) * P
    
    
    # Protección final
    if estimate != estimate:
    
        estimate = filtered
    
    if P != P:
    
        P = 1
    

    # ==========================
    # Guardar datos
    # ==========================

    front_raw.append(front)
    front_filtered.append(filtered)
    kalman_data.append(estimate)

    distance_data.append(avance)


    # ==========================
    # Detectar vueltas infinitas
    # ==========================

    if abs(avance)<0.001 and estimate>SAFE_DISTANCE:

        rotationCounter +=1

    else:

        rotationCounter=0
    
    
       # ==========================
    # Navegación estable
    # ==========================
    
    SAFE_DISTANCE = 90
    CRITICAL_DISTANCE = 180
    SIDE_LIMIT = 150
    
    
    # Mantener decisión previa
    if decisionMode:
    
        decisionCounter += 1
    
        if decisionDirection == "left":
    
            leftMotor.setVelocity(-1)
            rightMotor.setVelocity(3)
    
        else:
    
            leftMotor.setVelocity(3)
            rightMotor.setVelocity(-1)
    
        if decisionCounter > DECISION_STEPS:
    
            decisionMode = False
            decisionCounter = 0
    
    
    # Tomar nueva decisión
    elif estimate > SAFE_DISTANCE:
    
        decisionMode = True
    
        # diferencia significativa
        if abs(leftSide-rightSide) > 15:
    
            if leftSide > rightSide:
    
                decisionDirection = "right"
    
            else:
    
                decisionDirection = "left"
    
        else:
    
            # si son casi iguales, mantener última
            pass
    
    
    # Ejecutar navegación normal
    else:
    
        if leftSide > SIDE_LIMIT:
    
            leftMotor.setVelocity(2)
            rightMotor.setVelocity(3)
    
        elif rightSide > SIDE_LIMIT:
    
            leftMotor.setVelocity(3)
            rightMotor.setVelocity(2)
    
        else:
    
            leftMotor.setVelocity(2)
            rightMotor.setVelocity(2)
    if sample_count>=MAX_SAMPLES:

        break


# ==========================
# Gráficos
# ==========================

plt.figure()

plt.plot(front_raw,label="Cruda")
plt.plot(front_filtered,label="Filtrada")
plt.plot(kalman_data,label="Kalman")

plt.xlabel("Muestras")
plt.ylabel("Valor sensor")

plt.grid()
plt.legend()

plt.show()