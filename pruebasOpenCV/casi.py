import cv2
import numpy as np
import serial
import time

# Inicialización de la comunicación serial
# Reemplaza '/dev/ttyACM0' con tu puerto serial y 9600 con tu tasa de baudios
arduino = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)  # Da tiempo al Arduino para reiniciar y establecer la conexión
def calcular_centro_de_masa(contorno):
    M = cv2.moments(contorno)
    if M["m00"] != 0:
        centro_x = int(M["m10"] / M["m00"])
        centro_y = int(M["m01"] / M["m00"])
    else:
        centro_x, centro_y = 0, 0
    return centro_x, centro_y

def calcular_desviacion_y_grados(centro_x, ancho_frame, fov):
    centro_frame = ancho_frame // 2
    desviacion_centro = centro_x - centro_frame
    grados_a_girar = (desviacion_centro / ancho_frame) * fov
    return grados_a_girar

def detectar_objeto_en_cuadrantes(centro_x, centro_y, ancho_frame, alto_frame):
    cuadrante_ancho = ancho_frame // 5
    cuadrante_alto = alto_frame // 5
    # Definir cuadrantes centrales inferiores como 5, 6, 9, 10
    cuadrantes_centrales_inferiores = [(1,3),(1,4),(2,3),(2,4),(3,3),(3,4)]
    cuadrante_x = centro_x // cuadrante_ancho
    cuadrante_y = centro_y // cuadrante_alto
    if (cuadrante_x, cuadrante_y) in cuadrantes_centrales_inferiores:
        return 1
    return 0

cap = cv2.VideoCapture(2)
blackLow = np.array([0, 0, 0], np.uint8)
blackHigh = np.array([179, 255, 50], np.uint8)
blueLow = np.array([90, 150, 0], np.uint8)
blueHigh = np.array([130, 150, 255], np.uint8)
fov = 45

while True:
    ret, frame = cap.read()
    if ret:
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        maskNegro = cv2.inRange(frameHSV, blackLow, blackHigh)
        maskNegro = cv2.GaussianBlur(maskNegro, (5, 5), 0)
        maskNegro = cv2.dilate(maskNegro, None, iterations=2)
        maskNegro = cv2.erode(maskNegro, None, iterations=2)
        maskAzul = cv2.inRange(frameHSV, blueLow, blueHigh)
        maskAzul = cv2.GaussianBlur(maskAzul, (5, 5), 0)
        maskAzul = cv2.dilate(maskAzul, None, iterations=2)
        maskAzul = cv2.erode(maskAzul, None, iterations=1)

        contornosAzul, _ = cv2.findContours(maskAzul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contornos, _ = cv2.findContours(maskNegro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detenerse = 0
        objeto_en_cuadrantes = 0
        azul_encontrado = 0
        for ca in contornosAzul:
            centro_x, centro_y = calcular_centro_de_masa(ca)
            if detectar_objeto_en_cuadrantes(centro_x, centro_y, frame.shape[1], frame.shape[0]):
                azul_encontrado = 1
                break
        if contornos:
            c = max(contornos, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if 3500 <= area <= 8500:
                detenerse = 1
            centro_x, centro_y = calcular_centro_de_masa(c)
            objeto_en_cuadrantes = detectar_objeto_en_cuadrantes(centro_x, centro_y, frame.shape[1], frame.shape[0])
            grados_a_girar = calcular_desviacion_y_grados(centro_x, frame.shape[1], fov)
            
            output = "[{},{:.2f}, {:.2f}, {}, {}]".format(azul_encontrado, grados_a_girar, area, detenerse, objeto_en_cuadrantes)
            print(output)
            datos = "[{},{:.2f}, {:.2f}, {}, {}]\n".format(azul_encontrado, grados_a_girar, area, detenerse, objeto_en_cuadrantes)
            arduino.write(datos.encode())
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
            cv2.putText(frame, output, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('s'):
            break
cap.release()
cv2.destroyAllWindows()
arduino.close()