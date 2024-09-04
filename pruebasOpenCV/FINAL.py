import cv2
import numpy as np
import serial
import time

# Inicialización de la comunicación serial
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
    if grados_a_girar < -3: return -1
    if grados_a_girar > 3: return 1
    return 0  # Neutral

def detectar_objeto_en_cuadrantes_inferiores(centro_x, centro_y, ancho_frame, alto_frame):
    cuadrante_alto = alto_frame // 5
    cuadrante_y = centro_y // cuadrante_alto
    return cuadrante_y == 4  # True si está en los cuadrantes inferiores

def detectar_objeto_en_cuadrantes(centro_x, centro_y, ancho_frame, alto_frame):
    cuadrantes_centrales_inferiores = [(1,3),(1,4),(2,3),(2,4),(3,3),(3,4)]
    cuadrante_x = (centro_x // (ancho_frame // 5))
    cuadrante_y = (centro_y // (alto_frame // 5))
    return int((cuadrante_x, cuadrante_y) in cuadrantes_centrales_inferiores)

cap = cv2.VideoCapture(2)  # Verifica que este es el índice correcto para tu cámara

# Definición de los rangos HSV para los colores de interés
blackLow, blackHigh = np.array([0, 0, 0], np.uint8), np.array([179, 255, 50], np.uint8)
blueLow, blueHigh = np.array([90, 150, 0], np.uint8), np.array([130, 255, 255], np.uint8)
redLow1, redHigh1 = np.array([0, 150, 70], np.uint8), np.array([10, 255, 255], np.uint8)
redLow2, redHigh2 = np.array([170, 150, 70], np.uint8), np.array([180, 255, 255], np.uint8)
fov = 45

while True:
    ret, frame = cap.read()
    if ret:
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        maskNegro = cv2.inRange(frameHSV, blackLow, blackHigh)
        maskAzul = cv2.inRange(frameHSV, blueLow, blueHigh)
        maskRojo = cv2.bitwise_or(cv2.inRange(frameHSV, redLow1, redHigh1), cv2.inRange(frameHSV, redLow2, redHigh2))
        
        contornosRojo, _ = cv2.findContours(maskRojo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        desviacion_rojo = 2  # Asumimos inicialmente que no hay rojo detectado
        centro_x, centro_y = 0, 0  
        
        for contorno in contornosRojo:
            centro_x, centro_y = calcular_centro_de_masa(contorno)
            desviacion_rojo = calcular_desviacion_y_grados(centro_x, frame.shape[1], fov)  # Calcula desviación para contornos rojos
            if detectar_objeto_en_cuadrantes_inferiores(centro_x, centro_y, frame.shape[1], frame.shape[0]) or desviacion_rojo != 0:
                break  # Si se encuentra rojo en los cuadrantes inferiores o hay desviación significativa, termina el bucle

        azul_en_cuadrantes_inferiores, objeto_en_cuadrantes, detenerse, rojoDet = 0, 0, 0, 0
        if detectar_objeto_en_cuadrantes_inferiores(centro_x, centro_y, frame.shape[1], frame.shape[0]):
            rojoDet = 1 
        # Revisar azul en cuadrantes inferiores
        if any(calcular_centro_de_masa(contorno) and detectar_objeto_en_cuadrantes_inferiores(*calcular_centro_de_masa(contorno), frame.shape[1], frame.shape[0]) for contorno in cv2.findContours(maskAzul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]):
            azul_en_cuadrantes_inferiores = 1
        
        # Revisar objetos negros
        contornosNegro = cv2.findContours(maskNegro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        
        
        if contornosNegro:
            c = max(contornosNegro, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if 3500 <= area <= 8500: detenerse = 1
            centro_x, centro_y = calcular_centro_de_masa(c)
            objeto_en_cuadrantes = detectar_objeto_en_cuadrantes(centro_x, centro_y, frame.shape[1], frame.shape[0])
            grados_a_girar = calcular_desviacion_y_grados(centro_x, frame.shape[1], fov)
            
            # Preparación y envío de datos
            output = "[{},{},{}, {}, {}]".format(azul_en_cuadrantes_inferiores, grados_a_girar, objeto_en_cuadrantes, desviacion_rojo, rojoDet)
            print(output)
            datos = "[{},{}, {}, {}, {}]\n".format(azul_en_cuadrantes_inferiores, grados_a_girar, objeto_en_cuadrantes, desviacion_rojo, rojoDet)
            arduino.write(datos.encode())
            
            # Visualización en la imagen
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
            cv2.putText(frame, output, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('s'):
            break

cap.release()
cv2.destroyAllWindows()
arduino.close()
