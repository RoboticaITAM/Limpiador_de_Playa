import cv2
import numpy as np
import time

def encontrar_cuadricula(x, y, ancho, alto):
    # Divide el ancho y el alto en 4 para obtener las dimensiones de cada cuadrícula
    ancho_cuadricula = ancho // 4
    alto_cuadricula = alto // 4
    
    # Calcula el índice de la cuadrícula basándote en x, y
    indice_x = x // ancho_cuadricula
    indice_y = y // alto_cuadricula
    
    # Calcula el número de cuadrícula (de 0 a 15)
    num_cuadricula = indice_y * 4 + indice_x
    return num_cuadricula

def dibujar(mask, color, color_nombre, ancho, alto):
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contornos:
        area = cv2.contourArea(c)
        if area > 3000:
            M = cv2.moments(c)
            if (M["m00"] == 0): M["m00"] = 1
            x = int(M["m10"] / M["m00"])
            y = int(M['m01'] / M['m00'])
            # Encuentra en qué cuadrícula está el objeto
            cuadricula = encontrar_cuadricula(x, y, ancho, alto)
            coordenadas[color_nombre].append(cuadricula)

cap = cv2.VideoCapture(2)

# Obtiene el ancho y el alto de la ventana de la cámara
ret, frame = cap.read()
alto, ancho = frame.shape[:2]

azulBajo = np.array([95,115,20],np.uint8)
azulAlto = np.array([125,255,255],np.uint8)

blackLow = np.array([0, 0, 0], np.uint8)
blackHigh = np.array([179, 255, 50], np.uint8)

redBajo1 = np.array([0,100,20],np.uint8)
redAlto1 = np.array([5,255,255],np.uint8)

redBajo2 = np.array([175,100,20],np.uint8)
redAlto2 = np.array([179,255,255],np.uint8)

font = cv2.FONT_HERSHEY_SIMPLEX
ultimo_tiempo_impresion = time.time()

while True:
    ret, frame = cap.read()
    if ret:
        coordenadas = {'rojo': [], 'negro': [], 'azul': []}
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        maskAzul = cv2.inRange(frameHSV, azulBajo, azulAlto)
        maskNegro = cv2.inRange(frameHSV, blackLow, blackHigh)
        maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
        maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
        maskRed = cv2.add(maskRed1, maskRed2)
        dibujar(maskAzul, (255, 0, 0), 'azul', ancho, alto)
        dibujar(maskNegro, (0, 0, 0), 'negro', ancho, alto)
        dibujar(maskRed, (0, 0, 255), 'rojo', ancho, alto)
        cv2.imshow('frame', frame)

        if time.time() - ultimo_tiempo_impresion >= 1:
            if any(coordenadas.values()):
                print(coordenadas)
            ultimo_tiempo_impresion = time.time()

    if cv2.waitKey(1) & 0xFF == ord('s'):
        break

cap.release()
cv2.destroyAllWindows()
