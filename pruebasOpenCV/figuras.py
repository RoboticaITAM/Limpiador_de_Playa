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

def dibujar(mask, color_nombre, ancho, alto):
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contornos:
        # Obtén el rectángulo delimitador y sus dimensiones
        x, y, w, h = cv2.boundingRect(c)
        # Verifica si la forma es un rectángulo/cuadrado
        aspect_ratio = float(w) / h
        if aspect_ratio > 0.95 and aspect_ratio < 1.05:  # Cambia los valores para aceptar formas no perfectamente cuadradas
            area = cv2.contourArea(c)
            if area > 3000:
                # Encuentra en qué cuadrícula está el objeto
                cuadricula = encontrar_cuadricula(x, y, ancho, alto)
                coordenadas[color_nombre].append((cuadricula, (x, y, x+w, y+h)))  # Agrega las coordenadas del rectángulo

cap = cv2.VideoCapture(0)

# Obtiene el ancho y el alto de la ventana de la cámara
ret, frame = cap.read()
alto, ancho = frame.shape[:2]

blackLow = np.array([0, 0, 0], np.uint8)
blackHigh = np.array([179, 255, 50], np.uint8)

font = cv2.FONT_HERSHEY_SIMPLEX
ultimo_tiempo_impresion = time.time()

while True:
    ret, frame = cap.read()
    if ret:
        coordenadas = {'negro': []}
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        maskNegro = cv2.inRange(frameHSV, blackLow, blackHigh)
        dibujar(maskNegro, 'negro', ancho, alto)
        cv2.imshow('frame', frame)

        if time.time() - ultimo_tiempo_impresion >= 1:
            if coordenadas['negro']:
                print(coordenadas['negro'])
            ultimo_tiempo_impresion = time.time()

    if cv2.waitKey(1) & 0xFF == ord('s'):
        break

cap.release()
cv2.destroyAllWindows()
