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

# Función para dibujar contornos y actualizar las coordenadas
def dibujar(mask, color_nombre, frame, ancho, alto):
    contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    coords = []  # Lista para almacenar coordenadas de este color
    for c in contornos:
        area = cv2.contourArea(c)
        if area > 1500:
            x, y, w, h = cv2.boundingRect(c)
            # Se usa el centro del contorno para encontrar la cuadrícula
            centro_x = x + w // 2
            centro_y = y + h // 2
            num_cuadricula = encontrar_cuadricula(centro_x, centro_y, ancho, alto)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, str(num_cuadricula), (centro_x - 10, centro_y), font, 0.75, (0, 255, 0), 1, cv2.LINE_AA)
            coords.append(num_cuadricula)
    coordenadas[color_nombre]=coords

cap=cv2.VideoCapture(0) #cambiar a 0 en la rasp

#azulBajo = np.array([95,115,20],np.uint8)
#azulAlto = np.array([125,255,255],np.uint8)

blackLow = np.array([0, 0, 0], np.uint8)
blackHigh = np.array([179, 100, 50], np.uint8)

#redBajo1 = np.array([0,100,20],np.uint8)
#redAlto1 = np.array([5,255,255],np.uint8)

#redBajo2 = np.array([175,100,20],np.uint8)
#redAlto2 = np.array([179,255,255],np.uint8)

font = cv2.FONT_HERSHEY_SIMPLEX
ultimo_tiempo_impresion = time.time()

while True:
    ret, frame = cap.read()
    alto,ancho=frame.shape[:2]
    if ret:
        # Reinicia el diccionario de coordenadas para este frame
        coordenadas = {'negro': []}

        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #maskAzul = cv2.inRange(frameHSV, azulBajo, azulAlto)
        maskNegro = cv2.inRange(frameHSV, blackLow, blackHigh)
        #maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
        #maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
        #maskRed = cv2.add(maskRed1, maskRed2)
        #dibujar(maskAzul, (255, 0, 0), 'azul')
        dibujar(maskNegro,'negro',frame,ancho,alto)
        #dibujar(maskRed, (0, 0, 255), 'rojo')
        cv2.imshow('frame', frame)

        # Imprime solo si ha pasado un segundo y hay coordenadas detectadas
        if time.time() - ultimo_tiempo_impresion >= 1:
            if any(coordenadas.values()):  # Verifica si hay alguna coordenada detectada
                print(coordenadas)
            ultimo_tiempo_impresion = time.time()

    if cv2.waitKey(1) & 0xFF == ord('s'):
        break

cap.release()
cv2.destroyAllWindows()
