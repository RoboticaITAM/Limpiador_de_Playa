import cv2
import numpy as np

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
    # Asumiendo una relación lineal entre desviación en píxeles y grados
    grados_a_girar = (desviacion_centro / ancho_frame) * fov
    return grados_a_girar

cap = cv2.VideoCapture(2) # Asegúrate de que el índice de la cámara es correcto
blackLow = np.array([0, 0, 0], np.uint8)
blackHigh = np.array([179, 100, 50], np.uint8)
fov = 60  # Asumiendo un campo de visión horizontal de 60 grados para la cámara

while True:
    ret, frame = cap.read()
    if ret:
        frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        maskNegro = cv2.inRange(frameHSV, blackLow, blackHigh)
        
        # Aplicar desenfoque para reducir el ruido
        maskNegro = cv2.GaussianBlur(maskNegro, (5, 5), 0)

        # Dilatar y luego erosionar la máscara para cerrar huecos en el contorno
        maskNegro = cv2.dilate(maskNegro, None, iterations=2)
        maskNegro = cv2.erode(maskNegro, None, iterations=2)

        contornos, _ = cv2.findContours(maskNegro, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contornos:
            c = max(contornos, key=cv2.contourArea)
            area = cv2.contourArea(c)
            centro_x, centro_y = calcular_centro_de_masa(c)
            grados_a_girar = calcular_desviacion_y_grados(centro_x, frame.shape[1], fov)
            
            detenerse = 0
            if 900 <= area <= 1300:
                detenerse = 1
            
            output = "[{:.2f}, {:.2f}, {}]".format(grados_a_girar, area, detenerse)
            print(output)
            
            # Dibuja el contorno y muestra la información sobre el frame
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 3)
            cv2.putText(frame, output, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('s'):
            break

cap.release()
cv2.destroyAllWindows()
