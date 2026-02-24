import cv2
import numpy as np

def detectar_cruz_roja(frame):
    # Convertir a HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Rango para el color rojo (dos rangos por el círculo de HSV)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # Máscara combinada para el rojo
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    # Opcional: Filtrar ruido
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Buscar contornos
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    cruz_detectada = False
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:  # Ajustar según tamaño esperado
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h

            # Las barras de la cruz tienen aspect ratio cercano a 1 pero en intersección
            if 0.8 < aspect_ratio < 1.2:
                # Dibujar el contorno detectado
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, "CRUZ ROJA DETECTADA", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cruz_detectada = True

    return frame, cruz_detectada
