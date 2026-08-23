import cv2
import numpy as np

# 1. Cargá tus medidas físicas reales (EN METROS) y EN ORDEN
# Ejemplo: [Arriba-Izquierda, Arriba-Derecha, Abajo-Derecha, Abajo-Izquierda]
puntos_mundo = np.array([
    [0.208,  0.072], # Punto 1 (Arriba-Izquierda): Más lejos, hacia la izquierda
    [0.208, -0.072], # Punto 2 (Arriba-Derecha):   Más lejos, hacia la derecha
    [0.128, -0.072], # Punto 3 (Abajo-Derecha):    Más cerca, hacia la derecha
    [0.128,  0.072]  # Punto 4 (Abajo-Izquierda):  Más cerca, hacia la izquierda
], dtype=np.float32)

puntos_imagen = []

def clic_raton(evento, u, v, flags, param):
    """Captura las coordenadas en píxeles cuando haces clic izquierdo."""
    global puntos_imagen
    if evento == cv2.EVENT_LBUTTONDOWN:
        if len(puntos_imagen) < 4:
            puntos_imagen.append([u, v])
            print(f"Clic {len(puntos_imagen)} guardado: u={u}, v={v}")
            # Dibujar un círculo verde donde hiciste clic
            cv2.circle(frame_mostrado, (u, v), 5, (0, 255, 0), -1)
            cv2.imshow('Calibracion (Clic en los 4 puntos en orden)', frame_mostrado)

# Iniciar cámara
cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

print("--- INICIO DE CALIBRACIÓN ---")
print("Hacé clic en los 4 puntos de la mesa EN EL MISMO ORDEN que los definiste en el código.")
print("Presioná 'c' cuando hayas hecho los 4 clics para calcular la matriz.")
print("Presioná 'q' para salir sin guardar.")

cv2.namedWindow('Calibracion (Clic en los 4 puntos en orden)')
cv2.setMouseCallback('Calibracion (Clic en los 4 puntos en orden)', clic_raton)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Solo actualizamos el frame si no hemos empezado a hacer clics 
    # para que los puntos dibujados no se borren
    if len(puntos_imagen) == 0:
        frame_mostrado = frame.copy()
        cv2.imshow('Calibracion (Clic en los 4 puntos en orden)', frame_mostrado)

    key = cv2.waitKey(1) & 0xFF
    
    # Calcular Homografía al presionar 'c'
    if key == ord('c'):
        if len(puntos_imagen) == 4:
            puntos_imagen_np = np.array(puntos_imagen, dtype=np.float32)
            
            # Calcular la Matriz de Homografía
            H, estado = cv2.findHomography(puntos_imagen_np, puntos_mundo)
            
            print("\n¡Matriz de Homografía Calculada con Éxito!")
            print(H)
            
            # Guardar la matriz en un archivo para usarla en tu script de ROS2
            np.save('homografia.npy', H)
            print("Matriz guardada en 'homografia.npy'")
            break
        else:
            print(f"Faltan clics. Llevas {len(puntos_imagen)} de 4.")

    elif key == ord('q'):
        print("Calibración cancelada.")
        break

cap.release()
cv2.destroyAllWindows()
