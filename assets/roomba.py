import sys
import time
import math
import pycreate2
from pycreate2 import Create2
import threading

# Configuración inicial para odometría
DIAMETRO_RUEDAS = 72.0  # en mm
BASE_RUEDAS = 235.0  # en mm
TICKS_POR_REVOLUCION = 508.8
MM_POR_TICK = (math.pi * DIAMETRO_RUEDAS) / TICKS_POR_REVOLUCION
MAX_TICKS = 65536

modo_autonomo = False
detener_autonomo = False

class OdometriaRoomba:
    def __init__(self, roomba):
        self.roomba = roomba
        self.posicion = [0.0, 0.0]
        self.angulo = 0.0  # en radianes
        self.encoder_izq = 0
        self.encoder_der = 0


    def ajustar_overflow(self, nuevo_encoder, encoder_anterior):
        """Ajusta el desbordamiento de los encoders."""
        diferencia = nuevo_encoder - encoder_anterior
        if diferencia < -MAX_TICKS / 2:
            diferencia += MAX_TICKS
        elif diferencia > MAX_TICKS / 2:
            diferencia -= MAX_TICKS
        return diferencia

    def actualizar_posicion(self, encoder_izq, encoder_der):
        """
        Actualiza la posición y el ángulo.
        """
        # Se controla el reinicio de los encoders
        ticks_izq = self.ajustar_overflow(encoder_izq, self.encoder_izq_actual)
        ticks_der = self.ajustar_overflow(encoder_der, self.encoder_der_actual)

        # Se actualizan los encoders
        self.encoder_izq_actual = encoder_izq
        self.encoder_der_actual = encoder_der

        # Se calculan las distancias
        distancia_izq = ticks_izq * MM_POR_TICK
        distancia_der = ticks_der * MM_POR_TICK

        # Se calculan la distancia y el ángulo
        distancia = (distancia_izq + distancia_der) / 2
        delta_angulo = (distancia_der - distancia_izq) / BASE_RUEDAS

        self.angulo += delta_angulo

        # Se normaliza el ángulo acumulado a [-π, π]
        self.angulo = (self.angulo + math.pi) % (2 * math.pi) - math.pi

        # Se calcula la posición
        x = distancia * math.cos(self.angulo)
        y = distancia * math.sin(self.angulo)
        self.posicion[0] += x
        self.posicion[1] += y
    
    def obtener_posicion_actual(self):
        return {"x": self.posicion[0], "y": self.posicion[1], "theta": self.angulo}

    def establecer_posicion_actual(self, nueva_posicion):
        self.posicion[0] = nueva_posicion["x"]
        self.posicion[1] = nueva_posicion["y"]
        self.angulo = nueva_posicion["theta"]
        
    def imprimir_posicion(self):

        posicion = [coordenada / 10 for coordenada in self.posicion]
        angulo = math.degrees(self.angulo)

       # Imprimir los detalles de la odometría de una manera más amigable
        print("\n*** Odometría Actualizada ***")
        print(f"    -> Posición en cm: ({posicion[0]:.1f} cm, {posicion[1]:.1f} cm)")
        print(f"    -> Ángulo: {angulo:.2f}°")
        print("\n-----------------------------")

    def inicializar_odometria(self):
        """
        Inicializa la odometría.
        """
        try:
            sensores = self.roomba.get_sensors()
            if sensores is None:
                print("Error al obtener datos de los sensores")
                return
        except Exception as e:
            print(f"Error al obtener datos de los sensores: {e}")
            return

        self.encoder_izq_actual = sensores.encoder_counts_left
        self.encoder_der_actual = sensores.encoder_counts_right
        self.posicion = [0.0, 0.0]
        self.angulo = 0.0  
        print("Odometría inicial.")

    
class Controlador:
    def __init__(self, port):
        self.roomba = self.inicializar_roomba(port)
        self.posicion = OdometriaRoomba(self.roomba)
        self.posicion.inicializar_odometria()

    def inicializar_roomba(self, port):
        """Inicializa la conexión con la roomba."""
        roomba = None
        while roomba is None:
            try:
                print(f"Conectando la roomba al puerto {port}.")
                roomba = Create2(port=port)
                roomba.start()
                roomba.safe()
                print("Conexión realizada con la roomba.")
                return roomba
            except Exception as e:
                print(f"Error al conectar la roomba: {e}")
                sys.exit(1)

    def cerrar_roomba(self):
        """Cierra la conexión."""
        if self.roomba is not None:
            self.roomba.drive_stop()            
            self.roomba.close()
            self.roomba=None
        pycreate2.Create2.__del__=lambda self:None

    def adelante(self, distancia):
        self._mover_roomba(distancia, adelante=True)
    def atras(self, distancia):
        self._mover_roomba(distancia, adelante=False)

    def _mover_roomba(self, distancia, adelante=True):
        """Función común para mover la roomba hacia adelante o hacia atrás."""

        orientacion = 1 if adelante else - 1  # Dirección: 1 hacia adelante, -1 hacia atrás
        distancia = abs(distancia) 

        distancia_objetivo = distancia * 10
        distancia_total = 0

        vel = 100 * orientacion 

        # Intentos para obtener los sensores
        intentos_max = 3
        intento = 0

        # Última posición válida registrada
        ultima_posicion_valida = self.posicion.obtener_posicion_actual()

        while intento < intentos_max:
            # Obtener datos de los sensores
            try:
                sensor = self.roomba.get_sensors()
                if sensor is None:
                    print("Error: No se pueden obtener datos de sensores.")
                    try:
                        self.roomba.SCI.ser.reset_input_buffer() 
                        print("Buffer de entrada limpiado.")
                    except Exception as e:
                        print(f"Error al limpiar el buffer: {e}")
                    self._mover_roomba(distancia, adelante=adelante)
                    continue
            except Exception as e:
                print(f"Error al obtener datos de los sensores: {e}")
                return

            # Si uno de los encoders tiene un valor 0, guardar última posición válida y reiniciar encoders
            if int(sensor.encoder_counts_left) == 0 or int(sensor.encoder_counts_right) == 0:
                print("Error de encoders: Guardando posición y reiniciando encoders")
                self.posicion.establecer_posicion_actual(ultima_posicion_valida) 
                self.roomba.drive_stop()
                self.roomba.SCI.ser.reset_input_buffer() 
                time.sleep(1)
                intento += 1
                continue

            # Si no hay errores, actualizar la última posición válida
            ultima_posicion_valida = self.posicion.obtener_posicion_actual()
            break

        if intento >= intentos_max:
            print("Error: No se pudo obtener una lectura válida de los encoders después de varios intentos.")
            return

        encoder_izq_ini = int(sensor.encoder_counts_left)
        encoder_der_ini = int(sensor.encoder_counts_right)

        self.roomba.drive_direct(vel, vel)

        while abs(distancia_total) < abs(distancia_objetivo):
            try:
                sensor = self.roomba.get_sensors()
                if sensor is None:
                    print("Error: No se pueden obtener datos de sensores.")
                    self.roomba.drive_stop()
                    try:
                        self.roomba.SCI.ser.reset_input_buffer()
                        print("Buffer de entrada limpiado.")
                    except Exception as e:
                        print(f"Error al limpiar el buffer: {e}")
                    self._mover_roomba((distancia_objetivo - distancia_total) / 10, adelante=adelante)
                    return
            except Exception as e:
                print(f"Error al obtener datos de los sensores: {e}")
                return

            if int(sensor.encoder_counts_left) == 0 or int(sensor.encoder_counts_right) == 0:
                print("Error de encoders: Restaurando posición y reiniciando")
                self.roomba.drive_stop()
                self.posicion.establecer_posicion_actual(ultima_posicion_valida) 
                self.roomba.SCI.ser.reset_input_buffer()
                time.sleep(1)
                self._mover_roomba((distancia_objetivo - distancia_total) / 10, adelante=adelante)
                return

            # Detectar obstáculo y retroceder
            if sensor.bumps_wheeldrops.bump_left or sensor.bumps_wheeldrops.bump_right:
                print("¡Obstáculo detectado durante el movimiento! Deteniendo.")
                self.roomba.drive_stop()

                self.roomba.drive_direct(-vel, -vel)
                time.sleep(0.1)

                self.roomba.drive_stop()
                self.posicion.establecer_posicion_actual(self.posicion.obtener_posicion_actual())
                print("Posición actualizada tras el retroceso:")
                self.posicion.imprimir_posicion()
                return

            encoder_izq_nuevo = sensor.encoder_counts_left
            encoder_der_nuevo = sensor.encoder_counts_right

            cambio_encoder_izq = self.posicion.ajustar_overflow(encoder_izq_nuevo, encoder_izq_ini)
            cambio_encoder_der = self.posicion.ajustar_overflow(encoder_der_nuevo, encoder_der_ini)

            distancia_recorrida_izq = cambio_encoder_izq * MM_POR_TICK
            distancia_recorrida_der = cambio_encoder_der * MM_POR_TICK

            # Cálculo del avance lineal (promedio de ambas ruedas)
            distancia_recorrida_total = (distancia_recorrida_izq + distancia_recorrida_der) / 2

            distancia_total += distancia_recorrida_total

            self.posicion.actualizar_posicion(encoder_izq_nuevo, encoder_der_nuevo)

            encoder_izq_ini = encoder_izq_nuevo
            encoder_der_ini = encoder_der_nuevo

            time.sleep(0.02)

        self.roomba.drive_stop()
        print(f"Movimiento finalizado.")
        self.posicion.imprimir_posicion()
    
    

    def _girar_roomba(self, grados):
        """Gira la roomba una cantidad específica de grados (positivo: derecha, negativo: izquierda)."""        

        # Constantes
        correccion_velocidad = 0.501 
        velocidad = 95 
        angulo_total_girado = 0
        direccion = 1 if grados > 0 else -1

        intentos_max = 3
        intento = 0

        # Obtener última posición válida inicial
        ultima_posicion_valida = self.posicion.obtener_posicion_actual()

        while intento < intentos_max:
            # Obtener datos iniciales de los sensores
            try:
                sensor = self.roomba.get_sensors()
                if sensor is None:
                    print("Error: No se pueden obtener datos de sensores - _girar_roomba()")
                    try:
                        self.roomba.SCI.ser.reset_input_buffer()
                        print("Buffer de entrada limpiado.")
                    except Exception as e:
                        print(f"Error al limpiar el buffer: {e}")
                    intento += 1
                    time.sleep(1)
                    continue
            except Exception as e:
                print(f"Error al obtener datos de los sensores: {e}")
                return

            if int(sensor.encoder_counts_left) == 0 or int(sensor.encoder_counts_right) == 0:
                print("Error de encoders: Guardando posición y reiniciando encoders")
                self.posicion.establecer_posicion_actual(ultima_posicion_valida)
                self.roomba.drive_stop()
                self.roomba.SCI.ser.reset_input_buffer()
                self.roomba.SCI.ser.reset_output_buffer()

                time.sleep(1)
                intento += 1
                continue

            # Si no hay errores, actualizar la última posición válida
            ultima_posicion_valida = self.posicion.obtener_posicion_actual()
            break

        if intento >= intentos_max:
            print("Error: No se pudo obtener una lectura válida de los encoders después de varios intentos.")
            return  

        self.roomba.drive_direct(velocidad * direccion, -velocidad * direccion)

        encoder_izq_ini = int(sensor.encoder_counts_left)
        encoder_der_ini = int(sensor.encoder_counts_right)

        while abs(angulo_total_girado) < abs(grados):
            try:
                sensor = self.roomba.get_sensors()
                if sensor is None:
                    print("Error: No se pueden obtener datos de sensores.")
                    self.roomba.drive_stop()
                    try:
                        self.roomba.SCI.ser.reset_input_buffer()
                        print("Buffer de entrada limpiado.")
                    except Exception as e:
                        print(f"Error al limpiar el buffer: {e}")
                    self._girar_roomba(grados - angulo_total_girado)
                    return
            except Exception as e:
                print(f"Error al obtener datos de los sensores: {e}")
                return

            if int(sensor.encoder_counts_left) == 0 or int(sensor.encoder_counts_right) == 0:
                print("Error de encoders: Restaurando posición y reiniciando")
                self.roomba.drive_stop()
                self.posicion.establecer_posicion_actual(ultima_posicion_valida)
                time.sleep(1)
                self._girar_roomba(grados - angulo_total_girado)
                return

            encoder_izq_nuevo = sensor.encoder_counts_left
            encoder_der_nuevo = sensor.encoder_counts_right

            cambio_encoder_izq  = self.posicion.ajustar_overflow(encoder_izq_nuevo, encoder_izq_ini)
            cambio_encoder_der  = self.posicion.ajustar_overflow(encoder_der_nuevo, encoder_der_ini)

            # Distancia recorrida por cada rueda
            distancia_recorrida_izq = cambio_encoder_izq * MM_POR_TICK
            distancia_recorrida_der = cambio_encoder_der * MM_POR_TICK

            # Calcular el cambio angular en radianes
            cambio_angular = (distancia_recorrida_der - distancia_recorrida_izq) / BASE_RUEDAS
            cambio_angular *= correccion_velocidad 

            # Convertir a grados y acumular
            angulo_total_girado += math.degrees(cambio_angular * 2)
            self.posicion.actualizar_posicion(encoder_izq_nuevo, encoder_der_nuevo)

            # Actualizar valores iniciales de encoders
            encoder_izq_ini = encoder_izq_nuevo
            encoder_der_ini = encoder_der_nuevo

            time.sleep(0.005)

        self.roomba.drive_stop()
        print(f"Giro finalizado.")
        self.posicion.imprimir_posicion()
    
    def comportamiento_autonomo(controlador, *args, **kwargs):
        """Ejemplo de un comportamiento autónomo: mover en un cuadrado."""
        global modo_autonomo, detener_autonomo
        print("Modo autónomo iniciado. Presiona 'p' para detener.")
        lado = 50
        while modo_autonomo and not detener_autonomo:
            for _ in range(4): 
                controlador.adelante(lado)
                if detener_autonomo: 
                    break
                controlador._girar_roomba(90) 
                if detener_autonomo:
                    break
        print("Modo autónomo detenido.")
        modo_autonomo = False
        detener_autonomo = False


def main():
    global modo_autonomo, detener_autonomo

    if len(sys.argv) < 2:
        print("Uso: python3 roomba.py <puerto>")
        sys.exit(1)

    puerto = sys.argv[1]
    controlador = Controlador(puerto)

    # Imprimir odometría inicial
    print("\n===== ODOMETRÍA INICIAL =====")
    controlador.posicion.imprimir_posicion()

    # Verificar valores de los encoders al inicio
    try:
        sensor = controlador.roomba.get_sensors()
        if sensor is None:
            print("Error: No se pueden obtener datos de sensores.")
            return
    except Exception as e:
        print(f"Error al obtener datos de los sensores: {e}")
        return

    try:
        while True:
            print("\n===== MENÚ DE CONTROL =====")
            print("m <distancia en cm>   - Mover hacia adelante.")
            print("a <distancia en cm>   - Mover hacia atrás.")
            print("g <ángulo en grados>  - Girar la roomba (positivo: izquierda, negativo: derecha).")
            print("auto                  - Activar modo autónomo.")
            print("p                     - Detener modo autónomo.")
            print("e                     - Salir y detener la roomba.")
            print("======================================")

            command = input("Introduce tu instrucción: ").strip()

            if command.lower() == "e":
                break

            if command.lower() == "auto":
                if not modo_autonomo:
                    modo_autonomo = True
                    detener_autonomo = False
                    hilo_autonomo = threading.Thread(target=controlador.comportamiento_autonomo, args=(controlador,))
                    hilo_autonomo.start()
                else:
                    print("El modo autónomo ya está activo.")
                continue

            if command.lower() == "p":
                if modo_autonomo:
                    detener_autonomo = True
                else:
                    print("El modo autónomo no está activo.")
                continue

            parts = command.split()
            if len(parts) == 2:
                action = parts[0]
                try:
                    value = float(parts[1])
                except ValueError:
                    print("Debe ser un número válido.")
                    continue
            elif len(parts) == 1:
                action = parts[0]
                value = None
            else:
                print("Comando no válido.")
                continue

            if action.lower() == "m":
                if value < 0:
                    print("Error: Si quieres moverte hacia atrás, usa el comando 'a' con un valor negativo.")
                else:
                    controlador.adelante(value)
            elif action.lower() == "a":
                if value < 0:
                    print("Error: Si quieres moverte hacia adelante, usa el comando 'm' con un valor positivo.")
                else:
                    controlador.atras(value)
            elif action.lower() == "g":
                if value is None:
                    print("Error: Debes especificar un ángulo en grados.")
                else:
                    controlador._girar_roomba(value)
            else:
                print("Comando no reconocido.")

    except KeyboardInterrupt:
        print("\nInterrupción detectada.")
    finally:
        controlador.cerrar_roomba()


if __name__ == "__main__":
    main()