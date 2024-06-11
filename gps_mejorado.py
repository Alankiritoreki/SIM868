import machine  # Importa el módulo machine, que permite controlar el hardware de la Raspberry Pi Pico
import os  # Importa el módulo os para interactuar con el sistema operativo
import utime  # Importa el módulo utime para manejar el tiempo y retrasos
import binascii  # Importa el módulo binascii para conversiones entre binarios y ASCII

# Definición de pines
led_pin = 25  # Pin del LED a bordo
pwr_en = 14  # Pin para controlar la alimentación del módulo
uart_port = 0  # Puerto UART utilizado
uart_baute = 115200  # Velocidad en baudios para la comunicación UART

APN = "CMNET"  # APN para la configuración de la red celular

reading = 0  # Inicialización de variable de lectura
temperature = 0  # Inicialización de variable de temperatura

# Configuración de UART
uart = machine.UART(uart_port, uart_baute)  # Configura la UART con el puerto y la velocidad en baudios especificados
print(os.uname())  # Imprime información del sistema

# Indicador LED en la Raspberry Pi Pico
led_onboard = machine.Pin(led_pin, machine.Pin.OUT)  # Configura el pin del LED a bordo como salida

# Información del servidor MQTT
mqtt_host = '47.89.22.46'  # Dirección IP del servidor MQTT
mqtt_port = '1883'  # Puerto del servidor MQTT

# Definición de los temas MQTT
mqtt_topic1 = 'testtopic'
mqtt_topic2 = 'testtopic/led'
mqtt_topic3 = 'testtopic/temp'
mqtt_topic4 = 'testtopic/adc'
mqtt_topic5 = 'testtopic/tempwarning'
mqtt_topic6 = 'testtopic/warning'
mqtt_topic7 = 'testtopic/gpsinfo'

mqtt_msg = 'on'  # Mensaje MQTT predeterminado

# Filtro de Kalman
class KalmanFilter:
    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def update(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

        return self.posteri_estimate

# Inicialización de filtros de Kalman para latitud, longitud y altitud
kf_lat = KalmanFilter(1e-5, 1e-2)
kf_lon = KalmanFilter(1e-5, 1e-2)
kf_alt = KalmanFilter(1e-5, 1e-2)

# Función para parpadear el LED
def led_blink():
    led_onboard.value(1)  # Enciende el LED
    utime.sleep(1)  # Espera 1 segundo
    led_onboard.value(0)  # Apaga el LED
    utime.sleep(1)  # Espera 1 segundo
    led_onboard.value(1)  # Enciende el LED
    utime.sleep(1)  # Espera 1 segundo
    led_onboard.value(0)  # Apaga el LED

# Función para encender/apagar el módulo
def power_on_off():
    pwr_key = machine.Pin(pwr_en, machine.Pin.OUT)  # Configura el pin de control de alimentación como salida
    pwr_key.value(1)  # Enciende el módulo
    utime.sleep(2)  # Espera 2 segundos
    pwr_key.value(0)  # Apaga el módulo

# Función para convertir cadena hexadecimal a cadena normal
def hexstr_to_str(hex_str):
    hex_data = hex_str.encode('utf-8')  # Codifica la cadena hexadecimal a binario
    str_bin = binascii.unhexlify(hex_data)  # Convierte binario a cadena
    return str_bin.decode('utf-8')  # Decodifica la cadena a formato UTF-8

# Función para convertir cadena a cadena hexadecimal
def str_to_hexstr(string):
    str_bin = string.encode('utf-8')  # Codifica la cadena a binario
    return binascii.hexlify(str_bin).decode('utf-8')  # Convierte binario a hexadecimal y lo decodifica a UTF-8

# Función para esperar respuesta del módulo
def wait_resp_info(timeout=2000):
    prvmills = utime.ticks_ms()  # Obtiene el tiempo actual en milisegundos
    info = b""  # Inicializa la variable de información
    while (utime.ticks_ms()-prvmills) < timeout:  # Espera hasta que se agote el tiempo de espera
        if uart.any():  # Si hay datos en el buffer UART
            info = b"".join([info, uart.read(1)])  # Lee un byte de UART y lo añade a info
    print(info.decode())  # Imprime la información decodificada
    return info  # Devuelve la información

# Función para enviar comando AT
def send_at(cmd, back, timeout=2000):
    rec_buff = b''  # Inicializa el buffer de recepción
    uart.write((cmd+'\r\n').encode())  # Escribe el comando AT en UART
    prvmills = utime.ticks_ms()  # Obtiene el tiempo actual en milisegundos
    while (utime.ticks_ms()-prvmills) < timeout:  # Espera hasta que se agote el tiempo de espera
        if uart.any():  # Si hay datos en el buffer UART
            rec_buff = b"".join([rec_buff, uart.read(1)])  # Lee un byte de UART y lo añade al buffer de recepción
    if rec_buff != '':  # Si el buffer de recepción no está vacío
        if back not in rec_buff.decode():  # Si la respuesta no contiene el texto esperado
            print(cmd + ' back:\t' + rec_buff.decode())  # Imprime el comando y la respuesta recibida
            return 0  # Devuelve 0 si la respuesta no es la esperada
        else:
            print(rec_buff.decode())  # Imprime la respuesta recibida
            return 1  # Devuelve 1 si la respuesta es la esperada
    else:
        print(cmd + ' no responce')  # Imprime que no hubo respuesta

# Detección de arranque del módulo
def check_start():
    while True:  # Bucle infinito
        uart.write(bytearray(b'ATE1\r\n'))  # Envia comando ATE1 para activar eco
        utime.sleep(2)  # Espera 2 segundos
        uart.write(bytearray(b'AT\r\n'))  # Envia comando AT para comprobar la comunicación
        rec_temp = wait_resp_info()  # Espera respuesta
        if 'OK' in rec_temp.decode():  # Si la respuesta contiene 'OK'
            print('SIM868 is ready\r\n' + rec_temp.decode())  # Imprime que el SIM868 está listo y la respuesta
            break  # Sale del bucle
        else:
            power_on_off()  # Enciende/apaga el módulo
            print('SIM868 is starting up, please wait...\r\n')  # Imprime mensaje de inicio
            utime.sleep(8)  # Espera 8 segundos

# Comprobar el estado de la red
def check_network():
    for i in range(1, 3):  # Bucle para intentar dos veces
        if send_at("AT+CGREG?", "0,1") == 1:  # Envía comando AT+CGREG? para comprobar el registro en la red
            print('SIM868 is online\r\n')  # Imprime que el SIM868 está en línea
            break  # Sale del bucle
        else:
            print('SIM868 is offline, please wait...\r\n')  # Imprime que el SIM868 está fuera de línea
            utime.sleep(5)  # Espera 5 segundos
            continue  # Continúa con el siguiente intento
    send_at("AT+CPIN?", "OK")  # Comprueba el estado del PIN
    send_at("AT+CSQ", "OK")  # Comprueba la calidad de la señal
    send_at("AT+COPS?", "OK")  # Comprueba el operador actual
    send_at("AT+CGATT?", "OK")  # Comprueba el estado de la conexión GPRS
    send_at("AT+CGDCONT?", "OK")  # Comprueba el contexto de PDP
    send_at("AT+CSTT?", "OK")  # Comprueba el estado de la conexión a Internet
    send_at("AT+CSTT=\""+APN+"\"", "OK")  # Establece el APN
    send_at("AT+CIICR", "OK")  # Activa la conexión de datos
    send_at("AT+CIFSR", "OK")  # Obtiene la dirección IP

# Obtener información de GPS con mayor precisión
def get_gps_info():
    count = 0  # Inicializa el contador
    print('Start GPS session...')  # Imprime el inicio de la sesión GPS
    send_at('AT+CGNSPWR=1', 'OK')  # Enciende el receptor GNSS
    utime.sleep(2)  # Espera 2 segundos
    
    gps_data = []  # Inicializa la lista para datos GPS
    
    while count < 5:  # Recopilar 5 lecturas para promediar
        uart.write(bytearray(b'AT+CGNSINF\r\n'))  # Envía comando para obtener información GNSS
        rec_buff = wait_resp_info()  # Espera la respuesta
        
        if ',,,,' in rec_buff.decode():  # Si la respuesta indica que el GPS no está listo
            print('GPS is not ready')  # Imprime que el GPS no está listo
            utime.sleep(2)  # Espera 2 segundos
            continue  # Continúa con el siguiente intento
        
        gps_data.append(rec_buff.decode())  # Añade la respuesta a la lista de datos GPS
        count += 1  # Incrementa el contador
        print('GPS info:', rec_buff.decode())  # Imprime la información del GPS
        utime.sleep(2)  # Espera 2 segundos
    
    send_at('AT+CGNSPWR=0', 'OK')  # Apaga el receptor GNSS
    
    if gps_data:  # Si se ha recopilado información GPS
        average_gps_data(gps_data)  # Calcula el promedio de los datos GPS
    else:
        print('No valid GPS data collected')  # Imprime que no se ha recopilado información GPS válida

# Promediar los datos de GPS para mejorar la precisión
def average_gps_data(gps_data):
    latitudes = []  # Inicializa la lista para latitudes
    longitudes = []  # Inicializa la lista para longitudes
    altitudes = []  # Inicializa la lista para altitudes
    
    for data in gps_data:  # Itera sobre los datos GPS
        parts = data.split(',')  # Divide la cadena en partes
        if len(parts) > 8 and parts[1] == '1':  # Si hay suficientes partes y el estado del GPS es válido
            lat = float(parts[3])
            lon = float(parts[4])
            alt = float(parts[9])
            
            # Filtrar datos utilizando Kalman Filter
            lat_kf = kf_lat.update(lat)
            lon_kf = kf_lon.update(lon)
            alt_kf = kf_alt.update(alt)
            
            latitudes.append(lat_kf)  # Añade la latitud filtrada a la lista
            longitudes.append(lon_kf)  # Añade la longitud filtrada a la lista
            altitudes.append(alt_kf)  # Añade la altitud filtrada a la lista
    
    if latitudes and longitudes and altitudes:  # Si hay datos de latitud, longitud y altitud
        avg_lat = sum(latitudes) / len(latitudes)  # Calcula la latitud promedio
        avg_lon = sum(longitudes) / len(longitudes)  # Calcula la longitud promedio
        avg_alt = sum(altitudes) / len(altitudes)  # Calcula la altitud promedio
        print(f'Average Latitude: {avg_lat}')  # Imprime la latitud promedio
        print(f'Average Longitude: {avg_lon}')  # Imprime la longitud promedio
        print(f'Average Altitude: {avg_alt}')  # Imprime la altitud promedio
    else:
        print('Insufficient valid GPS data for averaging')  # Imprime que no hay suficientes datos GPS válidos

# Realizar una llamada telefónica
def phone_call(phone_num='10000', keep_time=10):
    send_at('AT+CHFA=1', 'OK')  # Establece el canal de audio
    send_at('ATD'+phone_num+';', 'OK')  # Marca el número de teléfono
    utime.sleep(keep_time)  # Espera durante el tiempo de la llamada
    send_at('AT+CHUP;', 'OK')  # Cuelga la llamada

# Prueba de Bluetooth
def bluetooth_test():
    send_at('AT+BTPOWER=1', 'OK', 3000)  # Enciende el Bluetooth
    send_at('AT+BTHOST?', 'OK', 3000)  # Consulta la dirección del host Bluetooth
    send_at('AT+BTSTATUS?', 'OK', 3000)  # Consulta el estado del Bluetooth
    send_at('AT+BTSCAN=1,10', 'OK', 8000)  # Escanea dispositivos Bluetooth durante 10 segundos
    send_at('AT+BTPOWER=0', 'OK')  # Apaga el Bluetooth

# Prueba de comandos AT
def at_test():
    print("---------------------------SIM868 TEST---------------------------")  # Imprime el encabezado de la prueba
    while True:  # Bucle infinito
        try:
            command_input = str(input('Please input the AT command,press Ctrl+C to exit: '))  # Solicita un comando AT al usuario
            send_at(command_input, 'OK', 2000)  # Envía el comando AT
        except KeyboardInterrupt:
            print("\r\nExit AT command test!\n")  # Imprime que se ha salido de la prueba de comandos AT
            power_on_off()  # Apaga el módulo
            print("Power off the module!\n")  # Imprime que el módulo se ha apagado
            break  # Sale del bucle

# Programa principal
check_start()  # Verifica el arranque del módulo
check_network()  # Verifica el estado de la red
get_gps_info()  # Obtiene información de GPS
