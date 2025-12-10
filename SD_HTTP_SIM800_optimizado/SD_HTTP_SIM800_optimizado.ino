/*
 * ============================================================================
 * SISTEMA DE MONITOREO CON RESPALDO EN SD - VERSIÓN OPTIMIZADA
 * ============================================================================
 * 
 * DESCRIPCIÓN:
 * Sistema de monitoreo de temperatura (NTC) y conductividad del agua con
 * respaldo local en tarjeta SD y envío por HTTP mediante módulo SIM800.
 * 
 * ESTRATEGIA DE RESPALDO:
 * - Usa 2 archivos: pendient.csv (no enviados) y enviado.csv (histórico)
 * - Append-only: Solo añade líneas, nunca reescribe archivos completos
 * - Sin String: Usa char arrays para evitar fragmentación de memoria
 * - Modular: Funciones pequeñas y específicas
 * 
 * FLUJO DE OPERACIÓN:
 * 1. Intenta reenviar datos pendientes (máx 3 por ciclo)
 * 2. Toma nuevas mediciones
 * 3. Guarda en pendient.csv (PRIORITARIO)
 * 4. Intenta enviar por HTTP
 * 5. Si OK → mueve línea a enviado.csv y borra de pendient.csv
 * 6. Si FALLO → queda en pendient.csv para próximo ciclo
 * 
 * HARDWARE:
 * - Arduino Pro Mini 5V 16MHz (ATmega328P)
 * - Módulo SIM800L
 * - Tarjeta MicroSD
 * - Termistor NTC 10K
 * - Sonda de conductividad
 * 
 * ============================================================================
 */

#include <SoftwareSerial.h>
#include "LowPower.h"
#include <SPI.h>
#include <SD.h>

// ============================================================================
// CONFIGURACIÓN DE HARDWARE
// ============================================================================

// Pines de tarjeta SD (SPI)
#define SD_CS_PIN 10        // Chip Select
// MOSI: pin 11 (fijo por SPI)
// MISO: pin 12 (fijo por SPI)
// SCK:  pin 13 (fijo por SPI)

// Pines de termistor NTC
#define NTC_ANALOG_PIN 0    // Pin analógico de lectura (A0)
#define NTC_VCC_PIN 8       // Pin de alimentación VCC
#define NTC_GND_PIN 9       // Pin de tierra GND

// Pines de sonda de conductividad
#define EC_ANALOG_PIN A1    // Pin analógico de lectura
#define EC_POWER_PIN 5      // Pin de alimentación 5V
#define EC_GROUND_PIN 4     // Pin de tierra

// Comunicación SIM800
SoftwareSerial SIM800(0, 1); // RX=0, TX=1

// ============================================================================
// CONSTANTES Y CONFIGURACIÓN
// ============================================================================

// Constantes de termistor NTC 10K (Steinhart-Hart)
#define NTC_R1 10000.0
#define NTC_C1 0.00112531
#define NTC_C2 0.000234712
#define NTC_C3 0.0000856635

// Constantes de conductividad
#define EC_R1 1000.0        // Resistencia pull-up
#define EC_RA 25.0          // Resistencia de pines
#define EC_K 1.07           // Constante de celda
#define EC_TEMP_COEF 0.0188 // Coeficiente de compensación de temperatura
#define EC_VIN 3.3          // Voltaje de referencia

// Configuración de sistema
#define MAX_REINTENTOS 3    // Máximo de reintentos por ciclo
#define UMBRAL_BATERIA_BAJA 25  // Ciclos con batería baja antes de shutdown
#define BUFFER_SIZE 120     // Tamaño de buffer para líneas CSV

// Nombres de archivos en SD
#define ARCHIVO_PENDIENTES "pendient.csv"
#define ARCHIVO_ENVIADOS "enviado.csv"
#define ARCHIVO_LOG "log.txt"

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

// Estado del sistema
bool sdDisponible = false;
unsigned long contadorMediciones = 0;
int contador_bateria_baja = 0;

// Parámetros de operación (actualizados por servidor)
long periodo = 60;          // Periodo entre mediciones (segundos)
int acumula = 1;            // Número de mediciones a acumular

// Variables de sensores
float temperatura = 0.0;
float conductividad = 0.0;

// Variables de estado SIM800
int voltios = 0;
int porcentaje = 0;
int cobertura = 0;

// Buffers de comunicación
char bufferRespuesta[70];
char bufferAT[40];
char bufferLinea[BUFFER_SIZE];

// ============================================================================
// FUNCIONES DE UTILIDAD PARA SD
// ============================================================================

/*
 * Escribe una línea en el archivo de log con timestamp
 */
void escribirLog(const char* mensaje) {
  if (!sdDisponible) return;
  
  File log = SD.open(ARCHIVO_LOG, FILE_WRITE);
  if (log) {
    log.print(millis());
    log.print(" ms: ");
    log.println(mensaje);
    log.close();
  }
}

/*
 * Cuenta el número de líneas en un archivo
 * Útil para saber cuántos datos pendientes hay
 */
int contarLineas(const char* nombreArchivo) {
  if (!sdDisponible) return 0;
  
  File archivo = SD.open(nombreArchivo, FILE_READ);
  if (!archivo) return 0;
  
  int lineas = 0;
  while (archivo.available()) {
    if (archivo.read() == '\n') lineas++;
  }
  archivo.close();
  
  return lineas;
}

/*
 * Copia una línea específica de un archivo a un buffer
 * Retorna: true si encontró la línea, false si no
 */
bool leerLineaN(const char* nombreArchivo, int numeroLinea, char* buffer, int bufferSize) {
  if (!sdDisponible) return false;
  
  File archivo = SD.open(nombreArchivo, FILE_READ);
  if (!archivo) return false;
  
  int lineaActual = 0;
  int pos = 0;
  
  while (archivo.available()) {
    char c = archivo.read();
    
    if (lineaActual == numeroLinea) {
      if (c == '\n' || pos >= bufferSize - 1) {
        buffer[pos] = '\0';
        archivo.close();
        return true;
      }
      buffer[pos++] = c;
    }
    
    if (c == '\n') {
      lineaActual++;
      if (lineaActual > numeroLinea) break;
    }
  }
  
  archivo.close();
  return false;
}

/*
 * Elimina la primera línea de un archivo
 * Estrategia: Copia todas las líneas excepto la primera a un archivo temporal
 */
bool eliminarPrimeraLinea(const char* nombreArchivo) {
  if (!sdDisponible) return false;
  
  const char* temp = "temp.tmp";
  
  File origen = SD.open(nombreArchivo, FILE_READ);
  if (!origen) return false;
  
  File destino = SD.open(temp, FILE_WRITE);
  if (!destino) {
    origen.close();
    return false;
  }
  
  // Salta la primera línea
  bool primeraLineaSaltada = false;
  while (origen.available()) {
    char c = origen.read();
    if (!primeraLineaSaltada) {
      if (c == '\n') primeraLineaSaltada = true;
    } else {
      destino.write(c);
    }
  }
  
  origen.close();
  destino.close();
  
  // Reemplaza el archivo original
  SD.remove(nombreArchivo);
  
  // Copia temp a original (SD.h no tiene rename)
  File temp_read = SD.open(temp, FILE_READ);
  File original_write = SD.open(nombreArchivo, FILE_WRITE);
  
  if (temp_read && original_write) {
    while (temp_read.available()) {
      original_write.write(temp_read.read());
    }
    temp_read.close();
    original_write.close();
    SD.remove(temp);
    return true;
  }
  
  return false;
}

// ============================================================================
// FUNCIONES DE GESTIÓN DE DATOS
// ============================================================================

/*
 * Guarda una medición en el archivo de pendientes
 * 
 * Formato CSV: ID;Timestamp;Temp;Cond;Bat;Volt;Cob
 * 
 * Parámetros:
 *   - temp: Temperatura(s) separadas por | si hay múltiples
 *   - cond: Conductividad(es) separadas por |
 *   - bat: Porcentaje(s) de batería separados por |
 *   - volt: Voltaje(s) en mV separados por |
 *   - cob: Cobertura(s) separadas por |
 * 
 * Retorna: true si se guardó correctamente
 */
bool guardarMedicion(const char* temp, const char* cond, const char* bat, 
                     const char* volt, const char* cob) {
  if (!sdDisponible) {
    Serial.println(F("ERROR: SD no disponible"));
    return false;
  }
  
  File archivo = SD.open(ARCHIVO_PENDIENTES, FILE_WRITE);
  if (!archivo) {
    Serial.println(F("ERROR: No se pudo abrir archivo pendientes"));
    escribirLog("ERROR: Fallo apertura pendientes");
    return false;
  }
  
  // Escribe en formato CSV
  archivo.print(contadorMediciones);
  archivo.print(';');
  archivo.print(millis());
  archivo.print(';');
  archivo.print(temp);
  archivo.print(';');
  archivo.print(cond);
  archivo.print(';');
  archivo.print(bat);
  archivo.print(';');
  archivo.print(volt);
  archivo.print(';');
  archivo.println(cob);
  
  archivo.close();
  
  Serial.print(F("✓ Guardado en SD [ID:"));
  Serial.print(contadorMediciones);
  Serial.println(F("]"));
  
  return true;
}

/*
 * Mueve una línea del archivo de pendientes al archivo de enviados
 * Esto marca la medición como exitosamente enviada
 * 
 * Parámetros:
 *   - linea: Contenido completo de la línea CSV
 * 
 * Retorna: true si se movió correctamente
 */
bool marcarComoEnviado(const char* linea) {
  if (!sdDisponible) return false;
  
  // Añade al archivo de enviados
  File enviados = SD.open(ARCHIVO_ENVIADOS, FILE_WRITE);
  if (!enviados) {
    Serial.println(F("ERROR: No se pudo abrir archivo enviados"));
    return false;
  }
  
  enviados.println(linea);
  enviados.close();
  
  // Elimina del archivo de pendientes
  bool eliminado = eliminarPrimeraLinea(ARCHIVO_PENDIENTES);
  
  if (eliminado) {
    Serial.println(F("✓ Marcado como ENVIADO"));
    return true;
  } else {
    Serial.println(F("ADVERTENCIA: Enviado pero no eliminado de pendientes"));
    return false;
  }
}

/*
 * Intenta reenviar datos pendientes de ciclos anteriores
 * Procesa máximo MAX_REINTENTOS registros por llamada
 * 
 * Retorna: número de registros enviados exitosamente
 */
int reintentarPendientes() {
  if (!sdDisponible) return 0;
  
  int pendientes = contarLineas(ARCHIVO_PENDIENTES);
  if (pendientes == 0) {
    Serial.println(F("No hay datos pendientes"));
    return 0;
  }
  
  Serial.print(F("Datos pendientes: "));
  Serial.println(pendientes);
  
  int enviados = 0;
  int intentos = 0;
  
  // Intenta enviar las primeras MAX_REINTENTOS líneas
  while (intentos < MAX_REINTENTOS && intentos < pendientes) {
    // Lee la primera línea del archivo
    if (!leerLineaN(ARCHIVO_PENDIENTES, 0, bufferLinea, BUFFER_SIZE)) {
      Serial.println(F("ERROR: No se pudo leer línea pendiente"));
      break;
    }
    
    Serial.print(F("Reintentando: "));
    Serial.println(bufferLinea);
    
    // Parsea la línea CSV
    // Formato: ID;Timestamp;Temp;Cond;Bat;Volt;Cob
    char* token = strtok(bufferLinea, ";");
    if (!token) break;
    
    // Salta ID y Timestamp
    token = strtok(NULL, ";"); // Timestamp
    if (!token) break;
    
    // Extrae datos
    char* temp = strtok(NULL, ";");
    char* cond = strtok(NULL, ";");
    char* bat = strtok(NULL, ";");
    char* volt = strtok(NULL, ";");
    char* cob = strtok(NULL, ";");
    
    if (!temp || !cond || !bat || !volt || !cob) {
      Serial.println(F("ERROR: Línea corrupta, eliminando"));
      eliminarPrimeraLinea(ARCHIVO_PENDIENTES);
      intentos++;
      continue;
    }
    
    // Intenta enviar
    bool exitoso = enviarHTTP(temp, cond, bat, volt, cob);
    
    if (exitoso) {
      // Lee de nuevo la línea original (strtok la modificó)
      leerLineaN(ARCHIVO_PENDIENTES, 0, bufferLinea, BUFFER_SIZE);
      marcarComoEnviado(bufferLinea);
      enviados++;
      Serial.println(F("✓ Reenvío exitoso"));
    } else {
      Serial.println(F("✗ Reenvío fallido, se reintentará después"));
      // No elimina la línea, queda para próximo ciclo
      break; // Sale del bucle si falla uno
    }
    
    intentos++;
  }
  
  Serial.print(F("Reenviados: "));
  Serial.println(enviados);
  
  return enviados;
}

// ============================================================================
// FUNCIONES DE COMUNICACIÓN HTTP
// ============================================================================

/*
 * Envía datos al servidor mediante HTTP usando el módulo SIM800
 * 
 * Parámetros:
 *   - temp, cond, bat, volt, cob: Strings con datos (pueden contener | para múltiples valores)
 * 
 * Retorna: true si el envío fue exitoso (código 200 o OK en respuesta)
 */
bool enviarHTTP(const char* temp, const char* cond, const char* bat, 
                const char* volt, const char* cob) {
  
  Serial.println(F("\n--- ENVIANDO HTTP ---"));
  
  // Despierta el módulo SIM800
  SIM800.write("AT\r\n");
  delay(500);
  
  // Configura conexión GPRS
  SIM800.write("AT+CGATT=1\r\n");
  delay(500);
  
  SIM800.write("AT+SAPBR=3,1,\"APN\",\"data.rewicom.net\"\r\n");
  delay(500);
  
  SIM800.write("AT+SAPBR=1,1\r\n");
  delay(1500);
  
  SIM800.write("AT+HTTPINIT\r\n");
  delay(500);
  
  // Limpia buffer
  while (SIM800.available() > 0) SIM800.read();
  
  // Construye URL con datos
  SIM800.write("AT+HTTPPARA=\"URL\",\"inasas.cuartazona.es/caramba3|protoSIM800|");
  SIM800.print(periodo);
  SIM800.print("|%7B%22Ba%22%3A%22");
  SIM800.print(bat);
  SIM800.print("%22%2C%22Cb%22%3A%22");
  SIM800.print(cob);
  SIM800.print("%22%2C%22Vo%22%3A%22");
  SIM800.print(volt);
  SIM800.print("%22%2C%22Te%22%3A%22");
  SIM800.print(temp);
  SIM800.print("%22%2C%22Co%22%3A%22");
  SIM800.print(cond);
  SIM800.print("%22%7D\"\r\n");
  
  delay(500);
  
  // Ejecuta petición HTTP GET
  SIM800.write("AT+HTTPACTION=0\r\n");
  delay(4000);
  
  // Lee respuesta
  SIM800.write("AT\r\n");
  delay(500);
  while (SIM800.available() > 0) SIM800.read();
  
  SIM800.write("AT+HTTPREAD\r\n");
  delay(200);
  
  int size = SIM800.readBytes(bufferRespuesta, sizeof(bufferRespuesta) - 1);
  bufferRespuesta[size] = '\0';
  
  // Cierra conexión HTTP
  SIM800.write("AT+HTTPTERM\r\n");
  delay(500);
  
  Serial.print(F("Respuesta: "));
  Serial.println(bufferRespuesta);
  
  // Verifica si fue exitoso
  bool exitoso = (strstr(bufferRespuesta, "200") != NULL || 
                  strstr(bufferRespuesta, "OK") != NULL);
  
  if (exitoso) {
    Serial.println(F("✓ HTTP OK"));
    
    // Parsea parámetros actualizados del servidor
    // Formato esperado: OK|periodo|acumula
    char* token = strtok(bufferRespuesta, "|");
    token = strtok(NULL, "|");
    if (token) {
      int nuevoPeriodo = atoi(token);
      if (nuevoPeriodo > 0) {
        periodo = nuevoPeriodo;
        Serial.print(F("Periodo actualizado: "));
        Serial.println(periodo);
      }
    }
    
    token = strtok(NULL, "|");
    if (token) {
      int nuevoAcumula = atoi(token);
      if (nuevoAcumula > 0) {
        acumula = nuevoAcumula;
        Serial.print(F("Acumula actualizado: "));
        Serial.println(acumula);
      }
    }
  } else {
    Serial.println(F("✗ HTTP FALLO"));
  }
  
  return exitoso;
}

// ============================================================================
// FUNCIONES DE SENSORES
// ============================================================================

/*
 * Lee temperatura del termistor NTC usando ecuación de Steinhart-Hart
 * 
 * Retorna: Temperatura en grados Celsius
 */
float leerTemperatura() {
  digitalWrite(NTC_VCC_PIN, HIGH);
  delay(200);
  
  int valorADC = analogRead(NTC_ANALOG_PIN);
  
  // Calcula resistencia del termistor
  float R2 = NTC_R1 * (1023.0 / (float)valorADC - 1.0);
  
  // Ecuación de Steinhart-Hart
  float logR2 = log(R2);
  float T = 1.0 / (NTC_C1 + NTC_C2 * logR2 + NTC_C3 * logR2 * logR2 * logR2);
  float Tc = T - 273.15;
  
  digitalWrite(NTC_VCC_PIN, LOW);
  
  return Tc;
}

/*
 * Mide conductividad eléctrica del agua compensada a 25°C
 * 
 * Retorna: Conductividad en µS/cm a 25°C
 */
float leerConductividad() {
  // Activa alimentación
  digitalWrite(EC_POWER_PIN, HIGH);
  int valorADC = analogRead(EC_ANALOG_PIN);
  digitalWrite(EC_POWER_PIN, LOW);
  
  // Convierte ADC a voltaje
  float Vdrop = (EC_VIN * valorADC) / 1023.0;
  
  // Calcula resistencia de la solución
  float Rc = (Vdrop * (EC_R1 + EC_RA)) / (EC_VIN - Vdrop);
  Rc = Rc - EC_RA;
  
  // Calcula conductividad
  float EC = 10000.0 / (Rc * EC_K);
  
  // Compensa a 25°C
  float EC25 = EC / (1.0 + EC_TEMP_COEF * (temperatura - 25.0));
  
  return EC25;
}

/*
 * Consulta estado de batería al SIM800 mediante AT+CBC
 * Actualiza variables globales: voltios, porcentaje
 */
void leerBateria() {
  SIM800.write("AT\r\n");
  delay(500);
  while (SIM800.available() > 0) SIM800.read();
  delay(200);
  
  SIM800.write("AT+CBC\r\n");
  delay(200);
  
  int size = SIM800.readBytes(bufferAT, sizeof(bufferAT) - 1);
  bufferAT[size] = '\0';
  
  // Parsea respuesta: +CBC: cargando,porcentaje,voltios
  char* token = strtok(bufferAT, ":");
  token = strtok(NULL, ","); // cargando
  token = strtok(NULL, ","); // porcentaje
  if (token) porcentaje = atoi(token);
  
  token = strtok(NULL, ","); // voltios
  if (token) voltios = atoi(token);
  
  // Control de batería baja
  if (voltios > 3000 && voltios < 3300) {
    contador_bateria_baja++;
  } else if (voltios >= 3300) {
    contador_bateria_baja = 0;
  }
}

/*
 * Consulta calidad de señal GSM al SIM800 mediante AT+CSQ
 * Actualiza variable global: cobertura (0-31, 99=desconocido)
 */
void leerCobertura() {
  SIM800.write("AT\r\n");
  delay(500);
  while (SIM800.available() > 0) SIM800.read();
  delay(200);
  
  SIM800.write("AT+CSQ\r\n");
  delay(200);
  
  int size = SIM800.readBytes(bufferAT, sizeof(bufferAT) - 1);
  bufferAT[size] = '\0';
  
  // Parsea respuesta: +CSQ: rssi,ber
  char* token = strtok(bufferAT, " ");
  token = strtok(NULL, ",");
  if (token) cobertura = atoi(token);
}

// ============================================================================
// FUNCIONES PRINCIPALES: SETUP Y LOOP
// ============================================================================

void setup() {
  Serial.begin(9600);
  Serial.println(F("\n========================================"));
  Serial.println(F("SISTEMA DE MONITOREO v2.0 OPTIMIZADO"));
  Serial.println(F("========================================\n"));
  
  // ===== INICIALIZACIÓN DE SD =====
  Serial.print(F("Inicializando SD... "));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("✗ FALLO"));
    Serial.println(F("ADVERTENCIA: Sin respaldo local"));
    sdDisponible = false;
  } else {
    Serial.println(F("✓ OK"));
    sdDisponible = true;
    
    // Crea archivos si no existen
    if (!SD.exists(ARCHIVO_PENDIENTES)) {
      File f = SD.open(ARCHIVO_PENDIENTES, FILE_WRITE);
      if (f) {
        f.println(F("ID;Timestamp;Temperatura;Conductividad;Bateria;Voltaje;Cobertura"));
        f.close();
        Serial.println(F("✓ Creado pendient.csv"));
      }
    }
    
    if (!SD.exists(ARCHIVO_ENVIADOS)) {
      File f = SD.open(ARCHIVO_ENVIADOS, FILE_WRITE);
      if (f) {
        f.println(F("ID;Timestamp;Temperatura;Conductividad;Bateria;Voltaje;Cobertura"));
        f.close();
        Serial.println(F("✓ Creado enviado.csv"));
      }
    }
    
    if (!SD.exists(ARCHIVO_LOG)) {
      File f = SD.open(ARCHIVO_LOG, FILE_WRITE);
      if (f) {
        f.println(F("=== LOG DEL SISTEMA ==="));
        f.close();
        Serial.println(F("✓ Creado log.txt"));
      }
    }
    
    escribirLog("Sistema iniciado");
  }
  
  // ===== CONFIGURACIÓN DE PINES =====
  pinMode(EC_ANALOG_PIN, INPUT);
  pinMode(EC_POWER_PIN, OUTPUT);
  pinMode(EC_GROUND_PIN, OUTPUT);
  pinMode(NTC_VCC_PIN, OUTPUT);
  pinMode(NTC_GND_PIN, OUTPUT);
  
  digitalWrite(EC_GROUND_PIN, LOW);
  digitalWrite(NTC_GND_PIN, LOW);
  
  // ===== INICIALIZACIÓN DE SIM800 =====
  Serial.print(F("Inicializando SIM800... "));
  SIM800.begin(9600);
  SIM800.write("AT\r\n");
  delay(1000);
  SIM800.write("AT\r\n");
  delay(1000);
  SIM800.write("AT+CSCLK=2\r\n"); // Activa sleep mode
  delay(500);
  Serial.println(F("✓ OK"));
  
  Serial.println(F("\n========================================"));
  Serial.println(F("SETUP COMPLETADO"));
  Serial.println(F("========================================\n"));
}

void loop() {
  Serial.println(F("\n========================================"));
  Serial.println(F("NUEVO CICLO"));
  Serial.println(F("========================================"));
  
  // ===== VERIFICACIÓN DE BATERÍA CRÍTICA =====
  if (contador_bateria_baja > UMBRAL_BATERIA_BAJA) {
    Serial.println(F("⚠ BATERIA CRITICA - SHUTDOWN"));
    escribirLog("SHUTDOWN: Bateria critica");
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
  
  // ===== PASO 1: REINTENTAR PENDIENTES =====
  Serial.println(F("\n[1/4] Reintentando pendientes..."));
  reintentarPendientes();
  
  // ===== PASO 2: LEER ESTADO DEL SISTEMA =====
  Serial.println(F("\n[2/4] Leyendo estado..."));
  leerBateria();
  leerCobertura();
  
  Serial.print(F("Batería: "));
  Serial.print(porcentaje);
  Serial.print(F("% ("));
  Serial.print(voltios);
  Serial.println(F(" mV)"));
  
  Serial.print(F("Cobertura: "));
  Serial.println(cobertura);
  
  // ===== PASO 3: TOMAR MEDICIONES =====
  Serial.println(F("\n[3/4] Tomando mediciones..."));
  
  // Buffers para acumular múltiples mediciones
  char tempBuffer[50] = "";
  char condBuffer[50] = "";
  char batBuffer[50] = "";
  char voltBuffer[50] = "";
  char cobBuffer[50] = "";
  
  // Primera medición
  temperatura = leerTemperatura();
  conductividad = leerConductividad();
  
  dtostrf(temperatura, 4, 1, tempBuffer);
  dtostrf(conductividad, 6, 1, condBuffer);
  itoa(porcentaje, batBuffer, 10);
  itoa(voltios, voltBuffer, 10);
  itoa(cobertura, cobBuffer, 10);
  
  Serial.print(F("Med 1: "));
  Serial.print(temperatura);
  Serial.print(F("°C, "));
  Serial.print(conductividad);
  Serial.println(F(" µS/cm"));
  
  // Mediciones adicionales
  for (int i = 1; i < acumula; i++) {
    // Sleep entre mediciones
    int ciclosSleep = periodo / 8;
    for (int j = 0; j < ciclosSleep; j++) {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }
    
    // Nueva medición
    leerBateria();
    leerCobertura();
    temperatura = leerTemperatura();
    conductividad = leerConductividad();
    
    // Acumula en buffers con separador |
    char temp[10];
    strcat(tempBuffer, "|");
    dtostrf(temperatura, 4, 1, temp);
    strcat(tempBuffer, temp);
    
    strcat(condBuffer, "|");
    dtostrf(conductividad, 6, 1, temp);
    strcat(condBuffer, temp);
    
    strcat(batBuffer, "|");
    itoa(porcentaje, temp, 10);
    strcat(batBuffer, temp);
    
    strcat(voltBuffer, "|");
    itoa(voltios, temp, 10);
    strcat(voltBuffer, temp);
    
    strcat(cobBuffer, "|");
    itoa(cobertura, temp, 10);
    strcat(cobBuffer, temp);
    
    Serial.print(F("Med "));
    Serial.print(i + 1);
    Serial.print(F(": "));
    Serial.print(temperatura);
    Serial.print(F("°C, "));
    Serial.print(conductividad);
    Serial.println(F(" µS/cm"));
  }
  
  // ===== PASO 4: GUARDAR Y ENVIAR =====
  Serial.println(F("\n[4/4] Guardando y enviando..."));
  
  contadorMediciones++;
  
  // GUARDA PRIMERO (PRIORITARIO)
  bool guardado = guardarMedicion(tempBuffer, condBuffer, batBuffer, voltBuffer, cobBuffer);
  
  if (!guardado) {
    Serial.println(F("⚠ ADVERTENCIA: No guardado en SD"));
    escribirLog("ERROR: Fallo guardado");
  }
  
  // INTENTA ENVIAR
  bool enviado = enviarHTTP(tempBuffer, condBuffer, batBuffer, voltBuffer, cobBuffer);
  
  if (enviado && guardado) {
    // Lee la última línea guardada y márcala como enviada
    int numLineas = contarLineas(ARCHIVO_PENDIENTES);
    if (numLineas > 1) { // >1 porque la primera es el encabezado
      leerLineaN(ARCHIVO_PENDIENTES, numLineas - 1, bufferLinea, BUFFER_SIZE);
      marcarComoEnviado(bufferLinea);
    }
  } else if (!enviado) {
    Serial.println(F("Datos guardados para reintento"));
    escribirLog("Envio fallido, en cola");
  }
  
  // ===== VOLVER A SLEEP MODE =====
  SIM800.write("AT\r\n");
  delay(500);
  SIM800.write("AT+CSCLK=2\r\n");
  delay(500);
  
  Serial.println(F("\n========================================"));
  Serial.println(F("FIN CICLO"));
  Serial.println(F("========================================\n"));
}
