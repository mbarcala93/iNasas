/*
================================================================================
  SISTEMA MODULAR DE MONITOREO CON RESPALDO EN SD
================================================================================
  
  Descripción:
  Sistema profesional y modular para monitoreo de múltiples sensores con:
  - Respaldo automático en tarjeta SD
  - Envío de datos por HTTP (SIM800L)
  - Arquitectura modular para activar/desactivar sensores fácilmente
  - Sistema de reintentos para datos no enviados
  - Optimizado para Arduino Pro Mini (baja RAM)
  
  Hardware:
  - Arduino Pro Mini 5V 16MHz
  - Módulo SIM800L
  - Tarjeta MicroSD
  - Sensores configurables (ver sección CONFIGURACIÓN)
    
================================================================================
*/

#include <SoftwareSerial.h>
#include "LowPower.h"
#include <SPI.h>
#include <SD.h>

// ============================================================================
// CONFIGURACIÓN DE SENSORES - ACTIVA/DESACTIVA AQUÍ
// ============================================================================
// Cambia true/false para activar/desactivar cada sensor

#define SENSOR_TEMPERATURA    true   // Termistor NTC 10K
#define SENSOR_CONDUCTIVIDAD  true   // Sonda de conductividad
#define SENSOR_OD             true   // Oxígeno Disuelto (DO)

// Sensores futuros (desactivados por ahora)
#define SENSOR_ULTRASONICO    false  // Distancia ultrasónica
#define SENSOR_TURBIDEZ       false  // Turbidez del agua
#define SENSOR_CORRIENTE      false  // Consumo energético

// ============================================================================
// CONFIGURACIÓN DE PINES
// ============================================================================

// Tarjeta SD (SPI)
#define SD_CS_PIN       10  // Chip Select
// MOSI: pin 11, MISO: pin 12, SCK: pin 13

// SIM800L
SoftwareSerial SIM800(0, 1); // RX=0, TX=1

// Sensor de Temperatura (NTC)
#if SENSOR_TEMPERATURA
  #define NTC_ANALOG_PIN    A0
  #define NTC_VCC_PIN       8
  #define NTC_GND_PIN       9
#endif

// Sensor de Conductividad
#if SENSOR_CONDUCTIVIDAD
  #define EC_ANALOG_PIN     A1
  #define EC_POWER_PIN      5
  #define EC_GROUND_PIN     4
#endif

// Sensor de Oxígeno Disuelto
#if SENSOR_OD
  #define DO_ANALOG_PIN     A2
  // Nota: Este sensor no necesita pines de alimentación adicionales
#endif

// ============================================================================
// CONFIGURACIÓN DEL SISTEMA
// ============================================================================

#define PERIODO_DEFAULT     60    // Segundos entre mediciones
#define ACUMULA_DEFAULT     1     // Mediciones a acumular antes de enviar
#define MAX_REINTENTOS      3     // Reintentos de envío por ciclo
#define UMBRAL_BATERIA_BAJA 25    // Ciclos con batería baja antes de apagar

// APN del operador
#define APN_OPERADOR        "data.rewicom.net"

// Archivos en SD
#define ARCHIVO_PENDIENTES  "pendient.csv"
#define ARCHIVO_ENVIADOS    "enviado.csv"
#define ARCHIVO_LOG         "log.txt"

// ============================================================================
// CONSTANTES DE CALIBRACIÓN - TEMPERATURA
// ============================================================================
#if SENSOR_TEMPERATURA
  const float NTC_R1 = 10000.0;           // Resistencia del termistor (10K)
  const float NTC_C1 = 0.00112531;        // Coeficientes Steinhart-Hart
  const float NTC_C2 = 0.000234712;
  const float NTC_C3 = 0.0000856635;
#endif

// ============================================================================
// CONSTANTES DE CALIBRACIÓN - CONDUCTIVIDAD
// ============================================================================
#if SENSOR_CONDUCTIVIDAD
  const float EC_R1 = 1000.0;             // Resistencia pull-up
  const float EC_RA = 25.0;               // Resistencia de pines
  const float EC_K = 1.07;                // Constante de celda
  const float EC_VIN = 3.3;               // Voltaje de referencia
  const float EC_TEMP_COEF = 0.0188;      // Coeficiente compensación temperatura
#endif

// ============================================================================
// CONSTANTES DE CALIBRACIÓN - OXÍGENO DISUELTO
// ============================================================================
#if SENSOR_OD
  #define DO_VREF           5000          // Voltaje de referencia (mV)
  #define DO_ADC_RES        1024          // Resolución ADC
  
  // Calibración (ajustar según tu calibración)
  // Modo: 0 = Un punto, 1 = Dos puntos
  #define DO_TWO_POINT_CAL  0
  
  // Calibración de un punto (saturación a temperatura conocida)
  #define DO_CAL1_V         1600          // Voltaje saturación (mV)
  #define DO_CAL1_T         25            // Temperatura calibración (°C)
  
  // Calibración de dos puntos (si DO_TWO_POINT_CAL = 1)
  #define DO_CAL2_V         1300          // Voltaje baja temperatura (mV)
  #define DO_CAL2_T         15            // Temperatura baja (°C)
  
  // Tabla de DO saturado vs temperatura (μg/L)
  const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
  };
#endif

// ============================================================================
// VARIABLES GLOBALES
// ============================================================================

// Sistema
long periodo = PERIODO_DEFAULT;
int acumula = ACUMULA_DEFAULT;
int contador_bateria_baja = 0;

// Estado del sistema
int voltios = 0;
int porcentaje = 0;
int cobertura = 0;

// Temperatura actual (compartida entre sensores)
float temperatura_actual = 25.0;

// Buffers de comunicación
char buffer_http[70];
char buffer_sim[40];

// ============================================================================
// ESTRUCTURA DE DATOS DE SENSORES
// ============================================================================

struct DatosSensores {
  #if SENSOR_TEMPERATURA
    float temperatura;
  #endif
  
  #if SENSOR_CONDUCTIVIDAD
    float conductividad;
  #endif
  
  #if SENSOR_OD
    float oxigeno_disuelto;
  #endif
  
  // Sistema
  int bateria;
  int voltaje;
  int cobertura_gsm;
  unsigned long timestamp;
};

// ============================================================================
// FUNCIONES DE LECTURA DE SENSORES
// ============================================================================

#if SENSOR_TEMPERATURA
/*
  Función: leerTemperatura
  Descripción: Lee temperatura del termistor NTC usando ecuación Steinhart-Hart
  Retorna: Temperatura en °C
*/
float leerTemperatura() {
  digitalWrite(NTC_VCC_PIN, HIGH);
  delay(200);  // Estabilización
  
  int lectura = analogRead(NTC_ANALOG_PIN);
  
  // Calcular resistencia
  float R2 = NTC_R1 * (1023.0 / (float)lectura - 1.0);
  float logR2 = log(R2);
  
  // Ecuación Steinhart-Hart
  float T = 1.0 / (NTC_C1 + NTC_C2 * logR2 + NTC_C3 * logR2 * logR2 * logR2);
  float Tc = T - 273.15;
  
  digitalWrite(NTC_VCC_PIN, LOW);
  
  return Tc;
}
#endif

// ----------------------------------------------------------------------------

#if SENSOR_CONDUCTIVIDAD
/*
  Función: leerConductividad
  Descripción: Lee conductividad del agua con compensación de temperatura
  Retorna: Conductividad en μS/cm compensada a 25°C
*/
float leerConductividad() {
  // Activar sensor
  digitalWrite(EC_POWER_PIN, HIGH);
  int lectura = analogRead(EC_ANALOG_PIN);
  digitalWrite(EC_POWER_PIN, LOW);
  
  // Convertir a voltaje
  float Vdrop = (EC_VIN * lectura) / 1023.0;
  
  // Calcular resistencia
  float Rc = (Vdrop * (EC_R1 + EC_RA)) / (EC_VIN - Vdrop);
  Rc = Rc - EC_RA;
  
  // Calcular conductividad
  float EC = 10000.0 / (Rc * EC_K);
  
  // Compensar a 25°C
  float EC25 = EC / (1.0 + EC_TEMP_COEF * (temperatura_actual - 25.0));
  
  return EC25;
}
#endif

// ----------------------------------------------------------------------------

#if SENSOR_OD
/*
  Función: leerOxigenoDisuelto
  Descripción: Lee oxígeno disuelto con compensación de temperatura
  Retorna: Oxígeno disuelto en mg/L
*/
float leerOxigenoDisuelto() {
  // Leer voltaje del sensor
  uint16_t adc_raw = analogRead(DO_ANALOG_PIN);
  uint16_t adc_voltage = (uint32_t)DO_VREF * adc_raw / DO_ADC_RES;
  
  // Obtener temperatura actual (limitada a rango de tabla)
  uint8_t temp = (uint8_t)temperatura_actual;
  if (temp > 40) temp = 40;
  if (temp < 0) temp = 0;
  
  // Calcular voltaje de saturación según calibración
  uint16_t V_saturation;
  
  #if DO_TWO_POINT_CAL == 0
    // Calibración de un punto
    V_saturation = (uint32_t)DO_CAL1_V + (uint32_t)35 * temp - (uint32_t)DO_CAL1_T * 35;
  #else
    // Calibración de dos puntos
    V_saturation = (int16_t)((int8_t)temp - DO_CAL2_T) * 
                   ((uint16_t)DO_CAL1_V - DO_CAL2_V) / 
                   ((uint8_t)DO_CAL1_T - DO_CAL2_T) + DO_CAL2_V;
  #endif
  
  // Calcular DO en μg/L
  uint16_t DO_ug = (uint32_t)adc_voltage * DO_Table[temp] / V_saturation;
  
  // Convertir a mg/L
  float DO_mg = DO_ug / 1000.0;
  
  return DO_mg;
}
#endif

// ============================================================================
// FUNCIONES DE SISTEMA (BATERÍA, COBERTURA)
// ============================================================================

/*
  Función: leerBateria
  Descripción: Lee estado de batería del SIM800
  Actualiza: voltios, porcentaje, contador_bateria_baja
*/
void leerBateria() {
  SIM800.write("AT\r\n");
  delay(500);
  while (SIM800.available() > 0) SIM800.read();
  delay(200);
  
  SIM800.write("AT+CBC\r\n");
  delay(200);
  
  int size = SIM800.readBytes(buffer_sim, 39);
  buffer_sim[size] = 0;
  
  // Parsear respuesta: +CBC: cargando,porcentaje,voltaje
  char* token = strtok(buffer_sim, "CBC:");
  token = strtok(0, ",");  // Cargando (no usado)
  token = strtok(0, ",");
  porcentaje = atoi(token);
  token = strtok(0, ",");
  voltios = atoi(token);
  
  // Control de batería baja
  if (voltios > 3000 && voltios < 3300) {
    contador_bateria_baja++;
  } else if (voltios > 3300 && voltios < 4200) {
    contador_bateria_baja = 0;
  }
}

// ----------------------------------------------------------------------------

/*
  Función: leerCobertura
  Descripción: Lee calidad de señal GSM del SIM800
  Actualiza: cobertura (0-31, 99=sin señal)
*/
void leerCobertura() {
  SIM800.write("AT\r\n");
  delay(500);
  while (SIM800.available() > 0) SIM800.read();
  delay(200);
  
  SIM800.write("AT+CSQ\r\n");
  delay(200);
  
  int size = SIM800.readBytes(buffer_sim, 39);
  buffer_sim[size] = 0;
  
  // Parsear respuesta: +CSQ: señal,ber
  char* token = strtok(buffer_sim, ": ");
  token = strtok(0, ",");
  cobertura = atoi(token);
}

// ============================================================================
// FUNCIONES DE ADQUISICIÓN DE DATOS
// ============================================================================

/*
  Función: leerTodosSensores
  Descripción: Lee todos los sensores activos y guarda en estructura
  Retorna: Estructura con todos los datos
*/
DatosSensores leerTodosSensores() {
  DatosSensores datos;
  
  // Leer estado del sistema
  leerBateria();
  leerCobertura();
  
  // Leer sensores en orden (temperatura primero para compensación)
  #if SENSOR_TEMPERATURA
    temperatura_actual = leerTemperatura();
    datos.temperatura = temperatura_actual;
  #endif
  
  #if SENSOR_CONDUCTIVIDAD
    datos.conductividad = leerConductividad();
  #endif
  
  #if SENSOR_OD
    datos.oxigeno_disuelto = leerOxigenoDisuelto();
  #endif
  
  // Datos del sistema
  datos.bateria = porcentaje;
  datos.voltaje = voltios;
  datos.cobertura_gsm = cobertura;
  datos.timestamp = millis();
  
  return datos;
}

// ============================================================================
// FUNCIONES DE ALMACENAMIENTO EN SD
// ============================================================================

/*
  Función: generarLineaCSV
  Descripción: Genera línea CSV con todos los sensores activos
  Parámetros: datos - estructura con mediciones
              buffer - buffer donde escribir (mín 150 chars)
*/
void generarLineaCSV(DatosSensores& datos, char* buffer) {
  char temp[20];
  buffer[0] = '\0';
  
  // Timestamp
  ultoa(datos.timestamp, temp, 10);
  strcat(buffer, temp);
  strcat(buffer, ";");
  
  // Sensores activos
  #if SENSOR_TEMPERATURA
    dtostrf(datos.temperatura, 5, 2, temp);
    strcat(buffer, temp);
    strcat(buffer, ";");
  #endif
  
  #if SENSOR_CONDUCTIVIDAD
    dtostrf(datos.conductividad, 6, 1, temp);
    strcat(buffer, temp);
    strcat(buffer, ";");
  #endif
  
  #if SENSOR_OD
    dtostrf(datos.oxigeno_disuelto, 5, 2, temp);
    strcat(buffer, temp);
    strcat(buffer, ";");
  #endif
  
  // Sistema
  itoa(datos.bateria, temp, 10);
  strcat(buffer, temp);
  strcat(buffer, ";");
  
  itoa(datos.voltaje, temp, 10);
  strcat(buffer, temp);
  strcat(buffer, ";");
  
  itoa(datos.cobertura_gsm, temp, 10);
  strcat(buffer, temp);
}

// ----------------------------------------------------------------------------

/*
  Función: guardarEnSD
  Descripción: Guarda medición en archivo pendientes (append-only)
  Retorna: true si éxito, false si error
*/
bool guardarEnSD(DatosSensores& datos) {
  char linea[150];
  generarLineaCSV(datos, linea);
  
  File archivo = SD.open(ARCHIVO_PENDIENTES, FILE_WRITE);
  if (!archivo) {
    Serial.println(F("ERROR: No se pudo abrir pendient.csv"));
    return false;
  }
  
  archivo.println(linea);
  archivo.close();
  
  Serial.print(F("✓ Guardado en SD: "));
  Serial.println(linea);
  
  return true;
}

// ----------------------------------------------------------------------------

/*
  Función: escribirLog
  Descripción: Escribe mensaje en archivo de log
*/
void escribirLog(const char* mensaje) {
  File archivo = SD.open(ARCHIVO_LOG, FILE_WRITE);
  if (archivo) {
    archivo.print(millis());
    archivo.print(F(": "));
    archivo.println(mensaje);
    archivo.close();
  }
}

// ============================================================================
// FUNCIONES DE ENVÍO HTTP
// ============================================================================

/*
  Función: construirURL
  Descripción: Construye URL con datos de sensores para envío HTTP
  Parámetros: datos - estructura con mediciones
              buffer - buffer donde escribir URL (mín 200 chars)
*/
void construirURL(DatosSensores& datos, char* buffer) {
  char temp[20];
  
  strcpy(buffer, "inasas.cuartazona.es/caramba2-protoSIM800-");
  
  // Periodo
  itoa(periodo, temp, 10);
  strcat(buffer, temp);
  strcat(buffer, "-%7B");
  
  // Batería
  strcat(buffer, "%22Ba%22%3A%22");
  itoa(datos.bateria, temp, 10);
  strcat(buffer, temp);
  strcat(buffer, "%22%2C");
  
  // Cobertura
  strcat(buffer, "%22Cb%22%3A%22");
  itoa(datos.cobertura_gsm, temp, 10);
  strcat(buffer, temp);
  strcat(buffer, "%22%2C");
  
  // Voltaje
  strcat(buffer, "%22Vo%22%3A%22");
  itoa(datos.voltaje, temp, 10);
  strcat(buffer, temp);
  strcat(buffer, "%22%2C");
  
  // Temperatura
  #if SENSOR_TEMPERATURA
    strcat(buffer, "%22Te%22%3A%22");
    dtostrf(datos.temperatura, 5, 1, temp);
    strcat(buffer, temp);
    strcat(buffer, "%22%2C");
  #endif
  
  // Conductividad
  #if SENSOR_CONDUCTIVIDAD
    strcat(buffer, "%22Co%22%3A%22");
    dtostrf(datos.conductividad, 6, 1, temp);
    strcat(buffer, temp);
    strcat(buffer, "%22%2C");
  #endif
  
  // Oxígeno Disuelto
  #if SENSOR_OD
    strcat(buffer, "%22OD%22%3A%22");
    dtostrf(datos.oxigeno_disuelto, 5, 2, temp);
    strcat(buffer, temp);
    strcat(buffer, "%22%2C");
  #endif
  
  // Cerrar JSON (eliminar última coma)
  int len = strlen(buffer);
  if (buffer[len-3] == '%' && buffer[len-2] == '2' && buffer[len-1] == 'C') {
    buffer[len-3] = '\0';
  }
  strcat(buffer, "%7D");
}

// ----------------------------------------------------------------------------

/*
  Función: enviarHTTP
  Descripción: Envía datos por HTTP usando SIM800
  Retorna: true si envío exitoso, false si error
*/
bool enviarHTTP(DatosSensores& datos) {
  char url[250];
  construirURL(datos, url);
  
  Serial.println(F("Enviando por HTTP..."));
  
  // Despertar SIM800
  SIM800.write("AT\r\n");
  delay(500);
  
  // Configurar GPRS
  SIM800.write("AT+CGATT=1\r\n");
  delay(500);
  
  SIM800.write("AT+SAPBR=3,1,\"APN\",\"" APN_OPERADOR "\"\r\n");
  delay(500);
  
  SIM800.write("AT+SAPBR=1,1\r\n");
  delay(1500);
  
  // Iniciar HTTP
  SIM800.write("AT+HTTPINIT\r\n");
  delay(500);
  
  // Limpiar buffer
  while (SIM800.available() > 0) SIM800.read();
  
  // Configurar URL
  SIM800.write("AT+HTTPPARA=\"URL\",\"");
  SIM800.print(url);
  SIM800.write("\"\r\n");
  delay(500);
  
  // Ejecutar petición GET
  SIM800.write("AT+HTTPACTION=0\r\n");
  delay(4000);
  
  // Leer respuesta
  SIM800.write("AT\r\n");
  delay(500);
  while (SIM800.available() > 0) SIM800.read();
  
  SIM800.write("AT+HTTPREAD\r\n");
  delay(200);
  
  int size = SIM800.readBytes(buffer_http, 70);
  buffer_http[size] = 0;
  
  // Terminar HTTP
  SIM800.write("AT+HTTPTERM\r\n");
  delay(500);
  
  // Verificar respuesta (buscar "200" o "OK")
  bool exito = (strstr(buffer_http, "200") != NULL || strstr(buffer_http, "OK") != NULL);
  
  if (exito) {
    Serial.println(F("✓ Envío HTTP exitoso"));
  } else {
    Serial.println(F("✗ Envío HTTP falló"));
    escribirLog("Error envio HTTP");
  }
  
  return exito;
}

// ============================================================================
// FUNCIONES DE GESTIÓN DE ARCHIVOS SD
// ============================================================================

/*
  Función: contarLineas
  Descripción: Cuenta líneas de un archivo
  Retorna: Número de líneas
*/
int contarLineas(const char* nombreArchivo) {
  File archivo = SD.open(nombreArchivo, FILE_READ);
  if (!archivo) return 0;
  
  int lineas = 0;
  while (archivo.available()) {
    if (archivo.read() == '\n') lineas++;
  }
  archivo.close();
  
  return lineas;
}

// ----------------------------------------------------------------------------

/*
  Función: leerPrimeraLinea
  Descripción: Lee la primera línea de un archivo
  Parámetros: nombreArchivo - nombre del archivo
              buffer - buffer donde escribir (mín 150 chars)
  Retorna: true si éxito, false si error o archivo vacío
*/
bool leerPrimeraLinea(const char* nombreArchivo, char* buffer) {
  File archivo = SD.open(nombreArchivo, FILE_READ);
  if (!archivo) return false;
  
  int i = 0;
  while (archivo.available() && i < 149) {
    char c = archivo.read();
    if (c == '\n' || c == '\r') break;
    buffer[i++] = c;
  }
  buffer[i] = '\0';
  archivo.close();
  
  return (i > 0);
}

// ----------------------------------------------------------------------------

/*
  Función: moverPrimeraLineaAEnviados
  Descripción: Mueve primera línea de pendientes a enviados
  Retorna: true si éxito, false si error
*/
bool moverPrimeraLineaAEnviados() {
  char linea[150];
  
  // Leer primera línea de pendientes
  if (!leerPrimeraLinea(ARCHIVO_PENDIENTES, linea)) {
    return false;
  }
  
  // Añadir a enviados
  File enviados = SD.open(ARCHIVO_ENVIADOS, FILE_WRITE);
  if (!enviados) return false;
  enviados.println(linea);
  enviados.close();
  
  // Eliminar primera línea de pendientes (copiar resto a temporal)
  File origen = SD.open(ARCHIVO_PENDIENTES, FILE_READ);
  File temp = SD.open("temp.tmp", FILE_WRITE);
  
  if (!origen || !temp) {
    if (origen) origen.close();
    if (temp) temp.close();
    return false;
  }
  
  // Saltar primera línea
  while (origen.available()) {
    char c = origen.read();
    if (c == '\n') break;
  }
  
  // Copiar resto
  while (origen.available()) {
    temp.write(origen.read());
  }
  
  origen.close();
  temp.close();
  
  // Reemplazar archivo
  SD.remove(ARCHIVO_PENDIENTES);
  SD.rename("temp.tmp", ARCHIVO_PENDIENTES);
  
  return true;
}

// ----------------------------------------------------------------------------

/*
  Función: reintentarPendientes
  Descripción: Intenta reenviar datos pendientes (máx MAX_REINTENTOS)
*/
void reintentarPendientes() {
  int pendientes = contarLineas(ARCHIVO_PENDIENTES);
  
  if (pendientes == 0) {
    Serial.println(F("No hay datos pendientes"));
    return;
  }
  
  Serial.print(F("Datos pendientes: "));
  Serial.println(pendientes);
  
  int reintentos = 0;
  char linea[150];
  
  while (reintentos < MAX_REINTENTOS && leerPrimeraLinea(ARCHIVO_PENDIENTES, linea)) {
    Serial.print(F("Reintento "));
    Serial.print(reintentos + 1);
    Serial.print(F("/"));
    Serial.println(MAX_REINTENTOS);
    
    // Parsear línea CSV a estructura
    DatosSensores datos;
    char* token = strtok(linea, ";");
    datos.timestamp = atol(token);
    
    #if SENSOR_TEMPERATURA
      token = strtok(NULL, ";");
      datos.temperatura = atof(token);
    #endif
    
    #if SENSOR_CONDUCTIVIDAD
      token = strtok(NULL, ";");
      datos.conductividad = atof(token);
    #endif
    
    #if SENSOR_OD
      token = strtok(NULL, ";");
      datos.oxigeno_disuelto = atof(token);
    #endif
    
    token = strtok(NULL, ";");
    datos.bateria = atoi(token);
    token = strtok(NULL, ";");
    datos.voltaje = atoi(token);
    token = strtok(NULL, ";");
    datos.cobertura_gsm = atoi(token);
    
    // Intentar enviar
    if (enviarHTTP(datos)) {
      moverPrimeraLineaAEnviados();
      Serial.println(F("✓ Dato pendiente enviado"));
    } else {
      Serial.println(F("✗ Fallo reintento, se intentará después"));
      break;  // No seguir intentando si falla
    }
    
    reintentos++;
    delay(1000);
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  Serial.println(F("========================================"));
  Serial.println(F("  SISTEMA MODULAR DE MONITOREO v3.0"));
  Serial.println(F("========================================"));
  
  // Mostrar sensores activos
  Serial.println(F("\nSensores activos:"));
  #if SENSOR_TEMPERATURA
    Serial.println(F("  ✓ Temperatura (NTC)"));
  #endif
  #if SENSOR_CONDUCTIVIDAD
    Serial.println(F("  ✓ Conductividad"));
  #endif
  #if SENSOR_OD
    Serial.println(F("  ✓ Oxígeno Disuelto"));
  #endif
  
  // Inicializar SD
  Serial.print(F("\nInicializando SD... "));
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("FALLO"));
    escribirLog("ERROR: SD no inicializada");
  } else {
    Serial.println(F("OK"));
    escribirLog("Sistema iniciado");
  }
  
  // Configurar pines de sensores
  #if SENSOR_TEMPERATURA
    pinMode(NTC_ANALOG_PIN, INPUT);
    pinMode(NTC_VCC_PIN, OUTPUT);
    pinMode(NTC_GND_PIN, OUTPUT);
    digitalWrite(NTC_GND_PIN, LOW);
  #endif
  
  #if SENSOR_CONDUCTIVIDAD
    pinMode(EC_ANALOG_PIN, INPUT);
    pinMode(EC_POWER_PIN, OUTPUT);
    pinMode(EC_GROUND_PIN, OUTPUT);
    digitalWrite(EC_GROUND_PIN, LOW);
  #endif
  
  #if SENSOR_OD
    pinMode(DO_ANALOG_PIN, INPUT);
  #endif
  
  // Inicializar SIM800
  Serial.print(F("Inicializando SIM800... "));
  SIM800.begin(9600);
  SIM800.write("AT\r\n");
  delay(1000);
  SIM800.write("AT\r\n");
  delay(1000);
  SIM800.write("AT+CSCLK=2\r\n");  // Sleep mode
  Serial.println(F("OK"));
  
  Serial.println(F("\n✓ Setup completado\n"));
  delay(2000);
}

// ============================================================================
// LOOP PRINCIPAL
// ============================================================================

void loop() {
  Serial.println(F("\n========== NUEVO CICLO =========="));
  
  // Verificar batería crítica
  Serial.print(F("Contador batería baja: "));
  Serial.println(contador_bateria_baja);
  
  if (contador_bateria_baja > UMBRAL_BATERIA_BAJA) {
    Serial.println(F("⚠ BATERÍA CRÍTICA - APAGANDO"));
    escribirLog("SHUTDOWN: Bateria critica");
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  }
  
  // Reintentar envío de datos pendientes
  reintentarPendientes();
  
  // Tomar mediciones
  Serial.println(F("\nTomando mediciones..."));
  DatosSensores datos = leerTodosSensores();
  
  // Mostrar datos
  Serial.println(F("\n--- Datos leídos ---"));
  #if SENSOR_TEMPERATURA
    Serial.print(F("Temperatura: "));
    Serial.print(datos.temperatura);
    Serial.println(F(" °C"));
  #endif
  
  #if SENSOR_CONDUCTIVIDAD
    Serial.print(F("Conductividad: "));
    Serial.print(datos.conductividad);
    Serial.println(F(" μS/cm"));
  #endif
  
  #if SENSOR_OD
    Serial.print(F("Oxígeno Disuelto: "));
    Serial.print(datos.oxigeno_disuelto);
    Serial.println(F(" mg/L"));
  #endif
  
  Serial.print(F("Batería: "));
  Serial.print(datos.bateria);
  Serial.print(F("% ("));
  Serial.print(datos.voltaje);
  Serial.println(F(" mV)"));
  
  Serial.print(F("Cobertura GSM: "));
  Serial.println(datos.cobertura_gsm);
  
  // PASO 1: Guardar en SD (PRIORITARIO)
  Serial.println(F("\n[1/2] Guardando en SD..."));
  if (!guardarEnSD(datos)) {
    escribirLog("ERROR: Fallo guardar SD");
  }
  
  // PASO 2: Intentar enviar por HTTP
  Serial.println(F("[2/2] Enviando por HTTP..."));
  if (enviarHTTP(datos)) {
    // Si se envió correctamente, mover de pendientes a enviados
    moverPrimeraLineaAEnviados();
  } else {
    Serial.println(F("⚠ Dato quedó pendiente para próximo ciclo"));
  }
  
  // Dormir SIM800
  SIM800.write("AT\r\n");
  delay(500);
  SIM800.write("AT+CSCLK=2\r\n");
  
  // Dormir Arduino
  Serial.print(F("\nDurmiendo "));
  Serial.print(periodo);
  Serial.println(F(" segundos...\n"));
  
  for (int i = 0; i < periodo / 8; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

// ============================================================================
// FIN DEL CÓDIGO
// ============================================================================