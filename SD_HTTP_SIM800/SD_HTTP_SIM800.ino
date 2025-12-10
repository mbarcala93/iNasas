/*
 * SISTEMA DE MONITOREO CON RESPALDO EN SD
 * ========================================
 * Este código mide temperatura (NTC) y conductividad del agua.
 * Los datos se guardan PRIMERO en tarjeta SD y DESPUÉS se envían por HTTP.
 * Si falla el envío, los datos quedan guardados para reintento posterior.
 * 
 * MEJORAS IMPLEMENTADAS:
 * - Guardado prioritario en SD antes de enviar
 * - Sistema de cola de datos pendientes
 * - Reintentos automáticos de envío
 * - Marcado de datos enviados exitosamente
 * - Logs de errores y estado
 */

#include <SoftwareSerial.h>
#include "LowPower.h"
#include <SPI.h>    // incluye libreria interfaz SPI
#include <SD.h>     // incluye libreria para tarjetas SD

//variables de modulo tarjeta SD
#define SSpin 10    // CS (Slave Select) en pin digital 10
    // MOSI en pin digital 11
    // MISO en pin digital 12
    // SCK en pin digital 13
    
File archivo;    // objeto archivo del tipo File de la librería SD.h

// VARIABLES PARA SISTEMA DE RESPALDO
bool sdDisponible = false;           // Flag para saber si la SD está funcionando
unsigned long contadorMediciones = 0; // Contador único para cada medición
#define MAX_REINTENTOS 3             // Número máximo de reintentos de envío por ciclo

SoftwareSerial SIM800(0,1); // RX | TX
// Connect the SIM800 TX to Arduino pin 0 RX.
// Connect the SIM800 RX to Arduino pin 1 TX.
char c = ' ';

//variables de termistor NTC 10K
int ThermistorPin = 0; // esto es 0 para las placas de JLCPCB (error soldando en RAFAELA, que es 1) ojoooo
int vccNTCPin = 8;
int groundNTCPin = 9;
int Vo;
float Rt1 = 10000;
float logR2, R2, T;
float c1 = 0.00112531; //these values hold for 10K NTC thermistors
float c2 = 0.000234712;
float c3 = 0.0000856635;

//variables de sonda de conductividad
float R1= 1000.0; //resistencia pull up
float Ra=25.0; //Resistance of powering Pins
float K=1.07; //cte de celda

int ECPin= A1; //pin de lectura, esto es A1 para las placas de JLCPCB(error soldando en RAFAELA, que es A0) ojoooo
int ECGround = 4; //reducidos en 1 en protoCaixa y thor porque me equivoqué al soldar
int ECPower = 5; //4 y 5 son los hechos tomando de partida protoCuarentena
float TemperatureCoef = 0.0188; // cte compensacion temperatura

// iniciamos variables para conductividad
float Tc=10;
float EC=0;
float EC25 =0; 
float raw= 0;
float Vin= 3.3;
float Vdrop= 0;
float Rc= 0;

//establecemos el periodo de toma de datos (en segundos) y el num de datos acumulados
long periodo = 10;
int acumula = 3;
char prueba2[70];

char prueba4[40];
int contador_bateria_baja;

int voltios;
int cargando;
int porcentaje;
int cobertura;

// Si se utiliza vscode hace falta llamar antes las funciones de que aparezcan los loop
float GetTemp ();
float GetEC();
void GetBateria ();
void GetCobertura ();

/*
 * FUNCIÓN: guardarDatosSD
 * =======================
 * Guarda los datos de medición en la tarjeta SD en formato CSV.
 * Cada línea contiene: ID, Timestamp, Temperatura, Conductividad, Batería, Voltaje, Cobertura, Estado
 * Estado: PENDIENTE (no enviado) o ENVIADO (confirmado en servidor)
 * 
 * Parámetros:
 *   - id: Identificador único de la medición
 *   - tempNTC: Temperatura medida
 *   - conductividad: Conductividad medida
 *   - bateria: Porcentaje de batería
 *   - volt: Voltaje en mV
 *   - cobert: Nivel de cobertura GSM
 * 
 * Retorna: true si se guardó correctamente, false si hubo error
 */
bool guardarDatosSD(unsigned long id, String tempNTC, String conductividad, String bateria, String volt, String cobert) {
  if (!sdDisponible) {
    Serial.println("ERROR: SD no disponible");
    return false;
  }
  
  // Abre el archivo de datos en modo escritura (append)
  archivo = SD.open("datos.csv", FILE_WRITE);
  
  if (archivo) {
    // Formato: ID;Timestamp;Temp;Cond;Bat;Volt;Cob;Estado
    archivo.print(id);
    archivo.print(";");
    archivo.print(millis()); // Timestamp en milisegundos desde inicio
    archivo.print(";");
    archivo.print(tempNTC);
    archivo.print(";");
    archivo.print(conductividad);
    archivo.print(";");
    archivo.print(bateria);
    archivo.print(";");
    archivo.print(volt);
    archivo.print(";");
    archivo.print(cobert);
    archivo.print(";");
    archivo.println("PENDIENTE"); // Marca como pendiente de envío
    
    archivo.close();
    Serial.print("✓ Datos guardados en SD con ID: ");
    Serial.println(id);
    return true;
  } else {
    Serial.println("ERROR: No se pudo abrir datos.csv");
    return false;
  }
}

/*
 * FUNCIÓN: marcarComoEnviado
 * ===========================
 * Marca una medición específica como enviada exitosamente al servidor.
 * Lee el archivo, busca la línea con el ID correspondiente y cambia su estado.
 * 
 * Parámetros:
 *   - id: Identificador de la medición a marcar
 * 
 * Retorna: true si se marcó correctamente, false si hubo error
 */
bool marcarComoEnviado(unsigned long id) {
  if (!sdDisponible) return false;
  
  // Crea un archivo temporal para la actualización
  if (SD.exists("temp.csv")) {
    SD.remove("temp.csv");
  }
  
  File archivoOriginal = SD.open("datos.csv", FILE_READ);
  File archivoTemp = SD.open("temp.csv", FILE_WRITE);
  
  if (!archivoOriginal || !archivoTemp) {
    Serial.println("ERROR: No se pudo actualizar estado");
    if (archivoOriginal) archivoOriginal.close();
    if (archivoTemp) archivoTemp.close();
    return false;
  }
  
  // Lee línea por línea y actualiza la que corresponda
  String linea = "";
  while (archivoOriginal.available()) {
    char c = archivoOriginal.read();
    if (c == '\n') {
      // Procesa la línea completa
      if (linea.startsWith(String(id) + ";")) {
        // Esta es la línea a actualizar
        int ultimoPuntoYComa = linea.lastIndexOf(';');
        linea = linea.substring(0, ultimoPuntoYComa + 1) + "ENVIADO";
        Serial.print("✓ Marcado como ENVIADO: ID ");
        Serial.println(id);
      }
      archivoTemp.println(linea);
      linea = "";
    } else {
      linea += c;
    }
  }
  
  // Escribe la última línea si no termina en \n
  if (linea.length() > 0) {
    if (linea.startsWith(String(id) + ";")) {
      int ultimoPuntoYComa = linea.lastIndexOf(';');
      linea = linea.substring(0, ultimoPuntoYComa + 1) + "ENVIADO";
    }
    archivoTemp.println(linea);
  }
  
  archivoOriginal.close();
  archivoTemp.close();
  
  // Reemplaza el archivo original con el temporal
  SD.remove("datos.csv");
  // Nota: SD.h no tiene rename(), hay que copiar manualmente
  File origen = SD.open("temp.csv", FILE_READ);
  File destino = SD.open("datos.csv", FILE_WRITE);
  
  if (origen && destino) {
    while (origen.available()) {
      destino.write(origen.read());
    }
    origen.close();
    destino.close();
    SD.remove("temp.csv");
    return true;
  }
  
  return false;
}

/*
 * FUNCIÓN: reintentarEnviosPendientes
 * ====================================
 * Busca en la SD los datos marcados como PENDIENTE e intenta enviarlos.
 * Procesa un máximo de registros por llamada para no bloquear el sistema.
 * 
 * Retorna: número de registros enviados exitosamente
 */
int reintentarEnviosPendientes() {
  if (!sdDisponible) return 0;
  
  Serial.println("--- Buscando datos pendientes de envío ---");
  
  File archivoLectura = SD.open("datos.csv", FILE_READ);
  if (!archivoLectura) {
    Serial.println("No hay archivo de datos");
    return 0;
  }
  
  int enviados = 0;
  int intentos = 0;
  String linea = "";
  
  // Lee el archivo línea por línea
  while (archivoLectura.available() && intentos < MAX_REINTENTOS) {
    char c = archivoLectura.read();
    if (c == '\n') {
      // Procesa la línea si está PENDIENTE
      if (linea.indexOf("PENDIENTE") > 0) {
        intentos++;
        Serial.print("Reintentando envío: ");
        Serial.println(linea);
        
        // Parsea los datos de la línea
        // Formato: ID;Timestamp;Temp;Cond;Bat;Volt;Cob;Estado
        int pos = 0;
        int indices[8];
        for (int i = 0; i < 8; i++) {
          indices[i] = linea.indexOf(';', pos);
          pos = indices[i] + 1;
        }
        
        unsigned long id = linea.substring(0, indices[0]).toInt();
        String temp = linea.substring(indices[1] + 1, indices[2]);
        String cond = linea.substring(indices[2] + 1, indices[3]);
        String bat = linea.substring(indices[3] + 1, indices[4]);
        String volt = linea.substring(indices[4] + 1, indices[5]);
        String cob = linea.substring(indices[5] + 1, indices[6]);
        
        // Intenta enviar
        char* respuesta = enviahttp(periodo/8*8, temp, cond, bat, volt, cob);
        
        // Verifica si el envío fue exitoso (busca "OK" o código 200 en respuesta)
        if (respuesta != NULL && (strstr(respuesta, "200") != NULL || strstr(respuesta, "OK") != NULL)) {
          marcarComoEnviado(id);
          enviados++;
          Serial.println("✓ Reenvío exitoso");
        } else {
          Serial.println("✗ Reenvío fallido");
        }
      }
      linea = "";
    } else {
      linea += c;
    }
  }
  
  archivoLectura.close();
  
  Serial.print("Datos reenviados: ");
  Serial.println(enviados);
  return enviados;
}

/*
 * FUNCIÓN: enviahttp
 * ==================
 * Envía los datos al servidor mediante HTTP usando el módulo SIM800.
 * Esta función NO guarda en SD, solo envía.
 * 
 * Retorna: Respuesta del servidor (char*)
 */
char* enviahttp(int periodo, String tempNTC, String conductividad, String bateria, String volt, String cobert) {
  Serial.print("\n--- ENVIANDO HTTP ---\n");
  Serial.print("Periodo: "); Serial.println(periodo);
  Serial.print("Bateria: "); Serial.println(bateria);
  Serial.print("Voltaje: "); Serial.println(volt);
  Serial.print("Temp: "); Serial.println(tempNTC);
  Serial.print("Cond: "); Serial.println(conductividad);
  Serial.print("Cobert: "); Serial.println(cobert);

  //despertamos el modulo SIM800 mini
  SIM800.write("AT\r\n");
  delay(500);
  SIM800.write("AT+CGATT=1\r\n");
  delay(500);
  SIM800.write("AT+SAPBR=3,1,\"APN\",\"data.rewicom.net\"\r\n");
  delay(500);
  SIM800.write("AT+SAPBR=1,1\r\n");
  delay(1500);
  SIM800.write("AT+HTTPINIT\r\n");
  delay(500);
  while ( SIM800.available() > 0) SIM800.read(); 
  SIM800.write("AT+HTTPPARA=\"URL\",\"inasas.cuartazona.es/caramba3|protoSIM800|");
  SIM800.print(periodo);
  SIM800.print("|%7B%22Ba%22%3A%22");
  SIM800.print(bateria);
  SIM800.print("%22%2C%22Cb%22%3A%22");
  SIM800.print(cobert);
  SIM800.print("%22%2C%22Vo%22%3A%22");
  SIM800.print(volt);
  SIM800.print("%22%2C%22Te%22%3A%22");
  SIM800.print(tempNTC);
  SIM800.print("%22%2C%22Co%22%3A%22");
  SIM800.print(conductividad);
  SIM800.print("%22%7D");
  SIM800.write("\"\r\n");

  Serial.print("URL: inasas.cuartazona.es/caramba3|protoSIM800|");
  Serial.print(periodo);
  Serial.print("|%7B%22Ba%22%3A%22");
  Serial.print(bateria);
  Serial.print("%22%2C%22Cb%22%3A%22");
  Serial.print(cobert);
  Serial.print("%22%2C%22Vo%22%3A%22");
  Serial.print(volt);
  Serial.print("%22%2C%22Te%22%3A%22");
  Serial.print(tempNTC);
  Serial.print("%22%2C%22Co%22%3A%22");
  Serial.print(conductividad);
  Serial.print("%22%7D\n");
  
  delay(500);
  SIM800.write("AT+HTTPACTION=0\r\n");
  delay(4000);
  
  SIM800.write("AT\r\n");
  delay(500);
  while ( SIM800.available() > 0) SIM800.read(); 
  SIM800.write("AT+HTTPREAD\r\n");  
  delay(200);

  char* prueba = prueba2;
  int size = SIM800.readBytes(prueba, 70);
  prueba[size] = 0;
    
  SIM800.write("AT+HTTPTERM\r\n");
  delay(500);
  Serial.print("Respuesta servidor: ");
  Serial.println(prueba);
  return prueba;
}

void setup() {
  Serial.begin(9600); 
  Serial.println("=== INICIANDO SISTEMA DE MONITOREO ===");
  
  // Inicialización de tarjeta SD con verificación
  Serial.print("Inicializando SD... ");
  if (!SD.begin(SSpin)) {
    Serial.println("✗ FALLO");
    Serial.println("ADVERTENCIA: Sistema funcionará sin respaldo local");
    sdDisponible = false;
  } else {
    Serial.println("✓ OK");
    sdDisponible = true;
    
    // Crea encabezado del archivo CSV si no existe
    if (!SD.exists("datos.csv")) {
      archivo = SD.open("datos.csv", FILE_WRITE);
      if (archivo) {
        archivo.println("ID;Timestamp;Temperatura;Conductividad;Bateria;Voltaje;Cobertura;Estado");
        archivo.close();
        Serial.println("✓ Archivo datos.csv creado");
      }
    }
    
    // Crea archivo de log si no existe
    if (!SD.exists("log.txt")) {
      archivo = SD.open("log.txt", FILE_WRITE);
      if (archivo) {
        archivo.println("=== LOG DEL SISTEMA ===");
        archivo.close();
      }
    }
  }
  
  // Configuración de pines
  pinMode(ECPin,INPUT);
  pinMode(ECPower,OUTPUT);
  pinMode(ECGround,OUTPUT);
  pinMode(vccNTCPin, OUTPUT);
  pinMode(groundNTCPin, OUTPUT);
  digitalWrite(ECGround,LOW);
  digitalWrite(groundNTCPin, LOW);
 
  delay(100);
  R1=(R1+Ra);

  // Inicialización del módulo SIM800
  Serial.println("Inicializando SIM800...");
  SIM800.begin(9600);
  SIM800.write("AT\r\n");
  delay(1000);
  SIM800.write("AT\r\n");
  delay(1000);
  SIM800.write("AT+CSCLK=2\r\n"); // Activa sleep mode
  
  Serial.println("=== SETUP COMPLETADO ===\n");  
}

void loop() {
  Serial.println("\n========== NUEVO CICLO ==========");
  Serial.print("Contador bateria baja: ");
  Serial.println(contador_bateria_baja);
  
  // Protección: Si batería muy baja, entra en sleep permanente
  if (contador_bateria_baja > 25) {
    Serial.println("BATERIA CRITICA - Entrando en sleep permanente");
    if (sdDisponible) {
      archivo = SD.open("log.txt", FILE_WRITE);
      if (archivo) {
        archivo.println("SHUTDOWN: Bateria critica");
        archivo.close();
      }
    }
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
  }

  // PASO 1: Intentar reenviar datos pendientes de ciclos anteriores
  Serial.println("\n[1/4] Verificando datos pendientes...");
  reintentarEnviosPendientes();

  // PASO 2: Obtener estado del sistema
  Serial.println("\n[2/4] Leyendo estado del sistema...");
  GetBateria();
  GetCobertura();
  
  // PASO 3: Tomar mediciones y acumular
  Serial.println("\n[3/4] Tomando mediciones...");
  Tc = GetTemp();
  String temper=String(Tc,1);
  String conduct=String(GetEC(),1);
  String bateria=String(porcentaje);
  String volt=String(voltios);
  String cobert=String(cobertura);
  
  Serial.print("Medición 1: Temp=");
  Serial.print(Tc);
  Serial.print("°C, Cond=");
  Serial.print(GetEC());
  Serial.println(" µS/cm");
  
  // Acumula mediciones adicionales
  for (int i=0; i<=acumula-2; i++) {
    GetBateria();
    GetCobertura();
    Tc = GetTemp();
    temper = temper + "|" + String(Tc,1);
    conduct = conduct + "|" + String(GetEC(),1);
    bateria = bateria + "|" + String(porcentaje);
    volt = volt + "|" + String(voltios);
    cobert = cobert + "|" + String(cobertura);
    
    Serial.print("Medición ");
    Serial.print(i+2);
    Serial.print(": Temp=");
    Serial.print(Tc);
    Serial.print("°C, Cond=");
    Serial.print(GetEC());
    Serial.println(" µS/cm");
    
    // Sleep entre mediciones
    for (int j=0;j<=periodo/8;j++){
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }
    Serial.println("Despierto para siguiente medición");
  }

  // PASO 4: GUARDAR EN SD PRIMERO (PRIORITARIO)
  Serial.println("\n[4/4] Guardando y enviando datos...");
  contadorMediciones++; // Incrementa ID único
  
  bool guardadoOK = guardarDatosSD(contadorMediciones, temper, conduct, bateria, volt, cobert);
  
  if (!guardadoOK) {
    Serial.println("ADVERTENCIA: Datos no guardados en SD");
    if (sdDisponible) {
      // Intenta escribir en log de errores
      archivo = SD.open("log.txt", FILE_WRITE);
      if (archivo) {
        archivo.print("ERROR guardado ID: ");
        archivo.println(contadorMediciones);
        archivo.close();
      }
    }
  }

  // PASO 5: Intentar envío HTTP
  Serial.println("Intentando envío HTTP...");
  char* respuesta = enviahttp(periodo/8*8, temper, conduct, bateria, volt, cobert);

  // PASO 6: Verificar si el envío fue exitoso
  bool envioExitoso = false;
  if (respuesta != NULL && (strstr(respuesta, "200") != NULL || strstr(respuesta, "OK") != NULL)) {
    Serial.println("✓ ENVIO HTTP EXITOSO");
    envioExitoso = true;
    
    // Marca como enviado en la SD
    if (guardadoOK) {
      marcarComoEnviado(contadorMediciones);
    }
  } else {
    Serial.println("✗ ENVIO HTTP FALLIDO");
    Serial.println("Los datos quedan guardados en SD para reintento");
    
    // Log del fallo
    if (sdDisponible) {
      archivo = SD.open("log.txt", FILE_WRITE);
      if (archivo) {
        archivo.print("FALLO envio ID: ");
        archivo.print(contadorMediciones);
        archivo.print(" - Respuesta: ");
        archivo.println(respuesta);
        archivo.close();
      }
    }
  }

  // Parsea respuesta del servidor para actualizar parámetros
  char* valor = strtok(respuesta, "|");
  valor = strtok(0,"|");
  if (valor != NULL) {
    periodo = atoi(valor);
    Serial.print("Periodo actualizado: ");
    Serial.println(periodo);
  }
  
  valor = strtok(0,"|");
  if (valor != NULL) {
    acumula = atoi(valor);
    Serial.print("Acumulaciones actualizadas: ");
    Serial.println(acumula);
  }
  
  // Valores por defecto si no se recibieron
  if (periodo == 0 || acumula == 0) {
    periodo = 60;
    acumula = 1;
    Serial.println("Usando valores por defecto");
  }
  
  // Vuelve a poner SIM800 en sleep mode
  SIM800.write("AT\r\n");
  delay(500);
  SIM800.write("AT+CSCLK=2\r\n");
  
  Serial.println("========== FIN CICLO ==========\n");
} 

/*
 * FUNCIÓN: GetTemp
 * ================
 * Lee la temperatura del termistor NTC usando la ecuación de Steinhart-Hart.
 * 
 * Retorna: Temperatura en grados Celsius
 */
float GetTemp(){
  digitalWrite(vccNTCPin, HIGH);
  delay(200);
  Vo = analogRead(ThermistorPin);
  R2 = Rt1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
  digitalWrite(vccNTCPin, LOW);
  return Tc;
}

/*
 * FUNCIÓN: GetEC
 * ==============
 * Mide la conductividad eléctrica del agua y la compensa a 25°C.
 * 
 * Retorna: Conductividad en µS/cm a 25°C
 */
float GetEC(){  
  // Activa pin 5V, lee voltaje, apaga pin 5V
  digitalWrite(ECPower,HIGH);
  raw= analogRead(ECPin);
  digitalWrite(ECPower,LOW);
  
  // Convierte voltaje a resistencia teniendo en cuenta pullup R1
  Vdrop= (Vin*raw)/1023.0;
  Rc=(Vdrop*R1)/(Vin-Vdrop);
  Rc=Rc-Ra; // Compensación por resistencia del pin digital
  EC = 10000/(Rc*K);
  
  // Convierte conductividad a 25°C
  EC25  =  EC/ (1+ TemperatureCoef*(Tc-25.0));
  return EC25;
}

/*
 * FUNCIÓN: GetBateria
 * ===================
 * Consulta al SIM800 el estado de la batería mediante comando AT+CBC.
 * Actualiza variables globales: cargando, porcentaje, voltios.
 * Incrementa contador si batería está baja.
 */
void GetBateria() {
  SIM800.write("AT\r\n");
  delay(500);
  while ( SIM800.available() > 0) SIM800.read(); 
  delay(200);
  SIM800.write("AT+CBC\r\n");  
  delay(200);
  
  char* prueba3 = prueba4;
  int size2 = SIM800.readBytes(prueba3, 39);
  prueba3[size2] = 0;

  char* valore = strtok(prueba3, "CBC:");
  
  valore = strtok(0,",");
  cargando = atoi(valore);

  valore = strtok(0,",");
  porcentaje = atoi(valore);
  
  valore = strtok(0,",");
  voltios = atoi(valore);

  // Control de batería baja
  if (voltios > 3000 && voltios < 3300) {
    contador_bateria_baja = contador_bateria_baja +1;
  } else if (voltios > 3300 && voltios < 4200) {
    contador_bateria_baja = 0;
  }
}

/*
 * FUNCIÓN: GetCobertura
 * =====================
 * Consulta al SIM800 la calidad de señal GSM mediante comando AT+CSQ.
 * Actualiza variable global: cobertura (0-31, 99=desconocido).
 */
void GetCobertura() {
  SIM800.write("AT\r\n");
  delay(500);
  while ( SIM800.available() > 0) SIM800.read(); 
  delay(200);
  SIM800.write("AT+CSQ\r\n");  
  delay(200);
  
  char* prueba3 = prueba4;
  int size2 = SIM800.readBytes(prueba3, 39);
  prueba3[size2] = 0;

  char* valore = strtok(prueba3, ": ");
  
  valore = strtok(0,",");
  cobertura = atoi(valore);
}
