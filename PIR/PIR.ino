// ============================================
// SENSOR DE MOVIMIENTO INFRARROJO HC-SR501
// Sensor PIR (Passive Infrared) para detección de presencia
// ============================================

#define PIN_PIR_SENSOR 9           // Pin digital para lectura del sensor
#define TIEMPO_CALIBRACION 60000   // 60 segundos de calibración inicial

class SensorMovimiento {
private:
    bool activo;
    bool movimiento_detectado;
    bool calibrado;
    unsigned long tiempo_inicio_calibracion;
    unsigned long tiempo_ultima_deteccion;
    unsigned long contador_detecciones;
    
public:
    SensorMovimiento() : activo(false), movimiento_detectado(false), 
                         calibrado(false), tiempo_inicio_calibracion(0),
                         tiempo_ultima_deteccion(0), contador_detecciones(0) {}
    
    void inicializar() {
        pinMode(PIN_PIR_SENSOR, INPUT);
        tiempo_inicio_calibracion = millis();
        activo = true;
        
        // Nota: El HC-SR501 requiere 60 segundos de calibración
        // durante los cuales puede dar lecturas erráticas
    }
    
    bool estaActivo() {
        return activo;
    }
    
    const char* getNombre() {
        return "Movimiento_Detecciones_UltimaDeteccion";
    }
    
    float leer() {
        if (!activo) return -999;
        
        // Verificar si ya pasó el tiempo de calibración
        if (!calibrado) {
            if (millis() - tiempo_inicio_calibracion >= TIEMPO_CALIBRACION) {
                calibrado = true;
            } else {
                // Aún en calibración
                return 0;
            }
        }
        
        // Leer estado del sensor
        int estado = digitalRead(PIN_PIR_SENSOR);
        
        if (estado == HIGH) {
            // Movimiento detectado
            if (!movimiento_detectado) {
                // Nueva detección
                movimiento_detectado = true;
                tiempo_ultima_deteccion = millis();
                contador_detecciones++;
            }
            return 1.0; // Movimiento presente
        } else {
            // No hay movimiento
            movimiento_detectado = false;
            return 0.0; // Sin movimiento
        }
    }
    
    const char* getUnidad() {
        return "bool|count|ms";
    }
    
    // Métodos adicionales
    bool hayMovimiento() {
        return movimiento_detectado;
    }
    
    unsigned long getContadorDetecciones() {
        return contador_detecciones;
    }
    
    unsigned long getTiempoUltimaDeteccion() {
        return tiempo_ultima_deteccion;
    }
    
    bool estaCalibrado() {
        return calibrado;
    }
    
    void resetearContador() {
        contador_detecciones = 0;
    }
    
    // Obtener tiempo desde última detección (en segundos)
    float getTiempoDesdeUltimaDeteccion() {
        if (tiempo_ultima_deteccion == 0) return -1;
        return (millis() - tiempo_ultima_deteccion) / 1000.0;
    }
};

// Declarar instancia global
SensorMovimiento sensorMovimiento;

// En setup() añadir:
// sensorMovimiento.inicializar();
// Serial.println("Calibrando sensor PIR... espere 60 segundos");
// gestorSensores.registrarSensor(&sensorMovimiento);