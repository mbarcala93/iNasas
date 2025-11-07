// ============================================
// PLANTILLA PARA NUEVO SENSOR
// ============================================

// 1. DEFINIR PINES Y CONSTANTES
#define PIN_NUEVO_SENSOR A0  // Cambiar según tu sensor
// Añadir otras constantes necesarias

// 2. CREAR LA CLASE DEL SENSOR
class SensorNuevo {
private:
    bool activo;
    float ultimaLectura;
    
public:
    SensorNuevo() : activo(false), ultimaLectura(0) {}
    
    void inicializar() {
        pinMode(PIN_NUEVO_SENSOR, INPUT);
        // Configuración adicional del sensor
        activo = true;
    }
    
    bool estaActivo() {
        return activo;
    }
    
    const char* getNombre() {
        return "NombreSensor";  // Nombre para el CSV
    }
    
    float leer() {
        if (!activo) return -999;
        
        // Código de lectura del sensor
        int valorRaw = analogRead(PIN_NUEVO_SENSOR);
        ultimaLectura = valorRaw * FACTOR_CONVERSION;
        
        return ultimaLectura;
    }
    
    const char* getUnidad() {
        return "unidad";  // ej: "cm", "NTU", "W", etc.
    }
};

// 3. DECLARAR INSTANCIA GLOBAL
SensorNuevo sensorNuevo;

// 4. ACTIVAR EN setup()
// En la función setup(), añadir:
// sensorNuevo.inicializar();

// 5. REGISTRAR EN EL GESTOR
// En setup(), después de inicializar:
// gestorSensores.registrarSensor(&sensorNuevo);