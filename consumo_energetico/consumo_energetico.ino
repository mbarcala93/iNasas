// ============================================
// SENSOR DE CONSUMO ENERGÉTICO SCT-013-030
// Sensor toroidal de corriente AC no invasivo
// ============================================

#define PIN_SCT013 A1              // Pin analógico para lectura
#define VOLTAJE_RED 230.0          // Voltaje de la red (230V en España)
#define RELACION_TRANSFORMACION 30.0  // 30A -> 1V
#define NUM_MUESTRAS 1480          // Muestras para cálculo RMS

class SensorConsumo {
private:
    bool activo;
    float corriente_rms;
    float potencia_watts;
    float energia_acumulada_kwh;
    unsigned long tiempo_anterior;
    
    // Calcular corriente RMS (Root Mean Square)
    float calcularIrms() {
        float suma = 0;
        
        for (int i = 0; i < NUM_MUESTRAS; i++) {
            int lectura = analogRead(PIN_SCT013);
            
            // Convertir a voltaje (0-1023 -> 0-5V)
            float voltaje = (lectura / 1023.0) * 5.0;
            
            // Restar offset (punto medio en 2.5V)
            voltaje = voltaje - 2.5;
            
            // Convertir a corriente usando relación del sensor
            // Calibración: Relación/Resistencia de carga
            // Para SCT-013-030: 1800 espiras / 62Ω ≈ 29
            float corriente = voltaje * 29.0;
            
            // Sumar cuadrado para RMS
            suma += corriente * corriente;
        }
        
        // Calcular RMS
        float rms = sqrt(suma / NUM_MUESTRAS);
        
        return rms;
    }
    
public:
    SensorConsumo() : activo(false), corriente_rms(0), potencia_watts(0), 
                      energia_acumulada_kwh(0), tiempo_anterior(0) {}
    
    void inicializar() {
        pinMode(PIN_SCT013, INPUT);
        tiempo_anterior = millis();
        activo = true;
    }
    
    bool estaActivo() {
        return activo;
    }
    
    const char* getNombre() {
        return "Corriente_Potencia_Energia";
    }
    
    float leer() {
        if (!activo) return -999;
        
        // Calcular corriente RMS
        corriente_rms = calcularIrms();
        
        // Calcular potencia (P = V * I)
        // Nota: Esto asume factor de potencia = 1 (carga resistiva)
        // Para cargas inductivas/capacitivas, multiplicar por factor de potencia
        potencia_watts = VOLTAJE_RED * corriente_rms;
        
        // Calcular energía acumulada
        unsigned long tiempo_actual = millis();
        float tiempo_transcurrido_horas = (tiempo_actual - tiempo_anterior) / 3600000.0;
        energia_acumulada_kwh += (potencia_watts / 1000.0) * tiempo_transcurrido_horas;
        tiempo_anterior = tiempo_actual;
        
        return corriente_rms;
    }
    
    const char* getUnidad() {
        return "A|W|kWh";
    }
    
    // Métodos para obtener valores individuales
    float getCorriente() { return corriente_rms; }
    float getPotencia() { return potencia_watts; }
    float getEnergia() { return energia_acumulada_kwh; }
    
    // Resetear energía acumulada
    void resetearEnergia() {
        energia_acumulada_kwh = 0;
    }
};

// Declarar instancia global
SensorConsumo sensorConsumo;

// En setup() añadir:
// sensorConsumo.inicializar();
// gestorSensores.registrarSensor(&sensorConsumo);
