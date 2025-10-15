/**
 * @file SistemaConfort_Optimizado.ino
 * @brief Sistema de confort térmico con lógica PMV y control inteligente de actuadores
 * 
 * Este proyecto de Arduino implementa un sistema de confort térmico con las siguientes características:
 * - Autenticación mediante RFID y contraseña
 * - Cálculo del índice PMV (Predicted Mean Vote) basado en múltiples sensores
 * - Control automático de ventilación y cortinas
 * - Sistema de alarmas para condiciones extremas
 * - Registro de usuarios con confirmación RFID
 * 
 * @author [Sebastián Ruiz, Brayan Charry]
 * @date [Fecha]
 * @version 2.1
 */

#include <LiquidCrystal.h>
#include <Keypad.h>
#include <SPI.h>
#include <MFRC522.h>
#include <DHT.h>
#include <Servo.h>

// ============== CONFIGURACIÓN DE PINES ==============
/**
 * @brief Definiciones de pines para sensores y actuadores
 */
#define DHTPIN 7              ///< Pin del sensor DHT11
#define DHTTYPE DHT11         ///< Tipo de sensor DHT
#define TEMP_ANALOG_PIN A1    ///< Pin del sensor de temperatura analógico
#define LDR_PIN A0            ///< Pin del sensor de luz (LDR)
#define RELAY_PIN 20          ///< Pin del relé para ventilador
#define SERVO_PIN 13          ///< Pin del servomotor
#define RED_PIN 22            ///< Pin LED RGB - Rojo
#define GREEN_PIN 24          ///< Pin LED RGB - Verde
#define BLUE_PIN 26           ///< Pin LED RGB - Azul
#define BUZZER_PIN 10         ///< Pin del buzzer
#define SS_PIN 53             ///< Pin SS del lector RFID
#define RST_PIN 49            ///< Pin RST del lector RFID

// ============== INICIALIZACIÓN DE COMPONENTES ==============
/**
 * @brief Configuración de la pantalla LCD
 */
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

/**
 * @brief Configuración del teclado matricial 4x4
 */
const byte ROWS = 4, COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte rowPins[ROWS] = {30, 32, 34, 36};
byte colPins[COLS] = {38, 40, 42, 44};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

/**
 * @brief Inicialización de objetos para sensores y actuadores
 */
MFRC522 mfrc522(SS_PIN, RST_PIN);
DHT dht(DHTPIN, DHTTYPE);
Servo servo;

// ============== ESTRUCTURAS Y VARIABLES GLOBALES ==============
/**
 * @brief Estructura para almacenar información de usuario
 */
struct Usuario {
  String nombre;  ///< Nombre del usuario
  String tag;     ///< UID del tag RFID
  String clave;   ///< Contraseña de 4 dígitos
};

/// Base de datos de usuarios (máximo 2)
Usuario usuarios[2] = {
  {"laura", "5CF8D773", "3456"},
  {"", "", ""}
};

int totalUsuarios = 1;                ///< Número de usuarios registrados
Usuario* usuarioActivo = nullptr;     ///< Puntero al usuario actualmente logueado
float pmv = 0.0;                      ///< Valor actual del PMV
int intentosFallidos = 0;             ///< Contador de intentos fallidos de login

// ============== PARÁMETROS DE ALARMA ==============
/**
 * @brief Umbrales y configuración de alarmas
 */
const float PMV_ALARM_THRESHOLD = 0.7;    ///< Umbral para alarma de calor extremo
const float PMV_BAJO_THRESHOLD = -0.7;    ///< Umbral para indicador de frío extremo
bool alarmActive = false;                  ///< Estado de la alarma principal
bool alarmSilenced = false;                ///< Indicador de alarma silenciada
unsigned long alarmSilenceUntil = 0;       ///< Tiempo hasta que termina el silencio
unsigned long lastAlarmToggle = 0;         ///< Último cambio de estado de alarma
const unsigned long alarmInterval = 500;   ///< Intervalo de parpadeo de alarma
bool alarmToggleState = false;             ///< Estado de parpadeo actual

/// Variables para control de PMV bajo (LED azul parpadeante)
bool pmvBajoActive = false;
unsigned long lastPmvBajoToggle = 0;
const unsigned long pmvBajoOnTime = 100;
const unsigned long pmvBajoOffTime = 400;
bool pmvBajoToggleState = false;

// ============== CONTROL DE BLOQUEO ==============
/**
 * @brief Variables para el sistema de bloqueo por intentos fallidos
 */
bool sistemaBloqueado = false;
unsigned long tiempoBloqueo = 0;
const unsigned long DURACION_BLOQUEO = 15000; ///< 15 segundos de bloqueo

// ============== CONTROL DE SERVO ==============
/**
 * @brief Variables para movimiento suave del servo
 */
int servoAnguloActual = 90;
int servoAnguloDeseado = 90;
unsigned long ultimoCambioServo = 0;
const unsigned long SERVO_DELAY = 3000; ///< 3 segundos entre cambios

// ============== ESTADOS DEL SISTEMA ==============
/**
 * @brief Enumeración de los estados principales del sistema
 */
enum Estado { MENU, LOGIN, CONFORT };

Estado estado = MENU; ///< Estado actual del sistema

/**
 * @brief Enumeración de estados del PMV para histéresis
 */
enum EstadoPMV { PMV_MUY_BAJO, PMV_BAJO, PMV_CONFORT, PMV_CALOR, PMV_MUY_CALOR };

EstadoPMV estadoPMVActual = PMV_CONFORT;
EstadoPMV estadoPMVAnterior = PMV_CONFORT;

// ============== PROTOTIPOS DE FUNCIONES ==============
void setColor(bool r, bool g, bool b);
void beep(int n);
void mostrarMenu();
String leerClave();
String leerTag();
String leerTexto(String prompt);
float calcularPMV(float tDHT, float tAnalog, float h, int luz);
void setActuador(float pmv);
void actualizarServo(int angulo);
void registrarPorTag();
float leerTempAnalogica();
void bloqueoSistema();
void checkAlarm(float pmv);
void alarmHandler();
void triggerAlarm();
void stopAlarm();
void verificarDesbloqueo();
void checkPmvBajo(float pmv);
void pmvBajoHandler();
void triggerPmvBajo();
void stopPmvBajo();

// ============== CONFIGURACIÓN INICIAL ==============
/**
 * @brief Función de configuración inicial del sistema
 * 
 * Inicializa:
 * - Comunicación serie
 * - Sensor RFID
 * - Sensor DHT11
 * - Servomotor
 * - Pines de entrada/salida
 * - Pantalla LCD
 */
void setup() {
  Serial.begin(9600);
  SPI.begin();
  mfrc522.PCD_Init();
  dht.begin();
  servo.attach(SERVO_PIN);
  servo.write(90);

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, LOW);
  setColor(false, false, true);

  lcd.begin(16, 2);
  lcd.print("Sistema RFID");
  delay(1500);
  lcd.clear();
  
  Serial.println("Sistema iniciado");
}

// ============== BUCLE PRINCIPAL ==============
/**
 * @brief Bucle principal del programa
 * 
 * Gestiona:
 * - Verificación de desbloqueo del sistema
 * - Transiciones entre estados (MENU, LOGIN, CONFORT)
 * - Lectura de sensores en modo CONFORT
 * - Control de actuadores basado en PMV
 * - Manejo de alarmas
 */
void loop() {
  verificarDesbloqueo();

  switch (estado) {
    case MENU: {
      mostrarMenu();
      char key = keypad.getKey();
      if (key == '1') {
        estado = LOGIN;
        lcd.clear();
      } else if (key == '2') {
        registrarPorTag();
        lcd.clear();
      }
    } break;

    case LOGIN: {
      lcd.clear();
      lcd.print("Acerque su tag");
      
      String tag = leerTag();
      if (tag != "") {
        Serial.print("Tag leído: ");
        Serial.println(tag);
        
        bool tagEncontrado = false;
        for (int i = 0; i < totalUsuarios; i++) {
          if (usuarios[i].tag == tag) {
            tagEncontrado = true;
            usuarioActivo = &usuarios[i];
            lcd.clear();
            lcd.print("Hola ");
            lcd.print(usuarioActivo->nombre);
            beep(2);
            delay(1000);

            lcd.clear();
            lcd.print("Clave:");
            String clave = leerClave();

            if (clave == usuarioActivo->clave) {
              intentosFallidos = 0;
              beep(3);
              setColor(false, true, false);
              lcd.clear();
              lcd.print("Acceso OK");
              delay(1500);
              
              servoAnguloActual = 90;
              servoAnguloDeseado = 90;
              servo.write(90);
              ultimoCambioServo = millis();
              estadoPMVActual = PMV_CONFORT;
              estadoPMVAnterior = PMV_CONFORT;
              
              estado = CONFORT;
            } else {
              intentosFallidos++;
              beep(1);
              setColor(true, false, false);
              lcd.clear();
              lcd.print("Clave Erronea");
              delay(1500);
              if (intentosFallidos >= 3) bloqueoSistema();
              estado = MENU;
            }
            
            mfrc522.PICC_HaltA();
            mfrc522.PCD_StopCrypto1();
            return;
          }
        }
        
        if(!tagEncontrado) {
          lcd.clear();
          lcd.print("Tag no reg.");
          beep(1);
          delay(2000);
        }
        
        mfrc522.PICC_HaltA();
        mfrc522.PCD_StopCrypto1();
        estado = MENU;
      }
    } break;

    case CONFORT: {
      float tDHT = dht.readTemperature();
      float h = dht.readHumidity();
      float tAnalog = leerTempAnalogica();
      int luz = analogRead(LDR_PIN);

      if (isnan(tDHT) || isnan(h)) {
        tDHT = 25.0;
        h = 50.0;
      }

      pmv = calcularPMV(tDHT, tAnalog, h, luz);
      
      Serial.print("T_DHT:");
      Serial.print(tDHT);
      Serial.print(" | T_Analog:");
      Serial.print(tAnalog);
      Serial.print(" | H:");
      Serial.print(h);
      Serial.print(" | Luz:");
      Serial.print(luz);
      Serial.print(" | PMV:");
      Serial.println(pmv);

      setActuador(pmv);
      checkAlarm(pmv);
      checkPmvBajo(pmv);
      
      if (alarmActive) {
        alarmHandler();
        char k = keypad.getKey();
        if (k == '*') {
          stopAlarm();
          alarmSilenced = true;
          alarmSilenceUntil = millis() + 60000;
        }
        if (k == '#') {
          stopAlarm();
          stopPmvBajo();
          estado = MENU;
          usuarioActivo = nullptr;
          intentosFallidos = 0;
          servoAnguloActual = 90;
          servo.write(90);
          lcd.clear();
          return;
        }
        if (alarmSilenced && millis() > alarmSilenceUntil) {
          alarmSilenced = false;
        }
        delay(200);
      } else if (pmvBajoActive) {
        pmvBajoHandler();
        
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(usuarioActivo ? usuarioActivo->nombre : "Usuario");
        lcd.print(" PMV:");
        lcd.print(pmv, 2);
        lcd.setCursor(0,1);
        lcd.print("T1:");
        lcd.print(tDHT,1);
        lcd.print(" T2:");
        lcd.print(tAnalog,1);
        
        char k = keypad.getKey();
        if (k == '#') {
          stopPmvBajo();
          estado = MENU;
          usuarioActivo = nullptr;
          servo.write(90);
          lcd.clear();
          return;
        }
        delay(200);
      } else {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(usuarioActivo ? usuarioActivo->nombre : "Usuario");
        lcd.print(" PMV:");
        lcd.print(pmv, 2);
        lcd.setCursor(0,1);
        lcd.print("T1:");
        lcd.print(tDHT,1);
        lcd.print(" T2:");
        lcd.print(tAnalog,1);
        
        char k = keypad.getKey();
        if (k == '#') {
          estado = MENU;
          usuarioActivo = nullptr;
          servo.write(90);
          setColor(false, false, true);
          lcd.clear();
          return;
        }
        delay(2000);

        lcd.clear();
        lcd.print("H:");
        lcd.print(h,0);
        lcd.print("% L:");
        lcd.print(luz);
        lcd.setCursor(0,1);
        lcd.print("Servo:");
        lcd.print(servoAnguloActual);
        lcd.print(" deg");
        delay(2000);
      }
    } break;
  }
}

// ============== FUNCIONES DE INTERFAZ ==============
/**
 * @brief Establece el color del LED RGB
 * @param r Estado del color rojo (true = encendido)
 * @param g Estado del color verde (true = encendido)
 * @param b Estado del color azul (true = encendido)
 */
void setColor(bool r, bool g, bool b) {
  digitalWrite(RED_PIN,   r ? LOW : HIGH);
  digitalWrite(GREEN_PIN, g ? LOW : HIGH);
  digitalWrite(BLUE_PIN,  b ? LOW : HIGH);
}

/**
 * @brief Genera n pitidos cortos con el buzzer
 * @param n Número de pitidos a generar
 */
void beep(int n) {
  for (int i = 0; i < n; i++) {
    tone(BUZZER_PIN, 1000, 200);
    delay(250);
  }
}

/**
 * @brief Muestra el menú principal en el LCD
 */
void mostrarMenu() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("1.Login");
  lcd.setCursor(0,1);
  lcd.print("2.Registro");
}

/**
 * @brief Lee una contraseña de 4 dígitos desde el teclado
 * @return String con la contraseña ingresada
 * 
 * Controles:
 * - '#' para confirmar
 * - '*' para borrar
 * - Muestra asteriscos por seguridad
 */
String leerClave() {
  String clave = "";
  lcd.setCursor(0,1);
  while (true) {
    char key = keypad.getKey();
    if (key) {
      if (key == '#') break;
      if (key == '*') {
        clave = "";
        lcd.setCursor(0,1);
        lcd.print("                ");
        lcd.setCursor(0,1);
      } else if (clave.length() < 4) {
        clave += key;
        lcd.print('*');
      }
    }
  }
  return clave;
}

/**
 * @brief Lee el UID de un tag RFID
 * @return String con el UID en formato hexadecimal
 */
String leerTag() {
  if (!mfrc522.PICC_IsNewCardPresent()) return "";
  if (!mfrc522.PICC_ReadCardSerial()) return "";
  String content = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    if (mfrc522.uid.uidByte[i] < 0x10) content += "0";
    content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  content.toUpperCase();
  return content;
}

/**
 * @brief Lee texto desde el teclado
 * @param prompt Mensaje a mostrar en pantalla
 * @return String con el texto ingresado
 */
String leerTexto(String prompt) {
  lcd.clear();
  lcd.print(prompt);
  lcd.setCursor(0,1);
  String texto = "";
  while (true) {
    char key = keypad.getKey();
    if (key) {
      if (key == '#') break;
      if (key == '*') {
        texto = "";
        lcd.setCursor(0,1);
        lcd.print("                ");
        lcd.setCursor(0,1);
      } else {
        texto += key;
        lcd.print(key);
      }
    }
  }
  return texto;
}

/**
 * @brief Lee la temperatura del sensor analógico
 * @return Temperatura en grados Celsius
 */
float leerTempAnalogica() {
  int valor = analogRead(TEMP_ANALOG_PIN);
  float voltaje = valor * (5.0 / 1023.0);
  return voltaje * 10.0;
}

// ============== CÁLCULO DEL PMV ==============
/**
 * @brief Calcula el índice PMV (Predicted Mean Vote)
 * 
 * El PMV es un índice que predice la sensación térmica promedio de un grupo de personas.
 * 
 * Escala:
 * - PMV < -0.7: Muy frío
 * - -0.7 ≤ PMV < -0.3: Frío
 * - -0.3 ≤ PMV ≤ 0.3: Confortable
 * - 0.3 < PMV ≤ 0.7: Cálido
 * - PMV > 0.7: Muy caliente
 * 
 * @param tDHT Temperatura del sensor DHT11 (°C)
 * @param tAnalog Temperatura del sensor analógico (°C)
 * @param h Humedad relativa (%)
 * @param luz Nivel de luz del LDR (0-1023)
 * @return Valor del PMV calculado
 */
float calcularPMV(float tDHT, float tAnalog, float h, int luz) {
  float tProm = (tDHT + tAnalog) / 2.0;
  
  float factorHumedad = 0.0;
  if (h < 40) {
    factorHumedad = (40 - h) / 40.0 * (-0.2);
  } else if (h > 60) {
    factorHumedad = (h - 60) / 40.0 * 0.3;
  }
  
  float factorLuz = (luz - 300.0) / 700.0 * 0.15;
  factorLuz = constrain(factorLuz, -0.15, 0.15);
  
  float tempConfort = 23.0;
  float desviacionTemp = tProm - tempConfort;
  
  float pmvCalculado = (desviacionTemp * 0.25) + factorHumedad + factorLuz;
  pmvCalculado = constrain(pmvCalculado, -2.0, 2.0);
  
  return pmvCalculado;
}

// ============== CONTROL DE ACTUADORES ==============
/**
 * @brief Controla actuadores basándose en el valor del PMV
 * 
 * Utiliza histéresis para evitar cambios constantes:
 * - Ventilador (relé): ON cuando hace calor
 * - Servo (cortina): Ángulo según nivel de calor/frío
 * - LED RGB: Código de colores según estado térmico
 * 
 * @param pmvVal Valor actual del PMV
 */
void setActuador(float pmvVal) {
  EstadoPMV nuevoEstado = estadoPMVActual;
  
  if (pmvVal > 0.8) {
    nuevoEstado = PMV_MUY_CALOR;
  } else if (pmvVal > 0.4) {
    if (estadoPMVActual != PMV_MUY_CALOR || pmvVal < 0.6) {
      nuevoEstado = PMV_CALOR;
    }
  } else if (pmvVal >= -0.4 && pmvVal <= 0.4) {
    nuevoEstado = PMV_CONFORT;
  } else if (pmvVal < 0.3 && pmvVal >= -0.8) {
    if (estadoPMVActual != PMV_MUY_BAJO || pmvVal > -0.6) {
      nuevoEstado = PMV_BAJO;
    }
  } else if (pmvVal < -0.8) {
    nuevoEstado = PMV_MUY_BAJO;
  }
  
  if (nuevoEstado != estadoPMVActual) {
    estadoPMVAnterior = estadoPMVActual;
    estadoPMVActual = nuevoEstado;
  }
  
  switch (estadoPMVActual) {
    case PMV_MUY_CALOR:
      digitalWrite(RELAY_PIN, HIGH);
      actualizarServo(0);
      if (!alarmActive) setColor(true, false, false);
      break;
      
    case PMV_CALOR:
      digitalWrite(RELAY_PIN, HIGH);
      actualizarServo(45);
      if (!alarmActive && !pmvBajoActive) setColor(true, true, false);
      break;
      
    case PMV_CONFORT:
      digitalWrite(RELAY_PIN, LOW);
      actualizarServo(90);
      if (!alarmActive && !pmvBajoActive) setColor(false, true, false);
      break;
      
    case PMV_BAJO:
      digitalWrite(RELAY_PIN, LOW);
      actualizarServo(135);
      if (!pmvBajoActive) setColor(false, true, true);
      break;
      
    case PMV_MUY_BAJO:
      digitalWrite(RELAY_PIN, LOW);
      actualizarServo(180);
      break;
  }
}

/**
 * @brief Actualiza la posición del servo con retardo anti-rebote
 * 
 * Solo mueve el servo si han pasado al menos 3 segundos desde
 * el último movimiento, evitando cambios constantes.
 * 
 * @param anguloDeseado Ángulo objetivo (0-180 grados)
 */
void actualizarServo(int anguloDeseado) {
  if (anguloDeseado != servoAnguloDeseado) {
    servoAnguloDeseado = anguloDeseado;
  }
  
  if (servoAnguloActual != servoAnguloDeseado && 
      (millis() - ultimoCambioServo >= SERVO_DELAY)) {
    servo.write(servoAnguloDeseado);
    servoAnguloActual = servoAnguloDeseado;
    ultimoCambioServo = millis();
  }
}

// ============== REGISTRO DE USUARIOS ==============
/**
 * @brief Registra un nuevo usuario mediante tag RFID
 * 
 * Proceso:
 * 1. Lee el tag RFID
 * 2. Solicita nombre del usuario
 * 3. Solicita contraseña de 4 dígitos
 * 4. Confirma con segunda lectura del tag
 * 5. Ingresa automáticamente al modo CONFORT
 */
void registrarPorTag() {
  if (totalUsuarios >= 2) {
    lcd.clear();
    lcd.print("Limite usuarios");
    beep(1);
    delay(1500);
    return;
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  
  lcd.clear();
  lcd.print("Acerque su tag");
  String tag = "";
  while (tag == "") {
    tag = leerTag();
    delay(100);
  }
  
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  beep(2);
  lcd.clear();
  lcd.print("Tag leido:");
  lcd.setCursor(0,1);
  lcd.print(tag);
  delay(2000);

  String nombre = leerTexto("Nombre:");
  
  lcd.clear();
  lcd.print("Ingrese clave:");
  String clave = leerClave();

  usuarios[totalUsuarios].tag = tag;
  usuarios[totalUsuarios].nombre = nombre;
  usuarios[totalUsuarios].clave = clave;
  
  totalUsuarios++;

  beep(3);
  lcd.clear();
  lcd.print("Usuario #");
  lcd.print(totalUsuarios);
  lcd.print(" OK");
  delay(2000);
  
  lcd.clear();
  lcd.print("Pase tag para");
  lcd.setCursor(0,1);
  lcd.print("confirmar");
  delay(1000);
  
  String tagConfirm = "";
  while (tagConfirm == "") {
    tagConfirm = leerTag();
    delay(100);
  }
  
  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
  
  if (tagConfirm == tag) {
    beep(3);
    lcd.clear();
    lcd.print("Confirmado!");
    delay(1500);
    usuarioActivo = &usuarios[totalUsuarios - 1];
    
    servoAnguloActual = 90;
    servoAnguloDeseado = 90;
    servo.write(90);
    ultimoCambioServo = millis();
    
    estado = CONFORT;
    lcd.clear();
  } else {
    beep(1);
    lcd.clear();
    lcd.print("Tag no coincide");
    delay(2000);
    estado = MENU;
  }
}

// ============== SISTEMA DE BLOQUEO ==============
/**
 * @brief Bloquea el sistema por 15 segundos tras 3 intentos fallidos
 * 
 * Durante el bloqueo:
 * - LED rojo parpadeante
 * - Buzzer intermitente
 * - Pantalla muestra tiempo restante
 */
void bloqueoSistema() {
  sistemaBloqueado = true;
  tiempoBloqueo = millis();
  intentosFallidos = 0;
  
  lcd.clear();
  lcd.print("!! BLOQUEADO !!");
  setColor(true, false, false);
  
  for (int i = 0; i < 10; i++) {
    tone(BUZZER_PIN, 1000, 150);
    delay(250);
  }
  
  noTone(BUZZER_PIN);
  lcd.clear();
  lcd.print("Sistema bloqueado");
}

/**
 * @brief Verifica si el tiempo de bloqueo ha terminado
 * 
 * Actualiza el estado del sistema durante el bloqueo y
 * desbloquea automáticamente al terminar el tiempo.
 */
void verificarDesbloqueo() {
  if (sistemaBloqueado) {
    if (millis() - tiempoBloqueo >= DURACION_BLOQUEO) {
      sistemaBloqueado = false;
      noTone(BUZZER_PIN);
      setColor(false, false, true);
      lcd.clear();
      lcd.print("Sistema OK");
      beep(2);
      delay(1500);
      estado = MENU;
      usuarioActivo = nullptr;
      lcd.clear();
    } else {
      unsigned long tiempoTranscurrido = millis() - tiempoBloqueo;
      if ((tiempoTranscurrido / 500) % 2 == 0) {
        setColor(true, false, false);
        tone(BUZZER_PIN, 1000);
      } else {
        setColor(false, false, false);
        noTone(BUZZER_PIN);
      }
      
      if ((tiempoTranscurrido / 1000) % 2 == 0) {
        unsigned long tiempoRestante = (DURACION_BLOQUEO - tiempoTranscurrido) / 1000;
        lcd.clear();
        lcd.print("BLOQUEADO");
        lcd.setCursor(0,1);
        lcd.print("Espere ");
        lcd.print(tiempoRestante);
        lcd.print("s");
      }
    }
  }
}

// ============== SISTEMA DE ALARMAS ==============
/**
 * @brief Verifica si se debe activar la alarma por PMV alto
 * 
 * La alarma se activa cuando el PMV supera el umbral de calor extremo.
 * Puede ser silenciada temporalmente por 60 segundos.
 * 
 * @param pmvVal Valor actual del PMV
 */
void checkAlarm(float pmvVal) {
  if (alarmSilenced && millis() < alarmSilenceUntil) {
    return;
  } else {
    alarmSilenced = false;
  }

  if (pmvVal > PMV_ALARM_THRESHOLD) {
    if (!alarmActive) triggerAlarm();
  } else {
    if (alarmActive) stopAlarm();
  }
}

/**
 * @brief Activa la alarma de PMV alto
 * 
 * Efectos:
 * - LED rojo encendido
 * - Buzzer a 2000 Hz
 * - Mensaje en LCD
 */
void triggerAlarm() {
  alarmActive = true;
  alarmToggleState = false;
  lastAlarmToggle = millis();
  setColor(true, false, false);
  tone(BUZZER_PIN, 2000);
  lcd.clear();
  lcd.print("!! ALARMA PMV !!");
}

/**
 * @brief Detiene la alarma y muestra confirmación
 */
void stopAlarm() {
  alarmActive = false;
  alarmToggleState = false;
  noTone(BUZZER_PIN);
  setColor(false, true, false);
  lcd.clear();
  lcd.print("Alarma Silenciada");
  delay(800);
}

/**
 * @brief Maneja el parpadeo y sonido de la alarma activa
 * 
 * Alterna cada 500ms entre:
 * - Estado 1: LED rojo, buzzer, mensaje de alarma
 * - Estado 2: LED apagado, sin sonido, instrucciones
 */
void alarmHandler() {
  unsigned long now = millis();
  if (now - lastAlarmToggle >= alarmInterval) {
    lastAlarmToggle = now;
    alarmToggleState = !alarmToggleState;
    if (alarmToggleState) {
      setColor(true, false, false);
      tone(BUZZER_PIN, 2000);
      lcd.clear();
      lcd.print("!! ALARMA PMV !!");
      lcd.setCursor(0,1);
      lcd.print("PMV:");
      lcd.print(pmv, 2);
      lcd.print(" *=Silenc");
    } else {
      setColor(false, false, false);
      noTone(BUZZER_PIN);
      lcd.clear();
      lcd.print("Presione * para");
      lcd.setCursor(0,1);
      lcd.print("silenciar 60s");
    }
  }
}

// ============== INDICADOR DE FRÍO EXTREMO ==============
/**
 * @brief Verifica si se debe activar el indicador de PMV bajo
 * 
 * Cuando el PMV es menor a -0.7, se activa el LED azul parpadeante
 * como indicador visual de frío extremo (no es alarma sonora).
 * 
 * @param pmvVal Valor actual del PMV
 */
void checkPmvBajo(float pmvVal) {
  if (pmvVal < PMV_BAJO_THRESHOLD) {
    if (!pmvBajoActive) triggerPmvBajo();
  } else {
    if (pmvBajoActive) stopPmvBajo();
  }
}

/**
 * @brief Activa el indicador de PMV bajo (LED azul parpadeante)
 */
void triggerPmvBajo() {
  pmvBajoActive = true;
  pmvBajoToggleState = true;
  lastPmvBajoToggle = millis();
  setColor(false, false, true);
  Serial.println("PMV_BAJO activado - LED azul parpadeante");
}

/**
 * @brief Detiene el indicador de PMV bajo
 */
void stopPmvBajo() {
  pmvBajoActive = false;
  pmvBajoToggleState = false;
  Serial.println("PMV_BAJO desactivado");
}

/**
 * @brief Maneja el parpadeo del LED azul para PMV bajo
 * 
 * Patrón de parpadeo:
 * - 100ms encendido
 * - 400ms apagado
 * 
 * Este patrón crea un efecto visual distintivo para indicar
 * condiciones de frío extremo sin ser intrusivo.
 */
void pmvBajoHandler() {
  unsigned long now = millis();
  unsigned long interval = pmvBajoToggleState ? pmvBajoOnTime : pmvBajoOffTime;
  
  if (now - lastPmvBajoToggle >= interval) {
    lastPmvBajoToggle = now;
    pmvBajoToggleState = !pmvBajoToggleState;
    
    if (pmvBajoToggleState) {
      setColor(false, false, true);
    } else {
      setColor(false, false, false);
    }
  }
}
        