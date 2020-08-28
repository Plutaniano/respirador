#define ci const int

#include <LiquidCrystal.h> //LCD
#include "RTClib.h" // RTC
#include <PN532_HSU.h> // NFC
#include <PN532.h>  // NFC

//Objetos
PN532_HSU pn532hsu(Serial1);
PN532 nfc(pn532hsu);
RTC_DS3231 rtc;
DateTime now;

// -----------------------------------------------

//Pinos do LCD
ci RSpin = 42;
ci Epin = 43;
ci D4pin = 44;
ci D5pin = 45;
ci D6pin = 46;
ci D7pin = 47;
LiquidCrystal lcd(RSpin, Epin, D4pin, D5pin, D6pin, D7pin);

//Pinos Step motor
ci ENplus_pin = 24;
ci ENminus_pin = 25;
ci DIRplus_pin = 26;
ci DIRminus_pin = 27;
ci PULplus_pin = 28;
ci PULminus_pin = 29;
ci stepsPerRevolution = 200;

//Pinos de Botões
ci SWITCHpin = 32;
ci ALARMEpin = 35;
ci B1pin = 34;
ci B2pin = 33;

//Pinos dos LEDs
ci LED1pin = 36;
ci LED2pin = 37;

//Pino Sensor de Pressão
ci AD0pin = A0;

//Pinos Potenciometros
ci AD1pin = A11; // IE
ci AD2pin = A12; // Pmax
ci AD3pin = A13; // Psen
ci AD4pin = A14; // BPM
ci AD5pin = A15; // O2

//Pino de referencia 2.048V e variavel com ajuste do ADC
ci REFpin = A7;
float ajuste_ADC = 0;

//Variaveis dos potenciometros
int AD0 = 0; // Pressão
int AD1 = 0; // IE
int AD2 = 0; // Pmax
int AD3 = 0; // Psen
int AD4 = 0; // BPM
int AD5 = 0; // O2

//Pinos NFC
// Serial1
// SCL = D18/TX1
// SDA = D19/RX1
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 }; // user ID do cartão NFC
uint8_t uidLength; // tamanho do user ID
String nomes[5] = {"hora  ", "minuto", "ano   ", "mes   ", "dia   "}; // lista de componentes da data

//Outros
bool SWITCH_hold;

// -----------------------------------------------

void setup() {
  lcd.begin(16, 2);
  Serial.begin(115200);
  init_pins();
  rtc_init();
  nfc_init();
  rotina_menus();
  ajuste_ADC = calcular_erro_ADC();
  subrotina_de_start_up();
}

void loop() {
  delay(2000);
  blink_LED(1, 500, 2);
  blink_LED(2, 500, 2);
  lcd.clear();
  while (1) {
    refresh_medidas();
    print_aligned(0, 0, AD0);
    print_aligned(5, 0, AD1);
    print_aligned(10, 0, AD2);
    print_aligned(0, 1, AD3);
    print_aligned(5, 1, AD4);
    print_aligned(10, 1, AD5);
    lcd.setCursor(14, 0);
    lcd.print(digitalRead(SWITCHpin));
    lcd.print(digitalRead(ALARMEpin));
    lcd.setCursor(14, 1);
    lcd.print(digitalRead(B1pin));
    lcd.print(digitalRead(B2pin));
  }
}

// -----------------------------------------------

void subrotina_de_start_up() {
  // Rotina que mostra nome, versão
  // e testa os LED, motor e caso o SWITCH
  // seja segurado por 2 segundos, entra na
  // rotina de calibração(não criada)
  lcd.clear();
  lcd.setCursor(0, 0); //coluna, linha
  lcd.print("Respirador ");
  lcd.print(now.hour());
  lcd.print(":");
  lcd.print(now.minute());
  lcd.setCursor(0, 1);
  lcd.print("Mackbreathe v1.0");
  blink_LED(1, 100, 2);
  voltas_motor(1, 1);
  blink_LED(2, 100, 2);
  voltas_motor(1, 0);
  delay(1000);
  if (digitalRead(SWITCHpin)) {
    delay(2000);
    if (digitalRead(SWITCHpin)) {
      subrotina_de_calibracao();
    }
  }
}

void subrotina_de_calibracao() {
  //placeholder para subrotina de calibração
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("<SUBROTINA>");
  lcd.setCursor(0, 1);
  lcd.print("<CALIBRACAO>");
  delay(5000);
}

int media_5_leituras(int pin) {
  // recebe um pin como input e retorna
  //a média de 5 leituras daquele pin
  int media = 0;
  for (int i = 0; i < 5; i++) {
    media = media + analogRead(pin);
    delay(5);
  }
  return media / 5;
}

void refresh_medidas() {
// atualiza todas as medidas dos ADs
  AD0 = int(media_5_leituras(AD0pin) * ajuste_ADC);
  AD1 = int(media_5_leituras(AD1pin));
  AD2 = int(media_5_leituras(AD2pin));
  AD3 = int(media_5_leituras(AD3pin));
  AD4 = int(media_5_leituras(AD4pin));
  AD5 = int(media_5_leituras(AD5pin));
}

void blink_LED(int led, int speed, int times) {
// pisca o LED "led" "times" vezes com o intervalo de "speed"
if (led == 1) {
    for (int a = 0; a < times; a++) {
      digitalWrite(LED1pin, HIGH);
      delay(speed);
      digitalWrite(LED1pin, LOW);
      delay(speed);
    }
  } else {
    for (int a = 0; a < times; a++) {
      digitalWrite(LED2pin, HIGH);
      delay(speed);
      digitalWrite(LED2pin, LOW);
      delay(speed);
    }
  }
}

void voltas_motor(int voltas, int dir) {
  // da "voltas" voltas com o motor na direção "dir"
  motorEN(1);
  motorDIR(dir);
  for (int i = 0; i < voltas * stepsPerRevolution; i++) {
    digitalWrite(PULplus_pin, HIGH);
    digitalWrite(PULminus_pin, LOW);
    delayMicroseconds(500);
    digitalWrite(PULplus_pin, LOW);
    digitalWrite(PULminus_pin, HIGH);
    delayMicroseconds(500);
  }
  motorEN(0);
  delay(50);
}

void motorEN(int i) {
  // habilita ou desabilita o motor
  if (i == 1) {
    digitalWrite(ENplus_pin, LOW);
    digitalWrite(ENminus_pin, HIGH);
  } else {
    digitalWrite(ENplus_pin, HIGH);
    digitalWrite(ENminus_pin, LOW);
  }
}

void motorDIR(int i) {
  // troca a direção do motor
  if (i == 1) {
    digitalWrite(DIRplus_pin, LOW);
    digitalWrite(DIRminus_pin, HIGH);
  } else {
    digitalWrite(DIRplus_pin, HIGH);
    digitalWrite(DIRminus_pin, LOW);
  }
}

void print_aligned(int x, int y, int value) {
  // escreve as leituras dos ADs de forma alinhada no LCD
  // se você imprimir um número "1111" na posição 0,0 do LCD
  // e depois mandar imprimir um número "999" na mesma posição
  // o lcd irá mostrar "9991".
  // essa função resolve esse problema colocando espaços antes
  // dos números com <4 digitos
  lcd.setCursor(x, y);
  if (value <= 9) {
    lcd.print("   ");
    lcd.print(value);
  } else if (value <= 99) {
    lcd.print("  ");
    lcd.print(value);
  } else if (value <= 999) {
    lcd.print(" ");
    lcd.print(value);
  } else {
    lcd.print(value);
  }
}

float calcular_erro_ADC() {
  // compara a leitura do ADC com o valor esperado de 2.048V
  // e retorna um multiplicador de correção
  int x;
  x = analogRead(REFpin);
  Serial.print("ADC 2.048V: ");
  Serial.print(map(x, 0, 1023, 0, 5000) * 0.001);   // https://www.arduino.cc/reference/pt/language/functions/math/map/
  Serial.println("V");
  Serial.print("Multiplicador: ");
  Serial.println(419.0208 / x);
  // ADC do arduino lê valores entre 0 e 5V (0 a 1023)
  // 2.048V corresponde ao valor 419 no range de 10bits
  return 419.0208 / x;
}



void rotina_ajuste_rtc() {
// Rotina para modificar a data e hora do rtc
// Usuario utiliza os botoes para selecionar entre
// ano, mes, dia, hora e minuto e pode incrementar ou 
// decrementar cada uma dos campos
// parecido com o ajuste de relógio de um microondas
  Serial.println("rotina_ajuste_rtc() acionada");
  lcd.clear();
  int posicao = 0;
  int ano_novo = now.year();
  int mes_novo = now.month();
  int dia_novo = now.day();
  int hora_novo = now.hour();
  int minuto_novo = now.minute();
  while (!digitalRead(SWITCHpin)) {
    delay(140);
    lcd.setCursor(0, 0);
    lcd.print(time_string());
    lcd.print(":");
    lcd.print(minuto_novo);
    lcd.print(" ");
    lcd.print(ano_novo);
    lcd.print("/");
    lcd.print(mes_novo);
    lcd.print("/");
    lcd.print(dia_novo);
    lcd.setCursor(0, 1);
    lcd.print(nomes[posicao]);

    if (digitalRead(ALARMEpin)) {
      ++posicao;
    }

    bool B1state = digitalRead(B1pin);
    bool B2state = digitalRead(B2pin);
    if (posicao == 0) {
      if (B1state) {
        ++hora_novo;
      } else if (B2state) {
        --hora_novo;
      }
    } else if (posicao == 1) {
      if (B1state) {
        ++minuto_novo;
      } else if (B2state) {
        --minuto_novo;
      }
    } else if (posicao == 2) {
      if (B1state) {
        ++ano_novo;
      } else if (B2state) {
        --ano_novo;
      }
    } else if (posicao == 3) {
      if (B1state) {
        ++mes_novo;
      } else if (B2state) {
        --mes_novo;
      }
    } else if (posicao == 4) {
      if (B1state) {
        ++dia_novo;
      } else if (B2state) {
        --dia_novo;
      }
    } else if (posicao == 5) {
      posicao = 0;
    }
  }
  rtc.adjust(DateTime(ano_novo, mes_novo, dia_novo, hora_novo, minuto_novo, 0));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(hora_novo);
  lcd.print(":");
  lcd.print(minuto_novo);
  lcd.print(" ");
  lcd.print(ano_novo);
  lcd.print("/");
  lcd.print(mes_novo);
  lcd.print("/");
  lcd.print(dia_novo);
  lcd.setCursor(0, 1);
  lcd.print("Data/Hora salva");
  delay(2000);
}

void rotina_menus() {
// rotina para entrar no ajuste de relogio
// se SWITCH for segurado por 2s
  delay(500);
  SWITCH_hold = digitalRead(SWITCHpin);
  int start = millis();
  int time_since_last_switch = millis();
  while (start + 2000 > millis()) {
    if (!digitalRead(SWITCHpin)) {
      SWITCH_hold = 0;
    }
  }
  if (SWITCH_hold) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("*** AJUSTAR  ***");
    lcd.setCursor(0, 1);
    lcd.print("*** RELOGIO  ***");
    delay(2000);
    rotina_ajuste_rtc();
  }
}

String time_string() {
// Retorna uma string com o tempo atual no formato
// aaaa/mm/dd hh:mm
  return String(now.year()) + "/" + String(now.month()) + "/" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
}

// Inicialização do nfc reader
void nfc_init() {
  Serial.println("nfc_init() acionada");
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.print("Placa PN53x nao encontrada...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PN35x erro");
    while (1); // halt
  }
  Serial.print("Encontrado chip PN5");
  Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware versao: ");
  Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.');
  Serial.println((versiondata >> 8) & 0xFF, DEC);
  nfc.setPassiveActivationRetries(0xFF);
  nfc.SAMConfig();
  Serial.println("Aguardando cartao ISO14443A...");
  Serial.println("");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Aprox cartao");
  // loop que espera a leitura de um cartão
  while (1) {
    boolean success;
    success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, & uid[0], & uidLength);
    if (success) {
      lcd.setCursor(0, 1);
      lcd.print("OK!");
      Serial.println("Cartao detectado");
      Serial.print("Tamanho do UID: ");
      Serial.print(uidLength, DEC);
      Serial.println(" bytes");
      Serial.print("UID: ");
      delay(1000);
      lcd.setCursor(0, 1);
      for (uint8_t i = 0; i < uidLength; i++) {
        Serial.print(" 0x");
        Serial.print(uid[i], HEX);
        lcd.print(uid[i], HEX);
      }
      Serial.println("");
      Serial.println("");
      delay(1000);
      break;
    }
  }
}

// Inicialização do rtc
void rtc_init() {
  rtc.begin();
  now = rtc.now();
  Serial.println("rtc_init() acionada");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

// Inicialização dos pinos
void init_pins() {

  // Pino Sensor de Pressao
  pinMode(AD0pin, INPUT);

  // Pinos Potenciometros
  pinMode(AD1pin, INPUT);
  pinMode(AD2pin, INPUT);
  pinMode(AD3pin, INPUT);
  pinMode(AD4pin, INPUT);
  pinMode(AD5pin, INPUT);
  pinMode(REFpin, INPUT);

  // Pinos Botoes
  pinMode(SWITCHpin, INPUT);
  pinMode(ALARMEpin, INPUT);
  pinMode(B1pin, INPUT);
  pinMode(B2pin, INPUT);

  //Pinos LEDs
  pinMode(LED1pin, OUTPUT);
  pinMode(LED2pin, OUTPUT);

  // Pinos stepmotor
  pinMode(ENplus_pin, OUTPUT);
  pinMode(ENminus_pin, OUTPUT);
  pinMode(DIRplus_pin, OUTPUT);
  pinMode(DIRminus_pin, OUTPUT);
  pinMode(PULplus_pin, OUTPUT);
  pinMode(PULminus_pin, OUTPUT);
}
