#include <SoftwareSerial.h>
#include <DallasTemperature.h>      // https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <avr/wdt.h>

SoftwareSerial SIM800(15, 14);            //  пины RX,TX соединяются с сим 800с
#define ONE_WIRE_BUS 16                   // A2 пин датчика DS18B20, библиотека тут https://github.com/PaulStoffregen/OneWire     
#define Open_Door     3                   // концевик дверей 
#define RESET_Pin     4                   //  НОГА  НА ТРАНЗИСТОР дёргающая reset
#define FIRST_Pin     5                   // К1 на плате  реле зажигания  реле                  обход сигналки
#define SECOND_Pin    7                   // К2 на плате реле ACC 
#define STARTER_Pin   8                   // К3 на плате реле СТАРТЕРА
#define Lock_Pin      9                   // К4 на плате пульт  на кнопку "заблокировать дверь"
#define Unlock_Pin    10                  // К5 на плате на пульт  на кнопку "разблокировать дверь"
#define K4            11                  // К6 на плате на реле K5  печка
#define SIG_Pin       12                  // К7 на плате на реле      обход сигналки
#define K6            13                  // К8 на плате при перезагрузке мк будет дёргаться...на реле К6 кресла
#define Pso_F         A3                  // 17 на плате неподписаный вывод подключен к центральному замку инфа о состоянии 
#define STOP_Pin      A4                  // 18 вход IN3 на концевик педали тормоза для отключения режима прогрева либо датчик нейтрали для ручных коробок
#define Feedback_Pin  A5                  // 19  обратная связь по реле K3, проверка на включенное зажигание делитель 100кОм и 47 кОм
#define Tacho_Pin     2                   // TACHO на плате обратная связь  проверка на ключ в замке
#define BAT_Pin       A7                  // 21  внутри платы соединен с +12, через делитель напряжения 100кОм / 47 кОм

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
/*  ----------------------------------------- НАСТРОЙКИ MQTT брокера---------------------------------------------------------   */

const char MQTT_user[10] = "00000000";      // api.cloudmqtt.com > Details > User
const char MQTT_pass[15] = "00000000";  // api.cloudmqtt.com > Details > Password
const char MQTT_type[15] = "MQIsdp";        // тип протокола НЕ ТРОГАТЬ !
const char MQTT_CID[15] = "a0000000o";        // уникальное имя устройства в сети MQTT
String MQTT_SERVER = "m20.cloudmqtt.com";   // api.cloudmqtt.com > Details > Server  сервер MQTT брокера
String PORT = "000000";                      // api.cloudmqtt.com > Details > Port    порт MQTT брокера НЕ SSL !



/*----------------------------------------- ИНДИВИДУАЛЬНЫЕ НАСТРОЙКИ !!!---------------------------------------------------------   */
String call_phone =  "+79520000000";       // телефон входящего вызова  для управления DTMF

String APN = "internet.tele2.ru";

/*  ----------------------------------------- ДАЛЕЕ НЕ ТРОГАЕМ ---------------------------------------------------------------   */

float Vstart = 13.00;                       // порог распознавания момента запуска по напряжению
String pin = "";                            // строковая переменная набираемого пинкода
float TempDS[11];                           // массив хранения температуры c рахных датчиков
float Vbat, V_min;                          // переменная хранящая напряжение бортовой сети
float m = 64.50;                            // делитель для перевода АЦП в вольты для резистров 39/11kOm 65.01=12.48 66.01=12.29
unsigned long Time1, Time2 = 0;
int Timer, inDS, count, error_CF, error_C;
int interval = 2;                           // интервал oтправки данных на сервер после загрузки ардуино
bool heating = false;                       // переменная состояния режим прогрева двигателя
bool ring = false;                          // флаг момента снятия трубки
bool broker = false;                        // статус подклюлючения к брокеру
bool Security = false;                      // состояние охраны после подачи питания

void setup() {
  wdt_enable (WDTO_8S);
  pinMode(FIRST_Pin,  OUTPUT);
  pinMode(SECOND_Pin,   OUTPUT);
  pinMode(STARTER_Pin,  OUTPUT);
  pinMode(Lock_Pin,     OUTPUT);
  pinMode(Unlock_Pin,   OUTPUT);
  pinMode(SIG_Pin,      OUTPUT);
  pinMode(K4,           OUTPUT);
  pinMode(RESET_Pin,    OUTPUT);
  pinMode(Feedback_Pin,  INPUT);
  pinMode(K6,           OUTPUT);
  pinMode(Open_Door,     INPUT);                      //  обратная связь на открытие двери Open door alarma

  delay(100);
  Serial.begin(9600);                       //скорость порта

  SIM800.begin(9600);                       //скорость связи с модемом

  //Serial.println("MQTT |02/02/2019");                 // 15.12.2018

  delay(100);
  SIM800_reset();
}

void loop()
{

  wdt_reset();

  if (SIM800.available()) {
    resp_modem();                                    // если что-то пришло от SIM800 в Ардуино отправляем для разбора
  }
  if (Serial.available()) {
    resp_serial();                                 // если что-то пришло от Ардуино отправляем в SIM800
  }
  if (millis() > Time2 + 60000) {                   //отсчёт времени 1 минута для отправки данных на сервер
    Time2 = millis();
    if (Timer > 0) {
      Timer--;
    }
  }

  if (millis() > Time1 + 10000) {                   // выполняем функцию detection () каждые 10 сек
    Time1 = millis(), detection();
  }
  if (heating == true &&  analogRead(STOP_Pin) > 500) {                  // проверка датчика нейтрали
    heatingstop();
  }
}

void enginestart()     // программа запуска двигателя
{
  // wdt_disable();              /*если на запуск двигателя нужно больше 8 сек то раскоментировать ватчдог в начале и в конце функции */

  if (digitalRead(Feedback_Pin) == LOW && analogRead(STOP_Pin) < 500) {                  // считываем состояние обратной связи если зажигание выключено и ручник задран                                                                         // если на входе STOP_Pin HIGH то забываем про пуск двигателя
    Timer = 20 ;                                                 // устанавливаем таймер на 10 минут
    digitalWrite(SIG_Pin, HIGH), delay (100);                    // включаем реле сигналки
    int StTime  = map(TempDS[1], 20, -20, 700, 7000);            // Задаем время работы стартера в зависимости т температуры
    StTime = constrain(StTime, 700, 6000);                       // ограничиваем нижний и верхний диапазон работы стартера от 0,7 до 6 сек.
    // Serial.println("Еgnition. ON");
    digitalWrite(FIRST_Pin, HIGH),   delay (2000);               // включаем реле первого положения замка зажигания, ждем 1 сек.
    //digitalWrite(SECOND_Pin,    HIGH),   delay (2000);           // включаем зажигание, и выжидаем 4 сек.
    if (TempDS[0] < -15 ) {
      digitalWrite(FIRST_Pin, LOW),   delay(500);                // если температура ниже -15 градусов, дополнителльно выключаем
      digitalWrite(FIRST_Pin, HIGH),   delay(3000);              // и снова включаем зажигание для прогрева свечей на дизелях
    }
   // Serial.println("ST. ON") ;
    digitalWrite(STARTER_Pin, HIGH), delay(StTime);              // включаем и выключаем стартер на время установленное ранее
    digitalWrite(STARTER_Pin, LOW);
    delay (2000) ;
 //   Serial.println("ST. OFF") ;
  }
  if (VoltRead() > Vstart) {                                  // проверяем идет ли зарядка АКБ
    heating = true     ;
    SIM800.println("AT+CIPSEND"),  delay (200);
    MQTT_PUB("C5/engine",   heating ? "start" : "stop");          // если двиг стартанул тут же отправляем данные
    MQTT_FloatPub("C5/timer",    Timer, 0);
    SIM800.write(0x1A);
   // Serial.println("Engine work");
  }
  else {
    heatingstop();
   // Serial.println(" Generator error ");

  }
  //  wdt_enable (WDTO_8S);
  // Serial.println("OUT");
}

float VoltRead() {
  float ADCC = analogRead(BAT_Pin);
  ADCC = ADCC / m ;
 // Serial.print("АКБ: "), Serial.print(ADCC), Serial.println("V");
  if (ADCC < V_min) {
    V_min = ADCC;
    return (ADCC);
  }
}

void heatingstop() {                                // программа остановки прогрева двигателя
  digitalWrite(SECOND_Pin,  LOW), delay     (100);
  digitalWrite(FIRST_Pin, LOW),     delay (100);
  digitalWrite(SIG_Pin, LOW),        delay  (100);
  digitalWrite(K4, LOW);
  digitalWrite(K6, LOW);
  heating = false, Timer = 0;
  SIM800.println("AT+CIPSEND"),  delay (200);
  MQTT_PUB("C5/engine",   heating ? "start" : "stop");                   // если двиг не стартанул или заглох тут же  отправляем данные
  MQTT_FloatPub("C5/timer",    Timer, 0);
  SIM800.write(0x1A);
  Serial.println("OUT");
}

void detection() {                                         // условия проверяемые каждые 10 сек
  Vbat = VoltRead();                                         // замеряем напряжение на батарее
  Serial.print("Инт:"), Serial.println(interval);
  inDS = 0;
  sensors.requestTemperatures();                             // читаем температуру с трех датчиков
  while (inDS < 10) {
    TempDS[inDS] = sensors.getTempCByIndex(inDS);            // читаем температуру
    if (TempDS[inDS] == -127.00) {
      TempDS[inDS] = 80;                                     // пока не доберемся до неподключенного датчика
      break;
    }
    inDS++;
  }
  
  for (int i = 0; i < inDS; i++) Serial.print("Temp"), Serial.print(i), Serial.print("= "), Serial.println(TempDS[i]);
  Serial.println("")  ;

  if (heating == true && VoltRead() < Vstart && Timer < 1) {                  //Остановка прогрева если нет зарядки
    heatingstop();
  }

  if (heating == true && Timer < 1) {                          // остановка прогрева если закончился отсчет таймера
    heatingstop();
  }

  if (heating == true && TempDS[1] > 90) {                      // остановить прогрев если температура выше 90 град
    heatingstop();
  }

  if (heating == true && TempDS[1] > 30 && TempDS [2] <  10) {   // включаем отопитель если на улице холодно и двигатель прогрет
    digitalWrite(K4, HIGH);
  }

  if (heating == true && TempDS[2] < 10) {                     // включаем подогрев сидений если в салоне прохладно
    digitalWrite(K6, HIGH);
  }

  interval--;
  if (interval < 1) {
    interval = 6, SIM800.println("AT+SAPBR=2,1"), delay(200);  // подключаемся к GPRS
  }
}
void resp_serial() { // ---------------- ТРАНСЛИРУЕМ КОМАНДЫ из ПОРТА В МОДЕМ ----------------------------------

  String at = "";
  int k = 0;
  while (Serial.available())k = Serial.read(), at += char(k), delay(1);
  SIM800.println(at), at = "";
}

void  MQTT_FloatPub(const char topic[15], float val, int x) {
  char st[10];
  dtostrf(val, 0, x, st), MQTT_PUB (topic, st);
}

void MQTT_CONNECT() {
  SIM800.println("AT+CIPSEND"), delay (200);
  SIM800.write(0x10);                                                              // маркер пакета на установку соединения
  SIM800.write(strlen(MQTT_type) + strlen(MQTT_CID) + strlen(MQTT_user) + strlen(MQTT_pass) + 12);
  SIM800.write((byte)0), SIM800.write(strlen(MQTT_type)), SIM800.write(MQTT_type); // тип протокола
  SIM800.write(0x03), SIM800.write(0xC2), SIM800.write((byte)0), SIM800.write(0x3C); // просто так нужно
  SIM800.write((byte)0), SIM800.write(strlen(MQTT_CID)),  SIM800.write(MQTT_CID);  // MQTT  идентификатор устройства
  SIM800.write((byte)0), SIM800.write(strlen(MQTT_user)), SIM800.write(MQTT_user); // MQTT логин
  SIM800.write((byte)0), SIM800.write(strlen(MQTT_pass)), SIM800.write(MQTT_pass); // MQTT пароль
  MQTT_PUB("C5/status",  "Reset0");
  MQTT_PUB("C5/status",    "fas0");
  MQTT_SUB("C5/comand");                                                          // пакет подписки на присылаемые команды
  MQTT_SUB("C5/settimer");                                                        // пакет подписки на присылаемые значения таймера
  SIM800.write(0x1A),  broker = true;                                              // маркер завершения пакета
}

void  MQTT_PUB(const char MQTT_topic[15], const char MQTT_messege[15]) {      // пакет на публикацию

  SIM800.write(0x30), SIM800.write(strlen(MQTT_topic) + strlen(MQTT_messege) + 2);
  SIM800.write((byte)0), SIM800.write(strlen(MQTT_topic)), SIM800.write(MQTT_topic); // топик
  SIM800.write(MQTT_messege);
}                                                  // сообщение

void  MQTT_SUB(const char MQTT_topic[15]) {                                     // пакет подписки на топик

  SIM800.write(0x82), SIM800.write(strlen(MQTT_topic) + 5);                       // сумма пакета
  SIM800.write((byte)0), SIM800.write(0x01), SIM800.write((byte)0);                // просто так нужно
  SIM800.write(strlen(MQTT_topic)), SIM800.write(MQTT_topic);                      // топик
  SIM800.write((byte)0);
}

void resp_modem() {  //------------------ АНЛИЗИРУЕМ БУФЕР ВИРТУАЛЬНОГО ПОРТА МОДЕМА------------------------------
  String at = "";
  int k = 0;
  while (SIM800.available())k = SIM800.read(), at += char(k), delay(1);
  Serial.println(at);

  if (at.indexOf("+CLIP: \"" + call_phone + "\",") > -1) {
    delay(200), SIM800.println("ATA"), ring = true;
  }
  else if (at.indexOf("+DTMF: ") > -1) {
    String key = at.substring(at.indexOf("") + 9, at.indexOf("") + 10);
    pin = pin + key;
    if (pin.indexOf("*") > -1)pin = "";
  }

  else if (at.indexOf("SMS Ready") > -1 || at.indexOf("NO CARRIER") > -1) {      // Активируем АОН и декодер DTMF
    SIM800.println("AT+CLIP=1;+DDET=1");
  }

  /*  -------------------------------------- проверяем соеденеиние с ИНТЕРНЕТ, конектимся к серверу------------------------------------------------------- */
  else if (at.indexOf("+SAPBR: 1,3") > -1) {
    SIM800.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""), delay(200);
  }
  else if (at.indexOf("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\r\nOK") > -1) {
    SIM800.println("AT+SAPBR=3,1, \"APN\",\"" + APN + "\""), delay (500);
  }
  else if (at.indexOf("AT+SAPBR=3,1, \"APN\",\"" + APN + "\"\r\r\nOK") > -1 ) {
    SIM800.println("AT+SAPBR=1,1"), interval = 2 ; // устанавливаем соеденение
  }
  else if (at.indexOf("+SAPBR: 1,1") > -1) {
    delay (200),  SIM800.println("AT+CIPSTART=\"TCP\",\"" + MQTT_SERVER + "\",\"" + PORT + "\""), delay (1000);
  }
  else if (at.indexOf("CONNECT FAIL") > -1) {
    SIM800.println("AT+CFUN=1,1"), error_CF++, delay(1000), interval = 3 ; // костыль 1
  }
  else if (at.indexOf("CLOSED") > -1 ) {
    SIM800.println("AT+CFUN=1,1"), error_C++,  delay(1000), interval = 3 ; // костыль 2
  }
  else if (at.indexOf("+CME ERROR:") > -1 ) {
    error_CF++;                                                   // костыль 4
    if (error_CF > 3) {
      error_CF = 0, delay(1000);
      Serial.println("Restart");
      digitalWrite(RESET_Pin, HIGH );
      SIM800.println("AT+CFUN=1,1");
    }
  }
  else if (at.indexOf("CONNECT OK") > -1) {
    MQTT_CONNECT();
  }

  else if (at.indexOf("+CIPGSMLOC: 0,") > -1) {
    String LOC = at.substring(26, 35) + "," + at.substring(16, 25);
    SIM800.println("AT+CIPSEND"), delay(200);
    MQTT_PUB ("C5/ussl", LOC.c_str()), SIM800.write(0x1A);
  }

  else if (at.indexOf("+CSQ:") > -1) {
    String RSSI = at.substring(at.lastIndexOf(":") + 1, at.lastIndexOf(",")); // +CSQ: 31,0
    SIM800.println("AT+CIPSEND"), delay (200);
    MQTT_PUB ("C5/rssi", RSSI.c_str()), SIM800.write(0x1A);
  }
  else if (at.indexOf("ALREAD") > -1) {
    SIM800.println("AT+CIPSEND"), delay (200);
    MQTT_FloatPub("C5/ds0",      TempDS[2], 2);                     
    MQTT_FloatPub("C5/vbat",     Vbat, 2);                             
 //     MQTT_PUB("C5/security", Security ?  "lock1" : "lock0");
    if (heating == true) {
      MQTT_FloatPub("C5/timer",    Timer, 0);
  //    MQTT_PUB("C5/engine",   heating ? "start" : "stop");
      MQTT_FloatPub("C5/ds2",      TempDS[0], 2);
      MQTT_FloatPub("C5/ds1",      TempDS[1], 2);
    }
    SIM800.write(0x1A);
  }
  else if (at.indexOf("C5/comandReset1", 4) > -1) {               // рестарт контролллера
    delay(300);                                                   // Serial.println("Restart"),
    digitalWrite(RESET_Pin, HIGH );
  }

  else if (at.indexOf("C5/comandlock1", 4) > -1) {                     //// команда постановки на охрану и включения прерывания по концевикам дверей
    if (heating == true) {                              // если прогрев вкл то отключаем сигу
      digitalWrite(SIG_Pin , HIGH);
    }
    blocking(1);
    attachInterrupt(1, callback, LOW);
    delay(500);                                       // задержка для того чтобы сигналка успела отдуплиться
    if ( analogRead ( Pso_F  ) > 300) {           // 450 расчёт для дениса для меня 300
   //   Security = false ;                       если центральный замок не закрылся посылаем инфу
      SIM800.println("AT+CIPSEND"), delay (200);
      MQTT_PUB("C5/security",  "lock0");
      SIM800.write(0x1A);
     // Serial.println( "не закрылась");
    }
  }

  else if (at.indexOf("C5/comandlock0", 4) > -1) {          // команда снятия с охраны и отключения прерывания на концевик дверей
    blocking(0);
    detachInterrupt(1);
    delay(1500);
    if ( analogRead ( Pso_F  ) < 300) {                     // если центральный замок не открылся посылаем инфу
  //    Security = true;                                    соответственно она снова становится на охрану наверное для открытии машины этот код не нужен
      SIM800.println("AT+CIPSEND"), delay (200);
      MQTT_PUB("C5/security", "lock1" );
      SIM800.write(0x1A);
    //  Serial.println( "не открылась");
    }

    if (heating == true) {                            // если прогрев вкл то отключаем сигу
      digitalWrite(SIG_Pin , LOW);
    }
  }
  else if (at.indexOf("C5/settimer", 4) > -1) {
    Timer = at.substring(at.indexOf("") + 15, at.indexOf("") + 18).toInt();
  }
  else if (at.indexOf("C5/comandfas1", 4) > -1) {
    Anti_Hijack();                             //
  }
  else if (at.indexOf("C5/comandrssi", 4) > -1) {
    SIM800.println("AT+CSQ");                             // запрос уровня сигнала
  }
  else if (at.indexOf("C5/comandlocation", 4) > -1) {
    SIM800.println("AT+CIPGSMLOC=1,1");                   // запрос локации
  }
  else if (at.indexOf("C5/comandstop", 4) > -1) {
    heatingstop();                                       // команда остановки прогрева
  }
  else if (at.indexOf("C5/comandstart", 4) > -1) {
    enginestart();                                       // команда запуска прогрева
  }

  //   Команда обнoвления

  else if (at.indexOf("C5/comandRefresh", 4) > -1) {
    SIM800.println("AT+CIPSEND"),  delay (200);
    MQTT_FloatPub("C5/ds0",      TempDS[0], 2);
    MQTT_FloatPub("C5/ds1",      TempDS[1], 2);
    MQTT_FloatPub("C5/ds2",      TempDS[2], 2);
    MQTT_FloatPub("C5/vbat",     Vbat, 2);
    MQTT_FloatPub("C5/timer",    Timer, 0);
  //  MQTT_PUB("C5/security",  Security ? "lock1" : "lock0");
  //  MQTT_PUB("C5/engine",   heating ? "start" : "stop");
    MQTT_FloatPub("C5/C",   error_C, 0);
    MQTT_FloatPub("C5/CF", error_CF, 0);
    MQTT_FloatPub("C5/uptime",   millis() / 3600000, 0);
    SIM800.write(0x1A);
    interval = 6;                   // швырнуть данные на сервер и ждать 60 сек
    at = "";                             // Возвращаем ответ можема в монитор порта , очищаем переменную

  }
  if (pin.indexOf("123") > -1) {
    pin = "", enginestart();
  }
  else if (pin.indexOf("777") > -1) {
    pin = "", SIM800.println("AT+CFUN=1,1");          // костыль 3
  }
  else if (pin.indexOf("789") > -1) {
    pin = "", delay(150), SIM800.println("ATH0"), heatingstop();
  }
  else if (pin.indexOf("#") > -1) {
    pin = "", SIM800.println("ATH0");
  }
  else if (pin.indexOf("000") > -1) {
    pin = "", delay(100);       // pin = "", Serial.println("Restart"), delay(1000);
    digitalWrite(RESET_Pin, HIGH);                  // костыль 5
  }
}

/* функция дергания реле блокировки/разблокировки дверей с паузой "удержания кнопки" в 0,5 сек.*/
void blocking (bool st) {
  digitalWrite(st ? Lock_Pin : Unlock_Pin, HIGH), delay(300), digitalWrite(st ? Lock_Pin : Unlock_Pin, LOW ), delay(300),
               Security = st;         //  Serial.println(st ? "На охране" : "Открыто");

}

// Принудительная блокировка двигателя
void Anti_Hijack() {
  digitalWrite(SIG_Pin ,  HIGH);
  digitalWrite(Lock_Pin ,  HIGH), delay(300);
  digitalWrite(SIG_Pin ,  LOW);
  if (heating = true) {
    heatingstop();
  }
  // Serial.println(" ARMED ");
}
void SIM800_reset() {
  SIM800.println("AT+CFUN=1,1");                  // перезагрузка модема
}
void callback() {
  SIM800.println("ATD" + call_phone + ";"),    delay(3000); // обратный звонок при появлении напряжения на входе IN1
}
