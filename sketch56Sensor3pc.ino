//---Тема, в которой обсуждается эта разработка: goo.gl/ijbJho---

// Параметры в структуре ИЗМЕНЕНЫ и перезапишут ваши данные при заливке в Ардуино! Если вы обновили данные в дефолтном профиле, измените значение EEPROM_No_Write
// Рампа для низа, Верха и коррекция общей мощности по отклонению температуры платы.

// управление включением/выключением пайки и переключением профилей с РС
// сохранение и загрузка профилей с/на диск PC
// не используется экран, только ПК
// ПИД по измерению dT
// добавлен 3 пид + integral antiwindup для всех 3 пид.

// Защита !!!!!
// 1. Перегрев платы выше max_temperature_pcb (240гр) автоматически останавливает выполнение профиля
// 2. Если этап преднагрева перед стартом профиля длится более max_warmup_duraton (300сек \ 5мин) то выполнение профиля останавливается.
// 3. Автопауза при выполнении профиля - при отклонении температуры более чем max_pcb_delta выполнение профиля приостанавливается на hold_lenght секунд
// 4. Мягкий старт нагревателей 5 секунд 3% мощности перед стартом профиля (можно изменить в переменной)


// Шаги ВИ,НИ, Платы не привязаны друг к другу по времени. Шаг для каждого может быть произвольной длинны до 250 секунд. Максимум 29 шагов (30 точек графика)
// Для подготовки профиля есть специальный файл Excel: Plot_Edit.xlsx


// Словарь команд для ручного управления. Большинство дублируется кнопками в Интерфейсе Port Plotter:

// E - прервать текущее задание = кнопка STOP !!!
// S - Начать выполнение профиля // Если указана темпетарура то нагрев до фикс температуры.
// J - Старт с соответствующей точки профиля если плата горячая.

// H - Включить HOLD - приостанавливаем выполнение профиля и переходим на удержание температуры платы.
// U - Прибавить температуру во время MANUAL HOLD.  При нажатии во время пайки скетч встаёт на паузу автоматически
// D - Убавить температуру во время MANUAL HOLD. При нажатии во время пайки скетч встаёт на паузу автоматически

// N+цифра - сохранить профиль в EEPROM.
// P - загружает профиль из EEPROM в оперативную память и выводит его в COM)
// R - следующий профиль
// L - предыдущий профиль

// M -  M 1 - не используется, M 2,150 - установить температуру НИ 150 и греть до отмены / М 3,150 - установить температуру платы и греть до отмены
// Q - Q 1,100 - установить мощность ВИ 100, /  Q 2,30 установить мощность НИ 30 и греть до отмены.


// TODO

// 1. Вывод всех ошибок в Буфер данных в 2 потока: Статусы, Информация для графиков.
// 2. +\-Отправка данных на пк: во время выполнения профиля: температура, мощности, время по графику, время факт. Включена ли пауза. Синхронизация данных о профиле в начале программы
// 3. +++ Получение данных с ПК: Профиль, команды вкл\выкл\пауза, переключение профилей. данные для отправки генерируются в эксельке.
// 4. +++ Режим удержания температуры платы на хх градусов - Включение паузы вручную в нужный момент
// 5. +++ Режим удержания температуры нагревателя на хх градусов
// 6. Автотюнинг ПИД
// 7. +++ DONE Исправить Перерегулирование ПИД платы на этапе Warmup ++++++++++++++ FIXED AntiWindup
// 8. +++ Исправить активацию автопаузы на этапе остывания (на последних 2 шагах платы отключена автопауза.) FIXED (?)
// 9. +++ Подумать как вносить коррекцию с учетом инерционности нагревателей. // Правильно отстроил ПИД платы
// 10. Метки времени \ температуры для подачи сигналов - Доп канал для статусов: 0 - не выполняется - 10 пауза - 100 Бип - 250 ошибка
// 11. +++ В режиме паузы ввести возможность корректировать температуру НИ\Платы. Если во время паузы температура платы ушла за макс отклонение, то после отключения паузы - перезапустить профиль.
// 12. +++ Изменить логику HotStart чтобы он явно запускался отдельной кнопкой. По умолчанию отключен. Убрать из профиля.
// 13. +++ Статусы отправляемые через ком (Idle, Hold, Pause, Sound, Error) (@0)
// 14. +++ Исправить логику Автопаузы при слишком быстром нагреве платы.
// 15. +++ Добавить команды управления мощностью нагревателей. (Q 1 = ВИ , Q 2 = НИ).
// 16. Исправить ПИД платы. Сейчас большой перелёт. Продумать алгоритм его настройки
// 17. Исправить логику удержания температуры платы \ НИ при переходе из пайки по профилю к ручному управлению
// 18. Возможность изменить делители для пвраметров ПИД
// 19. Проверить корректность работы UP\DOWN в разных режимах
// 20. Проверить логику HOTSTART при переходах из IDLE, HOLD_MANUAL, RUN. Возможно стоит использовать прыжок если идет перегрев платы относительно графика.
// 21. Определиться с форматом звуковых сигналов, и их выводом в плоттер \ на буззер
// 22. Привести к одному формату отправку служебной информации плоттером в порт (Serial.print, sprintf (buf,.., s->print(.. )
// 23. +++ Исправлена логика ручного управления температурой HOLD_MANUAL. ВИ принудительно отключается.

/**  Список статусов
 *
 *   @0 IDLE - режим ожидания
 *   @1 WARMUP - Этап Преднагрева нагревателей и платы до стартовых температур профиля
 *   @2 RUN - Собственно этап пайки по профилю
 *   @3 HOLD AUTO - включена автопауза
 *   @4 HOLD MANUAL - включена ручная пауза
 *   @5 COMPLETE - профиль выполнен
 *   @6 SIGNAL (Sound type)
 *
 *    Serial.println("@0;");
 */

#include <EEPROM.h>

// Выводить ли диагностические данные в СОМ----------------

#define show_diagnostics_com true;
// #define show_diagnostics_com_deep false;

// Секция ПИД----------------------------------------------

float integra1, integra2, integra3;       // интегральные составляющие ВИ и НИ и платы
float e1, p1, d1, e2, p2, d2, e3, p3, d3; // ошибка регулирования, П-составляющая, Д-составляющая для ВИ, НИ и платы соответственно

int SetPoint_Top;           // Задание по графику для Пид ВИ на текущую секунду
int SetPoint_Bottom;        // Задание по графику для Пид НИ на текущую секунду
int SetPoint_Pcb;           // Задание по графику для Пид ПЛАТЫ на текущую секунду
double Input1;              // Показания термопары ВИ в текущую секунду с фильтрацией
byte Output1;               // Выход ПИД ВИ - мощность на текущую секнду
double Input2;              // Показания термопары НИ в текущую секунду с фильтрацией
byte Output2;               // Выход ПИД НИ - мощность на текущую секнду
double Input3;              // Показания термопары Платы в текущую секунду с фильтрацией
float Output3;              // Выход ПИД Платы - коррекция уставки ВИ и НИ на текущую секнду
int tc1, tc2, tc3;          // Показания термопар равны input 1-2-3 целочисленные
int temp_correction_top;    // значение коррекции температуры ВИ регулируемое ПИД платы
int temp_correction_bottom; // значение коррекции температуры НИ регулируемое ПИД платы

#define i_min 0.0   // минимум И составляющей
#define i_max 100.0 // максимум И составляющей

// Переменные которые можно настраивать под себя--------------------------------------------

#define SENSOR_SAMPLING_TIME 250  // время чтения температуры и пересчёта ПИД(милисекунды)
#define COM_PORT_SAMPLE_TIME 1000 // время передачи данных на РС

const int max_temperature_pcb = 240; // Предел температуры платы. Если термопара покажет больше профиль автоматически будет остановлен.
const int max_warmup_duraton = 300;  // Максимальная длительность выхода на стартовые температуры профиля.
const byte PreHeatPower = 3;         // мощность на этапе преднагрева НИ в процентах
const byte PreHeatTime = 5;          // время преднагрева НИ в Секундах
const byte profile_steps_pcb = 10;   // количество шагов профиля Платы

byte max_profiles = 6; // Максимальное число профилей  !!!общий размер не должен превышать объем EEPROM!!!

//-----------------------------------------------------------

// секция алгоритма Брезенхема---Распределение мощности равномерно на 1 секунду----------------
int er1 = 1;
int er2 = 1;
int reg1;
int reg2;
boolean out1;
boolean out2;
//-----------------------------------------------------------

// Секция кнопок, пинов подключеня-----------------------------------------
#define RelayPin1 7 // назначаем пин "ВЕРХНЕГО" нагревателя
#define RelayPin2 6 // назначаем пин "НИЖНЕГО" нагревателя

// Выходы реле
#define P1_PIN 9           // назначаем пин реле 1
#define P2_PIN 10          // назначаем пин реле 2
#define P3_PIN 11          // назначаем пин реле 3
#define P4_PIN 12          // назначаем пин реле 4
byte buzzerPin = 8;        // пин пищалки
byte cancelSwitchPin = 21; // пин кнопки отмена или назад

// назначаем пины усилителя термопары MAX6675 "ВЕРХНЕГО" нагревателя   clk=sck cs=cs do=so
byte thermoCLK1 = 14; //=sck
byte thermoCS1 = 15;  //=cs //separately
byte thermoDO1 = 16;  //=so

// назначаем пины усилителя термопары MAX6675 "НИЖНЕГО" нагревателя clk=sck cs=cs do=so
byte thermoCLK2 = 14; //=sck
byte thermoCS2 = 17;  //=cs //separately
byte thermoDO2 = 16;  //=so

// назначаем пины усилителя термопары MAX6675 "ПЛАТЫ" clk=sck cs=cs do=so
byte thermoCLK3 = 14; //=sck
byte thermoCS3 = 19;  //=cs //separately
byte thermoDO3 = 16;  //=so

// Секция профиля----------------------------------------------------------
struct pr
{                                  // основные поля профиля
  int time_step_top[30];           // время от старта профиля до шага ВИ //потолок 9 часов
  int temperature_step_top[30];    // температура в НАЧАЛЕ текущего шага ВИ
  int time_step_bottom[30];        // время от старта профиля до шага НИ
  int temperature_step_bottom[30]; // температура в НАЧАЛЕ текущего шага НИ
  int time_step_pcb[10];           // время от старта профиля до шага Платы
  int temperature_step_pcb[10];    // температура в НАЧАЛЕ текущего шага Платы
  // int beep_time[10];            //время через которое подать сигнал
  // int beep_temp[10];            //Температура платы при которой подать сигнал
  byte profile_steps;          // количество шагов ВИ и НИ
  byte table_size;             // размер стола
  byte kp1;                    // пропорциональный коэффициент ВИ
  byte ki1;                    // интегральный коэффициент     ВИ
  byte kd1;                    // дифференциальный коэффициент ВИ
  byte kp2;                    // пропорциональный коэффициент НИ
  byte ki2;                    // интегральный коэффициент     НИ
  byte kd2;                    // дифференциальный коэффициент НИ
  byte kp3;                    // пропорциональный коэффициент Платы
  byte ki3;                    // интегральный коэффициент     Платы
  byte kd3;                    // дифференциальный коэффициент Платы
  byte max_correction_top;     // максимальная коррекция температуры ВИ в обе стороны
  byte max_correction_bottom;  // максимальная коррекция температуры НИ в обе стороны
  byte max_pcb_delta;          // максимальный недогрев Платы после которого включаеся Пауза
  byte hold_lenght;            // длительность паузы в секундах
  byte participation_rate_top; // коэфициент участия ВИ (от 0 - догреваем только низом, до 100 догреваем только верхом)(при отклонении температыры платы от профиля)
  char alias[20];
};

int SizeProfile = sizeof(pr); // длинна поля данных
pr profile;                   // структура для параметров

// Переменные для рассчета шагов по графику-------------------------------------------------------

byte max_correction_total;

byte ParticipationRateBottom;           // коэфициент участия низа. При включении вычисляется автоматом на основе profile.participation_rate_top
byte CurrentStepTop;                    // номер шага профиля ВИ
byte CurrentStepBottom;                 // номер шага профиля НИ
byte CurrentStepPcb;                    // номер шага профиля Платы
byte DurationCurrentStepTop;            // Длительность текущего шага ВИ
byte DurationCurrentStepBottom;         // Длительность текущего шага НИ
byte DurationCurrentStepPcb;            // Длительность текущего шага Платы
byte TimeCurrentStepTop;                // секунд от начала текущего шага ВИ
byte TimeCurrentStepBottom;             // секунд от начала текущего шага НИ
byte TimeCurrentStepPcb;                // секунд от начала текущего шага Платы
int TemperatureDeltaTop;                // дельта температур текущего и следующего шага ВИ
int TemperatureDeltaBottom;             // дельта температур текущего и следующего шага НИ
int TemperatureDeltaPcb;                // дельта температур текущего и следующего шага Платы
int TempSpeedTop;                       // Рассчитанная скорость роста температуры ВИ (может быть положительная и отрицательная) умножена на 100
int TempSpeedBottom;                    // Рассчитанная скорость роста температуры НИ (может быть положительная и отрицательная) умножена на 100
int TempSpeedPcb;                       // Рассчитанная скорость роста температуры Платы (может быть положительная и отрицательная) умножена на 100
unsigned int CurrentProfileSecond;      // Cчетчик времени по профилю
unsigned long CurrentProfileRealSecond; // Какая сейчас секунда профиля с момента запуска (с учетом пауз, прогрева )
int PcbDelta, BottomDelta, TopDelta;    // Отклонение температуры Платы от профиля
int manual_temperature;                 // Переменная для ручной регулировки температуры
int manual_power;                       // Переменная для ручной регулировки мощности

//------------------------------------------------------------------------

// Секция флагов-----------------------------------------------------------
bool manual_temp_changed = false;    // Флаг что мы вручную изменили температуру
bool manual_temperature_pcb = false; // Флаг что мы из Idle рулим темп платы

bool ManualHoldEnable = false; // переменная для ручного включения HOLD (удержания температуры нагревателей)
byte AutoHoldCounter = 0;      // счетчик сколько еще держать Hold
bool jump_if_pcb_warm = false; // начинать выполнение профиля с ближайшей подходящей точки графика платы (горячий старт)
bool blockflag = false;
//------------------------------------------------------------------------

// Секция переменных общего назначения-------------------------------------

int cancelSwitchState = 0;
long ms_button = 0;                 // время при котором была нажата кнопка
long previousMillis;                // это для счетчиков
unsigned long NextReadTemperatures; // переменная для обновления текущей температуры
byte Secs = 0;                      // Счетчик количества прерываний от детектра ноля. каждые 100 отправляет данные на ПК
long nextRead2 = 0;
byte currentProfile = 1; // номер профиля при включении (в вычислениях используется currentProfile-1)

//------------------------------------------------------------------------

// секция ввода/вывода для ПЭВМ-----------------------------------------------
char buf[60];  // буфер вывода сообщений через сом порт
 
//---------------------------------------------------------------------------

// these are the different states of the sketch. We call different ones depending on conditions
//  ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE : byte
{
  REFLOW_STATE_IDLE,
  REFLOW_STATE_INIT,         // Анализ с какого шага начинать профиль, фиксация температур
  REFLOW_STATE_PRE_HEATER,   // Прогрев ВИ и НИ на мощности PreHeatPower в течение PreHeatTime секунд, ПИД отключены. Чтобы уменьшить нагрузку на управляющие элементы.
  REFLOW_STATE_WARMUP,       // Преднагрев до начальных значений температуры в первом шаге профиля ВИ, НИ, Платы. ПИД включены.
  REFLOW_STATE_RUNNING,      // Выполнение профиля ВИ, НИ, Платы. (объединенные STEP)
  REFLOW_STATE_HOLD_AUTO,    // Приостановка выполнения профиля, фиксация температур ВИ и НИ по графику, чтобы плата догрелась либо остыла (коррекция с учетом отклонения температуры платы продолжается)
  REFLOW_STATE_HOLD_MANUAL,  // Приостановка выполнения профиля вручную, фиксация температуры ПЛАТЫ, если в профиле нет задания для платы то фиксация температуры Низа.
  REFLOW_STATE_POWER_MANUAL, // Режим ручного управления мощностью для этапа тюнинга
  REFLOW_STATE_COMPLETE      // Завершение профиля. Из-за ошибки \ принудительной остановки \ успешного выполнения.
} reflowState_t;

typedef enum REFLOW_STATUS : byte // this is simply to check if reflow should be running or not
{
  REFLOW_STATUS_OFF,
  REFLOW_STATUS_ON
} reflowStatus_t;
reflowStatus_t reflowStatus;
reflowState_t reflowState;

byte prev_reflowState = 0; // для сохранения состояния перед переходом в Hold

// Command Parser area ------------Parser Originally written by Christopher Wang aka Akiba.----------------------
//  ***** TYPE DEFINITIONS *****

#define MAX_CMD_SIZE 160

// command line structure
typedef struct _cmd_t
{
  char *cmd;
  void (*func)(int argc, char **argv);
  struct _cmd_t *next;
} cmd_t;

void cmdInit(Stream *);
void cmdPoll();
void cmdAdd(const char *name, void (*func)(int argc, char **argv));

Stream *cmdGetStream(void);
uint32_t cmdStr2Num(char *str, uint8_t base);

// command line message buffer and pointer
static uint8_t msg[MAX_CMD_SIZE];
static uint8_t *msg_ptr;

// linked list for command table
static cmd_t *cmd_tbl_list, *cmd_tbl;

// text strings for command prompt (stored in flash)

static Stream *stream;

// int led_pin = 50;
// bool led_blink_enb = false;
// int led_blink_delay_time = 1000;
// int pwm_pin = 51;

// END Command Parser area ---------------Parser Originally written by Christopher Wang aka Akiba.---------------------

void printProfileNumber()
{
  blockflag=true;
  Serial.print("!profile: ");
  Serial.print(currentProfile);
  Serial.println(";");
  Serial.print("!aliasprofile: ");
  Serial.print(profile.alias);
  Serial.println(";");
  blockflag=false;
}

void loadProfile() // this function loads whichever profile currentProfile variable is set to  TODO исправить под новый формат профиля
{

  EEPROM.get((currentProfile - 1) * SizeProfile, profile); // читаем профиль с EEPROM в память

  ParticipationRateBottom = 100 - profile.participation_rate_top; // считаем коэфициент участия низа
  max_correction_total = profile.max_correction_top + profile.max_correction_bottom;

  delay(100);
  printProfileNumber();
  // aliasprofile();

  return;
}

void SaveProfile()
{ //  сохранение текущего профиля в позицию currentProfile
  EEPROM.put((currentProfile - 1) * SizeProfile, profile);
}

void setup()
{
  Serial.begin(115200);
#ifdef show_diagnostics_com

  // Serial.begin(9600);
  Serial.println("Setup START"); // DIAGNOSTICS
#endif

  // setup reley pins as outputs
  pinMode(P1_PIN, OUTPUT);
  pinMode(P2_PIN, OUTPUT);
  pinMode(P3_PIN, OUTPUT);
  pinMode(P4_PIN, OUTPUT);
  pinMode(thermoCLK1, OUTPUT);
  pinMode(thermoCLK2, OUTPUT);
  pinMode(thermoCLK3, OUTPUT);
  pinMode(thermoDO1, INPUT);
  pinMode(thermoDO2, INPUT);
  pinMode(thermoDO3, INPUT);
  pinMode(thermoCS1, OUTPUT);
  pinMode(thermoCS2, OUTPUT);
  pinMode(thermoCS3, OUTPUT);
  pinMode(cancelSwitchPin, INPUT_PULLUP); // подключен подтягивающий резистор
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);

  initEeprom();
  loadProfile(); // вызов функции loadProfile для загрузки данных профиля из eeprom
  NextReadTemperatures = millis();
  attachInterrupt(0, Dimming, RISING); // настроить порт прерывания(0 или 1) 2й или 3й цифровой пин

  Serial.println("@0;");
  reflowState = REFLOW_STATE_IDLE;

  // LIST OF COMMANDS Alias-----cmdAdd("receive_from_com", execute_command_name);---receive_from_com - текст полученный из COM -------execute_command_name - имя функции которую выполнить------------------------
  cmdInit(&Serial);

  cmdAdd("time_step_top", update_time_step_top);
  cmdAdd("temperature_step_top", update_temperature_step_top);
  cmdAdd("time_step_bottom", update_time_step_bottom);
  cmdAdd("temperature_step_bottom", update_temperature_step_bottom);
  cmdAdd("time_step_pcb", update_time_step_pcb);
  cmdAdd("temperature_step_pcb", update_temperature_step_pcb);

  cmdAdd("kp1", update_kp1);
  cmdAdd("ki1", update_ki1);
  cmdAdd("kd1", update_kd1);
  cmdAdd("kp2", update_kp2);
  cmdAdd("ki2", update_ki2);
  cmdAdd("kd2", update_kd2);
  cmdAdd("kp3", update_kp3);
  cmdAdd("ki3", update_ki3);
  cmdAdd("kd3", update_kd3);
  cmdAdd("profile_steps", update_profile_steps);
  cmdAdd("table_size", update_table_size);
  cmdAdd("max_correction_top", update_max_correction_top);
  cmdAdd("max_correction_bottom", update_max_correction_bottom);
  cmdAdd("max_pcb_delta", update_max_pcb_delta);
  cmdAdd("hold_lenght", update_hold_lenght);
  cmdAdd("participation_rate_top", update_participation_rate_top);
  cmdAdd("aliasprofile", update_aliasprofile);

  cmdAdd("U", UP);
  cmdAdd("D", DOWN);
  cmdAdd("L", LEFT);
  cmdAdd("R", RIGHT);
  cmdAdd("N", SAVE);          // Save profile from memory to position
  cmdAdd("E", CANCEL);        // Stop executing profile
  cmdAdd("H", HOLD);          // Pause executing profile and hold current PCB temperature
  cmdAdd("S", START);         // OK // Start profile execution
  cmdAdd("J", HOTSTART);      // HotStart (J) // Start profile execution
  cmdAdd("M", MANUAL_TEMP);   // Manual Temp adjustment
  cmdAdd("Q", MANUAL_POWER);  // Manual Power adjustment
  cmdAdd("P", PRINT_PROFILE); // Print profile to COM
  cmdAdd("X", BLOCK);         // Edit profile mode - no upstream data from station
  cmdAdd("W", UNBLOCK);       // Exit edit profile mode

  // End LIST OF COMMANDS Alias---------------------------------------------------------------------------------------

#ifdef show_diagnostics_com
  Serial.println("Setup END"); // DIAGNOSTICS
#endif
}

const char EEPROM_No_Write = 'm'; // <----------------- Изменить значение на любое другое, чтобы обновить профиль в EEPROM при заливке скетча

void initEeprom() // Запись в 0 позицию профиля по умолчанию
{
#ifdef show_diagnostics_com
  Serial.println("EEPROM Init START"); // DIAGNOSTICS
#endif

  if (EEPROM.read(max_profiles * SizeProfile + 1) == EEPROM_No_Write) // записываем любое значение, при включении проверяем его, если совпадает по адресу, то профиль не перезаписываем.
  {
#ifdef show_diagnostics_com
    Serial.println("EEPROM Init Skipped"); // DIAGNOSTICS
#endif                                     // чтото только для уже инициализированой памяти
  }
  else
  {
    // профиль 1:
    static int rom1[] = {// ДАННЫЕ для шагов НИЖЕ МОЖНО СФОРМИРОВАТЬ В EXCELL файле ПОДГОТОВКА ПРОФИЛЯ.xls
                         // 30 время от старта профиля до шага ВИ
                         0, 10, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300, 320, 340, 360, 380, 400, 420, 440, 460, 477, 613, 0, 0, 0, 0,
                         // 30 температур по шагам ВИ
                         84, 84, 102, 112, 122, 131, 139, 146, 161, 166, 172, 170, 169, 197, 241, 271, 302, 328, 353, 377, 399, 418, 427, 429, 421, 50, 0, 0, 0, 0,
                         // 30 время от старта профиля до шага НИ
                         0, 4, 34, 54, 74, 94, 114, 134, 154, 174, 194, 214, 234, 254, 274, 294, 314, 334, 354, 374, 394, 414, 434, 454, 474, 494, 514, 613, 0, 0,
                         // 30 температур по шагам НИ
                         256, 256, 306, 312, 318, 309, 304, 300, 300, 295, 290, 285, 274, 286, 306, 313, 314, 315, 314, 313, 311, 309, 307, 304, 296, 256, 203, 104, 0, 0,
                         // 10 время от старта профиля до шага Платы
                         0, 4, 89, 159, 212, 252, 420, 462, 477, 613,
                         // 10 температур по шагам Платы
                         79, 79, 122, 148, 161, 165, 224, 235, 235, 99

    };
    static byte rom2[] = {
        // кол-во шагов, размер стола
        27,
        3,
        // PID верха,
        31,
        23,
        37,
        // PID низа,
        6,
        23,
        8,
        // PID Платы,
        5,
        15,
        12,
        // максимальная коррекция температуры ВИ, НИ
        30,
        80,
        // максимальное отклонение температуры Платы после которого включаеся Авто Пауза (0-250)
        5,
        // длительность паузы в секундах
        5,
        // коэфициент участия ВИ (0 - догреваем только низом\ 100 - только верхом)
        30,

    };

    static char aliast[20] = "Default Profile";
    EEPROM.put(0, rom1); // TODO добавить смещение записи
    EEPROM.put(280, rom2);
    EEPROM.put(296, aliast);

    EEPROM.write(max_profiles * SizeProfile + 1, EEPROM_No_Write); // Запись 

    // for (int i = SizeProfile+1; i < max_profiles * SizeProfile; i++){EEPROM.write(i, 0);}

#ifdef show_diagnostics_com
    Serial.print("EEPROM Profile 1 Updated. Size ");
    Serial.println(SizeProfile); // DIAGNOSTICS
#endif
  }
}

void loop()
{

  cmdPoll(); // Получить команды из ком порта если есть

  // Считываем состояние кнопок управления для обычных кнопок
  cancelSwitchState = !digitalRead(cancelSwitchPin);

  unsigned long currentMillis = millis();

  if (cancelSwitchState == HIGH && (millis() - ms_button) > 100) // Обработка нажатия start.
    if (reflowState == REFLOW_STATE_IDLE && (millis() - ms_button) > 2000)
    {
      ms_button = millis();
      START(0, 0);
    }
    else if ((millis() - ms_button) > 3000) // Обработка нажатия отмены.
    {
      ms_button = millis();
      CANCEL(0, 0);
    }

 
  switch (reflowState)
  {
  case REFLOW_STATE_IDLE:

    // блок обработки данных с ПЭВМ--------------------TODO Переписать прием-----------------------------------

    if (millis() > NextReadTemperatures)
    {
      // Read thermocouples next sampling period
      NextReadTemperatures = millis() + SENSOR_SAMPLING_TIME;
      Input1 = Input1 * 0.6 + 0.4 * (max6675_read_temp(thermoCLK1, thermoCS1, thermoDO1));
      Input2 = Input2 * 0.6 + 0.4 * (max6675_read_temp(thermoCLK2, thermoCS2, thermoDO2));
      Input3 = Input3 * 0.6 + 0.4 * (max6675_read_temp(thermoCLK3, thermoCS3, thermoDO3));

      tc1 = Input1;
      tc2 = Input2;
      tc3 = Input3;

      if (isnan(Input1))
        Serial.println("TC1 Error");
      if (isnan(Input2))
        Serial.println("TC2 Error");
      if (isnan(Input3))
        Serial.println("TC3 Error");
    }

    if (millis() > nextRead2) // TODO переписать отправку данных на ПК
    {
      nextRead2 = millis() + COM_PORT_SAMPLE_TIME;

      sprintf(buf, "$%03d %03d %03d %03d %03d %03d %03d %03d %03d;", (Output1), (Output2), (int)(Output3), tc1, tc2, tc3, TopDelta, BottomDelta, PcbDelta); // DIAGNOSTICS
 
    }

    break; // REFLOW_STATE_IDLE

  case REFLOW_STATE_INIT: // выбираем точку на графике, которая соответствует текущей температуре Платы
    previousMillis = millis();

    // сбрасываем переменные
    Output1 = 0;
    Output2 = 0;
    Output3 = 0;
    Serial.println("$#;"); // Команда - очистить график в Порт плоттере
    beep(1);               // Send sound Command to com

#ifdef show_diagnostics_com
    Serial.println("REFLOW_STATE_INIT");
#endif

    integra1 = 0;
    integra2 = 0;
    integra3 = 0;

    p1 = 0;
    p2 = 0;
    p3 = 0;

    d1 = 0;
    d2 = 0;
    d3 = 0;

    DurationCurrentStepTop = 0;
    DurationCurrentStepBottom = 0;
    DurationCurrentStepPcb = 0;

    TemperatureDeltaTop = 0;
    TemperatureDeltaBottom = 0;
    TemperatureDeltaPcb = 0;

    TempSpeedTop = 0;
    TempSpeedBottom = 0;
    TempSpeedPcb = 0;

    TimeCurrentStepTop = 0;
    TimeCurrentStepBottom = 0;
    TimeCurrentStepPcb = 0;

    CurrentStepTop = 0;
    CurrentStepBottom = 0;
    CurrentStepPcb = 0;

    CurrentProfileSecond = 0;
    CurrentProfileRealSecond = 0;

     // фиксируем размер стола
    if (profile.table_size == 1)
    {
      digitalWrite(P1_PIN, HIGH);
    }
    if (profile.table_size == 2)
    {
      digitalWrite(P1_PIN, HIGH);
      digitalWrite(P2_PIN, HIGH);
    }
    if (profile.table_size == 3)
    {
      digitalWrite(P1_PIN, HIGH);
      digitalWrite(P2_PIN, HIGH);
      digitalWrite(P3_PIN, HIGH);
    }

    // выбираем точку на графике, которая соответствует текущей температуре Платы

    if (jump_if_pcb_warm) // Если включен "Горячий старт"
    {
#ifdef show_diagnostics_com
      Serial.println("STATE_INIT HOT_Start calc\r\n"); // DIAGNOSTICS
#endif
      if (profile.temperature_step_pcb[0] > 0)
      {
        while (tc3 >= profile.temperature_step_pcb[CurrentStepPcb + 1]) // Прибавляем 1 шаг если температура платы сейчас ниже конечной температуры шага
        {
          CurrentStepPcb++;
#ifdef show_diagnostics_com
          Serial.println(CurrentStepPcb);          // DIAGNOSTICS
#endif
          if (CurrentStepPcb >= profile_steps_pcb) // если проверили все шаги, но температура платы сейчас выше максимальной температуры в профиле
          {
            Serial.print("ERROR Pcb Tempereture too high\r\n");
            reflowState = REFLOW_STATE_COMPLETE;
          }
        }
#ifdef show_diagnostics_com
        Serial.print("Calculated CurrentStepPcb:\r\n");
        Serial.println(CurrentStepPcb); // DIAGNOSTICS
#endif
        if (profile.temperature_step_pcb[CurrentStepPcb] != profile.temperature_step_pcb[CurrentStepPcb + 1])
        {
          DurationCurrentStepPcb = profile.time_step_pcb[CurrentStepPcb + 1] - profile.time_step_pcb[CurrentStepPcb];
          TemperatureDeltaPcb = profile.temperature_step_pcb[CurrentStepPcb + 1] - profile.temperature_step_pcb[CurrentStepPcb];
          TempSpeedPcb = TemperatureDeltaPcb * 100 / DurationCurrentStepPcb;                               // определяем скорость роста температуры на выбранном шаге
          TimeCurrentStepPcb = (tc3 - profile.temperature_step_pcb[CurrentStepPcb]) * 100L / TempSpeedPcb; // Домножаем на 100L, т.к делим на TempSpeedPcb которая уже умножена на 100
#ifdef show_diagnostics_com
          Serial.print("Calculated DurationCurrentStepPcb:\r\n"); // DIAGNOSTICS
          Serial.println(DurationCurrentStepPcb);                 // DIAGNOSTICS
          Serial.print("Calculated TemperatureDeltaPcb:\r\n");    // DIAGNOSTICS
          Serial.println(TemperatureDeltaPcb);                    // DIAGNOSTICS
          Serial.print("Calculated TempSpeedPcb:\r\n");           // DIAGNOSTICS
          Serial.println(TempSpeedPcb);                           // DIAGNOSTICS
          Serial.print("Calculated TimeCurrentStepPcb:\r\n");     // DIAGNOSTICS
          Serial.println(TimeCurrentStepPcb);                     // DIAGNOSTICS
#endif
        }
        // Вычисляем сколько секунд должно было пройти по профилю платы до текущей температуры
        CurrentProfileSecond = profile.time_step_pcb[CurrentStepPcb] + TimeCurrentStepPcb; // секунд от начала профиля до текущего шага платы + секунд от начала текущего шага до текущей температуры
        SetPoint_Pcb = profile.temperature_step_pcb[CurrentStepPcb] + TimeCurrentStepPcb * TempSpeedPcb / 100L;
#ifdef show_diagnostics_com
        Serial.print("Choosen CurrentProfileSecond:\r\n"); // DIAGNOSTICS
        Serial.println(CurrentProfileSecond);              // DIAGNOSTICS
        Serial.print("Choosen SetPoint_Pcb:\r\n");         // DIAGNOSTICS
        Serial.println(SetPoint_Pcb);                      // DIAGNOSTICS
#endif
      }

      if (profile.temperature_step_top[0] > 0) // определяем какую температуру ВИ выбрать (только если включен горячий страт)
      {
        while (CurrentProfileSecond >= profile.time_step_top[CurrentStepTop + 1]) // вычисляем с какого шшшага ВИ начинать
        {
          CurrentStepTop++;
#ifdef show_diagnostics_com
          Serial.println(CurrentStepTop); // DIAGNOSTICS
#endif
          if (CurrentStepTop >= profile.profile_steps) //
          {
            Serial.print("ERROR Top Steps Overcount\r\n");
            reflowState = REFLOW_STATE_COMPLETE;
            break;
          }
        }
        DurationCurrentStepTop = profile.time_step_top[CurrentStepTop + 1] - profile.time_step_top[CurrentStepTop];
        TemperatureDeltaTop = profile.temperature_step_top[CurrentStepTop + 1] - profile.temperature_step_top[CurrentStepTop];
        TempSpeedTop = TemperatureDeltaTop * 100 / DurationCurrentStepTop;
        TimeCurrentStepTop = CurrentProfileSecond - profile.time_step_top[CurrentStepTop];
        SetPoint_Top = profile.temperature_step_top[CurrentStepTop] + TimeCurrentStepTop * TempSpeedTop / 100L; // Находим значение температуры, которое выдать в задание преднагрева. Найти секунду от начала шага -> умножть разницу на гр\сек в текущем шаге
      }
      if (profile.temperature_step_bottom[0] > 0) // определяем какую температуру ВИ выбрать (только если включен горячий страт)
      {
        while (CurrentProfileSecond >= profile.time_step_bottom[CurrentStepBottom + 1]) // вычисляем с какого шшшага ВИ начинать
        {
          CurrentStepBottom++;
#ifdef show_diagnostics_com
          Serial.println(CurrentStepBottom); // DIAGNOSTICS
#endif
          if (CurrentStepBottom >= profile.profile_steps) //
          {
            Serial.print("ERROR Bottom Steps Overcount\r\n");
            reflowState = REFLOW_STATE_COMPLETE;
            break;
          }
        }
        DurationCurrentStepBottom = profile.time_step_bottom[CurrentStepBottom + 1] - profile.time_step_bottom[CurrentStepBottom];
        TemperatureDeltaBottom = profile.temperature_step_bottom[CurrentStepBottom + 1] - profile.temperature_step_bottom[CurrentStepBottom];
        TempSpeedBottom = TemperatureDeltaBottom * 100 / DurationCurrentStepBottom;
        TimeCurrentStepBottom = CurrentProfileSecond - profile.time_step_bottom[CurrentStepBottom];
        SetPoint_Bottom = profile.temperature_step_bottom[CurrentStepBottom] + TimeCurrentStepBottom * TempSpeedBottom / 100L;
      }
      if (CurrentProfileSecond == 0)
      {
        DurationCurrentStepPcb = profile.time_step_pcb[CurrentStepPcb + 1] - profile.time_step_pcb[CurrentStepPcb];
        DurationCurrentStepTop = profile.time_step_top[CurrentStepTop + 1] - profile.time_step_top[CurrentStepTop];
        DurationCurrentStepBottom = profile.time_step_bottom[CurrentStepBottom + 1] - profile.time_step_bottom[CurrentStepBottom];
      }
    } // конец логики горячего старта

    else // если горячий старт не включен, выставляем задание для ПИД равное температуре первой секунды первого шага ВИ, НИ и Платы.
    {
#ifdef show_diagnostics_com
      Serial.println("STATE_INIT COLD calc\r\n"); // DIAGNOSTICS
#endif
      SetPoint_Pcb = profile.temperature_step_pcb[0];
      SetPoint_Top = profile.temperature_step_top[0];
      SetPoint_Bottom = profile.temperature_step_bottom[0];

#ifdef show_diagnostics_com
      Serial.print("Calculated SetPoint_Pcb:\r\n");    // DIAGNOSTICS
      Serial.println(SetPoint_Pcb);                    // DIAGNOSTICS
      Serial.print("Calculated SetPoint_Top:\r\n");    // DIAGNOSTICS
      Serial.println(SetPoint_Top);                    // DIAGNOSTICS
      Serial.print("Calculated SetPoint_Bottom:\r\n"); // DIAGNOSTICS
      Serial.println(SetPoint_Bottom);                 // DIAGNOSTICS
#endif
    }

    // на выходе - номера шагов профиля ВИ, НИ, Платы (CurrentStepTop|Bottom|Pcb), а также количество секунд от их начала (TimeCurrentStepTop|Bottom|Pcb).
    // если температуры ВИ, НИ меньше 70 градусов -> preheat иначе warmup

#ifdef show_diagnostics_com
    Serial.print("Calculated CurrentStepPcb:\r\n");    // DIAGNOSTICS
    Serial.println(CurrentStepPcb);                    // DIAGNOSTICS
    Serial.print("Calculated CurrentStepTop:\r\n");    // DIAGNOSTICS
    Serial.println(CurrentStepTop);                    // DIAGNOSTICS
    Serial.print("Calculated CurrentStepBottom:\r\n"); // DIAGNOSTICS
    Serial.println(CurrentStepBottom);                 // DIAGNOSTICS
#endif
    if (tc2 < 70) // Если низ меньше 70 градусов - включаем плавный старт N секунд на малой мощности
    {

      reflowState = REFLOW_STATE_PRE_HEATER;
      Serial.println("@1;"); // отправить команду паузы отрисовки графика
      //  beep(1);
    }
    else
    {
      reflowStatus = REFLOW_STATUS_ON;
      reflowState = REFLOW_STATE_WARMUP;
      Serial.println("@1;");
      //  beep(1);
    }
    break;

  case REFLOW_STATE_PRE_HEATER:
    if (currentMillis - previousMillis > 1000)
    {
      Serial.println("STATE_PRE_HEATER\r\n"); // DIAGNOSTICS
      previousMillis = currentMillis;
      CurrentProfileRealSecond++;
      Output2 = PreHeatPower;
      if (profile.temperature_step_top[CurrentStepTop + 1] > 1) // Если включаем верх в профиле то прогреваем и его
      {
        Output1 = PreHeatPower;
      }
      Output3 = 0;
      if (CurrentProfileRealSecond >= PreHeatTime) // Если прошло N секунд
      {
        Output1 = 0;
        Output2 = 0;
        reflowStatus = REFLOW_STATUS_ON;
        reflowState = REFLOW_STATE_WARMUP;
        Serial.println("@1;");
        // beep(1);
      }
    }
    break;

  // преднагрев ВИ, НИ и платы до значений первого шага / секунды на которую перепрыгиваем. На этом этапе поградусная выдача задания для ПИД (Рампа) ОТКЛЮЧЕНА. ВИ и НИ греюся максимально быстро.
  // Проверяем только факт нагрева Платы и НИ до нужных температур. Целевые температуры определены на шаге REFLOW_STATE_INIT
  case REFLOW_STATE_WARMUP:
    if (currentMillis - previousMillis > 1000)
    {      
      Serial.println("STATE_WARMUP"); // DIAGNOSTICS
      Serial.println("@1;");
      Serial.println("!second: 1;");           // DIAGNOSTICS
      previousMillis = currentMillis;
      CurrentProfileRealSecond++;

      // Ждем когда показания термопар выйдут на нужные значения переход на Running
      if (tc3 >= (SetPoint_Pcb - profile.max_pcb_delta / 4)) // Если термопара Платы показывает значение больше чем половина  (задание - макс отклонение)
      {
        if (tc2 >= (SetPoint_Bottom - profile.max_correction_bottom)) // Если термопара Низа показывает значение больше чем (задание - макс коррекция)
        {
          reflowState = REFLOW_STATE_RUNNING;
          Serial.println("@2;");
          Serial.println("STATE_RUNNING;"); // DIAGNOSTICS
          beep(2);
        }
      }
      if (CurrentProfileRealSecond > max_warmup_duraton) // Если этап преднагрева затянулся больше чем на 4 минуты то отменяем профиль.
      {
        Serial.println("ERROR Warmup over 5 min\r\n"); // DIAGNOSTICS
        reflowState = REFLOW_STATE_COMPLETE;
      }
    }
    break;

  case REFLOW_STATE_RUNNING: // На входе номер шага НИ, ВИ, Платы, скорости роста температур, время внутри текущего шага, текущая секунда профиля.
    if (currentMillis - previousMillis > 1000)
    {
      previousMillis = currentMillis;

      // Логика для графика Платы
      if ((profile.time_step_pcb[CurrentStepPcb] > 0) || (profile.temperature_step_pcb[CurrentStepPcb] > 0)) // Если время шага и температура по профилю в начале текущего шага > 0
      {
        if (CurrentProfileSecond >= profile.time_step_pcb[CurrentStepPcb + 1]) // if (CurrentProfileSecond==0 ||  если счетчик времени по профилю 0 или >= времени начала следующего шага
        {                                                                      // переходим на следующий шаг профиля, считаем продолжительность, изменение температуры за шаг, скорость роста температуры
          CurrentStepPcb++;
          DurationCurrentStepPcb = profile.time_step_pcb[CurrentStepPcb + 1] - profile.time_step_pcb[CurrentStepPcb];
          TemperatureDeltaPcb = profile.temperature_step_pcb[CurrentStepPcb + 1] - profile.temperature_step_pcb[CurrentStepPcb];
          TempSpeedPcb = TemperatureDeltaPcb * 100 / DurationCurrentStepPcb;
          TimeCurrentStepPcb = 0; // определяем скорость роста температуры на текущем шаге
          if (TemperatureDeltaPcb < -10)
            beep(10); // Если дельта температур < -10 градусов, то даем сигнал что профиль все.
        }
        
        SetPoint_Pcb = profile.temperature_step_pcb[CurrentStepPcb] + TimeCurrentStepPcb * TempSpeedPcb / 100; // задание для ПИД = начальная температура шага + секунд от начала текущего шага * рост температуры в секунду (ошибка не накопится)
        TimeCurrentStepPcb++;                                                                                  // прибавляем +1 счетчику секунд от начала шага
      }
      else
      {
        SetPoint_Pcb = 0;
      }

      // Логика для графика НИ
      if (profile.time_step_bottom[CurrentStepBottom] > 0 || profile.temperature_step_bottom[CurrentStepBottom] > 0) // Если время шага и температура по профилю в начале текущего шага > 0
      {
        if (CurrentProfileSecond >= profile.time_step_bottom[CurrentStepBottom + 1]) // если счетчик времени по профилю >= времени начала следующего шага
        {                                                                            // переходим на следующий шаг профиля, считаем продолжительность, изменение температуры за шаг, скорость роста температуры
          CurrentStepBottom++;
          if (CurrentStepBottom == profile.profile_steps) // Если достигли последнего шага низа завершаем профиль
          {
            reflowState = REFLOW_STATE_COMPLETE;
            Serial.print("TX OK Last Bottom Step Complete\r\n");
            break;
          }
          DurationCurrentStepBottom = profile.time_step_bottom[CurrentStepBottom + 1] - profile.time_step_bottom[CurrentStepBottom];
          TemperatureDeltaBottom = profile.temperature_step_bottom[CurrentStepBottom + 1] - profile.temperature_step_bottom[CurrentStepBottom];
          TempSpeedBottom = TemperatureDeltaBottom * 100 / DurationCurrentStepBottom;
          TimeCurrentStepBottom = 0; // определяем скорость роста температуры на текущем шаге
        }
        SetPoint_Bottom = profile.temperature_step_bottom[CurrentStepBottom] + TimeCurrentStepBottom * TempSpeedBottom / (uint8_t)100; // задание для ПИД = начальная температура шага + секунд от начала текущего шага * рост температуры в секунду
        TimeCurrentStepBottom++;                                                                                                       // прибавляем +1 счетчику секунд от начала шага
      }
      else
      {
        SetPoint_Bottom = 0;
        Serial.print("SetPoint_Bottom: 0\r\n"); // DIAGNOSTICS
      }

      // Логика для графика ВИ
      if (profile.time_step_top[CurrentStepTop] > 0 || profile.temperature_step_top[CurrentStepTop] > 0) // Если время шага и температура по профилю в начале текущего шага > 1
      {
        if (CurrentProfileSecond >= profile.time_step_top[CurrentStepTop + 1]) // если счетчик времени по профилю >= времени начала следующего шага
        {                                                                      // переходим на следующий шаг профиля, считаем продолжительность, изменение температуры за шаг, скорость роста температуры
          CurrentStepTop++;
          DurationCurrentStepTop = profile.time_step_top[CurrentStepTop + 1] - profile.time_step_top[CurrentStepTop];
          TemperatureDeltaTop = profile.temperature_step_top[CurrentStepTop + 1] - profile.temperature_step_top[CurrentStepTop];
          TempSpeedTop = TemperatureDeltaTop * 100 / DurationCurrentStepTop;
          TimeCurrentStepTop = 0; // определяем скорость роста температуры на текущем шаге
        }
        SetPoint_Top = profile.temperature_step_top[CurrentStepTop] + TimeCurrentStepTop * TempSpeedTop / (uint8_t)100; // задание для ПИД = начальная температура шага + секунд от начала текущего шага * рост температуры в секунду
        TimeCurrentStepTop++;                                                                                           // прибавляем +1 счетчику секунд от начала шага
      }
      else
      {
        SetPoint_Top = 0;
      }

      if (SetPoint_Pcb == 0 && SetPoint_Bottom == 0) // если задание для ПИД платы и для низа по нолям -> закончить профиль (т.к. отдельно верхом мы не паяем)
      {
        reflowState = REFLOW_STATE_COMPLETE;
      }

      // Логика срабатывания HOLD
      if (SetPoint_Pcb - tc3 > profile.max_pcb_delta) // если температура платы не успевает за графиком больше чем на max_pcb_delta - переходим в режим HOLD (удержание температуры нагревателей, ждем пока плата догреется)
      {
        if (CurrentStepPcb < (profile_steps_pcb - 3)) // на последних 3 шагах когда идет финальная полка и охлаждение чтобы не вставало на автопаузу
        {
          if (AutoHoldCounter == 0) // DIAGNOSTICS(добавить =) выставляем счетчик по новой только если отсчитали предыдущий цикл полностью. Если за эти N секунд температура не вышла в диспазон допустимого отклонения - Можно включать HOLD по новой
          {
            AutoHoldCounter = profile.hold_lenght;
          }
          reflowState = REFLOW_STATE_HOLD_AUTO;
          Serial.println("@3;");
        }
        if (SetPoint_Pcb - tc3 < -profile.max_pcb_delta) // если температура платы перелетает уставку больше чем на max_pcb_delta - вопим что слишком большая мощность, но продолжаем паять.
        {
          Serial.println("WARNING Profile to powerfull");
          beep(7);
        }
      }

      CurrentProfileSecond++;
      CurrentProfileRealSecond++;
      Serial.print("RealSe: ");               // DIAGNOSTICS
      Serial.println(CurrentProfileRealSecond); // DIAGNOSTICS
      Serial.print("!second: ");           // DIAGNOSTICS
      Serial.print(CurrentProfileSecond);   // DIAGNOSTICS
      Serial.println(";");
    }
    break;

  case REFLOW_STATE_HOLD_AUTO: // попадаем в HOLD, задания для НИ, ВИ, и платы по графику остаются как в последнюю секунду. Только ПИД платы вносит небольшие коррективы в температуры ВИ и НИ .
    if (currentMillis - previousMillis > 1000)
    {
      Serial.println("HOLD_AUTO\r\n"); // DIAGNOSTICS
      // beep(1);  // Send sound Command to com every Second
      previousMillis = currentMillis;
      CurrentProfileRealSecond = CurrentProfileRealSecond + 1; // продолжаем считать полное время выполнения
      if (AutoHoldCounter > 0)
      {
        AutoHoldCounter--;
        Serial.print("Auto Hold Counting:"); // DIAGNOSTICS
        Serial.println(AutoHoldCounter);     // DIAGNOSTICS
        Serial.print("!second: ");           // DIAGNOSTICS
        Serial.print(CurrentProfileSecond);   // DIAGNOSTICS
        Serial.println(";");
      }
      else
      {
        reflowState = REFLOW_STATE_RUNNING;
        Serial.println("@2;");
        // beep(2); // Send sound Command to com when stop hold
      }
    }
    break;

  case REFLOW_STATE_HOLD_MANUAL:
    if (currentMillis - previousMillis > 1000) // 1 раз в секунду проверяем не нажали ли отмену \ отключение HOLD
    {
      if (prev_reflowState != REFLOW_STATE_IDLE && manual_temp_changed || manual_temperature_pcb)
      { // управление уставкой платы если перешли в паузу из профиля
        SetPoint_Pcb = manual_temperature;
      //  Serial.print("HOLD_MANUAL ENABLE Setpoint PCB Update ");
      //  Serial.println(SetPoint_Pcb);
      }
      else
      { // управление уставкой НИ если перешли в паузу из IDLE
        if (manual_temp_changed)
        {
          SetPoint_Bottom = manual_temperature;
       //   Serial.print("HOLD_MANUAL ENABLE Setpoint BOTTOM Update ");
       //   Serial.println(SetPoint_Bottom);
        }
      }
      Output1 = 0;
      manual_temp_changed = false;
      //manual_temperature_pcb = false;
      previousMillis = currentMillis;
      CurrentProfileRealSecond = CurrentProfileRealSecond + 1;
      if (CurrentProfileRealSecond % 60 == 0)
      {
        beep(1); // Если реальное время профиля кратно 60 скундам - сделать Бип
      }
      Serial.print("HOLD_MANUAL ENABLE  "); // DIAGNOSTICS
    }
    break;

  case REFLOW_STATE_POWER_MANUAL:              ////// Can be started only from IDLE.
    if (currentMillis - previousMillis > 1000) // 1 раз в секунду проверяем не нажали ли отмену
    {

      // manual_power_changed = false;
      previousMillis = currentMillis;
      CurrentProfileRealSecond = CurrentProfileRealSecond + 1;
      if (CurrentProfileRealSecond % 60 == 0)
      {
        beep(1); // Если реальное время профиля кратно 60 скундам - сделать Бип
        Serial.print("MANUAL_POWER: Top ");
        Serial.print(Output1);
        Serial.print("   Bottom: ");
        Serial.println(Output2);
      }
      
      // Serial.print("MANUAL_POWER ENABLE  "); //DIAGNOSTICS
    }
    break;

  // завершение пайки
  case REFLOW_STATE_COMPLETE:
    digitalWrite(P1_PIN, LOW);
    digitalWrite(P2_PIN, LOW);
    digitalWrite(P3_PIN, LOW);
    digitalWrite(P4_PIN, LOW);

    //  Serial.println("TXStop");
    Serial.println("REFLOW_STATE_COMPLETE");
    Serial.println("@5;");
    reflowStatus = REFLOW_STATUS_OFF;

    Output1 = 0;
    Output2 = 0;
    Output3 = 0;
    SetPoint_Top = 0;
    temp_correction_top = 0;
    SetPoint_Bottom = 0;
    temp_correction_bottom = 0;
    SetPoint_Pcb = 0;
    TopDelta = 0;
    BottomDelta = 0;
    PcbDelta = 0;
    manual_temperature = 0;
    manual_power = 0;

    beep(3);

    reflowState = REFLOW_STATE_IDLE;
    Serial.println("@0;");

    break;
  } // End of Switch

  // включение нагревателей
  if (reflowStatus == REFLOW_STATUS_ON)
  {

    if (millis() > NextReadTemperatures)
    {
      NextReadTemperatures = millis() + SENSOR_SAMPLING_TIME;
      Input1 = Input1 * 0.6 + 0.4 * (max6675_read_temp(thermoCLK1, thermoCS1, thermoDO1)); // сглаживание температуры на 12 измерений
      Input2 = Input2 * 0.6 + 0.4 * (max6675_read_temp(thermoCLK2, thermoCS2, thermoDO2)); // сглаживание температуры на 12 измерений
      Input3 = Input3 * 0.4 + 0.6 * (max6675_read_temp(thermoCLK3, thermoCS3, thermoDO3)); // сглаживание температуры на 8 измерений

      tc1 = Input1;
      tc2 = Input2;
      tc3 = Input3;

      if (reflowState != REFLOW_STATE_PRE_HEATER && reflowState != REFLOW_STATE_POWER_MANUAL) // не вычисляем PID в режиме преднагрева и ручного управления мощностью
      {
        //   Output3=(SetPoint_Pcb == 0) ? 0 : Pid3(Input3,SetPoint_Pcb,profile.kp3,profile.ki3,profile.kd3);         // если задание для ПИД платы 0 - не считаем его (не корректируем температуру)
        //   Output3=(SetPoint_Pcb == 0) ? 0 :Output3*0.92+0.08*Pid3(Input3,SetPoint_Pcb,profile.kp3,profile.ki3,profile.kd3);         // Сглаживание на 10 секунд 0.975 * 0.025//TODO Определится какой ПИД используем и какие настройки фильтрации значений выбрать.
        // Serial.println("PID Calculation");
        
        Output3 = (SetPoint_Pcb == 0) ? 0 : PidTEST4(Input3, SetPoint_Pcb, profile.kp3, profile.ki3, profile.kd3); //
        if (reflowState == REFLOW_STATE_WARMUP) {Output3 = 0;} // Won't correct temperature during Warmup period

        temp_correction_top = Output3 * profile.participation_rate_top / (uint8_t)100;                                               // вычисляем на сколько градусов скорректировать значние по графику
        temp_correction_top = (temp_correction_top < profile.max_correction_top) ? temp_correction_top : profile.max_correction_top; // проверяем не превышает ли макс значение коррекции

        temp_correction_bottom = Output3 * ParticipationRateBottom / (uint8_t)100;                                                                  // ParticipationRateBottom без profile.
        temp_correction_bottom = (temp_correction_bottom < profile.max_correction_bottom) ? temp_correction_bottom : profile.max_correction_bottom; // проверяем не превышает ли макс значение коррекции

        Output2 = PidTEST2(Input2, temp_correction_bottom + SetPoint_Bottom, profile.kp2, profile.ki2, profile.kd2);
        Output1 = (profile.temperature_step_top[CurrentStepTop] == 0) ? 0 : Pid1(Input1, temp_correction_top + SetPoint_Top, profile.kp1, profile.ki1, profile.kd1); // если для ВИ есть задание по графику, считаем ПИД верха

        // Вывод информации об отклонении температур от уставки, смещены на 50точек вниз, для удобства просмотра на 1 графике
        TopDelta = (tc1 - SetPoint_Top - temp_correction_top) - 50;
        BottomDelta = (tc2 - SetPoint_Bottom - temp_correction_bottom) - 50;
        PcbDelta = (tc3 - SetPoint_Pcb) - 50;
      }

      sprintf(buf, "$%03d %03d %03d %03d %03d %03d %03d %03d %03d;", (Output1), (Output2), (int)(Output3), tc1, tc2, tc3, TopDelta, BottomDelta, PcbDelta); // DIAGNOSTICS
      
      //  sprintf (buf, "OK%03d%03d%03d%03d%03d%03d%03d\r\n", (Output1), (Output2), (Output3), tc1, tc2, tc3, (CurrentProfileRealSecond)); // TODO исправить формат отправки для графика ПК
      // sprintf (buf, "$%03d %03d %03d %03d %03d %03d %03d %03d %03d;", (Output1), (Output2), (int)(Output3), tc1, tc2, tc3, (TopDelta), (BottomDelta), (PcbDelta)); // TODO исправить формат отправки для графика ПК
       // sprintf (buf, "$%03d %03d %03d %03d %03d %03d %03d %03d %03d;", (Output1), (Output2), (int)(Output3), tc1, tc2, tc3, (int)(p2), (int)(integra2), (int)(d2));  //DIAGNOSTICS

       if (isnan(Input1)) Serial.println("TXTC1Error");
       if (isnan(Input2)) Serial.println("TXTC2Error");
       if (isnan(Input3)) Serial.println("TXTC3Error");
    }
  if (tc3 >= max_temperature_pcb) // если температура платы достигла предела (250гр например) прервать выполнение профиля
  {
    reflowStatus = REFLOW_STATUS_OFF;
    reflowState = REFLOW_STATE_COMPLETE;
    Serial.println("ERROR PCB Overheat");
    beep(3);
  }
  }
}

void Dimming() // Вызывается по прерыванию от детектора ноля 100 раз в секунду
{
  OutPWR_TOP();
  OutPWR_BOTTOM();

  if (Secs >= 100) // каждые 100 прерываний отправляем данные через serial
  {
    if (!blockflag)
    {
      Serial.println(buf);
    }
    Secs = 1; // Начинаем считать с 1, т.к. в первый шаг мы только присваиваем значение, но не прибавляем.
  }
  else
    Secs++;
}

void OutPWR_TOP()
{
  reg1 = Output1 + er1; // pwr- задание выходной мощности в %,в текущем шаге профиля, er- ошибка округления
  if (reg1 < 50)
  {
    out1 = LOW;
    er1 = reg1; // reg- переменная для расчетов
  }
  else
  {
    out1 = HIGH;
    er1 = reg1 - 100;
  }
  digitalWrite(RelayPin1, out1); // пин через который осуществляется дискретное управление
}

void OutPWR_BOTTOM()
{
  reg2 = Output2 + er2; // pwr- задание выходной мощности в %, er- ошибка округления
  if (reg2 < 50)
  {
    out2 = LOW;
    er2 = reg2; // reg- переменная для расчетов
  }
  else
  {
    out2 = HIGH;
    er2 = reg2 - 100;
  }
  digitalWrite(RelayPin2, out2); // пин через который осуществляется дискретное управление
}

byte Pid1(double temp, int ust, byte kP, byte kI, byte kD)
{
  static byte out = 0;
  static float ed = 0;
  e1 = (ust - temp);   // ошибка регулирования
  p1 = (kP * e1); // П составляющая
  integra1 = ((out > 99) && (e1 * integra1 >= 0)) ? integra1 : (integra1 < i_min) ? i_min
                                                           : (integra1 > i_max)   ? i_max
                                                                                  : integra1 + (kI * e1) / 1000.0; // И составляющая
  d1 = kD * (temp - ed);                                                                                          // Д составляющая
  ed = temp;
  out = (p1 + integra1 - d1 >= 100) ? 100 : (p1 + integra1 - d1 <= 0) ? 0
                                                                      : p1 + integra1 - d1; //  Выход ПИД от 0 до 100.
  return out;
}

byte Pid2(double temp, int ust, byte kP, byte kI, byte kd)
{
  byte out = 0;
  static float ed = 0;
  e2 = (ust - temp); // ошибка регулирования
  p2 = (kP * e2);    // П составляющая
  integra2 = (integra2 < i_min) ? i_min : (integra2 > i_max) ? i_max
                                                             : integra2 + (kI * e2) / 1000.0; // И составляющая
  d2 = kd * (temp - ed);                                                                      // Д составляющая
  ed = temp;
  out = (p2 + integra2 - d2 >= 100) ? 100 : (p2 + integra2 - d2 <= 0) ? 0
                                                                      : p2 + integra2 - d2; //  Выход ПИД от 0 до 100.
  return out;
}

int8_t Pid3(double temp, int ust, byte kP, byte kI, byte kd) // Темп фактически, темп должно быть, коэфициенты пид, результат - значение коррекции для ПИД ВИ и НИ в градусах (от -100 до 100)
{
  int8_t out = 0;
  static float ed = 0;
  e3 = (ust - temp); // ошибка регулирования
  p3 = (kP * e3);    // П составляющая
  integra3 = (integra3 < i_min) ? i_min : (integra3 > i_max) ? i_max
                                                             : integra3 + (kI * e3) / 1000.0; // И составляющая
  d3 = kd * (temp - ed);                                                                      // Д составляющая
  ed = temp;
  out = (p3 + integra3 - d3 > 100) ? 100 : (p3 + integra3 - d3 < -100) ? -100
                                                                       : p3 + integra3 - d3; // Ограничиваем выхо��ные значения
  return out;
}

int8_t PidTEST(double temp, int ust, byte kP, byte kI, byte kd) // Сглаживание дифференциальной составляющей на 4 шага
{
  int8_t out = 0;
  static float ed = 0;
  e3 = e3 * 0.75 + 0.25 * (ust - temp); // ошибка регулирования
  p3 = (kP * e3);                       // П составляющая
  integra3 = (integra3 < i_min) ? i_min : (integra3 > i_max) ? i_max
                                                             : integra3 + (kI * e3) / 1000.0; // И составляющая
  // d3 = d3 * 0.75 + 0.25 * kd * (e3 - ed); //Д составляющая
  d3 = kd * (e3 - ed);
  ed = e3;
  out = (p3 + integra3 + d3 > 100) ? 100 : (p3 + integra3 + d3 < -100) ? -100
                                                                       : p3 + integra3 + d3; // Ограничиваем выходные значения
#ifdef show_diagnostics_com_deep
  Serial.println(p3);       // DIAGNOSTICS
  Serial.println(integra3); // DIAGNOSTICS
  Serial.println(d3);       // DIAGNOSTICS
#endif
  return out;
}

byte PidTEST2(double temp, int ust, byte kP, byte kI, byte kD) // Сглаживание дифференциальной составляющей на 4 шага
{
  static byte out;
  static int prev_ust;
  static byte lastOut; // предыдущее значение мощности
  static float integra_coef = 10000.0; // Делитель интегральной составляющей
  //int ust_delta = prev_ust - ust;
  static byte buff_position;
  static const byte buff_depth = 4; //  // глубина буфера 4 = 1 секунда, 8 = 2 секунды... MAX 250, увеличение кушает память.
  static float ed[buff_depth];      // массив предыдущих измерений ошибки (виден только внутри функции пид)
  e2 = (ust - temp);                // ошибка регулирования Сглаживание на 4 шага
  p2 = (kP * e2) ;              // П составляющая
  integra2 = ((out > 99) && (e2 * integra2 >= 0)) ? integra2 : (integra2 < i_min) ? i_min
                                                           : (integra2 > i_max)   ? i_max
                                                                                  : integra2 + (kI * e2) / integra_coef; // И составляющая
  //integra2 = (ust_delta == 0) ? integra2 : integra2 - (kI * ust_delta) / integra_coef;
  d2 = kD * (temp - ed[buff_position]);
  ed[buff_position] = temp;
  buff_position = (buff_position < buff_depth - 1) ? buff_position + 1 : 0;
  out = (p2 + integra2 - d2 >= 100) ? 100 : (p2 + integra2 - d2 <= 0) ? 0
                                                                      : p2 + integra2 - d2; // Ограничиваем выходные значения
  prev_ust = ust;
  lastOut = out;
  return out;
}

int8_t PidTEST4(double temp, int ust, byte kP, byte kI, byte kD) // Сглаживание дифференциальной составляющей на 4 шага
{
  int8_t out = 0;
  static byte lastOut; // предыдущее значение мощности
  byte kF = kD;        // Значение множителя К для Сопротивления изменению температуры (f = friction)
  float f3 = 0;
  static const byte buff_depth = 4; //  // глубина буфера 4 = 1 секунда, 8 = 2 секунды... MAX 250, увеличение кушает память.
  static float ed[buff_depth];      // массив предыдущих измерений ошибки (виден только внутри функции пид)
  static float ef[buff_depth];      // массив предыдущих измерений температуры (виден только внутри функции пид)
  static byte buff_position;
  e3 = e3 * 0.75 + 0.25 * (ust - temp); // ошибка регулирования Сглаживание на 11 шагов
  p3 = (kP * e3);                       // П составляющая
  // integra3 = (integra3 < -100) ? -100 : (integra3 > 100) ? 100 : integra3 + (kI * e3)/1000.0; //И составляющая
  integra3 = ((lastOut > max_correction_total - 1) && (e3 * integra3 >= 0)) ? integra3 : (integra3 < -max_correction_total) ? -max_correction_total
                                                                                     : (integra3 > max_correction_total)    ? max_correction_total
                                                                                                                            : integra3 + (kI * e3) / 1000.0;
  d3 = kD * (e3 - ed[buff_position]);                                       // Д составляющая // сравниваем со значением ошибки N шагов назад, если ошибка уменьшилась уменьшаем ПИД
  f3 = kF * (temp - ef[buff_position]);                                     // скорость изменения температуры, если быстро растет, гасим пид, если быстро падает - подпинываем
  ed[buff_position] = e3;                                                   // Запись ошибки buff_depth шагов назад
  ef[buff_position] = temp;                                                 // Запись температуры buff_depth шагов назад
  buff_position = (buff_position < buff_depth - 1) ? buff_position + 1 : 0; // если достигли последнего значения буфера начинаем писать с 0
#ifdef show_diagnostics_com_deep
  Serial.println(p3);       // DIAGNOSTICS
  Serial.println(integra3); // DIAGNOSTICS
  Serial.println(d3);       // DIAGNOSTICS
  Serial.println(f3);
#endif
  out = (p3 + integra3 + d3 - f3 > max_correction_total) ? max_correction_total : (p3 + integra3 + d3 - f3 < -max_correction_total) ? -max_correction_total
                                                                                                                                    : p3 + integra3 + d3 - f3; // Ограничиваем выходные значения -100 __ 100
  lastOut = out;
  return out;
}

double max6675_read_temp(int ck, int cs, int so)
{
  char i;
  int tmp = 0;
  digitalWrite(cs, LOW); // cs = 0;                            // Stop a conversion in progress
  asm volatile(" nop"
               "\n\t");
  for (i = 15; i >= 0; i--)
  {
    digitalWrite(ck, HIGH);
    asm volatile(" nop"
                 "\n\t");
    if (digitalRead(so))
      tmp |= (1 << i);
    digitalWrite(ck, LOW);
    asm volatile(" nop"
                 "\n\t");
  }
  digitalWrite(cs, HIGH);
  if (tmp & 0x4)
  {
    return NAN;
  }
  else
    return ((tmp >> 3)) * 0.25;
}

/**************************************************************************/
/*!
    Generate the main command prompt
*/
/**************************************************************************/
void cmd_display()
{
  char buf[180];
}

/**************************************************************************/
/*!
    Parse the command line. This function tokenizes the command input, then
    searches for the command table entry associated with the commmand. Once found,
    it will jump to the corresponding function.
*/
/**************************************************************************/
void cmd_parse(char *cmd)
{
  static uint8_t argc;

  uint8_t i = 0;

#define max_arguments 85
  char *argv[max_arguments]; // Max number of variables in recieved command line
  cmd_t *cmd_entry;

  // parse the command line statement and break it up into delimited
  // strings. the array of strings will be saved in the argv array.
  argv[i] = strtok(cmd, " ");
  do
  {
    // argv[++i] = strtok(NULL, " ");
    argv[++i] = strtok(NULL, ",");
  } while ((i < max_arguments) && (argv[i] != NULL));
  if (!argv[0])
    return; // if we found that 0 element = 0 than we have no command. no need to react.

  // save off the number of arguments for the particular command.
  argc = i;

  // parse the command table for valid command. used argv[0] which is the
  // actual command name typed in at the prompt

  for (cmd_entry = cmd_tbl; cmd_entry != NULL; cmd_entry = cmd_entry->next)
  {

    if (!strcmp(argv[0], cmd_entry->cmd))
    {
      cmd_entry->func(argc, argv);
      cmd_display(); // Echoe commands
      return;
    }
    if (cmd_entry == NULL)
    {
      cmd_display(); // Echoe commands
      return;
    }
  }

  // command not recognized. print message and re-generate prompt.

  stream->print("CMD Unknown: ");
  stream->println(argv[0]);
}

/**************************************************************************/
/*!
    This function processes the individual characters typed into the command
    prompt. It saves them off into the message buffer unless its a "backspace"
    or "enter" key.
*/
/**************************************************************************/
void cmd_handler()
{
  unsigned char c = stream->read();

  switch (c)
  {
  case ';':
    *msg_ptr = '\0';
    cmd_parse((char *)msg);
    msg_ptr = msg;

    break;
  case '\r':

    // ignore return characters. they usually come in pairs
    // with the \r characters we use for newline detection.

    break;

  case '\n':
    // terminate the msg and reset the msg ptr. then send
    // it to the handler for processing.
    *msg_ptr = '\0';
    cmd_parse((char *)msg);
    msg_ptr = msg;

    break;

  default:
    // normal character entered. add it to the buffer

    *msg_ptr++ = c;
    break;
  }
}

/**************************************************************************/
/*!
    This function should be set inside the main loop. It needs to be called
    constantly to check if there is any available input at the command prompt.
*/
/**************************************************************************/
void cmdPoll()
{
  while (stream->available())
  {
    cmd_handler();
  }
}

/**************************************************************************/
/*!
    Initialize the command line interface. This sets the terminal speed and
    and initializes things.
*/
/**************************************************************************/
void cmdInit(Stream *str)
{
  stream = str;
  // init the msg ptr
  msg_ptr = msg;

  // init the command table
  cmd_tbl_list = NULL;
}

/**************************************************************************/
/*!
    Add a command to the command table. The commands should be added in
    at the setup() portion of the sketch.
*/
/**************************************************************************/
void cmdAdd(const char *name, void (*func)(int argc, char **argv))
{
  // alloc memory for command struct
  cmd_tbl = (cmd_t *)malloc(sizeof(cmd_t));

  // alloc memory for command name
  char *cmd_name = (char *)malloc(strlen(name) + 1);

  // copy command name
  strcpy(cmd_name, name);

  // terminate the command name
  cmd_name[strlen(name)] = '\0';

  // fill out structure
  cmd_tbl->cmd = cmd_name;
  cmd_tbl->func = func;
  cmd_tbl->next = cmd_tbl_list;
  cmd_tbl_list = cmd_tbl;
}

/**************************************************************************/
/*!
    Get a pointer to the stream used by the interpreter. This allows
    commands to use the same communication channel as the interpreter
    without tracking it in the main program.
*/
/**************************************************************************/
Stream *cmdGetStream(void)
{
  return stream;
}

/**************************************************************************/
/*!
    Convert a string to a number. The base must be specified, ie: "32" is a
    different value in base 10 (decimal) and base 16 (hexadecimal).
*/
/**************************************************************************/
uint32_t cmdStr2Num(char *str, uint8_t base)
{
  return strtol(str, NULL, base);
}

// Array Variables Update Area -------------------------------------------

void update_time_step_top(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  for (int i = 1; i < arg_cnt; i++)
  {
    profile.time_step_top[i - 1] = atoi(args[i]);
  }
  s->print("!time_step_top: ");

  for (int i = 0; i < profile.profile_steps; i++)
  {
    s->print(profile.time_step_top[i]);
    s->print(",");
  }
  s->println(";");
}

void update_temperature_step_top(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  for (int i = 1; i < arg_cnt; i++)
  {
    profile.temperature_step_top[i - 1] = atoi(args[i]);
  }
  s->print("!temperature_step_top: ");
  for (int i = 0; i < profile.profile_steps; i++)
  {
    s->print(profile.temperature_step_top[i]);
    s->print(",");
  }
  s->println(";");
}

void update_time_step_bottom(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  for (int i = 1; i < arg_cnt; i++)
  {
    profile.time_step_bottom[i - 1] = atoi(args[i]);
  }
  s->print("!time_step_bottom: ");
  for (int i = 0; i < profile.profile_steps; i++)
  {
    s->print(profile.time_step_bottom[i]);
    s->print(",");
  }
  s->println(";");
}

void update_temperature_step_bottom(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  for (int i = 1; i < arg_cnt; i++)
  {
    profile.temperature_step_bottom[i - 1] = atoi(args[i]);
  }
  s->print("!temperature_step_bottom: ");
  for (int i = 0; i < profile.profile_steps; i++)
  {
    s->print(profile.temperature_step_bottom[i]);
    s->print(",");
  }
  s->println(";");
}

void update_time_step_pcb(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  for (int i = 1; i < arg_cnt; i++)
  {
    profile.time_step_pcb[i - 1] = atoi(args[i]);
  }
  s->print("!time_step_pcb: ");
  for (int i = 0; i < profile_steps_pcb; i++)
  {
    s->print(profile.time_step_pcb[i]);
    s->print(",");
  }
  s->println(";");
}

void update_temperature_step_pcb(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  for (int i = 1; i < arg_cnt; i++)
  {
    profile.temperature_step_pcb[i - 1] = atoi(args[i]);
  }
  s->print("!temperature_step_pcb: ");
  for (int i = 0; i < profile_steps_pcb; i++)
  {
    s->print(profile.temperature_step_pcb[i]);
    s->print(",");
  }
  s->println(";");
}

// Single Variables Update Area -------------------------------------------
void update_profile_steps(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!profile_steps: ");
  if (arg_cnt > 1)
  {
    profile.profile_steps = atoi(args[1]);
  }
  s->print(profile.profile_steps);
  s->println(";");
}

void update_aliasprofile(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!aliasprofile: ");
  if (arg_cnt > 1)
  {
    strcpy(profile.alias, args[1]);
  }
  s->print(profile.alias);
  s->println(";");
}

void update_table_size(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!table_size: ");
  if (arg_cnt > 1)
  {
    profile.table_size = atoi(args[1]);
  }
  s->print(profile.table_size);
  s->println(";");
}

void update_kp1(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!kp1: ");
  if (arg_cnt > 1)
  {
    profile.kp1 = atoi(args[1]);
  }
  s->print(profile.kp1);
  s->println(";");
}

void update_ki1(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!ki1: ");
  if (arg_cnt > 1)
  {
    profile.ki1 = atoi(args[1]);
  }
  s->print(profile.ki1);
  s->println(";");
}

void update_kd1(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!kd1: ");
  if (arg_cnt > 1)
  {
    profile.kd1 = atoi(args[1]);
  }
  s->print(profile.kd1);
  s->println(";");
}

void update_kp2(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!kp2: ");
  if (arg_cnt > 1)
  {
    profile.kp2 = atoi(args[1]);
  }
  s->print(profile.kp2);
  s->println(";");
}

void update_ki2(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!ki2: ");
  if (arg_cnt > 1)
  {
    profile.ki2 = atoi(args[1]);
  }
  s->print(profile.ki2);
  s->println(";");
}

void update_kd2(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!kd2: ");
  if (arg_cnt > 1)
  {
    profile.kd2 = atoi(args[1]);
  }
  s->print(profile.kd2);
  s->println(";");
}

void update_kp3(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!kp3: ");
  if (arg_cnt > 1)
  {
    profile.kp3 = atoi(args[1]);
  }
  s->print(profile.kp3);
  s->println(";");
}

void update_ki3(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!ki3: ");
  if (arg_cnt > 1)
  {
    profile.ki3 = atoi(args[1]);
  }
  s->print(profile.ki3);
  s->println(";");
}

void update_kd3(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!kd3: ");
  if (arg_cnt > 1)
  {
    profile.kd3 = atoi(args[1]);
  }
  s->print(profile.kd3);
  s->println(";");
}

void update_max_correction_top(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!max_correction_top: ");
  if (arg_cnt > 1)
  {
    profile.max_correction_top = atoi(args[1]);
  }
  s->print(profile.max_correction_top);
  s->println(";");
}

void update_max_correction_bottom(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!max_correction_bottom: ");
  if (arg_cnt > 1)
  {
    profile.max_correction_bottom = atoi(args[1]);
  }
  s->print(profile.max_correction_bottom);
  s->println(";");
}

void update_max_pcb_delta(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!max_pcb_delta: ");
  if (arg_cnt > 1)
  {
    profile.max_pcb_delta = atoi(args[1]);
  }
  s->print(profile.max_pcb_delta);
  s->println(";");
}

void update_hold_lenght(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!hold_lenght: ");
  if (arg_cnt > 1)
  {
    profile.hold_lenght = atoi(args[1]);
  }
  s->print(profile.hold_lenght);
  s->println(";");
}

void update_participation_rate_top(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  s->print("!participation_rate_top: ");
  if (arg_cnt > 1)
  {
    profile.participation_rate_top = atoi(args[1]);
  }
  s->print(profile.participation_rate_top);
  s->println(";");
}

// Statiion commands area ----------------------------------------------------------

void SAVE(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  if (arg_cnt <= 1)
  {
    printProfileNumber();
  }
  else
  {
    if (atoi(args[1]) <= 0 || atoi(args[1]) > max_profiles)
    {
      s->print("Profile number over limit");
    }
    else
    {
      currentProfile = atoi(args[1]);
      SaveProfile();
      s->print("Profile ");
      s->print(currentProfile);
      s->println(" updated");
      printProfileNumber();
       // Print profile Number
    }
  }
}

void UP(int arg_cnt, char **args) // Adjust PCB temperature up if called during profile running. BOTTOM temperature if called from IDLE
{
  Stream *s = cmdGetStream();

  if (arg_cnt > 1)
    s->print("UP sould have 0 params");
  else
  {
    if (reflowState == REFLOW_STATE_RUNNING || reflowState == REFLOW_STATE_HOLD_AUTO || reflowState == REFLOW_STATE_HOLD_MANUAL)
    {
      manual_temperature = (reflowState != REFLOW_STATE_HOLD_MANUAL) ? ((SetPoint_Pcb == 0) ? Input3 : SetPoint_Pcb) : manual_temperature + 2; // if it's first call during profile run - fix PCB current temp as setpoint
      manual_temp_changed = true;
      s->print("!Set Temp: ");
      s->print(manual_temperature);
      s->println(";");
      if (reflowState != REFLOW_STATE_HOLD_MANUAL){HOLD(0, 0);} //если мы не в режиме ручной паузы - включить её
    }
  }
}

void DOWN(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();

  if (arg_cnt > 1)
    s->print("DOWN sould have 0 params");
  else
  {
    if (reflowState == REFLOW_STATE_RUNNING || reflowState == REFLOW_STATE_HOLD_AUTO || reflowState == REFLOW_STATE_HOLD_MANUAL)
    {
      manual_temperature = (reflowState != REFLOW_STATE_HOLD_MANUAL) ? ((SetPoint_Pcb == 0) ? Input3 : SetPoint_Pcb) : manual_temperature - 2;
      manual_temp_changed = true;
      s->print("!Set Temp: ");
      s->print(manual_temperature);
      s->println(";");
      if (reflowState != REFLOW_STATE_HOLD_MANUAL){HOLD(0, 0);}
    }
  }
}

void RIGHT(int arg_cnt, char **args) // Switch to next profile in IDLE
{
  Stream *s = cmdGetStream();

  if (arg_cnt > 1)
    s->print("RIGHT sould have 0 params");
  else
  {
    if (reflowState == REFLOW_STATE_IDLE)
    {
      currentProfile = currentProfile + 1;
      if (currentProfile > max_profiles)
        currentProfile = 1;
      loadProfile();
    }
  }
}

void LEFT(int arg_cnt, char **args) // Switch to prev profile in IDLE
{
  Stream *s = cmdGetStream();

  if (arg_cnt > 1)
    s->print("LEFT sould have 0 params");
  else
  {
    if (reflowState == REFLOW_STATE_IDLE)
    {
      currentProfile = currentProfile - 1;
      if (currentProfile <= 0)
        currentProfile = max_profiles;
      loadProfile();
    }
  }
}

void CANCEL(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();

  if (arg_cnt > 1)
    s->print("CANCEL sould have 0 params");
  else
  {
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      Serial.println("STOP");
      reflowState = REFLOW_STATE_COMPLETE; // Обработка команды Отмены выполнения
    }
  }
}

void BLOCK(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  if (arg_cnt > 1)
    s->print("BLOCK sould have 0 params");
  else
  {
    if (reflowState == REFLOW_STATE_IDLE)
    {
      Serial.println("Edit mode enable");
      blockflag = true;
    }
  }
}

void UNBLOCK(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();
  if (arg_cnt > 1)
    s->print("UNBLOCK sould have 0 params");
  else
  {
    if (reflowState == REFLOW_STATE_IDLE)
    {
      Serial.println("Edit mode disable");
      blockflag = false;
    }
  }
}

void PRINT_PROFILE() // отправка профиля в КОМ Костыль))
{
  blockflag=true;
  printProfileNumber(); // печатает номер профиля
  update_time_step_top(0, 0);
  update_temperature_step_top(0, 0);
  update_time_step_bottom(0, 0);
  update_temperature_step_bottom(0, 0);
  update_time_step_pcb(0, 0);
  update_temperature_step_pcb(0, 0);
  update_kp1(0, 0);
  update_ki1(0, 0);
  update_kd1(0, 0);
  update_kp2(0, 0);
  update_ki2(0, 0);
  update_kd2(0, 0);
  update_kp3(0, 0);
  update_ki3(0, 0);
  update_kd3(0, 0);
  update_profile_steps(0, 0);
  update_table_size(0, 0);
  update_max_correction_top(0, 0);
  update_max_correction_bottom(0, 0);
  update_max_pcb_delta(0, 0);
  update_hold_lenght(0, 0);
  update_participation_rate_top(0, 0);
  update_aliasprofile(0, 0);
  Serial.println("!profile_transfer_finished:1;");
  blockflag=false;

  return;
}

void HOLD(int arg_cnt, char **args) // Ручная пауза выполнения профиля
{
  Stream *s = cmdGetStream();
  if (arg_cnt > 1)
    s->print("HOLD sould have 0 params");
  else
  {
    if (reflowStatus == REFLOW_STATUS_ON)
    {
      if (reflowState != REFLOW_STATE_HOLD_MANUAL) // Если не в Hold - перейти в него
      {
        manual_temperature = (SetPoint_Pcb == 0) ? Input3 : SetPoint_Pcb; // если задание платы = 0 (в профиле нет графика для платы) то задаём уставку равной текущей температуре платы
                                                                          // manual_temperature = SetPoint_Pcb;
        s->print("manual temp: ");
        s->print(manual_temperature);
        prev_reflowState = reflowState;
        reflowState = REFLOW_STATE_HOLD_MANUAL;
        Serial.println("@4;");
      
      }
      else // если уже в HOLD и еще раз пришла команда HOLD - выходим из удержания
      {
        if (!manual_temp_changed)
        {
          if (prev_reflowState == REFLOW_STATE_RUNNING && abs(tc3 - SetPoint_Pcb) > profile.max_pcb_delta) // если предыдущее состояние RUNNING и температура платы ушла больше чем на макс отклонение температуры - стартуем профиль на горячую
          {
            jump_if_pcb_warm = true;
            reflowState = REFLOW_STATE_INIT;
          }
          else
          {
            reflowState = prev_reflowState;
          }
          Serial.println("HOLD_MANUAL Disabled");
        }
      }
    }
  }
}

void START(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();

  if (arg_cnt > 1)
    s->print("START sould have 0 params");
  else
  {
    if (reflowState == REFLOW_STATE_IDLE)
    {
      jump_if_pcb_warm = false;
      Serial.println("Profile Start");
      reflowStatus = REFLOW_STATUS_OFF;
      reflowState = REFLOW_STATE_INIT;
    }
  }
}

void HOTSTART(int arg_cnt, char **args) // Может быть вызвана в любой момент.
{
  Stream *s = cmdGetStream();

  if (arg_cnt > 1)
    s->print("HOTSTART sould have 0 params");
  else
  {
    jump_if_pcb_warm = true;
    Serial.println("Profile HotStart");
    reflowStatus = REFLOW_STATUS_OFF;
    reflowState = REFLOW_STATE_INIT;
  }
}


void MANUAL_TEMP(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();

  if (arg_cnt < 3)
    s->println("MANUAL_TEMP sould have 2 params");
  else
  {
    if (reflowState == REFLOW_STATE_IDLE || REFLOW_STATE_HOLD_MANUAL)
    {
      manual_temperature = cmdStr2Num(args[2], 10);
      manual_temp_changed = true;
      s->print("manual_temperature: ");
      s->print(manual_temperature);
      s->println(";");

      s->print("!Set Temp: ");
      s->print(manual_temperature);
      s->println(";");

      prev_reflowState = reflowState;
      reflowStatus = REFLOW_STATUS_ON;
      reflowState = REFLOW_STATE_HOLD_MANUAL;
      Serial.println("@4;");

      if (profile.table_size == 1)
      {
        digitalWrite(P1_PIN, HIGH);
      }
      if (profile.table_size == 2)
      {
        digitalWrite(P1_PIN, HIGH);
        digitalWrite(P2_PIN, HIGH);
      }
      if (profile.table_size == 3)
      {
        digitalWrite(P1_PIN, HIGH);
        digitalWrite(P2_PIN, HIGH);
        digitalWrite(P3_PIN, HIGH);
      }

      switch (cmdStr2Num(args[1], 10)) // Определяем чем будем управлять
      {
      case 2: // Греть НИ до указанной температуры
        SetPoint_Bottom = manual_temperature;
        manual_temp_changed = true;
        s->print("ManualControl_Bottom: ");
        s->println(manual_temperature);
        break;

      case 3: // Греть плату при помощи НИ до указанной температуры
        SetPoint_Pcb = manual_temperature;
        SetPoint_Bottom = manual_temperature * 1,5;        // Уставка НИ = 1,5 уставки платы
        manual_temperature_pcb = true;
        profile.max_correction_bottom = manual_temperature; // коррекция НИ = уставки платы
        s->print("ManualControl_PCB: ");
        s->println(manual_temperature);
        break;

      default:
        s->println("ManualControl Wrong param");
        break;
      }
    }
    else
    {
      s->print("Wrong state. Cant set MANUAL TEMP");
      // break;
    }
  }
}

void MANUAL_POWER(int arg_cnt, char **args)
{
  Stream *s = cmdGetStream();

  if (arg_cnt < 3)
    s->println("MANUAL_POWER sould have 2 params");
  else
  {
    if (reflowState == REFLOW_STATE_IDLE || reflowState == REFLOW_STATE_POWER_MANUAL) // Can start only from IDLE and adjust from POWER_MANUAL
    {
      manual_power = cmdStr2Num(args[2], 10);
      s->print("manual_power ");
      s->print(manual_power);
      s->println(";");

      s->print("!Set Power ");
      s->print(manual_power);
      s->println(";");

      prev_reflowState = reflowState;
      reflowStatus = REFLOW_STATUS_ON;
      reflowState = REFLOW_STATE_POWER_MANUAL;
      Serial.println("@4;");

      if (profile.table_size == 1)
      {
        digitalWrite(P1_PIN, HIGH);
      }
      if (profile.table_size == 2)
      {
        digitalWrite(P1_PIN, HIGH);
        digitalWrite(P2_PIN, HIGH);
      }
      if (profile.table_size == 3)
      {
        digitalWrite(P1_PIN, HIGH);
        digitalWrite(P2_PIN, HIGH);
        digitalWrite(P3_PIN, HIGH);
      }

      switch (cmdStr2Num(args[1], 10))
      {
      case 1: // Греть ВИ указанной мощностью
        Output1 = constrain(manual_power, 0, 100);
        s->print("ManualControl_POWER_Top: ");
        s->println(Output1);
        break;

      case 2: // Греть НИ до указанной мощностью
        Output2 = constrain(manual_power, 0, 100);
        s->print("ManualControl_POWER_Bottom: ");
        s->println(Output2);
        break;

      default:
        s->println("ManualControl. Wrong parameter");
        break;
      }
    }
    else
    {
      s->print("Wrong state. Cant set MANUAL Power.");
      // break;
    }
  }
}

void beep(int **times)
{
  int t = 0;
  do
  {
    Serial.println("$@;");
    t++;
    if (times > t)
    {
      delay(400);
    }

  } while (t < times);
}
