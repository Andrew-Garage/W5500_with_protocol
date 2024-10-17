#define configUSE_TASK_NOTIFICATIONS  1 
#define INCLUDE_vTaskPrioritySet      1
#define INCLUDE_uxTaskPriorityGet     1

#include <stdint.h>
#include <SPI.h>
#include "wizchip_conf.h"
#include "socket.h"
#include "esp_heap_caps.h"

static const int spiClk = 1000000;  //  нормально работает на 10 MHz

SPIClass * vspi = NULL;             // Указатель на экз класса SPI 

portMUX_TYPE wizchip_spinlock = portMUX_INITIALIZER_UNLOCKED;    // ???
TaskHandle_t w5500_task, peripheral_task, print_request_task;
SemaphoreHandle_t NET_semaphore = NULL;
static QueueHandle_t receive_queue;

#define _WIZCHIP_ 5500                                     // Какой чип
#define _WIZCHIP_IO_MODE_ _WIZCHIP_IO_MODE_SPI_VDM_        // Как общаемся с чипом (VDM вариэйбл дата мессадьж)

#define button1_pin           36
#define button2_pin           39
#define button3_pin           34
#define button4_pin           35
#define potentiometr_pin      33
#define freq_converter_pin    26

#define W5500_iterrupt_pin     4
#define ISR_FUNC_work         21
#define ISR_HANDLER_work       2
#define ISR_noty               1


// Поля данных принятого сообщения
#define command_zone           0
#define bits_zone              1
#define bytes_zone             2
#define floats_zone            3


uint32_t ulNotifiedValue;

// Сетевые настройки чипа
wiz_NetInfo WIZNETINFO = {
    .mac = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06}, // MAC-адрес вашего устройства 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
    .ip = {1, 2, 3, 4}, // IP-адрес вашего устройства
    .sn = {255, 255, 255, 0}, // Маска подсети 255, 255, 255, 0
    .gw = {0, 0, 0, 0}, // IP-адрес шлюза 192, 168, 0, 1
    .dns = {0, 0, 0, 0}, // IP-адрес DNS-сервера
    .dhcp = NETINFO_STATIC // Режим настройки сетевых параметров, в данном случае статическая настройка
};

// Сокет
uint8_t socket_0 = 0;               // Какой сокет используем используем для обновления морды на компе
uint16_t port_s0 = 40000;           // Он доступен по этому порту
uint8_t socket_1 = 1;               // Какой сокет используем для получения управления с морды на компе 
uint16_t port_s1 = 40001;           // Он доступен по этому порту
uint8_t IP_comp [] = {1, 2, 3, 5};  // По этому IP сокет будет соединяться с другим хостом

// Всякие данные
uint8_t hello_buffer[] = "Hello, World!";   // Это отправляем на другой хост
uint8_t ADC_buffer [4] = {0, 0, 0, 0};      // Сохраняем значение с АЦП для отправки
uint8_t answer_buf [30] = {3,};             // Сюда будем забивать ответ на запрос
uint8_t receive_buf_0 [1024];                 // Сюда принимаем ответ от другого хоста
uint8_t receive_buf_1 [1024];
uint16_t receive_lenght_0 = 0;                // Сколько байт пришло от удаленного хоста
uint16_t receive_lenght_1 = 0;
uint8_t w5500_task_prio = 4;                // Приоритет w5500 задачи

// uint16_t size = 1;
// uint16_t counter = 0;
// uint8_t *my_Arr = (uint8_t *) heap_caps_malloc(size * sizeof(uint8_t), MALLOC_CAP_8BIT);

//--------------------------------------------------------DICTIONARY---------------------------------------

bool bool_values[] = {0, 0, 0, 0};      // Битовые поля

#define button1 bool_values[0]
#define button2 bool_values[1]
#define button3 bool_values[2]
#define button4 bool_values[3]

uint8_t byte_values[] = {255};            // Байтовые поля

#define freq byte_values[0]

float float_values[] = {0};             // Вещественные поля

#define ADC_value float_values[0]

//---------------------------------------------------------------------------------------------------------

void ISR() {
  if(uxTaskPriorityGetFromISR(w5500_task) == w5500_task_prio) vTaskPrioritySet(w5500_task, w5500_task_prio + 1); // увеличиваем на 1
  digitalWrite(ISR_FUNC_work, HIGH);
  BaseType_t pxHigherPriorityTaskWoken;   // static 
  pxHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(w5500_task, &pxHigherPriorityTaskWoken);
  digitalWrite(ISR_FUNC_work, LOW);
  if (pxHigherPriorityTaskWoken == pdTRUE) {
    digitalWrite(ISR_FUNC_work, HIGH);
    digitalWrite(ISR_FUNC_work, LOW);
    portYIELD_FROM_ISR();       // Не заметил что работает, скорее всего переключает по приоритету
  }
}

void setup() {
  Serial.begin(115200);

  vspi = new SPIClass(VSPI);
  
  //initialise vspi with default pins
  //SCLK = 18, MISO = 19, MOSI = 23, SS = 5
  vspi->begin();
  pinMode(vspi->pinSS(), OUTPUT);  //VSPI SS
  vspi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));     // Без нее почему-то тоже работает

  pinMode(ISR_HANDLER_work, OUTPUT);
  pinMode(ISR_FUNC_work, OUTPUT);
  pinMode(button1_pin, INPUT);
  pinMode(button2_pin, INPUT);
  pinMode(button3_pin, INPUT);
  pinMode(button4_pin, INPUT);

  ledcAttachPin(freq_converter_pin, 0);
  ledcSetup(0, 1000, 8);

  analogReadResolution(12);

  receive_queue = xQueueCreate(30, sizeof(uint8_t));

  reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);                                        // Команды для выбора чипа по SPI через CS 
  reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);                                    // Команды для чтения и записи 1 байта
  reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);                               // Команды для чтения и записи массива байт
  reg_wizchip_cris_cbfunc(wizchip_critical_section_enter, wizchip_critical_section_exit);     // Команды для входа и выхода из CRITICAL SECTION

  xTaskCreatePinnedToCore(w5500_task_code, "Task0", 10000, NULL, w5500_task_prio, &w5500_task, 1);
  xTaskCreatePinnedToCore(peripheral_task_code, "Task1", 10000, NULL, 6, &peripheral_task, 1);
  xTaskCreatePinnedToCore(print_request_task_code, "Task2", 10000, NULL, 4, &print_request_task, 1);
}

void loop() {
}

//---------------------------------------------------------------- W5500 TASK -----------------------------------------
void w5500_task_code(void* pvParameters) {
  w5500_task_init();
  uint8_t interrupt_register = 0;
  eTaskState info;
  uint32_t ulNotifiedValue;
  UBaseType_t my_priority = 3;
    for(;;) {
      if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) > 0) {
      //if(xTaskNotifyWait(ULONG_MAX, ULONG_MAX, &ulNotifiedValue, pdMS_TO_TICKS(300)) == pdPASS) {
        digitalWrite(ISR_HANDLER_work, HIGH);

        interrupt_register =  getSIR();
        
        if(interrupt_register == 2 || interrupt_register == 3) {
          receive_lenght_1 = getSn_RX_RSR(socket_1);                  // Сколько приняли
          recv(socket_1, (uint8_t*) receive_buf_1, receive_lenght_1);   // Принимаем в буфер
          //xTaskNotifyGive(print_request_task);                      // 
          xQueueSend(receive_queue, &socket_1, 100);
          setSn_IR(socket_1, Sn_IR_RECV);
        }
        else if(interrupt_register == 1) {
          receive_lenght_0 = getSn_RX_RSR(socket_0);
        
          recv(socket_0, (uint8_t*) receive_buf_0, receive_lenght_0);

          //xTaskNotifyGive(print_request_task);
          xQueueSend(receive_queue, &socket_0, 100);
          
          //if(receive_buf[0] == 0x01) xTaskNotifyGive(print_request_task);
          //else Serial.println("Command error");

          setSn_IR(socket_0, Sn_IR_RECV);   // Посылаем 1 в бит RECV, чем "гасим" его
        }

        digitalWrite(ISR_HANDLER_work, LOW);
        //send(socket_0, (uint8_t *) &ADC_buffer, sizeof(ADC_buffer)/sizeof(ADC_buffer[0]));
      
        if(uxTaskPriorityGet(NULL) == (w5500_task_prio + 1)) {
          vTaskPrioritySet(NULL, w5500_task_prio);
          //Serial.println("OP");
        } else {
          vTaskPrioritySet(NULL, w5500_task_prio); // уменьшаем до 3
          Serial.println("NE OP");
        }
      }
      else {
        print_socket_state(socket_0);
        print_socket_state(socket_1);
      }
  }
}
//---------------------------------------------------------- PERIPH TASK ------------------------------------------------
void peripheral_task_code(void* pvParameters) {
  byte old_freq = 0;
  const float ADC_step_value = 3.3/4095;
  uint32_t ADC_value_int = 0;
  ledcWrite(0, 0);
  for(;;) {

    ADC_value = analogRead(potentiometr_pin) * ADC_step_value;
    ADC_value_int = *reinterpret_cast<uint32_t*>(&ADC_value);

    ADC_buffer[0] = (ADC_value_int >> 24) & 0xFF;
    //Serial.printf("ADC_buffer[0] = %d \n", ADC_buffer[0]);
    ADC_buffer[1] = (ADC_value_int >> 16) & 0xFF;
    //Serial.printf("ADC_buffer[1] = %d \n", ADC_buffer[1]);
    ADC_buffer[2] = (ADC_value_int >> 8) & 0xFF;
    //Serial.printf("ADC_buffer[2] = %d \n", ADC_buffer[2]);
    ADC_buffer[3] = (ADC_value_int & 0xFF);
    //Serial.printf("ADC_buffer[3] = %d \n", ADC_buffer[3]);

    button1 = digitalRead(button1_pin);
    button2 = digitalRead(button2_pin);
    button3 = digitalRead(button3_pin);
    button4 = digitalRead(button4_pin);

    //  Serial.printf("b1:%d  b2:%d  b3:%d  b4:%d  potentiometr = %.3f \n", button1 ? 1 : 0
    //                                                                  , button2 ? 1 : 0 
    //                                                                  , button3 ? 1 : 0 
    //                                                                  , button4 ? 1 : 0 
    //                                                                  , ADC_value);

    if(freq != old_freq) {
      ledcWrite(0, freq);
      old_freq = freq;
    }
    //if(freq > 0) freq--;
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
//----------------------------------------------- PRINT TASK --------------------------------------------------------
void print_request_task_code(void* pvParameters) {
  uint8_t data_zone = command_zone;
  uint16_t next_value = 0;
  uint8_t command;
  uint8_t task_notify = 0;
  for(;;) {
    //if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {
    if(xQueueReceive(receive_queue, &task_notify, portMAX_DELAY) == pdPASS) {
      data_zone = command_zone;                                // Для разбиения пакета: 1 - биты, 2 - байты, 3 - плавающая точка
      //if(receive_buf[0] == 1) {
      if(task_notify == 0) {
        
        uint16_t size = 1;
        uint16_t counter = 0;
        uint8_t *my_Arr = (uint8_t *) heap_caps_malloc(size * sizeof(uint8_t), MALLOC_CAP_8BIT);
        my_Arr[counter] = 3;

        for(uint16_t i = 1; i < receive_lenght_0; i += 2) {
          next_value = (receive_buf_0[i] << 8) | receive_buf_0[i+1];

          if(next_value == 0xFFFF) {
            //Serial.println("\n Separator");
            data_zone++;

            size = size + 2;
            my_Arr = (uint8_t*) heap_caps_realloc(my_Arr, (size * sizeof(uint8_t)) , MALLOC_CAP_8BIT);
            counter++;
            my_Arr[counter] = 0xFF;
            counter++;
            my_Arr[counter] = 0xFF;

            //if(data_zone == bits_zone) Serial.print("Bits: ");
            //else if(data_zone == bytes_zone) Serial.print("Bytes: ");
            //else if(data_zone == floats_zone) Serial.print("Floats: ");
            continue;
          }
          else {
            //Serial.printf(" %x ", next_value);
            
            if(data_zone == bits_zone) {
              size = size + 2;
              my_Arr = (uint8_t*) heap_caps_realloc(my_Arr, (size * sizeof(uint8_t)) , MALLOC_CAP_8BIT);
              counter++;
              my_Arr[counter] = (uint8_t) (next_value >> 7) & 0xFF;
              counter++;
              my_Arr[counter] = (uint8_t) (next_value << 1) & 0xFF;
              if (bool_values[next_value]) my_Arr[counter] = my_Arr[counter] + 1;
            }
            else if(data_zone == bytes_zone) {
              size = size + 3;
              my_Arr = (uint8_t*) heap_caps_realloc(my_Arr, (size * sizeof(uint8_t)) , MALLOC_CAP_8BIT);
              counter++;
              my_Arr[counter] = (uint8_t) (next_value >> 8) & 0xFF;
              counter++;
              my_Arr[counter] = (uint8_t) next_value & 0xFF;
              counter++;
              my_Arr[counter] = byte_values[next_value];
            }
            else if(data_zone == floats_zone) {
              size = size + 6;
              my_Arr = (uint8_t*) heap_caps_realloc(my_Arr, (size * sizeof(uint8_t)) , MALLOC_CAP_8BIT);
              counter++;
              my_Arr[counter] = (uint8_t) (next_value >> 8) & 0xFF;
              counter++;
              my_Arr[counter] = (uint8_t) next_value & 0xFF;
              for(uint8_t i = 0; i < 4; i++) {
                counter++;
                my_Arr[counter] = ADC_buffer[i];
              }
            }
          }
        }

        send(socket_0, my_Arr, size);

        // Serial.println();
        // for(uint16_t i = 0; i < size; i++) {
        //   Serial.printf("%d  ", my_Arr[i]);
        // }
        // Serial.println();
        // heap_caps_free(my_Arr);

      }
      //else if(receive_buf[0] == 2) {
      else if(task_notify == 1) {
        freq = receive_buf_1[8];
        Serial.printf("Value to change %d\n", receive_buf_1[8]);
      }
      else {
        Serial.printf("Error! Command: %d\n", receive_buf_1[0]);
      }
      
      // Serial.println();
      // for(int i = 0; i < receive_lenght; i++) {
      //   Serial.print(receive_buf[i], HEX);
      // }
      // Serial.println();

    }
  }
}










void w5500_task_init () {
  uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
  wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

  wizchip_setnetinfo(&WIZNETINFO);               // Задаем сетевые настройки common register

  Serial.println();                              // Так надо чтоб не слипалось
  print_ISR_Regs(socket_0);                      // Выводим регистры прерываний нужного сокета и общие
  print_ISR_Regs(socket_1);

  // setSn_MSSR(socket_0, 1460);         // Макс размер пакета
  //Serial.printf("Retry Time-value: %d msec \n", (getRTR() / 10));  // Период на попытки выполнения команд сокетом
  
  uint8_t res = socket(socket_0, Sn_MR_TCP , port_s0, 0); // Должен вернуть номер включаемого сокета, если все ОК
  Serial.printf("\nSocket_0 init: %d \n", res);

  res = socket(socket_1, Sn_MR_TCP , port_s1, 0); // Должен вернуть номер включаемого сокета, если все ОК
  Serial.printf("\nSocket_1 init: %d \n", res);

  print_socket_state(socket_0);          // Смотрим статус регистра 0x13: сокет инициализирован
  print_socket_state(socket_1);

  //res = connect(socket_0, IP_comp, (port_s0 + 1)); // На счет (uint8_t*)& я хз. Подключаем сокет к другому хосту. 1 - соединение установлено.
  res = listen(socket_0);     // 1 - к нам подключились, соединение установлено.
  Serial.printf("We're listening: %d \n", res); 
  Serial.printf("Type of Service: %d \n", getSn_TOS(socket_0)); 
  
  res = listen(socket_1);
  Serial.printf("We're listening: %d \n", res); 
  Serial.printf("Type of Service: %d \n", getSn_TOS(socket_0)); 

  print_socket_state(socket_0);
  print_socket_state(socket_1);

  uint16_t intlevel = 4000;
  setINTLEVEL(intlevel)
  uint16_t qwe = getINTLEVEL();
  Serial.printf("INTLEVEL = %d \n", qwe);

  setSIMR(0b00000011);                  // Маска прерываний общего регистра 0b0000 0001 - включем прерывания по сокету 0
  setSn_IMR(socket_0, Sn_IR_RECV);      // Маска прерываний сокета 0, ставим 1 на прерывание по приему ( RECV )
  setSn_IMR(socket_1, Sn_IR_RECV);

  print_ISR_Regs(socket_0);
  print_ISR_Regs(socket_1);

  Serial.print("Delay");
  for(int i = 0; i <= 5; i++){ 
    vTaskDelay(pdMS_TO_TICKS(200));
    Serial.print(".");
  }

  Serial.println("\nStart");
  portMUX_INITIALIZE(&wizchip_spinlock);
  attachInterrupt(W5500_iterrupt_pin, ISR, FALLING);
}







void print_socket_state(uint8_t sock) {
  Serial.print("Socket state: ");
  switch (getSn_SR(sock)) {
    case 0x00:
      Serial.printf("SOCK_CLOSED socket_%d\n", sock);
      if(sock == 0) socket(sock, Sn_MR_TCP , port_s0, 0);     // Если сокет закрыт, инициализиуем его
      else if(sock == 1) socket(sock, Sn_MR_TCP , port_s1, 0);
      break;
    case 0x13:
      Serial.printf("SOCK_INIT socket_%d\n", sock);
      listen(sock);                             // Если сокет инициализирован, ставим его на прослушку
      break;
    case 0x14:
      Serial.printf("SOCK_LISTEN socket_%d\n", sock);
      break;
    case 0x17:
      Serial.printf("SOCK_ESTABLISHED socket_%d\n", sock);
      break;
    case 0x1C:
      Serial.printf("SOCK_CLOSE_WAIT socket_%d\n", sock);        // Получен запрос на отключение. Для полного отключения надо отключить сокет
      disconnect(sock);
      break;
    case 0x15:
      Serial.printf("SOCK_SYNSENT socket_%d\n", sock);
      break;
    case 0x16:
      Serial.printf("SOCK_SYNRECV socket_%d\n", sock);
      break;
    case 0x18:
      Serial.printf("SOCK_FIN_WAIT socket_%d\n", sock);
      break;
    case 0x1A:
      Serial.printf("SOCK_CLOSING socket_%d\n", sock);
      break;
    case 0x1B:
      Serial.printf("SOCK_TIME_WAIT socket_%d\n", sock);
      break;
    case 0x1D:
      Serial.printf("SOCK_LAST_ACK socket_%d\n", sock);
      break;
    default:
      Serial.println("UNDEFINED");
  }
}

void print_ISR_Regs (uint8_t sn) {
  Serial.print("IR = ");
  Serial.print( getIR() , BIN );
  
  Serial.print("   IMR = ");
  Serial.print( getIMR() , BIN );

  Serial.print("   SIR = ");
  Serial.print( getSIR() , BIN );

  Serial.print("   SIMR = ");
  Serial.print( getSIMR() , BIN );

  if(sn == 0) {
    Serial.print("   S0_IR = ");
    Serial.print( getSn_IR(sn), BIN );

    Serial.print("   S0_IMR = ");
    Serial.println( getSn_IMR(sn), BIN );
  }
  else if(sn == 1) {
    Serial.print("   S1_IR = ");
    Serial.print( getSn_IR(sn), BIN );

    Serial.print("   S1_IMR = ");
    Serial.println( getSn_IMR(sn), BIN );
  }
}

void W5500_Select(void) {
  digitalWrite(vspi->pinSS(), LOW);
}

void W5500_Unselect(void) {
  digitalWrite(vspi->pinSS(), HIGH);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
  //HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
  //void transferBytes(const uint8_t * data, uint8_t * out, uint32_t size);
  vspi->transferBytes(0, buff, len);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
  //HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
  //void transferBytes(const uint8_t * data, uint8_t * out, uint32_t size);
  vspi->transferBytes(buff, 0, len);
}

uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    byte = vspi->transfer(0);
    return byte;
}

void W5500_WriteByte(uint8_t byte) {
    vspi->transfer(byte);
}

static void wizchip_critical_section_enter(void) {
  // вход в критическую секцию
  taskENTER_CRITICAL(&wizchip_spinlock);
}

static void wizchip_critical_section_exit(void) {
  // выход из критической секции  
  taskEXIT_CRITICAL(&wizchip_spinlock);
}



      //send(socket_0, (uint8_t *) &hello_buffer, sizeof(hello_buffer)/sizeof(hello_buffer[0]));
      //send(socket_0, (uint8_t *) &ADC_buffer, sizeof(ADC_buffer)/sizeof(ADC_buffer[0]));
      //for(uint16_t i = 0; i <= 1000; i++) {  getSUBR(answer_buf);  }