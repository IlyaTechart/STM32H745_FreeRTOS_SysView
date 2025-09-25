/*
 * segger_uart_v2.c
 *
 *  Created on: Sep 25, 2025
 *      Author: q
 */

#include "SEGGER_SYSVIEW.h"

#if (SEGGER_UART_REC == 1) // Убедитесь, что этот макрос определен как 1 в SEGGER_SYSVIEW_Conf.h

#include "SEGGER_RTT.h"
#include "stm32h7xx.h"

// ВАЖНО: Установите это значение равным частоте вашего ядра D1 (HCLK)
// Обычно для H745 это 400МГц или 480МГц. Проверьте в CubeMX!
#define OS_FSYS 400000000UL

//
// =================== ИСПРАВЛЕНИЯ ДЛЯ STM32H7 ===================
//
// Базовые адреса для STM32H7
#define RCC_BASE_ADDR       0x58024400UL
#define GPIOD_BASE_ADDR     0x58020C00UL
#define USART3_BASE_ADDR    0x40004800UL

// Смещения регистров RCC для STM32H7
#define OFF_AHB4ENR         0x0F4 // GPIOx находятся на шине AHB4
#define OFF_APB1LENR        0x0E8 // USART2/3/4/5/7/8 находятся на шине APB1

// Регистры RCC
#define RCC_AHB4ENR         *(volatile uint32_t*)(RCC_BASE_ADDR + OFF_AHB4ENR)
#define RCC_APB1LENR        *(volatile uint32_t*)(RCC_BASE_ADDR + OFF_APB1LENR)

// Смещения регистров GPIO (остались те же)
#define OFF_MODER           0x00
#define OFF_AFRH            0x24

// Смещения регистров USART (SR стал ISR, остальные в основном те же)
#define OFF_ISR             0x1C        // Interrupt and Status register (вместо SR)
#define OFF_BRR             0x0C        // Baudrate register
#define OFF_TDR             0x28        // Transmit Data Register
#define OFF_RDR             0x24        // Receive Data Register
#define OFF_CR1             0x00        // Control register 1
#define OFF_CR2             0x04        // Control register 2
#define OFF_CR3             0x08        // Control register 3

// ВАЖНО: Убедитесь, что частота шины APB1 для USART3 соответствует этой формуле.
// Проверьте предделитель APB1 в CubeMX. Если он /2, то формула верна.
#define UART_BASECLK        (HAL_RCC_GetPCLK1Freq()) // Более надежный способ получить частоту

// Регистры USART
#define USART_ISR           *(volatile uint32_t*)(USART3_BASE_ADDR + OFF_ISR)
#define USART_BRR           *(volatile uint32_t*)(USART3_BASE_ADDR + OFF_BRR)
#define USART_RDR           *(volatile uint32_t*)(USART3_BASE_ADDR + OFF_RDR)
#define USART_TDR           *(volatile uint32_t*)(USART3_BASE_ADDR + OFF_TDR)
#define USART_CR1           *(volatile uint32_t*)(USART3_BASE_ADDR + OFF_CR1)
#define USART_CR2           *(volatile uint32_t*)(USART3_BASE_ADDR + OFF_CR2)
#define USART_CR3           *(volatile uint32_t*)(USART3_BASE_ADDR + OFF_CR3)

// Флаги в регистре ISR (номера битов те же, что и в SR)
#define USART_RXNE_POS          5     // Read data register not empty flag
#define USART_TC_POS            6     // Transmission complete flag
#define USART_TXE_POS           7     // Transmit data register empty

// Маски для флагов
#define USART_RXNE_FLAG         (1UL << USART_RXNE_POS)
#define USART_TC_FLAG           (1UL << USART_TC_POS)
#define USART_TXE_FLAG          (1UL << USART_TXE_POS)
#define USART_RX_ERROR_FLAGS    (1UL << 3 | 1UL << 1 | 1UL << 0) // ORE, FE, PE flags

// Биты в регистрах управления (номера те же)
#define USART_RXNEIE_POS        5     // RXNE interrupt enable (CR1)
#define USART_TXEIE_POS         7     // TXE interrupt enable (CR1)

// ================= КОНЕЦ ИСПРАВЛЕНИЙ ДЛЯ STM32H7 =================

typedef void UART_ON_RX_FUNC(uint8_t Data);
typedef int  UART_ON_TX_FUNC(uint8_t* pChar);

static UART_ON_RX_FUNC* _cbOnRx;
static UART_ON_TX_FUNC* _cbOnTx;

void HIF_UART_Init(uint32_t Baudrate, UART_ON_TX_FUNC* cbOnTx, UART_ON_RX_FUNC* cbOnRx);

static const U8 _abHelloMsg[] = { 'S', 'V', (SEGGER_SYSVIEW_VERSION / 10000), (SEGGER_SYSVIEW_VERSION / 1000) % 10 };
#define _TARGET_HELLO_SIZE        (sizeof(_abHelloMsg))
#define _SERVER_HELLO_SIZE        (4)

static struct {
  U8         NumBytesHelloRcvd;
  U8         NumBytesHelloSent;
  int        ChannelID;
} _SVInfo = {0, 0, 1};

static void _StartSysView(void) {
  if (SEGGER_SYSVIEW_IsStarted() == 0) {
    SEGGER_SYSVIEW_Start();
  }
}

static void _cbOnUARTRx(U8 Data) {
  if (_SVInfo.NumBytesHelloRcvd < _SERVER_HELLO_SIZE) {
    _SVInfo.NumBytesHelloRcvd++;
    return;
  }
  _StartSysView();
  SEGGER_RTT_WriteDownBuffer(_SVInfo.ChannelID, &Data, 1);
}

static int _cbOnUARTTx(U8* pChar) {
  if (_SVInfo.NumBytesHelloSent < _TARGET_HELLO_SIZE) {
    *pChar = _abHelloMsg[_SVInfo.NumBytesHelloSent];
    _SVInfo.NumBytesHelloSent++;
    return 1;
  }
  int r = SEGGER_RTT_ReadUpBufferNoLock(_SVInfo.ChannelID, pChar, 1);
  return (r < 0) ? 0 : r;
}

void SEGGER_UART_init(U32 baud) {
    HIF_UART_Init(baud, _cbOnUARTTx, _cbOnUARTRx);
}
void HIF_UART_EnableTXEInterrupt(void) {
  USART_CR1 |= (1UL << USART_TXEIE_POS);
}


void USART3_IRQHandler(void);
void USART3_IRQHandler(void) {
  uint32_t IsrStatus = USART_ISR;
  uint8_t v;
  int r;

  // Прерывание по приему данных (RXNE)
  if (IsrStatus & USART_RXNE_FLAG) {
    v = (uint8_t)USART_RDR;
    if ((IsrStatus & USART_RX_ERROR_FLAGS) == 0) {
      if (_cbOnRx) {
        _cbOnRx(v);
      }
    }
  }

  // Прерывание по пустому регистру передачи (TXE)
  if (IsrStatus & USART_TXE_FLAG) {
    if (_cbOnTx) {
      r = _cbOnTx(&v);
      if (r == 0) {
        USART_CR1 &= ~(1UL << USART_TXEIE_POS); // Отключаем прерывание TXE, если нечего отправлять
      } else {
        USART_TDR = v; // Отправляем следующий байт
      }
    }
  }
}

void HIF_UART_Init(uint32_t Baudrate, UART_ON_TX_FUNC* cbOnTx, UART_ON_RX_FUNC* cbOnRx) {
  // 1. Включение тактирования периферии
  RCC_APB1LENR |= (1 << 18); // Включить тактирование USART3
  RCC_AHB4ENR  |= (1 << 3);  // Включить тактирование GPIOD

  // 2. НАСТРОЙКА ПИНОВ PD8 и PD9
  // ВАЖНО!!! ЭТОТ ШАГ ЛУЧШЕ ВСЕГО СДЕЛАТЬ В CUBEMX!
  // В CubeMX:
  //   - Выберите PD8 -> USART3_TX
  //   - Выберите PD9 -> USART3_RX
  //   - CubeMX автоматически сгенерирует код для настройки GPIO_MODER и GPIO_AFRH.
  //   - Если вы не используете CubeMX, раскомментируйте и проверьте этот код:
  /*
  // Установка альтернативной функции AF7 для PD8 и PD9
  volatile uint32_t v_afrh = GPIOD->AFR[1]; // AFRH - это AFR[1]
  v_afrh &= ~((0xF << ((8 - 8) * 4)) | (0xF << ((9 - 8) * 4))); // Очистка битов для PD8, PD9
  v_afrh |=  ((7   << ((8 - 8) * 4)) | (7   << ((9 - 8) * 4))); // Установка AF7
  GPIOD->AFR[1] = v_afrh;

  // Установка режима "Alternate Function" для PD8 и PD9
  volatile uint32_t v_moder = GPIOD->MODER;
  v_moder &= ~((3UL << (8 * 2)) | (3UL << (9 * 2))); // Очистка битов для PD8, PD9
  v_moder |=  ((2UL << (8 * 2)) | (2UL << (9 * 2))); // Установка режима AF
  GPIOD->MODER = v_moder;
  */

  // 3. Расчет и установка скорости (Baudrate)
  USART_BRR = (uint16_t)(UART_BASECLK / Baudrate);

  // 4. Настройка и включение USART
  USART_CR1 = 0; // Сброс
  USART_CR1 = (1 << 0)  // UE: USART Enable
            | (1 << 2)  // RE: Receiver Enable
            | (1 << 3)  // TE: Transmitter Enable
            | (1 << 5); // RXNEIE: RXNE Interrupt Enable

  USART_CR2 = 0; // 1 стоп-бит по умолчанию
  USART_CR3 = 0; // Без DMA и прочего

  // 5. Настройка прерываний в NVIC
  _cbOnRx = cbOnRx;
  _cbOnTx = cbOnTx;
  NVIC_SetPriority(USART3_IRQn, 6);
  NVIC_EnableIRQ(USART3_IRQn);

  // Запускаем передачу "Hello" сообщения
  USART_CR1 |= (1UL << USART_TXEIE_POS); // Включаем прерывание TXE для начала передачи
}

#endif


