/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_rcc.h"

#include <stdlib.h>  // for atoi


/* Reserve Flash space for channel storage (10 channels * 2KB each) */
__attribute__((section(".user_data"))) const uint8_t flash_channels[10 * 2048];
/* Reserve one Flash page (1KB) for occupied channel map */
__attribute__((section(".occupied"))) const uint8_t occupied_page[1024];

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* IR signal processing thresholds */
#define BURST_GAP_US    70        /* Max duration to consider as part of a burst (carrier) */
#define MAX_SEGMENTS    256       /* Max number of burst/idle segments per signal */
#define MAX_DIFFS       3000      /* Reduced from 4096 to save RAM (8KB) */
#define TIMER_FREQ_HZ   1000000   /* TIM2 runs at 1 MHz → 1 µs per tick */
#define END_GAP_US      500000    /* 500 ms silence to force end of capture (fallback) */

/* Signal validation thresholds */
#define MIN_CARRIER_COUNT 5          /* Minimum pulses to calculate carrier frequency */
#define MIN_TOTAL_DURATION_US 50000  /* Minimum total signal duration (50 ms) */
#define MAX_TOTAL_DURATION_US 200000 /* Maximum total signal duration (200 ms) */
#define MIN_BURST_DURATION_US 100    /* Minimum burst length to reject noise */
#define MIN_IDLE_DURATION_US  50     /* Minimum idle length to reject short gaps */
#define CARRIER_FREQ_MIN_HZ   30000  /* Min carrier frequency (30 kHz) */
#define CARRIER_FREQ_MAX_HZ   50000  /* Max carrier frequency (50 kHz) */
#define MIN_DIFFS_FOR_PROCESS 10     /* Ignore signals with fewer diffs (noise) */

/* Flash memory layout */
#define FLASH_END_ADDRESS           0x08010000  /* End of 64KB Flash (base + 64KB) */
#define MY_FLASH_PAGE_SIZE          1024U       /* Page size (renamed to avoid conflict(redefining FLASH_PAGE_SIZE error)) */
#define FLASH_OCCUPIED_START ((uint32_t)occupied_page)
#define FLASH_CHANNELS_START ((uint32_t)flash_channels)
#define CHANNEL_SIZE_IN_FLASH       (MY_FLASH_PAGE_SIZE * PAGES_PER_CHANNEL) // حالا با MY_ استفاده می‌شه

/* IR channel configuration */
#define MAX_CHANNELS    10
#define PAGES_PER_CHANNEL    2   /* 2 pages = 2KB per channel */

#define IDLE_THRESHOLD 6  /* 6 * 5s = 30s idle timeout */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* EXTI flag for keypad interrupt */
volatile uint8_t exti_flag = 0;

/* Raw diff buffer (timing between edges) */
volatile uint32_t diffBuffer[MAX_DIFFS];
volatile uint32_t diffIndex = 0;

/* Capture state flags */
volatile uint8_t captureActive = 1;       /* Set to 0 to stop capture */
volatile uint8_t signalDetected = 0;      /* First valid edge detected */

/* Structure to hold processed IR signal data */
typedef struct {
    uint32_t carrierFreq;
    uint16_t segmentCount;
    uint8_t  type[MAX_SEGMENTS];      /* 0 = Idle, 1 = Burst */
    uint32_t duration[MAX_SEGMENTS];  /* microseconds */
} IR_RawData;

/* Current captured signal (volatile because used in ISR) */
volatile IR_RawData irData;

volatile uint32_t lastCapture = 0;
volatile uint8_t signalReady = 0;     /* Set when capture finished or overflow */
volatile uint8_t overflowError = 0;   /* Diff buffer overflow flag */

//phase 3
/* Temporary buffer for Flash read/write (one channel) */
IR_RawData tempChannelData;

/* State machine states */
typedef enum {
    IR_IDLE,         /* Waiting for signal or user input */
    IR_RECEIVING,    /* Capture in progress */
    IR_READY,        /* Signal ready, waiting for save/cancel */
    IR_TRANSMITTING, /* Replaying stored signal */
	IR_SELECTING     /* User selected channel, waiting for C (send) or D (delete) */
} IR_State;

volatile IR_State currentState = IR_IDLE;

/* Channel occupancy map (0 = free, 1 = occupied) */
uint8_t channelOccupied[MAX_CHANNELS] = {0};

/* Keyboard input buffer (max 2 digits + null terminator) */
uint8_t selectedChannel = 0;
uint8_t keyboardBuffer[3] = {0};
uint8_t keyboardIndex = 0;

/* User timeout duration (not used dynamically yet, kept for future) */
//uint32_t timeoutDuration = 0;

/* Button flags for polling in main loop (not used in current implementation) */
//uint8_t savePressed = 0, cancelPressed = 0, sendPressed = 0, deletePressed = 0;

/* Word-aligned buffer for Flash programming (BSS) */
static uint32_t saveBuf[(sizeof(IR_RawData) + 3) / 4];

//phase 4
/* Idle timeout handling for standby mode */
volatile uint32_t lastActivityTime = 0;
const uint32_t IDLE_TIMEOUT_MS = 30000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

//phase 3
/* Channel management */
static uint8_t FindNextFreeChannel(void);
static void SaveChannel(uint8_t ch);
static void DeleteChannel(uint8_t ch);

/* Flash operations */
HAL_StatusTypeDef SaveToFlash(uint8_t channelIndex, IR_RawData* data);
void LoadFromFlash(uint8_t channelIndex, IR_RawData* destination);
static void LoadOccupiedFromFlash(void);  /* Load channel occupancy from Flash at startup */
static HAL_StatusTypeDef SaveOccupiedToFlash(void);

/* User interface */
static uint8_t ScanKeyboard(void);  /* Scan 4x4 keypad, return ASCII key or 0 */
static void ProcessKeyboardInput(uint8_t key);
static void LED_Pattern(uint8_t type);  /* type: 1=save, 2=cancel, 3=send, 4=delete, 5=error */

/* Debug helpers */
static void DumpOccupiedFlashRaw(void);

/* User timeout management (TIM1) */
static inline void StartUserTimeout(void);
static inline void StopUserTimeout(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Initializes and starts IR capture mode.
 *        Resets buffers, flags, and starts TIM2 input capture interrupt.
 */
void IR_CaptureStart(void)
{
    memset((void*)diffBuffer, 0, sizeof(diffBuffer));
    diffIndex = 0;
    signalReady = 0;
    overflowError = 0;
    lastCapture = 0;
    captureActive = 1;
    signalDetected = 0;

    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    currentState = IR_IDLE;
	printf("Entered receiving state.\r\n");

}

/**
 * @brief TIM2 input capture interrupt callback.
 *        Measures time between falling edges, stores diffs in buffer,
 *        and starts TIM4 timeout on first valid edge.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2 || !captureActive) return;

    uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t diff = (now >= lastCapture) ? (now - lastCapture) : (0xFFFF - lastCapture + now + 1);

    /* Skip first capture (no valid diff) */
    if (lastCapture == 0) {
            lastCapture = now;
            return;
    }

    if (diffIndex < MAX_DIFFS) {
        diffBuffer[diffIndex++] = diff;
    } else {
    	/* Buffer overflow: stop capture and signal ready for processing */
        overflowError = 1;
        captureActive = 0;
        HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
        HAL_TIM_Base_Stop_IT(&htim4);
        signalReady = 1;
    }

    lastCapture = now;

    /* Start TIM4 timeout only on the first valid edge */
    if (!signalDetected) {
        signalDetected = 1;
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        HAL_TIM_Base_Start_IT(&htim4);
        currentState = IR_RECEIVING;

    }
}

/**
 * @brief TIM period elapsed callback (used by TIM4 and TIM1).
 *        TIM4: end-of-frame detection, stops capture.
 *        TIM1: user decision timeout (5s), discards signal or selection.
 */
__attribute__((used)) void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
    	/* End of capture timeout (no signal edge for 250ms) */
        if (captureActive) {
            captureActive = 0;
            HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
            HAL_TIM_Base_Stop_IT(&htim4);
            signalReady = 1;  /* Signal ready for processing */
        }
        return;
    }

    if (htim->Instance == TIM1) {

    	/* 5-second user timeout window (ready or channel selection) */
    	StopUserTimeout();

        if (currentState == IR_READY) {
        	lastActivityTime = HAL_GetTick();
        	printf("Timeout in ready state: Signal discarded.\r\n");
            //LED_Pattern(2);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* Turn LED off */
            currentState = IR_IDLE;
            selectedChannel = 0;
            keyboardIndex = 0;
            memset(keyboardBuffer, 0, sizeof(keyboardBuffer));
            //printf("Timeout in ready state: Signal discarded.\r\n");
            IR_CaptureStart(); /* Restart capture */
        }
        else if (currentState == IR_SELECTING && selectedChannel > 0) {
        	lastActivityTime = HAL_GetTick();
            printf("Timeout in channel selection: Channel %d discarded.\r\n", selectedChannel);
            selectedChannel = 0;
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* Turn LED off */
            currentState = IR_IDLE;
            IR_CaptureStart();
        }

        return;
    }
}


/**
 * @brief Retarget printf to UART1.
 * @param file unused
 * @param ptr data buffer
 * @param len length
 * @return number of bytes written
 */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/**
 * @brief Show visual feedback on LED (PC13, active low).
 * @param type 1=Save, 2=Cancel, 3=Send, 4=Delete, 5=Error
 */
void LED_Pattern(uint8_t type) {
    switch (type) {
        case 1:  /* Save: 3 quick blinks */
            for (int i = 0; i < 3; i++) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                HAL_Delay(150);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                HAL_Delay(150);
            }
            break;
        case 2:  /* Save: 3 quick blinks */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            HAL_Delay(500);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            HAL_Delay(200);
            break;
        case 3:  /* Send: 2 medium blinks */
            for (int i = 0; i < 2; i++) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                HAL_Delay(300);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                HAL_Delay(300);
            }
            break;
        case 4:  /* Delete: 4 fast blinks */
            for (int i = 0; i < 4; i++) {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
                HAL_Delay(100);
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
                HAL_Delay(100);
            }
            break;
        case 5:  /* Error: 5 very fast blinks */
			for (int i = 0; i < 5; i++) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_Delay(50);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_Delay(50);
			}
			break;
        default:
            break;
    }
}

/**
 * @brief Dynamically set TIM3 PWM frequency for IR carrier.
 * @param carrierHz Desired carrier frequency (30-56 kHz). Out-of-range defaults to 38 kHz.
 */
void TIM3_SetCarrierFreq(uint32_t carrierHz)
{
    if (carrierHz < 30000 || carrierHz > 50000) {
        carrierHz = 38000; /* Default to 38 kHz */
    }

    /* Calculate timer clock (APB1) */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();  // HCLK usually = SYSCLK
    uint32_t tim_clk = (pclk1 == hclk) ? pclk1 : 2 * pclk1;  /* If APB1 prescaler >1, timer clock = 2*PCLK1 */

    /* Adjust prescaler to avoid 16-bit overflow */
    uint32_t prescaler = 0;
    uint32_t period = tim_clk / carrierHz;
    while (period > 65535) {
        prescaler++;
        period = (tim_clk / (prescaler + 1)) / carrierHz;
    }
    if (period == 0) period = 1;  // prevents division by zero
    period -= 1; /* Final ARR value */

    /* Apply settings without reinitializing the timer */
    __HAL_TIM_SET_PRESCALER(&htim3, prescaler);
    __HAL_TIM_SET_AUTORELOAD(&htim3, period);

    /* Duty cycle ≈ 33% (can be changed to 50% by using period/2) */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, period / 2);
}

/**
 * @brief Enable PWM output on TIM3 channel 1.
 */
static inline void PWM_On(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

/**
 * @brief Disable PWM output on TIM3 channel 1.
 */
static inline void PWM_Off(void) {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

/**
 * @brief Microsecond delay using DWT cycle counter.
 * @param us Delay in microseconds
 */
static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000) * us;
    while ((DWT->CYCCNT - start) < cycles);
}

/**
 * @brief Replay a stored IR signal.
 * @param data Pointer to IR_RawData structure (carrier freq, segments, durations)
 */
void IR_Transmit(volatile IR_RawData *data)
{
    if (data->segmentCount == 0) {
        printf("Error: No segments to transmit.\r\n");
        return;
    }
    currentState = IR_TRANSMITTING;
	printf("Transmitting channel %d...\r\n", selectedChannel);
	LED_Pattern(3);   /* Send pattern */

    TIM3_SetCarrierFreq(data->carrierFreq); /* Dynamic frequency setting based on captured carrier */

    for (uint16_t i = 0; i < data->segmentCount; i++) {
        if (data->type[i] == 1) {
            PWM_On();
        } else {
            PWM_Off();
        }
        delay_us(data->duration[i]);
    }
    PWM_Off(); /* Final off */
	printf("Transmit complete.\r\n");
	currentState = IR_IDLE;
	selectedChannel = 0;
	IR_CaptureStart();  /* Back to idle, ready for next capture */
}

/**
 * @brief Process raw diff buffer and extract IR signal segments.
 * @return 1 if valid signal, 0 if discarded (noise or invalid)
 */
uint8_t ProcessDiffBuffer(void) {
	/* Too few diffs – likely noise */
	if (diffIndex < MIN_DIFFS_FOR_PROCESS) return 0;

	/* Step 1: Calculate total signal duration */
    uint32_t totalDuration = 0;
    for (uint32_t i = 0; i < diffIndex; i++) {
        totalDuration += diffBuffer[i];
    }

    /* Basic sanity checks */
    if (totalDuration < MIN_TOTAL_DURATION_US || totalDuration > MAX_TOTAL_DURATION_US || diffIndex < 10) {
        printf("Discard: Invalid total duration (%lu us) or too few diffs (%lu)\r\n", totalDuration, diffIndex);
        return 0;
    }

    /* Step 2: Calculate carrier frequency from short diffs (burst pulses) */
    uint32_t carrierSum = 0;
    uint32_t carrierCount = 0;
    for (uint32_t i = 0; i < diffIndex; i++) {
        if (diffBuffer[i] > 5 && diffBuffer[i] <= BURST_GAP_US) {  // carrier diffs (5-200us)
            carrierSum += diffBuffer[i];
            carrierCount++;
        }
    }
    uint32_t carrierFreq = 0;
    if (carrierCount >= MIN_CARRIER_COUNT) {
        uint32_t avgPeriod = carrierSum / carrierCount;
        carrierFreq = TIMER_FREQ_HZ / avgPeriod;
    } else {
        printf("Discard: Not enough carrier pulses (%lu < %d)\r\n", carrierCount, MIN_CARRIER_COUNT);
        return 0;
    }

    /* Step 3: Extract burst/idle segments from diff array */
    memset((void*)&irData, 0, sizeof(irData));
    uint8_t currentType = 1;  /* Assume starting with burst */
    uint32_t accumulatedDuration = diffBuffer[0];
    irData.segmentCount = 0;

    for (uint32_t i = 1; i < diffIndex; i++) {
        uint32_t diff = diffBuffer[i];
        if (diff <= BURST_GAP_US) {
            if (currentType == 1) {
                accumulatedDuration += diff;  /* Continue current burst */
            } else {
            	/* End of idle, start new burst */
                if (irData.segmentCount < MAX_SEGMENTS) {
                    irData.type[irData.segmentCount] = 0;  /* Idle segment */
                    irData.duration[irData.segmentCount++] = accumulatedDuration;
                }
                currentType = 1;
                accumulatedDuration = diff;
            }
        } else {
        	/* This diff is part of an idle gap */
            if (currentType == 0) {
                accumulatedDuration += diff;  /* Continue current idle */
            } else {
            	/* End of burst, start new idle */
                if (irData.segmentCount < MAX_SEGMENTS) {
                    irData.type[irData.segmentCount] = 1;  /* Burst segment */
                    irData.duration[irData.segmentCount++] = accumulatedDuration;
                }
                currentType = 0;
                accumulatedDuration = diff;
            }
        }
    }
    /* Add the final segment */
    if (accumulatedDuration > 0 && irData.segmentCount < MAX_SEGMENTS) {
        irData.type[irData.segmentCount] = currentType;
        irData.duration[irData.segmentCount++] = accumulatedDuration;
    }

    /* Check for segment overflow */
    if (irData.segmentCount >= MAX_SEGMENTS) {
        printf("Discard: Segment overflow (%d >= %d)\r\n", irData.segmentCount, MAX_SEGMENTS);
        return 0;
    }

    /* Step 4: Final validation checks */
    if (overflowError) {
        printf("Discard: Diff buffer overflow\r\n");
        return 0;
    }
    if (carrierFreq < CARRIER_FREQ_MIN_HZ || carrierFreq > CARRIER_FREQ_MAX_HZ) {
        printf("Discard: Invalid carrier freq (%lu Hz)\r\n", carrierFreq);
        return 0;
    }
    for (uint16_t i = 0; i < irData.segmentCount; i++) {
        if (irData.type[i] == 1 && irData.duration[i] < MIN_BURST_DURATION_US) {
            printf("Discard: Short burst (%lu us < %d)\r\n", irData.duration[i], MIN_BURST_DURATION_US);
            return 0;
        }
        if (irData.type[i] == 0 && irData.duration[i] < MIN_IDLE_DURATION_US) {
            printf("Discard: Short idle (%lu us < %d)\r\n", irData.duration[i], MIN_IDLE_DURATION_US);
            return 0;
        }
    }

    /* All checks passed */
    irData.carrierFreq = carrierFreq;
    return 1;
}

/**
 * @brief Save channel occupancy map to Flash (page at FLASH_OCCUPIED_START).
 * @return HAL_OK on success, otherwise error code.
 */
static HAL_StatusTypeDef SaveOccupiedToFlash(void) {
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    const uint16_t words = (sizeof(channelOccupied) + 3) / 4;
    uint32_t occBuf[(sizeof(channelOccupied) + 3) / 4];

    /* Prepare buffer: pad with 0xFF then copy actual bytes */
    for (uint16_t i = 0; i < words; ++i) occBuf[i] = 0xFFFFFFFFu;
    memcpy(occBuf, channelOccupied, sizeof(channelOccupied));

    /* Disable capture IRQs during Flash write (safer than full __disable_irq) */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_OCCUPIED_START;
    EraseInitStruct.NbPages = 1;

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    if (status != HAL_OK) {
        uint32_t ferr = HAL_FLASH_GetError();
        printf("SaveOccupiedToFlash: Erase failed. status=%ld, FLASH error=0x%08lX, PageError=0x%08lX\r\n",
               (long)status, (unsigned long)ferr, (unsigned long)PageError);
        HAL_FLASH_Lock();
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
        return status;
    }

    uint32_t addr = FLASH_OCCUPIED_START;
    for (uint16_t i = 0; i < words; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, occBuf[i]);
        if (status != HAL_OK) {
            uint32_t ferr = HAL_FLASH_GetError();
            printf("SaveOccupiedToFlash: Program failed at 0x%08lX. FLASH error=0x%08lX\r\n", (unsigned long)addr, (unsigned long)ferr);
            HAL_FLASH_Lock();
            HAL_NVIC_EnableIRQ(TIM4_IRQn);
            HAL_NVIC_EnableIRQ(TIM2_IRQn);
            return status;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();

    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    return HAL_OK;
}



/**
 * @brief Load channel occupancy map from Flash into RAM.
 *        Handles empty (0xFF) or corrupted pages by resetting map to all free.
 */
static void LoadOccupiedFromFlash(void) {
    uint8_t temp[sizeof(channelOccupied)];
    memcpy(temp, (void*)FLASH_OCCUPIED_START, sizeof(temp));

    /* Debug: print raw bytes */
    printf("Raw occupied flash bytes: ");
    for (int i = 0; i < (int)sizeof(temp); i++) {
        printf("%02X ", temp[i]);
    }
    printf("\r\n");

    /* Check if page is completely erased (0xFF) – new / empty */
    uint8_t all_ff = 1;
    for (int i = 0; i < (int)sizeof(temp); i++) {
        if (temp[i] != 0xFF) { all_ff = 0; break; }
    }
    if (all_ff) {
        memset(channelOccupied, 0, sizeof(channelOccupied));
        printf("Occupied status empty (0xFF). Defaults set (all free).\r\n");
        return;
    }

    /* Validate each byte: only 0x00, 0x01 or 0xFF are allowed */
    uint8_t valid = 1;
    for (int i = 0; i < (int)sizeof(temp); i++) {
        if (!(temp[i] == 0x00 || temp[i] == 0x01 || temp[i] == 0xFF)) { valid = 0; break; }
    }

    if (valid) {
        for (int i = 0; i < MAX_CHANNELS; i++) {
            channelOccupied[i] = (temp[i] == 0x01) ? 1 : 0;
        }
        printf("Occupied status loaded from flash (validated).\r\n");
        /* Attempt to erase the page manually (SaveOccupiedToFlash would write current RAM, which is not intended here) */
    } else {
        printf("Occupied page invalid/corrupt. Erasing occupied page and resetting RAM map.\r\n");
        if (SaveOccupiedToFlash() == HAL_OK) {

        	/* SaveOccupiedToFlash erases and writes current channelOccupied (already zeroed later), but we just want erase.
        	So we do a manual erase instead: */
            FLASH_EraseInitTypeDef EraseInitStruct;
            uint32_t PageError = 0;
            HAL_FLASH_Unlock();
            EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.PageAddress = FLASH_OCCUPIED_START;
            EraseInitStruct.NbPages = 1;
            HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
            HAL_FLASH_Lock();
        } else {
        	/* Erase failed, but still reset RAM */
        }
        memset(channelOccupied, 0, sizeof(channelOccupied));
        printf("Occupied status reset in RAM.\r\n");
    }

    printf("Occupied map: ");
    for (int i = 0; i < MAX_CHANNELS; i++) {
        printf("%d", channelOccupied[i] ? 1 : 0);
        if (i < MAX_CHANNELS-1) printf(",");
    }
    printf("\r\n");
}


/**
 * @brief Wrapper to safely save a captured signal to Flash.
 *        Disables capture IRQs, writes to Flash, updates occupancy map.
 * @param ch Channel number (1‑based)
 */
static void SaveChannel(uint8_t ch) {
    if (ch < 1 || ch > MAX_CHANNELS) return;
    ch--; /* Convert to 0-based index */

    /* Validate captured data */
    if (irData.segmentCount == 0 || irData.segmentCount > MAX_SEGMENTS) {
        printf("Error: invalid capture; not saved.\r\n");
        LED_Pattern(5);
        return;
    }

    /* Stop capture interrupts to avoid interference during Flash write */
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Base_Stop_IT(&htim4);
    // optionally stop main polling briefly
    __disable_irq();   /* Also disable all interrupts for safety */

    HAL_StatusTypeDef st = SaveToFlash(ch, (IR_RawData*)&irData);

    __enable_irq();
    // Restore capture (IR_CaptureStart will also set state)
    if (st == HAL_OK) {
        channelOccupied[ch] = 1;
        if (SaveOccupiedToFlash() != HAL_OK) {
            printf("Warning: failed to persist occupied map.\r\n");
        }
        printf("Channel %d saved.\r\n", ch + 1);
        LED_Pattern(1);
    } else {
        printf("Error saving channel %d. Flash error: %lu\r\n", ch + 1, HAL_FLASH_GetError());
        LED_Pattern(5);
    }

    IR_CaptureStart();  /* Restart capture */
}

/**
 * @brief Low-level Flash write for one channel.
 *        Erases the channel's Flash pages, then programs word by word.
 * @param channelIndex 0-based channel index
 * @param data Pointer to IR_RawData to store
 * @return HAL_OK on success, otherwise error code.
 */
HAL_StatusTypeDef SaveToFlash(uint8_t channelIndex, IR_RawData* data)
{
    if (channelIndex >= MAX_CHANNELS) return HAL_ERROR;

    uint32_t dataSize = sizeof(IR_RawData);
    if (dataSize > CHANNEL_SIZE_IN_FLASH) {
        printf("Error: Data size (%lu) exceeds channel size (%d)\r\n", dataSize, CHANNEL_SIZE_IN_FLASH);
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    uint32_t targetAddress = FLASH_CHANNELS_START + (channelIndex * CHANNEL_SIZE_IN_FLASH);

    /* Check for Flash overflow */
    if (targetAddress + CHANNEL_SIZE_IN_FLASH > FLASH_END_ADDRESS) {
        printf("Error: Flash overflow for channel %d (address 0x%08lX)\r\n", channelIndex + 1, (unsigned long)targetAddress);
        return HAL_ERROR;
    }

    /* Copy data to word‑aligned buffer and pad with 0xFF */
    uint16_t numWords = (dataSize + 3) / 4;
    for (uint16_t i = 0; i < numWords; ++i) saveBuf[i] = 0xFFFFFFFFu;
    memcpy(saveBuf, data, dataSize);

    /* Disable only capture‑related IRQs during Flash operation */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

    HAL_FLASH_Unlock();

    /* Erase the allocated pages for this channel */
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = targetAddress;
    EraseInitStruct.NbPages = PAGES_PER_CHANNEL;

    status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
    if (status != HAL_OK) {
        uint32_t ferr = HAL_FLASH_GetError();
        printf("Flash erase failed (ch %d). status=%ld, FLASH error=0x%08lX, PageError=0x%08lX\r\n",
               channelIndex + 1, (long)status, (unsigned long)ferr, (unsigned long)PageError);
        HAL_FLASH_Lock();
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
        return status;
    }

    /* Program word by word */
    uint32_t addr = targetAddress;
    for (uint16_t i = 0; i < numWords; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, saveBuf[i]);
        if (status != HAL_OK) {
            uint32_t ferr = HAL_FLASH_GetError();
            printf("Flash program failed at 0x%08lX (ch %d). status=%ld, FLASH error=0x%08lX\r\n",
                   (unsigned long)addr, channelIndex + 1, (long)status, (unsigned long)ferr);
            HAL_FLASH_Lock();
            HAL_NVIC_EnableIRQ(TIM4_IRQn);
            HAL_NVIC_EnableIRQ(TIM2_IRQn);
            return status;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();

    /* Re‑enable capture IRQs */
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    return HAL_OK;
}




/**
 * @brief Delete a channel by writing an empty IR_RawData structure to Flash.
 * @param ch Channel number (1‑based)
 */
static void DeleteChannel(uint8_t ch) {
    if (ch < 1 || ch > MAX_CHANNELS) {
        printf("Error: Invalid channel %d.\r\n", ch);
        return;
    }
    ch--;  /* Convert to 0‑based */
    memset(&tempChannelData, 0, sizeof(IR_RawData));
    if (SaveToFlash(ch, &tempChannelData) == HAL_OK) {
        channelOccupied[ch] = 0;
        SaveOccupiedToFlash();  /* Persist occupancy map */
        printf("Channel %d deleted.\r\n", ch + 1);
        LED_Pattern(4);   /* Delete pattern */
    } else {
        printf("Error deleting channel %d. Flash error: %lu\r\n", ch + 1, HAL_FLASH_GetError());
        LED_Pattern(5);  /* Error pattern */
    }
}

/**
 * @brief Load one channel's IR data from Flash into a RAM buffer.
 * @param channelIndex 0‑based channel index
 * @param destination Pointer to destination IR_RawData buffer
 */
void LoadFromFlash(uint8_t channelIndex, IR_RawData* destination)
{
    if (channelIndex >= MAX_CHANNELS) return;

    uint32_t sourceAddress = FLASH_CHANNELS_START + (channelIndex * CHANNEL_SIZE_IN_FLASH);

    if (sourceAddress + sizeof(IR_RawData) > FLASH_END_ADDRESS) {
        printf("Error: Flash overflow on load for channel %d\r\n", channelIndex + 1);
        memset(destination, 0, sizeof(IR_RawData));
        return;
    }

    memcpy(destination, (void*)sourceAddress, sizeof(IR_RawData));
}


/**
 * @brief Find the first free (unoccupied) channel.
 * @return Channel number (1‑based) or 0 if all channels are occupied.
 */
static uint8_t FindNextFreeChannel(void) {
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        if (!channelOccupied[i]) return i + 1;
    }
    return 0;
}

/**
 * @brief Scan the 4x4 matrix keypad and return the pressed key.
 * @return ASCII character of the pressed key, or 0 if none.
 */
static uint8_t ScanKeyboard(void) {
    const uint8_t keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };

//    // *** CHANGE: Using an array for non-contiguous row pins ***
//    const uint16_t row_pins[4] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_8};
//    const uint16_t col_pins[4] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};

    /* Row pins (all on GPIOB) */
    const uint16_t row_pins[4] = {GPIO_PIN_9, GPIO_PIN_8, GPIO_PIN_7, GPIO_PIN_6};

    /* Column ports and pins (PB3‑PB5 on GPIOB, PA15 on GPIOA) */
    GPIO_TypeDef* col_ports[4] = {GPIOB, GPIOB, GPIOB, GPIOB};
    const uint16_t col_pins[4] = {GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_3, GPIO_PIN_15};

    for (uint8_t row = 0; row < 4; row++) {
    	 /* Drive current row low */
        HAL_GPIO_WritePin(GPIOB, row_pins[row], GPIO_PIN_RESET);
        //for (volatile int d = 0; d < 100; d++);


        for (uint8_t col = 0; col < 4; col++) {
        	 /* Read column */
            if (HAL_GPIO_ReadPin(col_ports[col], col_pins[col]) == GPIO_PIN_RESET) {
            	/* Key pressed – debounce with 50 ms delay */
				//for (volatile int d = 0; d < 100; d++);  // ≈ ۱-۲µs بسته به clock
            	//for (volatile int d = 0; d < 100; d++);
            	HAL_Delay(50);

            	/* Confirm still pressed */
                if (HAL_GPIO_ReadPin(col_ports[col], col_pins[col]) == GPIO_PIN_RESET) {
                	/* Restore row to high before returning */
                    HAL_GPIO_WritePin(GPIOB, row_pins[row], GPIO_PIN_SET);
                    return keys[row][col];
                }
            }
        }
        /* Restore row to high before moving to next row */
        HAL_GPIO_WritePin(GPIOB, row_pins[row], GPIO_PIN_SET);
    }
    return 0;  /* No key pressed */
}

/**
 * @brief EXTI interrupt callback (triggered by keypad column pins).
 *        Simple debounce and flag setting – heavy processing is done in main loop.
 */
static uint32_t last_key_ms = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t now = HAL_GetTick();
	    if (now - last_key_ms < 40) {
	    	/* Debounce: ignore if less than 40 ms since last interrupt */
	        return;
	    }
	    last_key_ms = now;
	    /* Set flag for main loop to process */
        exti_flag = 1;
        /* Optional visual feedback (quick LED toggle) */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

}




/**
 * @brief Process keypad input and manage state machine transitions.
 * @param key ASCII character of the pressed key
 */
static void ProcessKeyboardInput(uint8_t key) {

	/* Digit entry (max 2 digits) */
    if (key >= '0' && key <= '9' && keyboardIndex < 2) {
        keyboardBuffer[keyboardIndex++] = key;
        keyboardBuffer[keyboardIndex] = '\0';
        printf("Key pressed: %c\r\n", key);

    }
    /* '#' – confirm channel selection (only in IDLE) */
    else if (key == '#') {
    	printf(">> State changed to IR_SELECTING (%d)\r\n", currentState);
        if (currentState != IR_IDLE) return;
        selectedChannel = atoi((char*)keyboardBuffer);
        if (selectedChannel < 1 || selectedChannel > MAX_CHANNELS) {
            printf("Error: Invalid channel %d.\r\n", selectedChannel);
            selectedChannel = 0;
            LED_Pattern(5);  /* Error pattern */
        } else {
            printf("Channel %d selected. Press C (Send) or D (Delete) within 5s.\r\n", selectedChannel);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  /* LED on */
            currentState = IR_SELECTING;
            printf(">> State changed to IR_SELECTING (%d)\r\n", currentState);
            StartUserTimeout();  /* Start 5s timeout */
        }
        keyboardIndex = 0;
        memset(keyboardBuffer, 0, sizeof(keyboardBuffer));
    }
    /* '*' – cancel current operation */
    else if (key == '*') {  // لغو
        keyboardIndex = 0;
        selectedChannel = 0;
        printf("Keyboard input canceled.\r\n");
        if (currentState == IR_READY) {
            LED_Pattern(2);  /* Cancel pattern */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  /* LED off */
            currentState = IR_IDLE;
            StopUserTimeout();
            IR_CaptureStart();
        }
    }
    /* A, B, C, D – action keys */
    else if (key == 'A' || key == 'B' || key == 'C' || key == 'D') {
    	printf("Key=%c, State=%d, selCh=%d\r\n", key, currentState, selectedChannel);

    	/* In READY state – save or cancel the captured signal */
    	if (currentState == IR_READY) {
            if (key == 'A') {  /* Save */
                uint8_t ch;
                if (keyboardIndex > 0) {
                    ch = atoi((char*)keyboardBuffer);
                    keyboardIndex = 0;
                    memset(keyboardBuffer, 0, sizeof(keyboardBuffer));
                    if (ch < 1 || ch > MAX_CHANNELS) {
                        printf("Error: Invalid channel %d for save.\r\n", ch);
                        LED_Pattern(5);  // Error pattern
                        return;
                    }
                } else {
                    ch = FindNextFreeChannel();  // از occupied برای پیدا کردن خالی
                }
                if (ch == 0) {  // همه پر، overwrite کانال 1
                    ch = 1;
                    printf("All %d channels full. Overwriting channel 1.\r\n", MAX_CHANNELS);
                } else {
                    printf("Saving to free channel %d.\r\n", ch);
                }

                if (channelOccupied[ch - 1] == 1) {
                    printf("Warning: Overwriting channel %d.\r\n", ch);
                }
                SaveChannel(ch);
                selectedChannel = 0;
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  /* LED off */
                currentState = IR_IDLE;
                StopUserTimeout();
                IR_CaptureStart();
            } else if (key == 'B') {  /* Cancel */
                printf("Signal canceled.\r\n");
                LED_Pattern(2);  // Cancel pattern
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  // LED off
                currentState = IR_IDLE;
                StopUserTimeout();
                keyboardIndex = 0;
                memset(keyboardBuffer, 0, sizeof(keyboardBuffer));
                IR_CaptureStart();
            }
        }
    	/* In SELECTING state – send or delete a previously selected channel */
    	else if (selectedChannel > 0 && currentState == IR_SELECTING) {
            printf("Key=%c, State=%d, selCh=%d\r\n", key, currentState, selectedChannel);

            if (key == 'C') {  /* Send */
                if (channelOccupied[selectedChannel - 1] == 0) {
                    printf("Error: Channel %d empty.\r\n", selectedChannel);
                    LED_Pattern(5);
                } else {

                    LoadFromFlash(selectedChannel - 1, &tempChannelData);
                    printf("\r\n--- Loaded Signal from Channel %d ---\r\n", selectedChannel);
                            printf("Carrier Frequency: %lu Hz\r\n", tempChannelData.carrierFreq);
                            printf("Segment Count: %d\r\n", tempChannelData.segmentCount);
                            for (int i = 0; i < tempChannelData.segmentCount; i++) {
                                const char* typeStr = (tempChannelData.type[i] == 1) ? "Burst" : "Idle ";
                                printf("Segment %02d: [%s] -> %lu us\r\n", i, typeStr, tempChannelData.duration[i]);
                            }
                            printf("--- End of Loaded Signal ---\r\n\r\n");
                    IR_Transmit(&tempChannelData);
                }
            } else if (key == 'D') {  /* Delete */
                if (channelOccupied[selectedChannel - 1] == 0) {
                    printf("Error: Channel %d already empty.\r\n", selectedChannel);
                    LED_Pattern(5);
                } else {
                    DeleteChannel(selectedChannel);
                }
            }

            /* After C or D, always return to IDLE */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            StopUserTimeout();
            selectedChannel = 0;
            currentState = IR_IDLE;
            IR_CaptureStart();
        }

    }
}


/**
 * @brief Start the 5‑second user decision timeout (TIM1).
 *        Clears all flags and pending IRQs before starting.
 */
static inline void StartUserTimeout(void)
{
	/* Reset counter */
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    /* Clear status register and all pending flags */
    htim1.Instance->SR = 0;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    /* Clear NVIC pending for this IRQ */
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

    /* Start the timer (only if not already running) */
    if ((htim1.Instance->CR1 & TIM_CR1_CEN) == 0) {
        HAL_TIM_Base_Start_IT(&htim1);
    }

    /* Debug: print register status */
    printf("StartUserTimeout after start: CR1=0x%04lX, SR=0x%08lX, DIER=0x%08lX\n",
           (unsigned long)htim1.Instance->CR1,
           (unsigned long)htim1.Instance->SR,
           (unsigned long)htim1.Instance->DIER);
}

/**
 * @brief Stop the user decision timeout and clear all pending flags.
 */
static inline void StopUserTimeout(void)
{
    HAL_TIM_Base_Stop_IT(&htim1);
    /* Clear all flags and pending IRQs */
    htim1.Instance->SR = 0;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
    HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
}
static void DumpOccupiedFlashRaw(void) {
    uint8_t *p = (uint8_t*)FLASH_OCCUPIED_START;
    printf("Raw occupied flash bytes: ");
    for (int i = 0; i < (int)sizeof(channelOccupied); i++) {
        printf("%02X ", p[i]);
    }
    printf("\r\n");
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* Configure PA0 as input with pull-down for standby wake-up (WKUP) */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_0;  // PA0
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;  // pull-down
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Disable wakeup pin initially (used for standby mode) */
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);


  /* Check if the microcontroller is waking up from STANDBY mode */
  if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
      /*
       * STANDBY mode is the deepest sleep mode. When the MCU wakes up,
       * it performs a power-on reset and restarts execution from main().
       * The PWR_FLAG_SB flag indicates that the reset was caused by
       * waking from STANDBY rather than a normal power-up or hardware reset.
       */

      /* Clear the STANDBY flag to avoid re‑detection on next reset */
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

      /* Also clear the wake‑up flag (set when a wake‑up event occurred) */
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

      /*
       * Disable the wake‑up pin (PA0) to prevent immediate re‑entry into
       * standby after we exit this boot sequence. The pin will be re‑enabled
       * later when the idle timeout triggers standby again.
       */
      HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

      /* Optional debug message to confirm wake‑up source */
      printf("Woke up from STANDBY. Restarting init.\r\n");
  }
	/* Reset idle counter */
    lastActivityTime = HAL_GetTick();

    /* Enable DWT cycle counter for microsecond delays */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	LoadOccupiedFromFlash();
	printf("Occupied map: ");
	for (int i=0;i<MAX_CHANNELS;i++){
		printf("%d", channelOccupied[i] ? 1 : 0);
		if (i < MAX_CHANNELS-1) printf(",");
	}
	printf("\r\n");
	DumpOccupiedFlashRaw();

	printf("IR Raw Receiver Started... Flash base: 0x%08lX (%d channels, %d pages each)\r\n",
		   (unsigned long)FLASH_CHANNELS_START, MAX_CHANNELS, PAGES_PER_CHANNEL);
	LED_Pattern(1);   /* Test LED */
	//  printf("IR Raw Receiver Started...\r\n");
	//  BlinkPattern_OK();
	IR_CaptureStart();

	 /* Set PA5 low (active state) – used as a wake-up indicator */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
	  // UPDATED: اسکن کیبورد (در هر state)
//		uint8_t key = ScanKeyboard();
//		if (key) {
//			ProcessKeyboardInput(key);
//		}
	  /* Check idle timeout – enter STANDBY after 30 seconds of inactivity */
	  if (currentState == IR_IDLE && (HAL_GetTick() - lastActivityTime >= IDLE_TIMEOUT_MS)) {
		  printf("30s idle timeout. Entering Standby mode...\r\n");

		  /* Set PA5 high to indicate standby state (active low before) */
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

		  /* Stop all peripherals to save power */
		  HAL_TIM_Base_Stop_IT(&htim1);
		  HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		  HAL_TIM_Base_Stop_IT(&htim4);

		  /* Enable wake‑up pin and enter STANDBY mode */
		  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
		  HAL_PWR_EnterSTANDBYMode();
		  /* The MCU resets after waking from STANDBY */
	  }
	  /* Process EXTI flag (keypad interrupt) – consume and optionally debug */
	  if (exti_flag) {
		  exti_flag = 0;                     // consume the flag (one-shot)
		  printf("EXTI event detected! (flag processed)\r\n");
		  /* Additional processing can be added here if needed */
	  }
	  /* Scan keypad and handle user input */
	  uint8_t key = ScanKeyboard();
		if (key) {
			lastActivityTime = HAL_GetTick();  /* Reset idle timer on any key press */
			ProcessKeyboardInput(key);
		}

		/* If a signal capture is complete, process it */
		if (signalReady) {
			/* Valid signal – enter READY state and turn LED on solid */
			signalReady = 0;
			uint8_t result = ProcessDiffBuffer();
			lastActivityTime = HAL_GetTick();  /* User interaction resets idle timer */

			if (result) {
				currentState = IR_READY;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

				/* Print captured signal details for debugging */
				printf("\r\n--- New Valid Signal Captured ---\r\n");
				printf("Carrier Frequency: %lu Hz\r\n", irData.carrierFreq);
				printf("Segment Count: %d\r\n", irData.segmentCount);
				for (int i = 0; i < irData.segmentCount; i++) {
					const char* typeStr = (irData.type[i] == 1) ? "Burst" : "Idle ";
					printf("Segment %02d: [%s] -> %lu us\r\n", i, typeStr, irData.duration[i]);
				}
				printf("--- End of Signal ---\r\n\r\n");

				/* Suggest next free channel or overwrite option */
				uint8_t nextCh = FindNextFreeChannel();
				if (nextCh == 0) {
					printf("All channels full. Enter number then A to overwrite, or B (Cancel).\r\n");
					nextCh = 1;  /* Default to overwrite channel 1 */
				} else {
					printf("Save to channel %d? Press A (Save), or enter number then A, or B (Cancel).\r\n", nextCh);
				}
				/* Start 5-second timeout for user decision */
				StartUserTimeout();
			} else {
				/* Invalid signal – discard and restart capture */
				if (diffIndex >= MIN_DIFFS_FOR_PROCESS) {
					LED_Pattern(2);  // Cancel pattern
					printf("\r\n--- Invalid Signal Discarded. Resetting... ---\r\n");
				}
				IR_CaptureStart();
			}
		}

		/* Small delay to prevent excessive CPU usage while polling */
		HAL_Delay(100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 49999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
  HAL_NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

  HAL_NVIC_SetPriority(TIM1_UP_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);


  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1894;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 631;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7199;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA7 PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB15 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
