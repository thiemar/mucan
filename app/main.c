/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 04/05/2015 18:45:31
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"


volatile extern uint32_t _stext;           /* Start of .text */
volatile extern uint32_t _etext;           /* End_1 of .text + .rodata */
volatile extern uint32_t _eronly;          /* End+1 of read only section (.text + .rodata) */
volatile extern uint32_t _sdata;           /* Start of .data */
volatile extern uint32_t _edata;           /* End+1 of .data */
volatile extern uint32_t _sbss;            /* Start of .bss */
volatile extern uint32_t _ebss;            /* End+1 of .bss */


extern USBD_HandleTypeDef  *hUsbDevice_0;


/* TX buffer is TX from MCU to USB */
#define TX_BUFFER_SIZE 1024u
static uint8_t tx_buffer_0[TX_BUFFER_SIZE];
static uint8_t tx_buffer_1[TX_BUFFER_SIZE];
static uint8_t active_buffer;
static uint8_t active_buffer_length;


/* RX buffer is RX from USB to MCU */
#define RX_BUFFER_SIZE 128u
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_buffer_length;


const uint8_t hex_chars[] = "0123456789ABCDEF";
const uint32_t bitrate_lookup[] = {10000, 20000, 50000, 100000, 125000,
                                   250000, 500000, 800000, 1000000};


#define EXT_MESSAGE_LEN 27
#define STD_MESSAGE_LEN 22


#define CAN_NORMAL_QUANTA 8
#define CAN_NORMAL_SJW 0
#define CAN_NORMAL_BS1 CAN_BS1_6TQ
#define CAN_NORMAL_BS2 0u
#define CAN_1MBAUD_PRESCALER (SystemCoreClock/1000000/CAN_NORMAL_QUANTA)


#define CAN_800KBAUD_QUANTA 15
#define CAN_800KBAUD_SJW 0
#define CAN_800KBAUD_BS1 CAN_BS1_12TQ
#define CAN_800KBAUD_BS2 CAN_BS2_2TQ
#define CAN_800KBAUD_PRESCALER (SystemCoreClock/800000/CAN_800KBAUD_QUANTA)



void SystemClock_Config(void);
static uint8_t write_ext_message(uint8_t *dest, CanRxMsgTypeDef *msg);
static uint8_t write_std_message(uint8_t *dest, CanRxMsgTypeDef *msg);


void HAL_MspInit(void) {
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


inline static uint8_t nibble_from_hex(uint8_t ch) {
    if (ch >= '0' && ch <= '9') {
        return ch - '0';
    } else if (ch >= 'A' && ch <= 'F') {
        return ch - 'A' + 10u;
    } else if (ch >= 'a' && ch <= 'f') {
        return ch - 'a' + 10u;
    } else {
        return 0;
    }
}


static uint8_t write_ext_message(uint8_t *dest, CanRxMsgTypeDef *msg) {
    uint8_t i;

    *dest++ = 'T';

    /* Message ID, 00000000-1FFFFFFF */
    *dest++ = hex_chars[(msg->ExtId >> 28u) & 0xFu];
    *dest++ = hex_chars[(msg->ExtId >> 24u) & 0xFu];
    *dest++ = hex_chars[(msg->ExtId >> 20u) & 0xFu];
    *dest++ = hex_chars[(msg->ExtId >> 16u) & 0xFu];
    *dest++ = hex_chars[(msg->ExtId >> 12u) & 0xFu];
    *dest++ = hex_chars[(msg->ExtId >>  8u) & 0xFu];
    *dest++ = hex_chars[(msg->ExtId >>  4u) & 0xFu];
    *dest++ = hex_chars[(msg->ExtId >>  0u) & 0xFu];

    /* Length */
    *dest++ = '0' + msg->DLC;

    /* Data */
    for (i = 0; i < msg->DLC; i++) {
        *dest++ = hex_chars[(msg->Data[i] >> 4u) & 0xFu];
        *dest++ = hex_chars[(msg->Data[i] >> 0u) & 0xFu];
    }

    *dest++ = '\r';

    return 1u + 8u + 1u + (msg->DLC << 1u) + 1u;
}


static uint8_t write_std_message(uint8_t *dest, CanRxMsgTypeDef *msg) {
    uint8_t i;

    *dest++ = 'T';

    /* Message ID, 000-7FF */
    *dest++ = hex_chars[(msg->StdId >>  8u) & 0xFu];
    *dest++ = hex_chars[(msg->StdId >>  4u) & 0xFu];
    *dest++ = hex_chars[(msg->StdId >>  0u) & 0xFu];

    /* Length */
    *dest++ = '0' + msg->DLC;

    /* Data */
    for (i = 0; i < msg->DLC; i++) {
        *dest++ = hex_chars[(msg->Data[i] >> 4u) & 0xFu];
        *dest++ = hex_chars[(msg->Data[i] >> 0u) & 0xFu];
    }

    *dest++ = '\r';

    return 1u + 3u + 1u + (msg->DLC << 1u) + 1u;
}


static uint8_t read_ext_message(CanTxMsgTypeDef *dest, uint8_t *msg,
                                uint8_t msg_length) {
    uint8_t i;

    if (msg[0] > '1' || msg[8] < '0' || msg[8] > '8' ||
            msg_length != 8u + 1u + ((msg[8] - '0') << 1u)) {
        return 0;
    }

    dest->ExtId = (nibble_from_hex(msg[0]) << 28u) |
                  (nibble_from_hex(msg[1]) << 24u) |
                  (nibble_from_hex(msg[2]) << 20u) |
                  (nibble_from_hex(msg[3]) << 16u) |
                  (nibble_from_hex(msg[4]) << 12u) |
                  (nibble_from_hex(msg[5]) <<  8u) |
                  (nibble_from_hex(msg[6]) <<  4u) |
                  (nibble_from_hex(msg[7]) <<  0u);
    dest->IDE = CAN_ID_EXT;
    dest->RTR = 0;
    dest->DLC = msg[8] - '0';

    for (i = 0; i < dest->DLC; i++) {
        dest->Data[i] = (nibble_from_hex(msg[9u + (i << 1u)]) << 4u) |
                        (nibble_from_hex(msg[9u + (i << 1u) + 1u]) << 0u);
    }

    return 1u;
}


static uint8_t read_std_message(CanTxMsgTypeDef *dest, uint8_t *msg,
                                uint8_t msg_length) {
    uint8_t i;

    if (msg[0] > '7' || msg[3] < '0' || msg[3] > '8' ||
            msg_length != 3u + 1u + ((msg[3] - '0') << 1u)) {
        return 0;
    }

    dest->ExtId = (nibble_from_hex(msg[0]) <<  8u) |
                  (nibble_from_hex(msg[1]) <<  4u) |
                  (nibble_from_hex(msg[2]) <<  0u);
    dest->IDE = CAN_ID_EXT;
    dest->RTR = 0;
    dest->DLC = msg[3] - '0';

    for (i = 0; i < dest->DLC; i++) {
        dest->Data[i] = (nibble_from_hex(msg[4u + (i << 1u)]) << 4u) |
                        (nibble_from_hex(msg[4u + (i << 1u) + 1u]) << 0u);
    }

    return 1u;
}


static uint8_t set_can_bitrate(CAN_HandleTypeDef *hcan, uint32_t bitrate) {
    if (bitrate == 800000u) {
        /*
        800 kbaud is a special case because we can't get away with
        BS1=6, BS2=0
        */
        hcan->Init.Prescaler = CAN_800KBAUD_PRESCALER;
        hcan->Init.SJW = CAN_800KBAUD_SJW;
        hcan->Init.BS1 = CAN_800KBAUD_BS1;
        hcan->Init.BS2 = CAN_800KBAUD_BS2;
    } else if (bitrate == 1000000u || bitrate == 500000u ||
               bitrate == 250000u || bitrate == 125000u ||
               bitrate == 100000u || bitrate == 50000u || bitrate == 20000u ||
               bitrate == 10000u) {
        hcan->Init.Prescaler = CAN_1MBAUD_PRESCALER * (1000000u / bitrate);
        hcan->Init.SJW = CAN_NORMAL_SJW;
        hcan->Init.BS1 = CAN_NORMAL_BS1;
        hcan->Init.BS2 = CAN_NORMAL_BS2;
    } else {
        return 0u;
    }

    HAL_CAN_Init(hcan);
    return 1u;
}


static void set_dfu_option_bytes(uint8_t force_bootloader) {
    FLASH_OBProgramInitTypeDef ob_cfg;
    uint8_t selected_cfg;

    HAL_FLASHEx_OBGetConfig(&ob_cfg);

    if (force_bootloader) {
        /*
        From ST AN2606, pattern 6:
        nBoot0(bit) = 0, nBoot1(bit) = 1 and nBoot0_SW(bit) = 0
        */
        selected_cfg = OB_BOOT_SEL_RESET | OB_RAM_PARITY_CHECK_RESET |
                       OB_VDDA_ANALOG_ON | OB_BOOT1_SET |
                       OB_BOOT0_RESET | OB_STDBY_NO_RST | OB_STOP_NO_RST |
                       OB_WDG_SW;
    } else {
        /* Set option bytes to the default value */
        selected_cfg = OB_BOOT_SEL_SET | OB_RAM_PARITY_CHECK_RESET |
                       OB_VDDA_ANALOG_ON | OB_BOOT1_SET |
                       OB_BOOT0_SET | OB_STDBY_NO_RST | OB_STOP_NO_RST |
                       OB_WDG_SW;
    }

    if (selected_cfg != ob_cfg.USERConfig) {
        ob_cfg.OptionType = OPTIONBYTE_USER;
        ob_cfg.USERConfig = selected_cfg;

        HAL_FLASH_Unlock();
        HAL_FLASH_OB_Unlock();
        HAL_FLASHEx_OBErase();
        HAL_FLASHEx_OBProgram(&ob_cfg);
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
        HAL_FLASH_OB_Launch();
    }
}


int __start(void) {
    const volatile uint32_t *src;
    volatile uint32_t *dest;

    /* Set up BSS and copy data from flash */
    for (src = &_eronly, dest = &_sdata; dest < &_edata;) {
        *dest++ = *src++;
    }

    for (dest = &_sbss; dest < &_ebss;) {
        *dest++ = 0;
    }

    SystemInit();
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /*
    Make sure the flash option bytes are set such that we don't re-enter
    bootloader mode
    */
    set_dfu_option_bytes(0u);

    MX_USB_DEVICE_Init();

    /* Configure GPIO for the CAN_SILENT signal */
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __CRS_CLK_ENABLE();

    /*Configure GPIO pin : PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_CAN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Configure the bxCAN transceiver */
    CAN_HandleTypeDef hcan;

    __CAN_CLK_ENABLE();

    hcan.Instance = CAN;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.TTCM = DISABLE;
    hcan.Init.ABOM = ENABLE;
    hcan.Init.AWUM = ENABLE;
    hcan.Init.NART = DISABLE;
    hcan.Init.RFLM = DISABLE;
    hcan.Init.TXFP = DISABLE;

    /* Set the bitrate and init */
    set_can_bitrate(&hcan, 1000000u);

    CanRxMsgTypeDef rx_msg;
    hcan.pRxMsg = &rx_msg;

    CanTxMsgTypeDef tx_msg;
    hcan.pTxMsg = &tx_msg;

    CAN_FilterConfTypeDef filter;
    filter.FilterNumber = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.BankNumber = 0;
    filter.FilterMaskIdLow = filter.FilterIdLow = filter.FilterMaskIdHigh =
        filter.FilterIdHigh = 0;
    HAL_CAN_ConfigFilter(&hcan, &filter);

    __HAL_UNLOCK(&hcan);

    /*
    Now that USB is active, we need to sync with the 1 kHz SOF packets to
    tune the HSI48 so it's accurate enough not to disrupt the CAN bus.
    */
    RCC_CRSInitTypeDef crs_config;
    crs_config.Prescaler = RCC_CRS_SYNC_DIV1;
    crs_config.Source = RCC_CRS_SYNC_SOURCE_USB;
    crs_config.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    crs_config.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
    crs_config.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
    crs_config.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;

    HAL_RCCEx_CRSConfig(&crs_config);

    /* Don't continue unless we're synced */
    while (!(HAL_RCCEx_CRSWaitSynchronization(1000000u) & RCC_CRS_SYNCOK));

    while (1) {
        /* Set up the command record */
        size_t i;
        uint8_t current_cmd;
        uint8_t current_cmd_data[32];
        uint8_t current_cmd_data_length;
        uint8_t channel_open;

        current_cmd = current_cmd_data_length = i = 0;
        channel_open = 0;
        rx_buffer_length = 0;

        while (hUsbDevice_0 && hUsbDevice_0->pClassData) {
            /*
            If the RX buffer is non-empty, process any commands/messages in it
            and then acknowledge the transfer to the USB stack
            */
            if (rx_buffer_length) {
                /*
                Exit if we've read the entire buffer, or if transmit mailboxes
                are full and the current command is a transmit.
                */
                for (; i < rx_buffer_length &&
                        ((current_cmd != 't' && current_cmd != 'T') ||
                         __HAL_CAN_GET_FLAG(&hcan, CAN_FLAG_TME0)); i++) {
                    if (rx_buffer[i] == '\r') {
                        uint8_t *buf = active_buffer ? tx_buffer_1 :
                                                       tx_buffer_0;

                        /* End of command -- process and reset state */
                        switch (current_cmd) {
                            case 'S':
                                /*
                                Setup bitrate -- data should be a number from
                                0 to 8.
                                */
                                if (current_cmd_data[0] < '0' ||
                                        current_cmd_data[0] > '8') {
                                    buf[active_buffer_length++] = '\a';
                                } else if (set_can_bitrate(
                                        &hcan,
                                        bitrate_lookup[current_cmd_data[0] - '0'])) {
                                    buf[active_buffer_length++] = '\r';
                                } else {
                                    buf[active_buffer_length++] = '\a';
                                }
                                break;
                            case 'O':
                                /*
                                Open channel -- take CAN out of silent mode
                                and start receiving.

                                TODO: enable silent mode in bxCAN.
                                */
                                channel_open = 1;
                                buf[active_buffer_length++] = '\r';
                                break;
                            case 'L':
                                /*
                                Open in listen-only -- CAN should already be
                                in silent mode but just make sure.

                                TODO: enable silent mode in bxCAN.
                                */
                                channel_open = 1;
                                buf[active_buffer_length++] = '\r';
                                break;
                            case 'C':
                                /*
                                Close channel -- put CAN back in silent mode.

                                TODO: enable silent mode in bxCAN.
                                */
                                channel_open = 0;
                                buf[active_buffer_length++] = '\r';
                                break;
                            case 'V':
                                /* Hardware version */
                                buf[active_buffer_length++] = 'V';
                                buf[active_buffer_length++] = '0';
                                buf[active_buffer_length++] = '0';
                                buf[active_buffer_length++] = '0';
                                buf[active_buffer_length++] = '1';
                                buf[active_buffer_length++] = '\r';
                                break;
                            case 'v':
                                /* Major/minor version */
                                buf[active_buffer_length++] = 'v';
                                buf[active_buffer_length++] = '0';
                                buf[active_buffer_length++] = '0';
                                buf[active_buffer_length++] = '0';
                                buf[active_buffer_length++] = '1';
                                buf[active_buffer_length++] = '\r';
                                break;
                            case 'N':
                                /* Serial number */
                                buf[active_buffer_length++] = 'N';
                                buf[active_buffer_length++] = 'F';
                                buf[active_buffer_length++] = 'F';
                                buf[active_buffer_length++] = 'F';
                                buf[active_buffer_length++] = 'F';
                                buf[active_buffer_length++] = '\r';
                                break;
                            case 'T':
                                /* Extended message */
                                if (read_ext_message(&tx_msg, current_cmd_data,
                                                     current_cmd_data_length) &&
                                        HAL_CAN_Transmit(&hcan, 0) == HAL_OK) {
                                    buf[active_buffer_length++] = '\r';
                                } else {
                                    buf[active_buffer_length++] = '\a';
                                }
                                break;
                            case 't':
                                /* Standard message */
                                if (read_std_message(&tx_msg, current_cmd_data,
                                                     current_cmd_data_length) &&
                                        HAL_CAN_Transmit(&hcan, 0) == HAL_OK) {
                                    buf[active_buffer_length++] = '\r';
                                } else {
                                    buf[active_buffer_length++] = '\a';
                                }
                                break;
                            case '_':
                                /* Bootloader request */
                                if (current_cmd_data[0] == '_' &&
                                        current_cmd_data[1] == 'd' &&
                                        current_cmd_data[2] == 'f' &&
                                        current_cmd_data[3] == 'u') {
                                    /*
                                    Set option bytes to force bootloader entry
                                    */
                                    set_dfu_option_bytes(1u);

                                    NVIC_SystemReset();
                                } else {
                                    buf[active_buffer_length++] = '\a';
                                }
                                break;
                            case 'R':
                                /* Extended RTR */
                            case 'r':
                                /* Standard RTR */
                            case 'F':
                                /* Read status flags -- return 4 hex digits */
                            case 'Z':
                                /* Timestamp on/off -- 0=off, 1=on */
                            case 'M':
                                /* Acceptance mask -- 8 hex digits */
                            case 'm':
                                /* Acceptance value -- 8 hex digits */
                            case 's':
                                /* Set bitrate register -- 6 hex digits */
                            default:
                                /* NACK */
                                buf[active_buffer_length++] = '\a';
                                break;
                        }

                        current_cmd = current_cmd_data_length = 0;
                    } else if (current_cmd) {
                        /* Command data -- save it */
                        current_cmd_data[current_cmd_data_length++] =
                            rx_buffer[i];

                        /* Reset command state if the data is too long */
                        if (current_cmd_data_length == 32u) {
                            current_cmd = current_cmd_data_length = 0;
                        }
                    } else {
                        /*
                        The first letter of a line is the command type --
                        don't bother validating as we can't NACK until the
                        data has been received.
                        */
                        current_cmd = rx_buffer[i];
                    }
                }

                if (i == rx_buffer_length) {
                    /* Mark last packet as received */
                    rx_buffer_length = i = 0;
                    USBD_CDC_ReceivePacket(hUsbDevice_0);
                }
            }

            /*
            Check the CAN controller for messages and process them if the
            channel is open
            */
            if (HAL_CAN_Receive(&hcan, CAN_FIFO0, 0) == HAL_OK && hUsbDevice_0 &&
                    channel_open) {
                /*
                Format the message and send it to the host.

                The data format for a standard ID message is:
                tiiildddddddddddddddd\r

                The data format for an extended ID message is:
                Tiiiiiiiildddddddddddddddd\r
                */
                if (rx_msg.IDE == CAN_ID_EXT) {
                    if (active_buffer_length + EXT_MESSAGE_LEN > TX_BUFFER_SIZE) {
                        /* Reset buffer if we're too far behind */
                        active_buffer_length = 0;
                    }

                    active_buffer_length += write_ext_message(
                        &((active_buffer ? tx_buffer_1 : tx_buffer_0)[active_buffer_length]),
                        &rx_msg);
                } else {
                    if (active_buffer_length + STD_MESSAGE_LEN > TX_BUFFER_SIZE) {
                        /* Reset buffer if we're too far behind */
                        active_buffer_length = 0;
                    }

                    active_buffer_length += write_std_message(
                        &((active_buffer ? tx_buffer_1 : tx_buffer_0)[active_buffer_length]),
                        &rx_msg);
                }
            }

            if (!hUsbDevice_0 || !hUsbDevice_0->pClassData) {
                /* Reset TX buffer if USB is not connected */
                active_buffer_length = 0;
            } else if (active_buffer_length &&
                        ((USBD_CDC_HandleTypeDef*)hUsbDevice_0->pClassData)->TxState == 0) {
                /* Transmit the next buffer if one is available */
                CDC_Transmit_FS(active_buffer ? tx_buffer_1 : tx_buffer_0,
                                active_buffer_length);
                active_buffer_length = 0;
                active_buffer = active_buffer ? 0 : 1;
            }
        }
    }
}


void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    __SYSCFG_CLK_ENABLE();
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
