/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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


#include <stdarg.h>
#include <string.h>
#include <stdint.h>

/* USER CODE BEGIN Includes */

#include "main.h"

/* USER CODE END Includes */


// Uncomment this line to enable debug messages over debug UART
//#define BL_DEBUG_MSG_EN


/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

uint8_t supported_commands[] = {
                               BL_GET_VER,
                               BL_GET_HELP,
                               BL_GET_CID,
                               BL_GET_RDP_STATUS,
                               BL_GO_TO_ADDR,
                               BL_FLASH_ERASE,
                               BL_MEM_WRITE,
                               BL_READ_SECTOR_P_STATUS};

#define D_UART   &huart3
#define C_UART   &huart2

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void printmsg(char *format,...);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

char somedata[] = "Hello from Bootloader\r\n";

#define BL_RX_LEN  200
uint8_t bl_rx_buffer[BL_RX_LEN];

/* USER CODE END 0 */


void flash_testing(void)
{
    uint8_t protection_mode = 2;
    uint8_t sector_details = 0x80;

    // Flash option control register (OPTCR)
    volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

    // Option byte configuration unlock
    HAL_FLASH_OB_Unlock();

    // Wait until no active operation on flash
    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

    // Here we are setting read and write protection for the sectors
    *pOPTCR |= (1 << 31);  // Set the 31st bit

    *pOPTCR &= ~(0xff << 16);
    *pOPTCR |= (sector_details << 16);

    *pOPTCR |= (1 << 1);  // Set the option start bit (OPTSTRT) in the FLASH_OPTCR register

    while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

    HAL_FLASH_OB_Lock();
}


int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  /*
   * Check the state of the user button to determine whether to stay in bootloader mode or jump to the user application:
   * - If the button is pressed, the bootloader remains active and listens for commands via UART.
   * - Otherwise, the bootloader transfers control to the user application stored in the Flash memory.
   */
  if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
  {
      printmsg("BL_DEBUG_MSG:Button is pressed .. going to BL mode\n\r");
      bootloader_uart_read_data();  // Remain in bootloader mode and listen for UART commands.
  }
  else
  {
      printmsg("BL_DEBUG_MSG:Button is not pressed .. executing user app\n");
      bootloader_jump_to_user_app();  // Transfer control to the user application.
  }
}




/**
 * @brief Function to read commands sent from the host through the UART interface.
 * This function continuously waits for incoming commands, processes them, and
 * invokes the corresponding command handler function based on the command code.
 */
void bootloader_uart_read_data(void)
{
    uint8_t rcv_len = 0;  // Variable to store the length of the received command packet

    while(1)
    {
        memset(bl_rx_buffer, 0, 200);  // Reset the receive buffer to clear any old data

        // Read the first byte which contains the length of the command packet from the host
        HAL_UART_Receive(C_UART, bl_rx_buffer, 1, HAL_MAX_DELAY);

        // Store the length of the received data to determine how many more bytes to read
        rcv_len = bl_rx_buffer[0];

        // Read the remaining bytes based on the specified length
        HAL_UART_Receive(C_UART, &bl_rx_buffer[1], rcv_len, HAL_MAX_DELAY);

        // Decode the command sent by the host, which is located at index 1 of the buffer
        switch(bl_rx_buffer[1])
        {
            case BL_GET_VER:
                bootloader_handle_getver_cmd(bl_rx_buffer);  // Handle 'GET Version' command
                break;
            case BL_GET_HELP:
                bootloader_handle_gethelp_cmd(bl_rx_buffer);  // Handle 'GET Help' command
                break;
            case BL_GET_CID:
                bootloader_handle_getcid_cmd(bl_rx_buffer);  // Handle 'GET Chip ID' command
                break;
            case BL_GET_RDP_STATUS:
                bootloader_handle_getrdp_cmd(bl_rx_buffer);  // Handle 'GET Read Protection Status' command
                break;
            case BL_GO_TO_ADDR:
                bootloader_handle_go_cmd(bl_rx_buffer);  // Handle 'Go to Address' command
                break;
            case BL_FLASH_ERASE:
                bootloader_handle_flash_erase_cmd(bl_rx_buffer);  // Handle 'Flash Erase' command
                break;
            case BL_MEM_WRITE:
                bootloader_handle_mem_write_cmd(bl_rx_buffer);  // Handle 'Memory Write' command
                break;
            case BL_EN_RW_PROTECT:
                bootloader_handle_en_rw_protect(bl_rx_buffer);  // Handle 'Enable Read/Write Protect' command
                break;
            case BL_MEM_READ:
                bootloader_handle_mem_read(bl_rx_buffer);  // Handle 'Memory Read' command
                break;
            case BL_READ_SECTOR_P_STATUS:
                bootloader_handle_read_sector_protection_status(bl_rx_buffer);  // Handle 'Read Sector Protection Status' command
                break;
            case BL_OTP_READ:
                bootloader_handle_read_otp(bl_rx_buffer);  // Handle 'Read OTP Memory' command
                break;
            case BL_DIS_R_W_PROTECT:
                bootloader_handle_dis_rw_protect(bl_rx_buffer);  // Handle 'Disable Read/Write Protection' command
                break;
            default:
                printmsg("BL_DEBUG_MSG:Invalid command code received from host\n");  // Print error message for invalid command
                break;
        }
    }
}



/**
 * @brief Function to jump to the user application stored in the flash memory.
 *
 * This function assumes that the user application is stored at a specific flash memory address,
 * which is defined by `FLASH_SECTOR2_BASE_ADDRESS`. It reads the MSP (Main Stack Pointer) and
 * Reset Handler address from this location and transfers control to the user application.
 */
void bootloader_jump_to_user_app(void)
{
    // 1. Declare a function pointer to hold the address of the user application's Reset Handler.
    void (*app_reset_handler)(void);

    // 2. Debug message to indicate that we have entered the bootloader_jump_to_user_app function.
    printmsg("BL_DEBUG_MSG:bootloader_jump_to_user_app\n");

    // 3. Read the value of the Main Stack Pointer (MSP) from the base address of the user application.
    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;

    // Print the MSP value for debugging purposes.
    printmsg("BL_DEBUG_MSG:MSP value : %#x\n", msp_value);

    // 4. Set the MSP with the value retrieved from the user application's base address.
    __set_MSP(msp_value);

    // 5. Retrieve the Reset Handler address from the user application's vector table.
    uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

    // 6. Initialize the function pointer with the address of the Reset Handler.
    app_reset_handler = (void*) resethandler_address;

    // 7. Print the Reset Handler address for debugging purposes.
    printmsg("BL_DEBUG_MSG: app reset handler addr : %#x\n", app_reset_handler);

    // 8. Jump to the user application's Reset Handler to transfer execution.
    app_reset_handler();
}

/**
 * @brief Print formatted debug messages over the UART interface.
 *
 * This function is used to print debug messages through the UART interface, which helps in monitoring
 * the bootloader's activities. It accepts a format string and additional arguments similar to `printf`.
 *
 * @param format: The format string for the message to be printed.
 */
void printmsg(char *format,...)
{
#ifdef BL_DEBUG_MSG_EN
    char str[80];  // Buffer to store the formatted message string

    // Extract the argument list using VA (Variable Argument) macros
    va_list args;
    va_start(args, format);

    // Format the string based on the provided format and arguments
    vsprintf(str, format, args);

    // Transmit the formatted string over the UART interface
    HAL_UART_Transmit(D_UART, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);

    // End the argument extraction
    va_end(args);
#endif
}



/**
 * @brief  Configures the system clock for the microcontroller.
 *
 * This function sets up the High-Speed Internal (HSI) oscillator and configures the Phase-Locked Loop (PLL)
 * parameters to generate a stable clock source for the CPU and peripherals. The configured system clock
 * will be used for the AHB and APB buses as well as other core functionalities.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;    // Structure for configuring oscillators
    RCC_ClkInitTypeDef RCC_ClkInitStruct;    // Structure for configuring clock settings

    /** Enable the power control clock for voltage regulation */
    __HAL_RCC_PWR_CLK_ENABLE();

    /** Configure the main internal regulator output voltage */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /** Initialize the CPU, AHB, and APB buses clocks using the HSI oscillator */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Set the oscillator type to HSI
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;                   // Enable the HSI oscillator
    RCC_OscInitStruct.HSICalibrationValue = 16;                // Set the HSI calibration value
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;               // Enable the PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;       // Set HSI as the PLL source
    RCC_OscInitStruct.PLL.PLLM = 16;                           // Set PLLM divisor
    RCC_OscInitStruct.PLL.PLLN = 336;                          // Set PLLN multiplier
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;                // Set PLLP divisor for main clock
    RCC_OscInitStruct.PLL.PLLQ = 2;                            // Set PLLQ divisor for USB clock
    RCC_OscInitStruct.PLL.PLLR = 2;                            // Set PLLR divisor
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);                    // Handle initialization error
    }

    /** Initialize the CPU, AHB, and APB buses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Use PLL output as system clock
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // Set AHB clock divider
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;          // Set APB1 clock divider
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;          // Set APB2 clock divider

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);                    // Handle configuration error
    }

    /** Configure the Systick interrupt time */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);          // Set SysTick to generate 1ms tick interrupts

    /** Configure the Systick clock source */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);       // Use HCLK as SysTick clock source

    /* Set SysTick_IRQn interrupt priority to 0 (highest priority) */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * @brief  Initializes the CRC peripheral.
 *
 * This function configures and initializes the CRC (Cyclic Redundancy Check) peripheral, which
 * is used for calculating CRC values for data integrity checks in the bootloader.
 */
static void MX_CRC_Init(void)
{
    hcrc.Instance = CRC;               // Select the CRC peripheral instance
    if (HAL_CRC_Init(&hcrc) != HAL_OK)  // Initialize the CRC peripheral
    {
        _Error_Handler(__FILE__, __LINE__);  // Handle initialization error
    }
}

/**
 * @brief  Initializes the USART2 peripheral for UART communication.
 *
 * This function sets up the parameters for the UART interface, including baud rate, word length,
 * stop bits, and hardware flow control. USART2 is typically used for general UART communication.
 */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;          // Select the USART2 peripheral instance
    huart2.Init.BaudRate = 115200;      // Set baud rate to 115200 bps
    huart2.Init.WordLength = UART_WORDLENGTH_8B;  // 8-bit word length
    huart2.Init.StopBits = UART_STOPBITS_1;        // 1 stop bit
    huart2.Init.Parity = UART_PARITY_NONE;         // No parity bit
    huart2.Init.Mode = UART_MODE_TX_RX;            // Enable TX and RX mode
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;   // No hardware flow control
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;  // Set oversampling to 16
    if (HAL_UART_Init(&huart2) != HAL_OK)           // Initialize UART
    {
        _Error_Handler(__FILE__, __LINE__);        // Handle initialization error
    }
}

/**
 * @brief  Initializes the USART3 peripheral for UART communication.
 *
 * Similar to the USART2 initialization, this function sets up the parameters for the UART interface.
 * USART3 is configured for a different UART instance, which may be used for communication with other devices.
 */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;          // Select the USART3 peripheral instance
    huart3.Init.BaudRate = 115200;      // Set baud rate to 115200 bps
    huart3.Init.WordLength = UART_WORDLENGTH_8B;  // 8-bit word length
    huart3.Init.StopBits = UART_STOPBITS_1;        // 1 stop bit
    huart3.Init.Parity = UART_PARITY_NONE;         // No parity bit
    huart3.Init.Mode = UART_MODE_TX_RX;            // Enable TX and RX mode
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;   // No hardware flow control
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;  // Set oversampling to 16
    if (HAL_UART_Init(&huart3) != HAL_OK)           // Initialize UART
    {
        _Error_Handler(__FILE__, __LINE__);        // Handle initialization error
    }
}


/**
 * @brief Configures GPIO pins as Analog, Input, Output, EVENT_OUT, or EXTI.
 *
 * This function is used to set up the GPIO pins for different purposes, such as digital input, output,
 * analog functions, external interrupt sources (EXTI), or other custom configurations. It utilizes
 * the HAL library to initialize the GPIO pins of the microcontroller.
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;  // Structure for GPIO configuration

    /* Enable clocks for GPIO ports C, H, A, and B */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pin Output Level for LD2 (an onboard LED) */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);  // Set the LD2 LED pin to a low state

    /* Configure GPIO pin : B1_Pin (User button) */
    GPIO_InitStruct.Pin = B1_Pin;                // Select pin B1
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Configure as external interrupt on falling edge
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // No pull-up or pull-down resistors
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);  // Initialize the pin configuration

    /* Configure GPIO pin : LD2_Pin (LED) */
    GPIO_InitStruct.Pin = LD2_Pin;               // Select pin LD2
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Configure as push-pull output mode
    GPIO_InitStruct.Pull = GPIO_NOPULL;          // No pull-up or pull-down resistors
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set output speed to low
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);  // Initialize the pin configuration
}

/**
 * @brief  This function is executed in case of error occurrence.
 *
 * This is a general error handler function used for reporting errors or unexpected behavior
 * in the program. When called, it will enter an infinite loop, halting the execution of the code.
 * Users can customize this function to handle errors more gracefully.
 *
 * @param file: Name of the source file where the error occurred.
 * @param line: Line number in the source file where the error occurred.
 */
void _Error_Handler(char * file, int line)
{
  while(1)
  {
      // Infinite loop to indicate error state
  }
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 *        where the assert_param error has occurred.
 *
 * This function is used by the `assert_param` macro to report errors in case a parameter value is invalid.
 * It prints the source file and line number where the error was detected. This is typically used in debugging
 * to identify and fix issues.
 *
 * @param file: Pointer to the source file name.
 * @param line: Line number where the assert_param error was triggered.
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  // User can implement custom error reporting here, e.g., printing error message or logging
}

#endif

/************** Implementation of Bootloader Command Handle Functions *********/

/**
 * @brief Handle the 'Get Version' (BL_GET_VER) command from the host.
 *
 * This function is used to respond to the host with the current version of the bootloader.
 * It reads the command packet, verifies its integrity using CRC, and sends the bootloader
 * version if the CRC check passes. Otherwise, it responds with a NACK.
 *
 * @param bl_rx_buffer: Pointer to the received command buffer containing the command packet.
 */
void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer)
{
    uint8_t bl_version;  // Variable to store the bootloader version

    // 1) Print a debug message indicating entry into the function.
    printmsg("BL_DEBUG_MSG:bootloader_handle_getver_cmd\n");

    // Calculate the total length of the received command packet.
    // The first byte of the packet contains the total length of the message.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extract the CRC32 sent by the host at the end of the command packet.
    // CRC is usually stored in the last 4 bytes of the received message.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Verify the integrity of the received packet by calculating the local CRC
    // and comparing it with the one received. If `bootloader_verify_crc` returns `0`, the CRC is valid.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // If CRC is correct, print a message indicating successful checksum verification.
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        // Send an acknowledgment (ACK) message to the host.
        // The first parameter is the command code, and the second is the length of the response data.
        bootloader_send_ack(bl_rx_buffer[0], 1);

        // Retrieve the current bootloader version using the `get_bootloader_version()` function.
        bl_version = get_bootloader_version();

        // Print the bootloader version in the debug terminal.
        printmsg("BL_DEBUG_MSG:BL_VER : %d %#x\n", bl_version, bl_version);

        // Send the bootloader version to the host via the UART interface.
        bootloader_uart_write_data(&bl_version, 1);
    }
    else
    {
        // If the CRC is incorrect, print a message indicating checksum failure.
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");

        // Send a negative acknowledgment (NACK) message to the host.
        bootloader_send_nack();
    }
}


/**
 * @brief Handles the 'Get Help' (BL_GET_HELP) command from the host.
 *
 * This command is used to inform the host of all supported commands by the bootloader.
 * When called, this function sends a list of all the supported commands back to the host.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet from the host.
 */
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer)
{
    // Print a debug message indicating that the 'Get Help' command is being processed.
    printmsg("BL_DEBUG_MSG:bootloader_handle_gethelp_cmd\n");

    // 1. Calculate the total length of the command packet.
    // The first byte of `bl_rx_buffer` contains the length of the data to follow.
    // Add 1 to include this byte in the total length.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // 2. Extract the CRC32 value sent by the host.
    // The CRC32 is stored in the last 4 bytes of the command packet.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // 3. Verify the validity of the CRC32 to ensure that the data has not been altered.
    // The `bootloader_verify_crc` function takes the following parameters:
    // - `&bl_rx_buffer[0]`: Pointer to the start of the command packet.
    // - `command_packet_len - 4`: Length of the data excluding the CRC.
    // - `host_crc`: CRC32 sent by the host for verification.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // If the CRC is valid, send a success message for checksum verification.
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        // 4. Send an acknowledgment (ACK) with the number of bytes the bootloader will send in response.
        // The second parameter of `bootloader_send_ack` is the length of the list of supported commands.
        bootloader_send_ack(pBuffer[0], sizeof(supported_commands));

        // 5. Send the list of supported commands back to the host.
        // `supported_commands` is a global array containing all the supported command codes.
        bootloader_uart_write_data(supported_commands, sizeof(supported_commands));
    }
    else
    {
        // If the CRC is incorrect, send a debug message indicating the failure of verification.
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");

        // 6. Send a negative acknowledgment (NACK) to indicate that the verification failed.
        bootloader_send_nack();
    }
}


/**
 * @brief Handles the 'Get Chip ID' (BL_GET_CID) command from the host.
 *
 * This command is used to read the unique chip ID of the microcontroller's architecture.
 * The chip ID allows the host to determine the ST part number and silicon revision of the device.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet from the host.
 */
void bootloader_handle_getcid_cmd(uint8_t *pBuffer)
{
    uint16_t bl_cid_num = 0;  // Variable to store the unique chip ID of the microcontroller.

    // Print a debug message indicating that the command is being processed.
    printmsg("BL_DEBUG_MSG:bootloader_handle_getcid_cmd\n");

    // 1. Calculate the total length of the command packet.
    // The first byte of `bl_rx_buffer` contains the length of the data to follow.
    // Add 1 to include this byte in the total length.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // 2. Extract the CRC32 value sent by the host.
    // The CRC32 is stored in the last 4 bytes of the command packet.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // 3. Verify the validity of the CRC32.
    // The `bootloader_verify_crc` function takes the following parameters:
    // - `&bl_rx_buffer[0]`: Pointer to the start of the command packet.
    // - `command_packet_len - 4`: Length of the data excluding the CRC.
    // - `host_crc`: CRC32 sent by the host for verification.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // If the CRC is valid, print a message indicating successful checksum verification.
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        // 4. Send an acknowledgment (ACK) with the length of the response (2 bytes for the Chip ID).
        bootloader_send_ack(pBuffer[0], 2);

        // 5. Retrieve the chip ID of the microcontroller.
        bl_cid_num = get_mcu_chip_id();
        printmsg("BL_DEBUG_MSG:MCU id : %d %#x !!\n", bl_cid_num, bl_cid_num);

        // 6. Send the chip ID (2 bytes) to the host.
        bootloader_uart_write_data((uint8_t *)&bl_cid_num, 2);
    }
    else
    {
        // If the CRC is not valid, print a message indicating checksum failure.
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");

        // Send a negative acknowledgment (NACK) to indicate that the verification failed.
        bootloader_send_nack();
    }
}



/**
 * @brief Handles the 'Get Read Protection Status' (BL_GET_RDP_STATUS) command.
 *
 * This function retrieves the Read Protection (RDP) level of the Flash memory in the microcontroller
 * and sends this information to the host system over UART.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet from the host.
 */
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer)
{
    // Variable to store the Read Protection Level (RDP) of the Flash memory.
    uint8_t rdp_level = 0x00;

    // Print a debug message indicating entry into the RDP command handler function.
    printmsg("BL_DEBUG_MSG:bootloader_handle_getrdp_cmd\n");

    // Calculate the total length of the command packet by adding 1 to the value in the first byte.
    // The first byte of `bl_rx_buffer` represents the length of the data to follow.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extract the CRC32 sent by the host, which is located in the last 4 bytes of the command packet.
    // This calculation is done by pointing to the position of the CRC and converting it into a 32-bit pointer.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Verify the CRC of the command packet to ensure data integrity.
    // The `bootloader_verify_crc` function returns 0 if the CRC is correct.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Print a debug message indicating that the CRC is valid.
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        // Send an acknowledgment (ACK) along with the length of the data to follow (1 byte in this case).
        // `pBuffer[0]` represents the command code sent by the host.
        bootloader_send_ack(pBuffer[0], 1);

        // Retrieve the current RDP level from the Flash memory.
        // This function reads the byte corresponding to the Read Protection level.
        rdp_level = get_flash_rdp_level();

        // Print the RDP level in the debug console in both decimal and hexadecimal formats.
        printmsg("BL_DEBUG_MSG:RDP level: %d %#x\n", rdp_level, rdp_level);

        // Send the RDP level to the host system over UART.
        // The `rdp_level` is sent as a single 1-byte value.
        bootloader_uart_write_data(&rdp_level, 1);

    }
    else
    {
        // If the CRC is incorrect, print a debug error message.
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");

        // Send a Negative Acknowledgment (NACK) to indicate that the CRC is invalid.
        bootloader_send_nack();
    }
}


/**
 * @brief Handles the 'Go to Address' (BL_GO_TO_ADDR) command.
 *
 * This command is used to jump to an address specified by the host.
 * The function verifies the integrity of the command packet via the CRC, validates the address,
 * and if everything is correct, it jumps to the specified address.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet from the host.
 */
void bootloader_handle_go_cmd(uint8_t *pBuffer)
{
    // Variable to store the address where the bootloader should jump.
    uint32_t go_address = 0;

    // Variables to indicate whether the address is valid or not.
    uint8_t addr_valid = ADDR_VALID;
    uint8_t addr_invalid = ADDR_INVALID;

    // Print a debug message indicating entry into the GO command handler function.
    printmsg("BL_DEBUG_MSG:bootloader_handle_go_cmd\n");

    // Calculate the total length of the command packet (the first byte contains the length of the data to follow).
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extract the CRC32 sent by the host, which is located in the last 4 bytes of the packet.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Verify the CRC of the command packet to ensure data integrity.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        // Print a message indicating that the CRC is valid.
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        // Send an acknowledgment (ACK) to the host to confirm correct reception of the command.
        bootloader_send_ack(pBuffer[0], 1);

        // Extract the address where the host wants the bootloader to jump (4 bytes starting from the 3rd byte).
        go_address = *((uint32_t *)&pBuffer[2]);
        printmsg("BL_DEBUG_MSG:GO addr: %#x\n", go_address);

        // Verify if the extracted address is a valid execution address.
        if (verify_address(go_address) == ADDR_VALID)
        {
            // Indicate to the host that the address is valid.
            bootloader_uart_write_data(&addr_valid, 1);

            /* Adjust the address by adding 1 to set the Thumb bit (T bit) to 1.
             * This is necessary because ARM Cortex M processors use the T bit
             * to indicate that the instructions to execute are in Thumb mode.
             */
            go_address += 1;

            // Declare a function pointer to jump to the specified address.
            void (*lets_jump)(void) = (void *)go_address;

            // Print a message indicating that the bootloader is jumping to the specified address.
            printmsg("BL_DEBUG_MSG: jumping to go address! \n");

            // Jump to the specified address and execute the code at that location.
            lets_jump();
        }
        else
        {
            // If the address is not valid, print a debug message.
            printmsg("BL_DEBUG_MSG:GO addr invalid ! \n");

            // Indicate to the host that the address is invalid by sending an error code.
            bootloader_uart_write_data(&addr_invalid, 1);
        }
    }
    else
    {
        // If the CRC is incorrect, print an error message.
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");

        // Send a Negative Acknowledgment (NACK) to indicate that the CRC is invalid.
        bootloader_send_nack();
    }
}


/**
 * @brief Handles the 'Flash Erase' (BL_FLASH_ERASE) command.
 *
 * This function processes the 'Flash Erase' command received from the host.
 * It validates the CRC for the received command packet, extracts the erase details,
 * performs the Flash erase operation, and sends the status back to the host.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet.
 */
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer)
{
    uint8_t erase_status = 0x00; // Variable to capture the erase status.
    printmsg("BL_DEBUG_MSG:bootloader_handle_flash_erase_cmd\n");

    // Calculate the total length of the command packet.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extract the CRC32 value sent by the host.
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    // Verify the CRC to ensure data integrity.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        // Send an acknowledgment (ACK) to the host.
        bootloader_send_ack(pBuffer[0], 1);

        // Extract erase information: initial sector and number of sectors.
        printmsg("BL_DEBUG_MSG:initial_sector : %d  no_ofsectors: %d\n", pBuffer[2], pBuffer[3]);

        // Turn on an LED during the erase operation to indicate that the operation is ongoing.
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
        erase_status = execute_flash_erase(pBuffer[2], pBuffer[3]); // Call the Flash erase function.
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0); // Turn off the LED after the erase operation.

        // Display the erase status.
        printmsg("BL_DEBUG_MSG: flash erase status: %#x\n", erase_status);

        // Send the erase status back to the host.
        bootloader_uart_write_data(&erase_status, 1);
    }
    else
    {
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        // If the CRC is invalid, send a NACK to the host.
        bootloader_send_nack();
    }
}


/**
 * @brief Handles the 'Memory Write' (BL_MEM_WRITE) command.
 *
 * This function processes the 'Memory Write' command received from the host.
 * It validates the CRC for the received command packet, checks if the memory address is valid,
 * performs the write operation, and sends the write status back to the host.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet.
 */
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer)
{
    // Initialize status variables.
    uint8_t addr_valid = ADDR_VALID;          // Indicator if the address is valid.
    uint8_t write_status = 0x00;              // Status of the write operation.
    uint8_t chksum = 0, len = 0;              // Variables for length and checksum verification.

    // Read the command packet length.
    len = pBuffer[0];

    // Length of the payload (actual data to be written).
    uint8_t payload_len = pBuffer[6];

    // Extract the memory address where the write should begin.
    uint32_t mem_address = *((uint32_t *) ( &pBuffer[2]) );

    // Extract the checksum sent by the host to verify data integrity.
    chksum = pBuffer[len];

    // Print a debug message indicating execution of the command.
    printmsg("BL_DEBUG_MSG:bootloader_handle_mem_write_cmd\n");

    // Calculate the total length of the command packet.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extract the CRC32 sent by the host for verification.
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    // Verify the CRC to ensure data integrity.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg("BL_DEBUG_MSG:checksum success !!\n");

        // Send an acknowledgment (ACK) to the host.
        bootloader_send_ack(pBuffer[0], 1);

        // Print the target memory address for the write operation.
        printmsg("BL_DEBUG_MSG: mem write address : %#x\n", mem_address);

        // Check if the target memory address is valid.
        if (verify_address(mem_address) == ADDR_VALID)
        {
            printmsg("BL_DEBUG_MSG: valid mem write address\n");

            // Turn on the LED to indicate that the write operation is ongoing.
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

            // Perform the memory write operation.
            write_status = execute_mem_write(&pBuffer[7], mem_address, payload_len);

            // Turn off the LED to indicate the end of the write operation.
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

            // Send the write status back to the host (success or failure).
            bootloader_uart_write_data(&write_status, 1);
        }
        else
        {
            printmsg("BL_DEBUG_MSG: invalid mem write address\n");
            write_status = ADDR_INVALID;

            // Send the invalid address status to the host.
            bootloader_uart_write_data(&write_status, 1);
        }
    }
    else
    {
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");

        // Send a NACK if the CRC verification fails.
        bootloader_send_nack();
    }
}


/**
 * @brief Handles the 'Enable/Disable Read/Write Protection' (BL_EN_RW_PROTECT) command.
 *
 * This function processes the command to enable or disable read/write protection on Flash sectors.
 * It verifies the CRC of the received command packet, applies the desired protection,
 * and sends the status back to the host.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet.
 */
void bootloader_handle_en_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;  // Variable to store the protection status.
    printmsg("BL_DEBUG_MSG:bootloader_handle_endis_rw_protect\n");  // Debug message indicating command processing.

    // Calculate the total length of the command packet.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extract the CRC32 value sent by the host.
    uint32_t host_crc = *((uint32_t *) (bl_rx_buffer + command_packet_len - 4));

    // Verify the CRC to ensure data integrity.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg("BL_DEBUG_MSG:checksum success !!\n");  // CRC is valid, proceed with processing.

        // Send an acknowledgment (ACK) to the host.
        bootloader_send_ack(pBuffer[0], 1);

        // Enable or disable the protection based on the command parameters.
        status = configure_flash_sector_rw_protection(pBuffer[2], pBuffer[3], 0);

        printmsg("BL_DEBUG_MSG: flash erase status: %#x\n", status);

        // Send the protection status back to the host.
        bootloader_uart_write_data(&status, 1);
    }
    else
    {
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");  // CRC is invalid, send a NACK.
        bootloader_send_nack();
    }
}



/**
 * @brief Handles the 'Disable Read/Write Protection' (BL_DIS_RW_PROTECT) command.
 *
 * This function processes the command to disable read/write protection on Flash sectors.
 * It verifies the CRC of the received command packet, disables the protection,
 * and sends the status back to the host.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet.
 */
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer)
{
    uint8_t status = 0x00;
    printmsg("BL_DEBUG_MSG:bootloader_handle_dis_rw_protect\n");

    // Calculate the total length of the command packet.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extract the CRC32 value sent by the host.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Verify the CRC to ensure data integrity.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg("BL_DEBUG_MSG:checksum success !!\n");
        bootloader_send_ack(pBuffer[0], 1);

        // Disable the read/write protection.
        status = configure_flash_sector_rw_protection(0, 0, 1);

        printmsg("BL_DEBUG_MSG: flash erase status: %#x\n", status);

        // Send the status of the protection disabling operation to the host.
        bootloader_uart_write_data(&status, 1);
    }
    else
    {
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bootloader_send_nack();
    }
}

/**
 * @brief Handles the 'Memory Read' (BL_MEM_READ) command.
 *
 * This function is a placeholder for implementing the memory read functionality
 * to read a specified memory address and return the data to the host.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet.
 */
void bootloader_handle_mem_read(uint8_t *pBuffer)
{
    // To be implemented in the future for reading memory contents.
}

/**
 * @brief Handles the 'Read Sector Protection Status' (BL_READ_SECTOR_P_STATUS) command.
 *
 * This function reads the status of the sector-level read/write protection.
 * It validates the CRC for the received command packet, retrieves the protection status,
 * and sends it back to the host.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet.
 */
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer)
{
    uint16_t status; // Variable to store the protection status.
    printmsg("BL_DEBUG_MSG:bootloader_handle_read_sector_protection_status\n");

    // Calculate the total length of the command packet.
    uint32_t command_packet_len = bl_rx_buffer[0] + 1;

    // Extract the CRC32 value sent by the host.
    uint32_t host_crc = *((uint32_t *)(bl_rx_buffer + command_packet_len - 4));

    // Verify the CRC to ensure data integrity.
    if (!bootloader_verify_crc(&bl_rx_buffer[0], command_packet_len - 4, host_crc))
    {
        printmsg("BL_DEBUG_MSG:checksum success !!\n");
        bootloader_send_ack(pBuffer[0], 2);

        // Read the protection status from the Option Bytes (OB).
        status = read_OB_rw_protection_status();

        printmsg("BL_DEBUG_MSG: nWRP status: %#x\n", status);

        // Send the read/write protection status to the host.
        bootloader_uart_write_data((uint8_t *)&status, 2);
    }
    else
    {
        printmsg("BL_DEBUG_MSG:checksum fail !!\n");
        bootloader_send_nack();
    }
}

/**
 * @brief Handles the 'Read OTP Memory' (BL_OTP_READ) command.
 *
 * This function is a placeholder for implementing the OTP (One-Time Programmable) memory read functionality.
 *
 * @param pBuffer: Pointer to the buffer containing the received command packet.
 */
void bootloader_handle_read_otp(uint8_t *pBuffer)
{
    // To be implemented in the future for reading OTP memory contents.
}

/**
 * @brief Sends an acknowledgment (ACK) message with a follow-up data length.
 *
 * This function is used to indicate that the command sent by the host has been successfully
 * received and processed by the bootloader.
 *
 * @param command_code: The command code for which the ACK is being sent.
 * @param follow_len: Length of the follow-up data to be sent.
 */
void bootloader_send_ack(uint8_t command_code, uint8_t follow_len)
{
    // Declare a 2-byte array to hold the ACK message.
    uint8_t ack_buf[2];

    // Set the first byte of the array to the ACK value (defined as BL_ACK).
    ack_buf[0] = BL_ACK;

    // Set the second byte to the length of the follow-up data as specified by `follow_len`.
    ack_buf[1] = follow_len;

    // Use the HAL_UART_Transmit function to send the 2-byte ACK message over UART.
    // `C_UART`: the UART configuration used.
    // `ack_buf`: the data array to be sent.
    // `2`: number of bytes to send.
    // `HAL_MAX_DELAY`: wait indefinitely until transmission is complete.
    HAL_UART_Transmit(C_UART, ack_buf, 2, HAL_MAX_DELAY);
}

/**
 * @brief Sends a Negative Acknowledgment (NACK) message.
 *
 * This function is used to indicate that the CRC of the command sent by the host is incorrect
 * or that the command could not be processed.
 */
void bootloader_send_nack(void)
{
    // Declare a variable to hold the NACK code.
    uint8_t nack = BL_NACK;

    // Use HAL_UART_Transmit to send the NACK code over UART.
    // `C_UART`: the UART configuration used.
    // `&nack`: address of the variable holding the NACK code.
    // `1`: number of bytes to send (1 byte for NACK).
    // `HAL_MAX_DELAY`: wait indefinitely until transmission is complete.
    HAL_UART_Transmit(C_UART, &nack, 1, HAL_MAX_DELAY);
}

/**
 * @brief Verifies the CRC (Cyclic Redundancy Check) of a received data buffer.
 *
 * This function takes as input a pointer to the data (`pData`), the length of the data (`len`),
 * and the CRC value sent by the host (`crc_host`).
 * If the calculated CRC matches the host CRC, it returns `VERIFY_CRC_SUCCESS`.
 * Otherwise, it returns `VERIFY_CRC_FAIL`.
 *
 * @param pData: Pointer to the data buffer to verify.
 * @param len: Length of the data to process.
 * @param crc_host: CRC value received from the host.
 * @retval VERIFY_CRC_SUCCESS if CRC is correct, VERIFY_CRC_FAIL otherwise.
 */
uint8_t bootloader_verify_crc(uint8_t *pData, uint32_t len, uint32_t crc_host)
{
    // Initialize the variable to store the calculated CRC value.
    // Default value is set to 0xff, which is the initial value of CRC.
    uint32_t uwCRCValue = 0xff;

    // Loop through each byte in the data to accumulate the CRC.
    for (uint32_t i = 0; i < len; i++)
    {
        // Convert the current byte to a 32-bit value for the CRC calculation.
        uint32_t i_data = pData[i];

        // Calculate and accumulate the CRC for each byte using the HAL_CRC_Accumulate API.
        // `&hcrc`: pointer to the CRC peripheral structure.
        // `&i_data`: address of the data to calculate the CRC on.
        // `1`: number of values (bytes) to process.
        uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);
    }

    // Reset the CRC calculation unit to prepare the CRC peripheral for the next calculation.
    __HAL_CRC_DR_RESET(&hcrc);

    // Check if the calculated CRC (`uwCRCValue`) matches the received CRC (`crc_host`).
    if (uwCRCValue == crc_host)
    {
        // If the values match, the CRC is correct.
        return VERIFY_CRC_SUCCESS;  // Return 0 to indicate success.
    }

    // If the CRC does not match, return a value indicating failure.
    return VERIFY_CRC_FAIL;  // Return 1 to indicate failure.
}



/**
 * @brief Sends data to the serial terminal via UART.
 * This function uses the HAL_UART_Transmit API to send data over the specified UART.
 *
 * @param pBuffer: Pointer to the buffer containing the data to be transmitted.
 * @param len: Length of the data to be transmitted (in bytes).
 */
void bootloader_uart_write_data(uint8_t *pBuffer, uint32_t len)
{
    // Use the HAL_UART_Transmit API to send the data.
    // C_UART is the UART peripheral configured for communication (e.g., UART2).
    // pBuffer: Address of the memory location containing the data to be transmitted.
    // len: Number of bytes to be transmitted.
    // HAL_MAX_DELAY: Indicates that the function should wait indefinitely if the UART is busy.
    HAL_UART_Transmit(C_UART, pBuffer, len, HAL_MAX_DELAY);
}

/**
 * @brief Returns the bootloader version.
 * This function simply returns the value defined in the macro `BL_VERSION`.
 *
 * @retval The bootloader version as an 8-bit integer.
 */
uint8_t get_bootloader_version(void)
{
    // Return the bootloader version defined by the `BL_VERSION` macro.
    return (uint8_t)BL_VERSION;
}

/**
 * @brief Reads the unique chip identifier of the microcontroller.
 * The microcontroller's unique identifier is stored in the `IDCODE` register of the `DBGMCU` component.
 * The first 12 bits of this register contain the Device ID, which identifies the MCU's serial number.
 *
 * @retval uint16_t: The unique identifier of the microcontroller.
 */
uint16_t get_mcu_chip_id(void)
{
    /*
     * STM32F4xx MCUs integrate an identification code (ID code).
     * This code is contained in the `DBGMCU->IDCODE` register.
     * Bits [11:0] of this register identify the Device ID.
     * The `DBGMCU->IDCODE` register is mapped at address 0xE0042000.
     */
    uint16_t cid;

    // Read the `IDCODE` register and mask the lower 12 bits to get the Device ID.
    cid = (uint16_t)(DBGMCU->IDCODE) & 0x0FFF;

    // Return the unique identifier.
    return cid;
}

/**
 * @brief Reads the current Read Protection Level (RDP) of the Flash memory.
 * The Read Protection Level is used to protect the Flash memory content from read, write, and erase operations.
 *
 * @retval A `uint8_t` representing the current RDP level:
 *         - 0xAA: Level 0 (No protection)
 *         - 0xBB: Level 1 (Protection enabled)
 *         - 0xCC: Level 2 (Maximum protection, irreversible)
 */
uint8_t get_flash_rdp_level(void)
{
    // Variable to store the read RDP level.
    uint8_t rdp_status = 0;

    // Compilation directive to select the implementation based on the preferred method.
#if 0
    // Method 1: Using the HAL library to read the option bytes.

    // HAL structure to hold the configuration of the option bytes.
    FLASH_OBProgramInitTypeDef ob_handle;

    // Retrieve the current configuration of the option bytes, including the RDP level.
    HAL_FLASHEx_OBGetConfig(&ob_handle);

    // The RDP level is stored in the `RDPLevel` field of the `ob_handle` structure.
    rdp_status = (uint8_t)ob_handle.RDPLevel;

#else
    // Method 2: Directly reading from the memory address where the option bytes are stored.

    // Start address of the option bytes in STM32F446xx.
    // The option bytes are stored at address 0x1FFFC000 in the Flash memory.
    volatile uint32_t *pOB_addr = (uint32_t*)0x1FFFC000;

    // Read bits 8 to 15 to get the RDP level.
    // Perform a right shift by 8 bits to isolate the second set of 8 bits in the 32-bit register.
    rdp_status = (uint8_t)(*pOB_addr >> 8);

#endif

    // Return the read RDP level.
    return rdp_status;
}


/**
 * @brief Verifies the address sent by the host.
 * Returns ADDR_VALID if the address is valid for code execution.
 * Otherwise, returns ADDR_INVALID.
 */
uint8_t verify_address(uint32_t go_address)
{
    // Check if the address belongs to SRAM1 memory
    if (go_address >= SRAM1_BASE && go_address <= SRAM1_END)
    {
        return ADDR_VALID;
    }
    // Check if the address belongs to SRAM2 memory
    else if (go_address >= SRAM2_BASE && go_address <= SRAM2_END)
    {
        return ADDR_VALID;
    }
    // Check if the address belongs to Flash memory (user memory)
    else if (go_address >= FLASH_BASE && go_address <= FLASH_END)
    {
        return ADDR_VALID;
    }
    // Check if the address belongs to Backup SRAM memory
    else if (go_address >= BKPSRAM_BASE && go_address <= BKPSRAM_END)
    {
        return ADDR_VALID;
    }
    else
    {
        return ADDR_INVALID; // The address is not in a valid range
    }
}

/**
 * @brief Erases Flash memory sectors.
 *
 * We have a total of 8 sectors (from 0 to 7) in this microcontroller.
 * The number of sectors should be in the range of 0 to 7.
 * If the sector = 0xFF, this indicates that a mass erase (complete erase) is requested.
 */
uint8_t execute_flash_erase(uint8_t sector_number, uint8_t number_of_sector)
{
    FLASH_EraseInitTypeDef flashErase_handle; // Structure to manage the erase process
    uint32_t sectorError; // Variable to capture sector errors
    HAL_StatusTypeDef status; // Return status of the HAL API

    // If the number of sectors exceeds 8, return an invalid sector error
    if (number_of_sector > 8)
        return INVALID_SECTOR;

    // Ensure the sector is either 0xFF (mass erase) or between 0 and 7
    if ((sector_number == 0xFF) || (sector_number <= 7))
    {
        // If it's a mass erase
        if (sector_number == 0xFF)
        {
            flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE; // Indicate a full memory erase
        }
        // If it's specific sector erase
        else
        {
            // Calculate remaining sectors from the starting sector
            uint8_t remaining_sector = 8 - sector_number;
            if (number_of_sector > remaining_sector)
            {
                number_of_sector = remaining_sector;
            }
            flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS; // Indicate sector-by-sector erase
            flashErase_handle.Sector = sector_number; // First sector to erase
            flashErase_handle.NbSectors = number_of_sector; // Number of sectors to erase
        }
        flashErase_handle.Banks = FLASH_BANK_1; // Erasing on Flash Bank 1

        // Unlock the Flash memory for write access
        HAL_FLASH_Unlock();
        flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3; // Voltage range used by the microcontroller
        // Call the HAL API to perform the erase operation
        status = (uint8_t) HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
        // Lock the Flash memory after erasing
        HAL_FLASH_Lock();

        return status; // Return the status of the operation (OK, error, etc.)
    }

    return INVALID_SECTOR; // If the sector number is invalid, return an error
}

/**
 * @brief Writes the contents of pBuffer to "mem_address" byte by byte.
 *
 * Note1: Currently this function supports writing to Flash only.
 * Note2: This function does not check whether "mem_address" is a valid address in the Flash range.
 */
uint8_t execute_mem_write(uint8_t *pBuffer, uint32_t mem_address, uint32_t len)
{
    uint8_t status = HAL_OK;

    // Unlock Flash memory for write operations
    HAL_FLASH_Unlock();

    // Write each byte to the memory
    for(uint32_t i = 0; i < len; i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, mem_address + i, pBuffer[i]);
    }

    // Lock the Flash memory after the operation
    HAL_FLASH_Lock();

    return status;
}




/**
 * @brief Modifies user option bytes to configure Read/Write protection.
 * To modify the user option value, follow the sequence below:
 * 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_SR register.
 * 2. Write the desired option value in the FLASH_OPTCR register.
 * 3. Set the option start bit (OPTSTRT) in the FLASH_OPTCR register.
 * 4. Wait for the BSY bit to be cleared.
 *
 * This function configures sector-level read/write protection based on the provided mode:
 * - If `disable` is set, it disables read/write protection on all sectors.
 * - If `protection_mode` is 1, it enables write-only protection for the specified sectors.
 * - If `protection_mode` is 2, it enables read/write protection for the specified sectors.
 *
 * @param sector_details : Bitmask for sectors to be protected.
 * @param protection_mode : The type of protection (1: Write Protection, 2: Read/Write Protection).
 * @param disable : If set, disables the protection.
 *
 * @return 0 on success.
 */
uint8_t configure_flash_sector_rw_protection(uint8_t sector_details, uint8_t protection_mode, uint8_t disable)
{
    // Pointer to the Flash option control register (OPTCR)
    volatile uint32_t *pOPTCR = (uint32_t*) 0x40023C14;

    // Disable read/write protection if `disable` argument is set
    if(disable)
    {
        HAL_FLASH_OB_Unlock();  // Unlock the Flash option bytes

        // Wait until Flash is not busy
        while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);

        // Clear protection on all sectors
        *pOPTCR &= ~(1 << 31);  // Clear the SPRMOD bit (set for read/write protection)
        *pOPTCR |= (0xFF << 16);  // Set all bits to disable protection on all sectors

        *pOPTCR |= (1 << 1);  // Start the option programming

        while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Wait until Flash is not busy

        HAL_FLASH_OB_Lock();  // Lock the Flash option bytes
        return 0;
    }

    // Configure protection based on the selected mode
    if(protection_mode == (uint8_t) 1)  // Mode 1: Write-only protection
    {
        HAL_FLASH_OB_Unlock();  // Unlock the Flash option bytes

        while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Wait until Flash is not busy

        *pOPTCR &= ~(1 << 31);  // Disable read protection (bit 31 to 0)
        *pOPTCR &= ~(sector_details << 16);  // Enable write protection on the specified sectors

        *pOPTCR |= (1 << 1);  // Start the option programming

        while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Wait until Flash is not busy

        HAL_FLASH_OB_Lock();  // Lock the Flash option bytes
    }
    else if (protection_mode == (uint8_t) 2)  // Mode 2: Read/Write protection
    {
        HAL_FLASH_OB_Unlock();  // Unlock the Flash option bytes

        while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Wait until Flash is not busy

        *pOPTCR |= (1 << 31);  // Enable read protection (bit 31 to 1)
        *pOPTCR &= ~(0xff << 16);  // Clear all protection bits
        *pOPTCR |= (sector_details << 16);  // Enable read/write protection on the specified sectors

        *pOPTCR |= (1 << 1);  // Start the option programming

        while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY) != RESET);  // Wait until Flash is not busy

        HAL_FLASH_OB_Lock();  // Lock the Flash option bytes
    }

    return 0;
}

/**
 * @brief Reads the read/write protection status of the Flash option bytes.
 *
 * This function reads the Option Byte (OB) configuration to determine the current read/write protection status.
 * The protection status is stored in the `WRPSector` field of the OB structure.
 *
 * @return A 16-bit value representing the sectors with active read/write protection.
 */
uint16_t read_OB_rw_protection_status(void)
{
    // This structure is provided by the ST Flash driver to hold the OB (Option Byte) contents.
    FLASH_OBProgramInitTypeDef OBInit;

    // First, unlock the OB (Option Byte) memory access
    HAL_FLASH_OB_Unlock();
    // Retrieve the OB configuration details
    HAL_FLASHEx_OBGetConfig(&OBInit);
    // Lock the OB memory access
    HAL_FLASH_Lock();

    // We are only interested in the r/w protection status of the sectors.
    return (uint16_t)OBInit.WRPSector;
}
