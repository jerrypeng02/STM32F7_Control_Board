#include "main.h"
#include "stm32f7xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <string.h>

#define MAX_DEVICE_COUNT 8	// The maximum number of devices a single array can contain
#define VL_DEF_SLAVE_ADDR 0x52	// The shifted default sensor (VL6180X) I2C ddevice address
#define PCA_SLAVE_ADDR 0x40	// The I2C address of the testing bottom array's gpio expander
#define IDEN_MODEL_ID_ADDR 0x00	// The identification register address in the device
#define IDEN_MODEL_ID_VAL 0xB4	// The identification register value in the device

#define RANGE_ONLY
#define SEND_ONLY
//#define CALLBACK

#ifdef RAN_CAL	// Not yet used MACRO
#define RANGE_REG 0x0062 // VL6180X range measurement register address
#define RANGE_RATE_REG 0x0066 // VL6180x range rate measurement register address
#define RANGE_RATE_THRESH_REG 0x0026 // VL6180X register for setting the required signal level
#define INTERMEASUREMENT_REG 0x001B // VL6180X register for time between range measurements 
#define MAX_CONVERGENCE_REG 0x001C // VL6180X register for max time length of measurement 
#define RANGE_OFFSET_REG 0x0024 // VL6180x register for range offset calibration value
#define CROSSTALK_COMPENSATION_REG 0x001E // VL6180X register for cross-talk calibration value
#define MAX_INTERMEASUREMENT_PERIOD_MS 76 // The longest possible time between measurements
#endif

#define MAX_REGISTERS 5 // The maximum number of registers in the register list

typedef struct _pca {
	uint16_t addr;	// The slave address of the gpio expander
	uint8_t cur_device_output;	// The current output of the device
}pca;

typedef struct _vl6180x {
	uint16_t addr;	// The slave address of the sensor
	int8_t expander_id;	// The pin on the expander this device is connected to, -1 if not used
	pca* expander;	// The expander used to toggle GPIO0, 0 if not used
}vl6180x;

// Data structure that represents a single array of devices
typedef struct _vl_array {
	uint8_t device_count; // Number of devices in array
	vl6180x device_queue[MAX_DEVICE_COUNT]; // The devices in the array
	uint16_t cur_reg_addrs[MAX_REGISTERS]; // The addresses in the register list
	uint8_t cur_reg_lengths[MAX_REGISTERS]; // The lengths of registers in the register list
	uint8_t cur_reg_count; // The number of registers in the register list
	uint8_t cur_data[4 * MAX_REGISTERS * MAX_DEVICE_COUNT];  // Buffer for serialized register data
	//uint8_t calib_result[2 * MAX_DEVICE_COUNT]; // Buffer for results of calibration
}vl_array;

vl_array bottom;
volatile uint8_t timer_flag;	// Not used for now

void write_byte(vl6180x * vlPtr, uint16_t reg_addr, uint8_t data);
void write_byte_pca(uint16_t slave_addr, uint8_t reg_addr, uint8_t data);
void read_bytes(uint16_t slave_addr, uint16_t reg_addr, uint8_t* data, uint16_t n);
uint8_t vl6180x_init(vl6180x * vlPtr, uint8_t addr);
void change_address(vl6180x * vlPtr, uint8_t new_addr);
void toggle_measurement(vl6180x * vlPtr);
void pca_setup(pca * expander);
void pca_set_pin(vl6180x * vlPtr);
void pca_reset_pin(vl6180x * vlPtr);

#ifdef CALLBACK
uint8_t myBuf[20] = "Hello World \r\n"; 
uint8_t dataa[] = "Received a \r\n";
uint8_t datas[] = "Received s \r\n";
uint8_t initialData[] = "Initial data \r\n";
uint8_t * ptr;	// Pointer that will point to different data buffer above
#endif

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();

	// Initialize the bottom gpio expander device
	pca expander;
	expander.addr = PCA_SLAVE_ADDR;
	expander.cur_device_output = 0xff;
	pca_setup(&expander);

	timer_flag = 0; // Not used for now

	HAL_Delay(100);

	bottom.device_count = 8; // Set to correct device count
	bottom.cur_reg_count = 0; // Initialize number of registers in register list

	for(int i = 0; i < bottom.device_count; i++) {
		// Set bottom gpio expander as method for initializing array
		bottom.device_queue[i].expander = &expander;
		// Devices are not in descending order in hardware, so we correct for this in software
		if(i < 4) {
				bottom.device_queue[i].expander_id = i;
		} else {
				bottom.device_queue[i].expander_id = (bottom.device_count - 1) + (4 - i);
		}
	}

	// Pull all VL6180X GPIO 0 low for first step of re-addressing each device
	for(int i = 0; i < bottom.device_count; i++) {
		pca_reset_pin(&(bottom.device_queue[i])); // Expander will pull VL6180X gpio low
		HAL_Delay(10);
	}

	HAL_Delay(200);

	// One-by-one, pull VL6180X out of reset and assign to unique address
	uint8_t init_count = 0;
	for(int i = 0; i < bottom.device_count; i++) {
		pca_set_pin(&(bottom.device_queue[i])); // Expander unresets device
		HAL_Delay(1);

		// Configure the sensor
		vl6180x_init(&(bottom.device_queue[i]), VL_DEF_SLAVE_ADDR + 2 * (init_count + 1)); // Initialize VL6180X device with unique address
		init_count += 1;
	}

	HAL_Delay(100);

	// Test if the new addresses were set to the VL6180X devices in the bottom array
	#ifdef TEST_I2C
	uint8_t tempData[8];
	for(int i = 0; i < bottom.device_count; i++) {
		//read_bytes(VL_DEF_SLAVE_ADDR + 2 * (init_count + 1), IDEN_MODEL_ID_ADDR, &tempData[i], 1);
		read_bytes(bottom.device_queue[i].addr, IDEN_MODEL_ID_ADDR, &tempData[i], 1);
		if(tempData[i] == IDEN_MODEL_ID_VAL) {
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
			HAL_Delay(200);
		}
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
		HAL_Delay(200);
	}
	#endif

	// Offset initialization of measurements from each device to try to de-correlate inter-device noise
	#ifdef INIT_MEAS
	for(int i = 0; i < bottom.device_count; i++) {
		HAL_Delay(30); // Wait 30 ms
		for(int j = 0; j < i; j++) { // Additional delay depending on device idx
			HAL_Delay(100);
		}
		toggle_measurement(&(bottom.device_queue[i])); // Start measuring
		timer_flag = 0; // Not used for now
	}
	#endif
	
	#ifdef CALLBACK
	ptr = &initialData[0];
	#endif

	#ifdef SEND_ONLY
	uint8_t myBuf[20] = "Hello World \r\n";
	#endif

	while (1)
	{
		// Sending initial data at first 
		// And then sending different data
		// According to the pressed key
		#ifdef CALLBACK
		CDC_Transmit_HS(ptr, strlen((char *)ptr));
		#endif
		
		// Only send hello world over the VCP
		#ifdef SEND_ONLY
		CDC_Transmit_HS(myBuf, strlen((char *)myBuf));
		// Toggle the GPIO E2 pin to see if the data sending is successful
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
		#endif
		
		HAL_Delay(100);

		// Measure process from the sensors to the data buffer
		#ifdef START_MEAS
		uint32_t data_count = 0;
		for(unsigned int i = 0; i < bottom.device_count; i++) {
			for(unsigned int j = 0; j < bottom.cur_reg_count; j++) {
				// Read register and serialize into device array's data buffer for quick transmission
				read_bytes(bottom.device_queue[i].addr, bottom.cur_reg_addrs[j], bottom.cur_data + data_count, bottom.cur_reg_lengths[j]);
				data_count += bottom.cur_reg_lengths[j];
			}
		}
		#endif
	}
}

/*
	Call back function

	When receive key a, send data buffer dataa
	when receive key s
*/
#ifdef CALLBACK
void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len) {
	if(buf[0] == 'a') {
		ptr = &dataa[0];
	} else if(buf[0] == 's') {
		ptr = &datas[0];
	} else {
		ptr = &myBuf[0];
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
	}
}
#endif

/*
	Use I2C to write bytes to a device
	vlPtr: The device with a I2C slave address
	reg_addr: Device register address to write to
	data: The data to write
*/
void write_byte(vl6180x * vlPtr, uint16_t reg_addr, uint8_t data) {
	uint8_t buffer[3];
	buffer[0] = reg_addr >> 8;
	buffer[1] = reg_addr;
	buffer[2] = data;
	HAL_I2C_Master_Transmit(&hi2c1, vlPtr->addr, buffer, 3, 10);
}

/*
	Use I2C to write bytes to a device
	slave_addr: I2C address of device
	reg_addr: Device register address to write to
	data: The data to write
*/
void write_byte_pca(uint16_t slave_addr, uint8_t reg_addr, uint8_t data) {
	uint8_t buffer[2];
	buffer[0] = reg_addr;
	buffer[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, slave_addr, buffer, 2, 10);
}

/* 
	Read from a device over I2C
	slave_addr: The I2C adddress of the device
	reg_addr: Device register address to write to
	data: The data to write to
	n: The number of bytes to write
*/
void read_bytes(uint16_t slave_addr, uint16_t reg_addr, uint8_t* data, uint16_t n) {
	uint8_t buffer[2];
	buffer[0] = reg_addr >> 8;
	buffer[1] = reg_addr;
	HAL_I2C_Master_Transmit(&hi2c1, slave_addr, buffer, 2, 10);
	HAL_I2C_Master_Receive(&hi2c1, slave_addr, data, n, 10);
}

/* 
	Initialize a VL6180X sensor
	vl6Ptr: The device to be initialized
	address: The I2C slave address to assign to the device
*/
uint8_t vl6180x_init(vl6180x * vlPtr, uint8_t addr) {
	change_address(vlPtr, addr); // Change the ddevice address
	
	// Mandatory : private registers
	write_byte(vlPtr, 0x0207, 0x01);
	write_byte(vlPtr, 0x0208, 0x01);
	write_byte(vlPtr, 0x0096, 0x00);
	write_byte(vlPtr, 0x0097, 0xfd);
	write_byte(vlPtr, 0x00e3, 0x00);
	write_byte(vlPtr, 0x00e4, 0x04);
	write_byte(vlPtr, 0x00e5, 0x02);
	write_byte(vlPtr, 0x00e6, 0x01);
	write_byte(vlPtr, 0x00e7, 0x03);
	write_byte(vlPtr, 0x00f5, 0x02);
	write_byte(vlPtr, 0x00d9, 0x05);
	write_byte(vlPtr, 0x00db, 0xce);
	write_byte(vlPtr, 0x00dc, 0x03);
	write_byte(vlPtr, 0x00dd, 0xf8);
	write_byte(vlPtr, 0x009f, 0x00);
	write_byte(vlPtr, 0x00a3, 0x3c);
	write_byte(vlPtr, 0x00b7, 0x00);
	write_byte(vlPtr, 0x00bb, 0x3c);
	write_byte(vlPtr, 0x00b2, 0x09);
	write_byte(vlPtr, 0x00ca, 0x09);
	write_byte(vlPtr, 0x0198, 0x01);
	write_byte(vlPtr, 0x01b0, 0x17);
	write_byte(vlPtr, 0x01ad, 0x00);
	write_byte(vlPtr, 0x00ff, 0x05);
	write_byte(vlPtr, 0x0100, 0x05);
	write_byte(vlPtr, 0x0199, 0x05);
	write_byte(vlPtr, 0x01a6, 0x1b);
	write_byte(vlPtr, 0x01ac, 0x3e);
	write_byte(vlPtr, 0x01a7, 0x1f);
	write_byte(vlPtr, 0x0030, 0x00);
	
	//Recommended : Public registers - See data sheet for more detail
	write_byte(vlPtr, 0x0011, 0x10); // Enables polling for ?New Sample ready? when measurement completes
	write_byte(vlPtr, 0x010a, 0x30); // Set the averaging sample period (compromise between lower noise and increased execution time)
	write_byte(vlPtr, 0x003f, 0x46); // Sets the light and dark gain (upper nibble). Dark gain should not be changed.
	write_byte(vlPtr, 0x0031, 0xFF); // sets the # of range measurements after which auto calibration of system is performed
	write_byte(vlPtr, 0x0040, 0x32); // Set ALS integration time to 50ms
	write_byte(vlPtr, 0x002e, 0x01); // perform a single temperature calibration of the ranging sensor  
	write_byte(vlPtr, 0x003e, 0x09); // Set default ALS inter-measurement period to 100ms
	write_byte(vlPtr, 0x0014, 0x24); // Configures interrupt on ?New Sample Ready threshold event?
	#ifdef RANGE_ONLY
			write_byte(vlPtr, 0x001b, 0x02); // Set default ranging inter-measurement period to 30ms
			write_byte(vlPtr, 0x001c, 0x14); // Set Max Range convergence time to 20ms
	#else
			write_byte(vlPtr, 0x001b, 0x09); // Set default ranging inter-measurement period to 100ms
			write_byte(vlPtr, 0x001c, 0x1e); // Set Max Range convergence time to 30ms
			write_byte(vlPtr, 0x0120, 0x0F); // Scale als result by 15
			write_byte(vlPtr, 0x02A3, 0x01); // Enable interleaved mode
	#endif

	write_byte(vlPtr, 0x003f, 0x47); // Set the ALS analog gain to 40
	write_byte(vlPtr, 0x002d, 0x13); // Enable range ignore
	write_byte(vlPtr, 0x0025, 0xFF); // Set range ignore height to 255
	
	return vlPtr->addr;
}

/*
	Change the I2C slave address of the device
	vlPtr: The device whose address should be changed
	new_addr: The desired I2C address
*/
void change_address(vl6180x * vlPtr, uint8_t new_addr) {
	uint8_t data[1];
	vlPtr->addr = VL_DEF_SLAVE_ADDR;
	write_byte(vlPtr, 0x0212, new_addr >> 1); // Write new address using default slave address
	read_bytes(new_addr, 0x0212, data, 1); // Read the address back
	vlPtr->addr = (data[0]) << 1; // Set the received address and save it in the structure
}

/*
	Toggle measurement mode on and off
	vl6Ptr: The device whose measurement should be toggled
*/
void toggle_measurement(vl6180x * vlPtr) {

	#ifdef RANGE_ONLY
    	write_byte(vlPtr, 0x0018, 0x03); // Write 0x03 to SYSRANGE_START (0x18) for continuous measurements
	#else
		write_byte(vlPtr, 0x0038, 0x03);
	#endif
}

/*
Initialize the GPIO expander
*/
void pca_setup(pca * expander) {
	write_byte_pca(expander->addr, 0x03, 0x00); // Set all pins to be outputs
	write_byte_pca(expander->addr, 0x01, 0x00); // Pull all pins low
}

/*
Set the pin connected to the passed VL6180X device to be high
vlPtr: The device that should be brought out of reset
*/
void pca_set_pin(vl6180x * vlPtr) {
	vlPtr->expander->cur_device_output |= 1 << vlPtr->expander_id;
	write_byte_pca(vlPtr->expander->addr, 0x01, vlPtr->expander->cur_device_output);
}

/*
Set the pin connected to the passed VL6180X device to be low
vlPtr: The device that should be brought out of reset
*/
void pca_reset_pin(vl6180x * vlPtr) {
	vlPtr->expander->cur_device_output &= ~(1 << vlPtr->expander_id);
	write_byte_pca(vlPtr->expander->addr, 0x01, vlPtr->expander->cur_device_output);
}

void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
