#include "stm32f1xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

#include "gnss.h"
#include "gnss_fusion.h"
#include "gnss_uart.h"
#include "pps.h"
#include "spi_fusion.h"

void Rtt_Init(void);
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);

extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

static void LedTask(void *argument)
{
	(void)argument;
	while (1)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		vTaskDelay(pdMS_TO_TICKS(250));
	}
}

static const char *FusionStatusToString(GnssFusionStatus status)
{
	switch (status)
	{
	case GNSS_FUSION_NO_FIX:
		return "NO_FIX";
	case GNSS_FUSION_OK:
		return "OK";
	case GNSS_FUSION_DEGRADED:
		return "DEGRADED";
	case GNSS_FUSION_INTERFERENCE:
		return "INTERFERENCE";
	default:
		return "?";
	}
}

static void PrintCoordE7(const char *name, int32_t value_e7)
{
	int64_t v = (int64_t)value_e7;
	char sign = '+';
	if (v < 0)
	{
		sign = '-';
		v = -v;
	}

	int32_t deg = (int32_t)(v / 10000000);
	int32_t frac = (int32_t)(v % 10000000);
	printf("%s=%c%ld.%07ld", name, sign, (long)deg, (long)frac);
}

/* PCB reference of the ATGM336H wired to each firmware module index (1..8). */
static const char *const kGnssBoardRef[GNSS_MODULE_COUNT] = {
	"U1", "U4", "U6", "U8", "U10", "U12", "U14", "U16",
};

static void PrintDbhzX10(const char *name, uint16_t value_x10)
{
	printf("%s=%u.%u", name, (unsigned)(value_x10 / 10u), (unsigned)(value_x10 % 10u));
}

static const char *const kConstellationTag[GNSS_CONST_COUNT] = {"gps", "bds", "glo", "oth"};

/* Ends the module line: " cn0 top4=.. avg=.. max=.. trk=tracked/in_view gps=../.. bds=../.." */
static void PrintModuleCn0(const GnssModuleState *m, const GnssCn0Summary *s, uint32_t now)
{
	if (!s->valid)
	{
		printf(" cn0 n/a\r\n");
		return;
	}
	printf(" cn0 ");
	PrintDbhzX10("top4", s->top4_x10);
	printf(" ");
	PrintDbhzX10("avg", s->mean_x10);
	printf(" max=%u trk=%u/%u", (unsigned)s->max_dbhz, (unsigned)s->tracked, (unsigned)s->in_view);
	for (size_t c = 0; c < GNSS_CONST_COUNT; c++)
	{
		const GnssCn0Constellation *cc = &m->cn0[c];
		if (cc->update_tick == 0 || (now - cc->update_tick) > GNSS_CN0_STALE_MS || cc->in_view == 0)
		{
			continue;
		}
		printf(" %s=%u/%u", kConstellationTag[c], (unsigned)cc->tracked, (unsigned)cc->in_view);
	}
	printf("\r\n");
}

/*
 * One line for the whole array: mean of the per-module top4 C/N0, and the weakest/strongest module.
 * Watch this line while switching drone subsystems on and off: a drop on every module at once is
 * interference reaching all antennas, a drop on one module only is local (antenna, cable, routing).
 */
static void PrintArrayCn0(const GnssCn0Summary *summaries)
{
	uint32_t sum = 0;
	unsigned count = 0;
	uint8_t min_i = 0, max_i = 0;
	for (uint8_t i = 0; i < GNSS_MODULE_COUNT; i++)
	{
		if (!summaries[i].valid)
		{
			continue;
		}
		if (count == 0 || summaries[i].top4_x10 < summaries[min_i].top4_x10)
		{
			min_i = i;
		}
		if (count == 0 || summaries[i].top4_x10 > summaries[max_i].top4_x10)
		{
			max_i = i;
		}
		sum += summaries[i].top4_x10;
		count++;
	}

	if (count == 0)
	{
		printf("[cn0 array] no GSV data\r\n");
		return;
	}
	printf("[cn0 array] modules=%u ", count);
	PrintDbhzX10("top4", (uint16_t)(sum / count));
	printf(" ");
	PrintDbhzX10("min", summaries[min_i].top4_x10);
	printf(" (#%u) ", (unsigned)(min_i + 1u));
	PrintDbhzX10("max", summaries[max_i].top4_x10);
	printf(" (#%u)\r\n", (unsigned)(max_i + 1u));
}

static void PrintModulesStatus(void)
{
	uint32_t now = HAL_GetTick();
	GnssCn0Summary cn0[GNSS_MODULE_COUNT] = {0};
	for (uint8_t i = 1; i <= GNSS_MODULE_COUNT; i++)
	{
		const GnssModuleState *m = Gnss_GetModule(i);
		if (m == NULL)
		{
			continue;
		}

		printf("[gnss #%u %-3s] ", (unsigned)i, kGnssBoardRef[i - 1]);
		if (m->last_rx_tick == 0)
		{
			printf("silent (err=%lu)\r\n", (unsigned long)m->nmea_checksum_errors);
			continue;
		}

		bool alive = (now - m->last_rx_tick) <= 2000u;
		printf("%s nmea=%lu err=%lu ",
		       alive ? "alive" : "LOST ",
		       (unsigned long)m->nmea_sentences,
		       (unsigned long)m->nmea_checksum_errors);
		if (m->has_fix && (now - m->last_fix_tick) <= 2000u)
		{
			printf("fix q=%u sats=%u hdop=%u.%02u ",
			       (unsigned)m->fix_quality,
			       (unsigned)m->satellites,
			       (unsigned)(m->hdop_centi / 100u),
			       (unsigned)(m->hdop_centi % 100u));
			PrintCoordE7("lat", m->lat_e7);
			printf(" ");
			PrintCoordE7("lon", m->lon_e7);
		}
		else
		{
			printf("no fix sats=%u", (unsigned)m->satellites);
		}
		Gnss_GetCn0Summary(m, now, &cn0[i - 1]);
		PrintModuleCn0(m, &cn0[i - 1], now);
	}
	PrintArrayCn0(cn0);
}

/* Local clock error comes out for free: the timer counts HSI microseconds between UTC seconds. */
static void PrintPps(void)
{
	PpsInfo p = {0};
	if (!Pps_GetLast(&p))
	{
		printf("[pps #6] none (edges=%lu)\r\n", (unsigned long)p.count);
		return;
	}
	printf("[pps #6] edges=%lu age=%lums",
	       (unsigned long)p.count,
	       (unsigned long)(HAL_GetTick() - p.last_tick));
	if (p.period_us != 0)
	{
		printf(" period=%luus clk=%+ldppm",
		       (unsigned long)p.period_us,
		       (long)((int32_t)p.period_us - 1000000));
	}
	printf("\r\n");
}

static void FusionPrintTask(void *argument)
{
	(void)argument;
	while (1)
	{
		PrintModulesStatus();
		PrintPps();

		GnssFusionResult r = {0};
		if (GnssFusion_GetResult(&r) && r.has_fix)
		{
			printf("[fusion %s used=%u rej=%u hdop=%u res=%ucm] ",
			       FusionStatusToString(r.status),
			       (unsigned)r.used_modules,
			       (unsigned)r.rejected_modules,
			       (unsigned)r.avg_hdop_centi,
			       (unsigned)r.max_residual_cm);
			PrintCoordE7("lat", r.lat_e7);
			printf(" ");
			PrintCoordE7("lon", r.lon_e7);
			/* newlib-nano printf has no %f: print centimetres as metres with integers. */
			int32_t alt_abs = (r.alt_cm < 0) ? -r.alt_cm : r.alt_cm;
			uint32_t t = r.epoch_time_ms;
			printf(" alt=%s%ld.%02ldm utc=%02lu:%02lu:%02lu.%03lu\r\n",
			       (r.alt_cm < 0) ? "-" : "",
			       (long)(alt_abs / 100),
			       (long)(alt_abs % 100),
			       (unsigned long)(t / 3600000u),
			       (unsigned long)((t / 60000u) % 60u),
			       (unsigned long)((t / 1000u) % 60u),
			       (unsigned long)(t % 1000u));
		}
		else
		{
			printf("[fusion %s] no fix\r\n", FusionStatusToString(r.status));
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void SVC_Handler(void)
{
	vPortSVCHandler();
}

void PendSV_Handler(void)
{
	xPortPendSVHandler();
}

void SysTick_Handler(void)
{
	HAL_IncTick();
	/*
	 * HAL_Init() starts SysTick before the scheduler exists. Ticking the kernel then walks
	 * uninitialised task lists (NULL) and hard-faults, so only forward once it is running.
	 */
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
	{
		xPortSysTickHandler();
	}
}

void USART1_IRQHandler(void)
{
	GnssUart_IrqHandler(USART1);
}

void USART2_IRQHandler(void)
{
	GnssUart_IrqHandler(USART2);
}

void USART3_IRQHandler(void)
{
	GnssUart_IrqHandler(USART3);
}

void SPI1_IRQHandler(void)
{
	SpiFusion_SpiIrqHandler();
}

void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void TIM2_IRQHandler(void)
{
	Pps_TimIrqHandler();
}

void TIM3_IRQHandler(void)
{
	GnssUart_TimIrqHandler();
}

int main(void)
{
	Rtt_Init();
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();

	Gnss_Init(9600);
	Pps_Init();
	GnssFusion_Init();
	SpiFusion_Init();

	xTaskCreate(Gnss_Task, "gnss", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(GnssFusion_Task, "fusion", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(FusionPrintTask, "print", 384, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(LedTask, "led", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
	vTaskStartScheduler();

	while (1)
	{
	}
}

static void SystemClock_Config(void)
{
	RCC_OscInitTypeDef oscillator_config = {0};
	RCC_ClkInitTypeDef clock_config = {0};

	/*
	 * HSI 8MHz / 2 * 16 -> 64MHz SYSCLK (max reachable without HSE).
	 * The board's HSE crystal (Y2) does not start (HSERDY never set), so HSE is not used.
	 */
	oscillator_config.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	oscillator_config.HSIState = RCC_HSI_ON;
	oscillator_config.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	oscillator_config.PLL.PLLState = RCC_PLL_ON;
	oscillator_config.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	oscillator_config.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&oscillator_config) != HAL_OK)
	{
		while (1)
		{
		}
	}

	clock_config.ClockType =
		RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clock_config.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clock_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clock_config.APB1CLKDivider = RCC_HCLK_DIV2;
	clock_config.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&clock_config, FLASH_LATENCY_2) != HAL_OK)
	{
		while (1)
		{
		}
	}
}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef gpio = {0};
	gpio.Pin = GPIO_PIN_8;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &gpio);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	while (1)
	{
	}
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
	(void)task;
	(void)task_name;
	taskDISABLE_INTERRUPTS();
	while (1)
	{
	}
}
