
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"

#include "app_timer.h"

#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "simple_ble.h"
#include "nrf_drv_saadc.h"
#include "transfer_handler.h"

#include "Cellwise CW201x Driver for MCU.h"



#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


extern STRUCT_CW_BATTERY   cw_bat;
int a=0;
int b=0;
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
//static void timers_init(void)
//{
//    ret_code_t err_code = app_timer_init();
//    APP_ERROR_CHECK(err_code);
//}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_evt_t * p_evt)
{

    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {

        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

    }

}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
}

void saadc_init(void)
{
    ret_code_t err_code;
	
    nrf_saadc_channel_config_t channel_0_config =
    NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_0_config);
    APP_ERROR_CHECK(err_code);

}

void gpio_init(void)
{
	nrf_gpio_cfg_input(MCU_KEY, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_output(MCU_EN);
	nrf_gpio_cfg_output(LED1);
	nrf_gpio_cfg_output(LED2);
	nrf_gpio_cfg_output(LED3);
	nrf_gpio_cfg_output(LED4);
	nrf_gpio_cfg_output(LED4);
}


static void m_sample_timer_handler(void *p_context)
{

//		cw_bat_work();
//		a = cw_bat.capacity;
//		b = cw_bat.voltage;

}
/**@brief Application main function.
 */
int main(void)
{
	bool erase_bonds;

	unsigned char reg_val = 0;

//	unsigned char vcell[2] = {0x02, 0x03};
	// Initialize.
	log_init();
	lfclk_config();
	buttons_leds_init(&erase_bonds);
	gpio_init();
	
	nrf_gpio_pin_set(MCU_EN);
	
	power_management_init();
	sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
	timer_init(m_sample_timer_handler);
	timers_start(5000);
	iic_init();
	reg_val = cw_bat_init();
//	nrf_delay_ms(3);
//	reg_val = MODE_NORMAL;
//	cw_write(REG_MODE, &reg_val);
//	cw_read(REG_VERSION, &reg_val);
	Debug("%x",reg_val);
//	cw_read(REG_VCELL, &vcell[0]);
//	cw_read(REG_VCELL + 1, &vcell[1]);
//	Debug("%x,%x",vcell[0],vcell[1]);
	simple_ble_init();

	// Start execution.
	advertising_start();
	NRF_LOG_INFO("BLE Template Init.");

	// Enter main loop.
	for (;;)
	{
		idle_state_handle();
		cw_bat_work();
		a = cw_bat.capacity;
		b = cw_bat.voltage;
		Debug("%d,%d\r\n",a,b);
		nrf_delay_ms(5000);
//		show_number(cw_bat.capacity);
//		show_number(cw_bat.voltage);
		
	}
}


/**
 * @}
 */
