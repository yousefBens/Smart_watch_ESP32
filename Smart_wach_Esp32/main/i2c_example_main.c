/* File    : 17-2_lvgl_ME_task.c
 * date   : 11/03/2024
 * author : NR
 * Status : OK
 *
 * Actions :
 *           - When the BootBtn is pressed  (detected from GPIO IRQ)
 *              + the task "blink_Led" is resumed or suspended
 *              + the widget (text block) of page 0 is modified
 *               (Hello World -> Cornebidouille <-> Tarabistouille)
 *
 *           - Each 2 seconds (timer IRq), the current page to
 *           		   display is changed (detected from Timer IRQ)
 *           	                page 0-> 1 -> 2 -> 0 ...
 *
 *           - The display is refreshed every 500 ms using a Timer
 *                interrupt also
 *
 * Comments : provide the skeleton of any real-time application with
 *            GUI (lvgl here) : interaction by interrupts (GPIO,
 *            timers, ...) and a single call to refresh the HMI (so
 *            lv_task_handler() in LVGL case).
 *
 *            Updating the widgets in the corresponding tasks/fct
 *            allow to obtain a more dynamic interface. The required
 *            time for displaying the page is fixed and short.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "sdkconfig.h"

#include "lsm6dso_reg.h"
#include "driver/i2c.h"
#include "LSM6DO_capteur.h"
#include "LIS2MDL_Capteur.h"

#include "ble_init.h"
#include "gatt_server_profile.h"
#include "gatt_ser_gatt.h"
#include "WIFI_SNTP_RTC_SYNC.h"
#include "Gestion_affichage.h"


//******************************************************************
//**************************      1. USER INCLUDES     *************
//******************************************************************
/* Additional peripherals and t"lsm6dsoools */
#include "driver/gptimer.h"  // replace driver/timer.h (since 2023)
#include "esp_log.h"		//  for debug fct :ESP_LOGI/W/E/D/V( )

/* Littlevgl specific */
#include "lvgl.h"
#include "lvgl_helpers.h"

/* Pull Test  */


//******************************************************************
//**************************      2. DEFINES      ******************
//******************************************************************
#define NB_MAX_SCREEN 6		// number of different pages for the HMI

#define LV_TICK_PERIOD_MS 1

// Used GPIO port
#define BootBtn 0 // Boot Button :Take Care : same GPIO port than
				  //     the Green Led

#define RedLed 0
#define GreenLed 2
#define BlueLed 4	// To select for applications

#define QUEUE_LENGTH 5
#define ITEM_SIZE sizeof(float)
#define ITEM_SIZE1 sizeof(struct ValueToSend_Task)

//******************************************************************
//***************************    3.GLOBAL VARIABLES    *************
//******************************************************************
TaskHandle_t  blink_task_Handle    = NULL;
TaskHandle_t  state_machine_Handle = NULL;
TaskHandle_t  whoima_LSM6DO_task_Handle    = NULL;
TaskHandle_t  whoima_LIS2MDL_task_Handle    = NULL;
TaskHandle_t  BLE_task_Handle    = NULL;
TaskHandle_t Lecture_Horloge_Universelle_Handle = NULL;
TaskHandle_t Affichage_LCD_Handle = NULL;
lv_obj_t *tableau_mesures;
//extern TaskHandle_t whoami_task_Handle;

QueueHandle_t xQueue_T;
QueueHandle_t xQueue_Acc;
QueueHandle_t xQueue_Ang;
QueueHandle_t xQueue_Mag;


extern int a;

struct ValueToSend_Task {
  float x;
  float y;
  float z;
};


char temp_tab[20];
char Temperature_buf[20];
char Acc_buf[100];
char Ang_buf[100];
char Mag_buf[100];

SemaphoreHandle_t Btn_Semaphore        = NULL; // IRQ Button
SemaphoreHandle_t Acq_Timer_Semaphore  = NULL; // IRQ Timer Acq
SemaphoreHandle_t Aff_update_Semaphore = NULL; // IRQ Timer Display
SemaphoreHandle_t WIFI_RTC_SNTP_SYNC_Semaphore = NULL; // IRQ RTC SYNC
SemaphoreHandle_t screen_number_mutex;


//SemaphoreHandle_t Ac_semaphore;

QueueSetHandle_t Semaphore_QueueSet = NULL;  // The "queue set"  is
		// merging   the different semaphores for the state machine


// NB : VERSION LVGL UTILISEE : 7.11

// LVGL widgets management
lv_obj_t *screen[NB_MAX_SCREEN]; // table of the 3 LVGL screens
lv_obj_t *scr1_label1,			 // Text Object Label
		 *scr2_label1,			 // Text Object Label
		 *scr3_label1;			 // Text Object Label
lv_obj_t *tableau_mesures;
lv_obj_t *Graphe_Temperature;
lv_obj_t *Graphe_Magnetometrie;
lv_obj_t *Graphe_Gyroscopie;
lv_obj_t *Graphe_Accelerometrie;
lv_chart_series_t *temp_series, *Mag_series_x, *Mag_series_y, *Mag_series_z;
lv_chart_series_t *Ang_series_x, *Ang_series_y, *Ang_series_z;
lv_chart_series_t *Acc_series_x, *Acc_series_y, *Acc_series_z;
lv_style_t style_custom;
lv_style_t style_txt;
lv_style_t style_screen_black;
lv_style_t style_label_tab_mesures;
lv_style_t style_label_tableau_mesures;
lv_style_t style_chart;
lv_style_t graph_gyr;
lv_style_t style_label_graphs;
lv_style_t style_chart_graphs;
lv_style_t style_time;
uint8_t screen_number = 0;
extern char temperature_str[60];
QueueHandle_t temperatureQueue;



static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;

    conf.clk_flags = 0;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE; // Pullup resistors are already present on X-NUCLEO-IKS01A3
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


//******************************************************************
//***************************    4. IRQ Functions     **************
//******************************************************************
// All the interrupt Routines are sending a binary semaphore
// The semaphores are merged in a Queue for a single wait function
// in the state-machine (the simpler way to do it).
// So each routine uses is own semaphores to allow the detection
// of the sending routine.

//==================================================================
// gpio_IRQ_handler() : wake-up after BootButton was pressed
//==================================================================
static void IRAM_ATTR gpio_IRQ_handler(void *args)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// The return BootBtn will be (args = 0) if the button is pushed
	xSemaphoreGiveFromISR( Btn_Semaphore, &xHigherPriorityTaskWoken);
}

//==================================================================
// Acq_timer_IRQ_handler() : wake-up after 2s (Timer interrupt)
//==================================================================
static bool IRAM_ATTR Acq_timer_IRQ_handler(gptimer_handle_t timer,
		    const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
	// SM task has a higher priority, so no necessity to wake up it
	// with an higher priority
	static BaseType_t  xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( Acq_Timer_Semaphore,
			               &xHigherPriorityTaskWoken );
	return true;
}

//==================================================================
// Display_timer_IRQ_handler() : wake-up after 500ms (Timer IRQ)
//==================================================================
static bool IRAM_ATTR Display_timer_IRQ_handler(gptimer_handle_t timer,
		    const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
	// SM task has a higher priority, so no necessity to wake up it
	// with an higher priority
	static BaseType_t  xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR( Aff_update_Semaphore,
						   &xHigherPriorityTaskWoken );
	return true;
}
//==================================================================
// WIFI_RTC_SNTP_SYNC_IRQ_handler() : RTC clock sync every period
//==================================================================
static bool IRAM_ATTR WIFI_RTC_SNTP_SYNC_IRQ_Handler(gptimer_handle_t timer,
		    const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
	// SM task has a higher priority, so no necessity to wake up it
	// with an higher priority
	static BaseType_t  xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR( WIFI_RTC_SNTP_SYNC_Semaphore,
			               &xHigherPriorityTaskWoken );
	return true;
}
//==================================================================
// lv_tick_task() : wake-up after 1ms (Timer interrupt)
//==================================================================
static void lv_tick_task(void *arg)
{
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

//******************************************************************
//***************************************    5. Functions     ******
//******************************************************************
//==================================================================
// Init_Acq_Timer( period in µs ): configure and install the IRQ
// Timer for Acquisition (change msg)
//==================================================================
// Must be runned after running the tasks, who are waiting the IRQ

void Init_Acq_Timer(uint64_t NR_delay)
{
	gptimer_handle_t NR_timer_handle = NULL;  // handle of the timer

	//..............................................................
	// Part 1 : configure the data structure
	//..............................................................
	gptimer_config_t NR_timer_config = {	// timer config struct
	    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
	    .direction = GPTIMER_COUNT_UP,
	    .resolution_hz = 1000 * 1000, //Timer frequency => Fe =1MHz,
									  // so 1 tick = 1micro second
	};
	// create the new timer
	ESP_ERROR_CHECK(gptimer_new_timer(&NR_timer_config,
			                          &NR_timer_handle));

	//..............................................................
	// Part 2 : configure the alarm action
	//..............................................................
	gptimer_alarm_config_t NR_alarm_config ={
			.alarm_count = NR_delay,// alarm every NR_delay * 1 ms
			.reload_count = 0,      // for reload: restart to 0
			.flags.auto_reload_on_alarm  = true,
	};
	ESP_ERROR_CHECK(gptimer_set_alarm_action(NR_timer_handle,
			        &NR_alarm_config));

	//..............................................................
	// Part 3: configure the callback (interruption)
	//..............................................................
	gptimer_event_callbacks_t NR_cbs_config = {
	        .on_alarm = Acq_timer_IRQ_handler, //call the right fct!
	    };
	    ESP_ERROR_CHECK(gptimer_register_event_callbacks(NR_timer_handle,
	    		        &NR_cbs_config, NULL));

	//..............................................................
	// Part 4 : Start the timer
	//..............................................................
	ESP_ERROR_CHECK(gptimer_enable(NR_timer_handle));
	ESP_ERROR_CHECK(gptimer_start(NR_timer_handle));

} // end of void NR_Init_timer()
//==================================================================

//==================================================================
// Init_Display_Timer( period in µs ):
//          configure and install the IRQ Timer for display update
//			copy/paste from the previous function, could be improved
//==================================================================
// Must be runned after running the tasks, who are waiting the IRQ,
// so the semaphores
// Exactly the same function than the previous one, just changing
// the period and callback fct.
void Init_Display_Timer(uint64_t NR_delay)
{
	gptimer_handle_t NR_timer_handle = NULL;  // handle of the timer

	//..............................................................
	// Part 1 : configure the data structure
	//..............................................................
	gptimer_config_t NR_timer_config = {	// Timer config struct
	    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
	    .direction = GPTIMER_COUNT_UP,
	    .resolution_hz = 1000 * 1000, // Timer freq ==> Fe =1MHz,
									  // so 1 tick = 1micro second
	};
	// create the new timer
	ESP_ERROR_CHECK(gptimer_new_timer(&NR_timer_config,
			                                     &NR_timer_handle));

	//..............................................................
	// Part 2 : configure the alarm action
	//..............................................................
	gptimer_alarm_config_t NR_alarm_config ={
			.alarm_count = NR_delay,// alarm every NR_delay * 1 ms
			.reload_count = 0,      // for reload: restart to 0
			.flags.auto_reload_on_alarm  = true,
	};
	ESP_ERROR_CHECK(gptimer_set_alarm_action(NR_timer_handle,
			                                     &NR_alarm_config));

	//..............................................................
	// Part 3: configure the callback (interruption)
	//..............................................................
	gptimer_event_callbacks_t NR_cbs_config = {
	        .on_alarm = Display_timer_IRQ_handler,//call right fct !
	    };
	    ESP_ERROR_CHECK(gptimer_register_event_callbacks(NR_timer_handle,
	    		                             &NR_cbs_config, NULL));

	//..............................................................
	// Part 4 : Start the timer
	//..............................................................
	ESP_ERROR_CHECK(gptimer_enable(NR_timer_handle));
	ESP_ERROR_CHECK(gptimer_start(NR_timer_handle));

} // end of void NR_Init_timer()
//==================================================================
// Init_RTC_SYNC_Timer( period in µs ): configure and install the IRQ
// Timer for SNTP RTC Sync
//==================================================================
// Must be runned after running the tasks, who are waiting the IRQ
void Init_RTC_SYNC_Timer(uint64_t NR_delay)
{
	gptimer_handle_t NR_timer_handle = NULL;  // handle of the timer
	//..............................................................
	// Part 1 : configure the data structure
	//..............................................................
	gptimer_config_t NR_timer_config = {	// timer config struct
	    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
	    .direction = GPTIMER_COUNT_UP,
	    .resolution_hz = 1000 * 1000, //Timer frequency => Fe =1MHz,
									  // so 1 tick = 1micro second
	};
	// create the new timer
	ESP_ERROR_CHECK(gptimer_new_timer(&NR_timer_config,
			                          &NR_timer_handle));
	//..............................................................
	// Part 2 : configure the alarm action
	//..............................................................
	gptimer_alarm_config_t NR_alarm_config ={
			.alarm_count = NR_delay,// alarm every NR_delay * 1 ms
			.reload_count = 0,      // for reload: restart to 0
			.flags.auto_reload_on_alarm  = true,
	};
	ESP_ERROR_CHECK(gptimer_set_alarm_action(NR_timer_handle,
			        &NR_alarm_config));
	//..............................................................
	// Part 3: configure the callback (interruption)
	//..............................................................
	gptimer_event_callbacks_t NR_cbs_config = {
	        .on_alarm = WIFI_RTC_SNTP_SYNC_IRQ_Handler, //call the right fct!
	    };
	    ESP_ERROR_CHECK(gptimer_register_event_callbacks(NR_timer_handle,
	    		        &NR_cbs_config, NULL));
	//..............................................................
	// Part 4 : Start the timer
	//..............................................................
	ESP_ERROR_CHECK(gptimer_enable(NR_timer_handle));
	ESP_ERROR_CHECK(gptimer_start(NR_timer_handle));
} // end of void NR_Init_timer()
//==================================================================

//==================================================================
//  create_GUI_widgets() :
//create all the graphic objects required by the interface
//used labels and dynamical elements must be shared (global variable
//                                                         or other)
//3 screens are constructed (only one is downloaded at a given time)
//==================================================================
static void create_GUI_widgets(void)
{
	//The following objects must be created as global variables
	// and updated in the state-machine.
	// lv_obj_t screen[3];
	// lv_obj_t *scr1_label1,
	//			*scr2_label1,
	//			*scr3_label1;

		//--------------------------------------------------------------
	//                      Screen 1 : display horodate
	//--------------------------------------------------------------
	/* Create the Screen 1 */

			init_screen_1_Affichage_Horodate();
	//--------------------------------------------------------------
	//                      Screen 2 : data
	//--------------------------------------------------------------
			init_screen_2_tableau_mesures();
	//--------------------------------------------------------------
	//                      Screen 3 : Graphe de temperature
	//--------------------------------------------------------------
			init_screen_3_graphe_temperature();
	//--------------------------------------------------------------
	//                      Screen 4 : Graphe de Magnetometrie
	//--------------------------------------------------------------
			init_screen_4_graphe_magnetometre();
	//--------------------------------------------------------------
	//                      Screen 5 : Graphe de Gyroscopie
	//--------------------------------------------------------------
			init_screen_5_graphe_gyroscope();
	//--------------------------------------------------------------
	//                      Screen 6 : Graphe d'Accelerometrie
	//--------------------------------------------------------------
			init_screen_6_graphe_accelerometre();
	//--------------------------------------------------------------
	//              For the initialisation : load Screen 0
	//--------------------------------------------------------------
	lv_scr_load(screen[0]);          /*Load screen on the display */
	lv_style_set_text_font(&style_custom, LV_STATE_DEFAULT, LV_FONT_MONTSERRAT_12);
}



//==================================================================

//==================================================================
// init_lvgl() : initialize the display and the LVGL interface,
//				 run the timer for LVGL tick management
//==================================================================
 void init_lvgl(void)
 {
	 printf("init lvgl init\r\n");
	 lv_init();

	 /* Initialize SPI or I2C bus used by the drivers */
	 lvgl_driver_init();

	 static lv_color_t buf1[DISP_BUF_SIZE];

 /* Use double buffered when not working with monochrome displays */
	 static lv_color_t *buf2 = NULL;

	 static lv_disp_buf_t disp_buf;

	 uint32_t size_in_px = DISP_BUF_SIZE;

 /* Initialize the working buffer depending on the selected display.
 * NOTE: buf2 == NULL when using monochrome displays. */
	 lv_disp_buf_init(&disp_buf, buf1, buf2, size_in_px);

	 lv_disp_drv_t disp_drv;
	 lv_disp_drv_init(&disp_drv);
	 disp_drv.flush_cb = disp_driver_flush;

	 disp_drv.buffer = &disp_buf;
	 lv_disp_drv_register(&disp_drv);

 /* Create and start a periodic timer interrupt to call lv_tick_inc */
	 const esp_timer_create_args_t periodic_timer_args = {
		 .callback = &lv_tick_task,
		 .name = "periodic_gui"
	 };
	 esp_timer_handle_t periodic_timer;
	 ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args,
			                          &periodic_timer));
	 ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer,
			                             LV_TICK_PERIOD_MS * 1000));

 }

//******************************************************************
//***************************     Task Functions     ***************
//******************************************************************

//==================================================================
// blink_task() : toggle the Blue Led in an infinite loop
//==================================================================
void blink_task(void *arg)
{
   // Or convergence point
    while(1)
    {
        // Step B2 : Led On
        gpio_set_level(BlueLed, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // Step B3 : Led Off
        gpio_set_level(BlueLed, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    } // branch to while(1)
}



//==================================================================
// state_machine() :
//    manage the lvgl display refresh,
//    and the different tasks (no sleep mode in this version)
//==================================================================
void LSM6DO_task(void *args){
	//xSemaphoreTake(Ac_semaphore,  0 );
	//while(1){
		whoami_LSM6DO_task();
	//}

	//xSemaphoreGive(Ac_semaphore);
}

void BLE_task(){

   // for (;;) {

printf("********** Bluetooth ON !!!! *********\n********** Bluetooth ON !!!! *********\n********** Bluetooth ON !!!! *********\n");

        esp_ble_gap_register_callback(gap_event_handler);



        esp_ble_gatts_register_callback(gatts_event_handler);

        esp_ble_gatts_app_register(PROFILE_APP_ID);
  // }

}


void state_machine()
{
	uint8_t state_blink_task = 1;
	float receivedValue_T;
	struct ValueToSend_Task receivedValue_Acc;
	struct ValueToSend_Task receivedValue_Ang;
	struct ValueToSend_Task receivedValue_Mag;
	char *TAG ="State_machine Task";
	char bufferr[50] = "Initial T";
	char bufferrAcc[80]= "Initial Acc";
	char bufferrAng[80]= "Initial Ang";
	char bufferrMag[80]= "Initial Mag";

	// Local "Queue set" merging the wait of binary semaphores
	//   coming from the different IRQs
	QueueSetMemberHandle_t received_semaphore;

	//-------------------------- Step sm1: start Blinking Led ------
	vTaskResume(blink_task_Handle);

	//--------------------- Infinite loop waiting for semaphore ----
	// recall: Screen1 is downloaded by the Create_GUI_widgets() fct

	while(1)
	{



		//---- Step sm2 (wait for a semaphore from the queue set) --
		//           this following line justifies the RTOS use
		// (imagine how to proceed without Queue, considering random
		//                                   order of coming events)
		received_semaphore = xQueueSelectFromSet(Semaphore_QueueSet,
				                                     portMAX_DELAY);

		//==========================================================
		//  Or divergence (ordered selection of sequences)
		//==========================================================

		//----------- Sequence 1: Button was pushed ----------------
		// No periodicity : event
		if( received_semaphore == Btn_Semaphore  )
		{	// Step 1.1
			// the token must be removed from the semaphore
			xSemaphoreTake(Btn_Semaphore, 0);

			// Step 1.2
			// Resume or Suspend the Blink Task
			// 					and modify the associated widget
			state_blink_task = !state_blink_task;
			ESP_LOGE(TAG, "ME: Button was pushed, blink_state :%s",
						( (state_blink_task==1 ) ? "off" : "on") );
			if(xSemaphoreTake(screen_number_mutex, portMAX_DELAY) == pdTRUE) {
			    screen_number = ( screen_number < (NB_MAX_SCREEN-1) ? screen_number+1 : 0);
			    xSemaphoreGive(screen_number_mutex);
			}
			if (state_blink_task == 1)
			{
				vTaskResume(blink_task_Handle);

			}
			else
			{
				vTaskSuspend(blink_task_Handle);
			}
			lv_task_handler();
		}
		//------ Sequence 2 : Timer semaphore is arrived -----------
		//             Periodicity : every 2 seconds
		else if (received_semaphore == Acq_Timer_Semaphore)
		{   // Step 2.1
			// the token must be removed from the semaphore
			xSemaphoreTake(Acq_Timer_Semaphore, 0);
			ESP_LOGW(TAG, "ME: Acquisition Timer semaphore");
			vTaskResume(whoima_LSM6DO_task_Handle);
			vTaskResume(whoima_LIS2MDL_task_Handle );


			if (xQueueReceive(xQueue_T, &receivedValue_T, portMAX_DELAY)) { // Attend indéfiniment jusqu'à ce qu'une valeur soit disponible
				            sprintf(bufferr, "Temperature [degC]:%6.2f", receivedValue_T);
				        }

			else if (xQueueReceive(xQueue_Acc, &receivedValue_Acc, portMAX_DELAY)) { // Attend indéfiniment jusqu'à ce qu'une valeur soit disponible
				            sprintf(bufferrAcc, "Acceleration [mg]:\r\n%4.2f, %4.2f, %4.2f",
				            		receivedValue_Acc.x, receivedValue_Acc.y, receivedValue_Acc.z);
			}

			else if (xQueueReceive(xQueue_Ang, &receivedValue_Ang, portMAX_DELAY)) { // Attend indéfiniment jusqu'à ce qu'une valeur soit disponible
				            sprintf(bufferrAng, "Angular rate [mdps]:\r\n%4.2f, %4.2f, %4.2f",
				            		receivedValue_Ang.x, receivedValue_Ang.y, receivedValue_Ang.z);
			}
			else if (xQueueReceive(xQueue_Mag, &receivedValue_Mag, portMAX_DELAY)) { // Attend indéfiniment jusqu'à ce qu'une valeur soit disponible
				            sprintf(bufferrMag, "Magnetic field [mG]:\r\n%4.2f, %4.2f, %4.2f",
				            		receivedValue_Mag.x, receivedValue_Mag.y, receivedValue_Mag.z);
			}
		}
		//------------ Sequence 3: Display Update ---------------
		//               Periodicity : every 500 ms
		else if (received_semaphore == Aff_update_Semaphore)
		{   // Step 3.1
			// the token must be removed from the semaphore
			xSemaphoreTake(Aff_update_Semaphore, 0);
			ESP_LOGI(TAG, "ME: Display_update");
			vTaskResume(Affichage_LCD_Handle);
			//vTaskResume(BLE_task_Handle );
			//vTaskSuspend( whoima_task_Handle );

			// Step 3.2 : whatever the duration of the following
			//            function, the state-machine will end it
			//            before to continue.
			lv_task_handler();
		}
		//------------ Sequence 4: Periodic RTC SYNC ---------------
				//               Periodicity : every 24 h
		else if (received_semaphore == WIFI_RTC_SNTP_SYNC_Semaphore)
						{
							ESP_LOGW(TAG, "ME: RTC SYNC Timer semaphore");
							xSemaphoreTake(WIFI_RTC_SNTP_SYNC_Semaphore, 0);
							vTaskResume(Lecture_Horloge_Universelle_Handle);
						}
		// else:no necessity, the ME go back in 'wait for semaphore'

	}
}

//##################################################################
//						app_main() function
//##################################################################
void app_main(void)
{
	a = a+1;
	char * TAG="App_main() :";
	ESP_LOGI(TAG, "Start App_main");
	//==============================================================
	// Step 1. Initialisation
	//==============================================================
	// Step 1.1 Peripherals initialization
	gpio_reset_pin(BootBtn);
	gpio_set_direction(BootBtn, GPIO_MODE_INPUT);
	gpio_set_intr_type(BootBtn, GPIO_INTR_NEGEDGE);	//default lvl= 1

	gpio_reset_pin(BlueLed);
	gpio_set_direction(BlueLed, GPIO_MODE_OUTPUT);
	gpio_set_level(BlueLed, 0);

	// Step 1.2 : Semaphore and Queue Set creation
	//            (The queue set merge the different semaphores)

	Btn_Semaphore        = xSemaphoreCreateBinary();
	Acq_Timer_Semaphore  = xSemaphoreCreateBinary();
	Aff_update_Semaphore = xSemaphoreCreateBinary();
	WIFI_RTC_SNTP_SYNC_Semaphore = xSemaphoreCreateBinary();
	screen_number_mutex = xSemaphoreCreateMutex();
	Semaphore_QueueSet = xQueueCreateSet( 4 );
	xQueueAddToSet( Btn_Semaphore,        Semaphore_QueueSet );
	xQueueAddToSet( Acq_Timer_Semaphore,  Semaphore_QueueSet );
	xQueueAddToSet( Aff_update_Semaphore, Semaphore_QueueSet );
	xQueueAddToSet( WIFI_RTC_SNTP_SYNC_Semaphore, Semaphore_QueueSet );
	xQueue_T = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
	xQueue_Acc = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE1);
	xQueue_Ang = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE1);
	xQueue_Mag = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE1);

	// Step 1.3 : LVGL initialisation

	init_lvgl();
	create_GUI_widgets();
	i2c_master_init();
	ble_init();

	//==============================================================
	// Step 2. Tasks installation
	//==============================================================
	// 2.1 Install the Blink task and Suspend it
	xTaskCreate(blink_task, "blink_task", 10000, NULL, 1,
			                                    &blink_task_Handle);
	vTaskSuspend( blink_task_Handle );

	xTaskCreate(whoami_LSM6DO_task, "LSM6DO_TASK", 10000, NULL, 2, &whoima_LSM6DO_task_Handle);

	vTaskSuspend( whoima_LSM6DO_task_Handle );

	xTaskCreate(whoami_LIS2MDL_task, "LIS2MDL_TASK", 10000, NULL, 2, &whoima_LIS2MDL_task_Handle);

	vTaskSuspend( whoima_LIS2MDL_task_Handle );

	/* RTC Update task should keep the highest priority just after the state machine */
	xTaskCreate(Lecture_Horloge_Universelle, "Update_RTC", 4096, NULL, 5, &Lecture_Horloge_Universelle_Handle);
	/* No need for placing here the vTaskSuspend for Lecture_Horloge_Universelle it's already
	  done inside the task, so therefore it's going to be suspended just after the first RTC UPDATE */

	xTaskCreate(state_machine, "state_machine", 10000, NULL, 6,
												 &state_machine_Handle);

	xTaskCreate(Affichage_LCD, "Affichage_LCD", 20000, NULL, 4,  &Affichage_LCD_Handle);

	vTaskSuspend(Affichage_LCD_Handle);


	//xTaskCreate(BLE_task, "BLE_TASK", 10000, NULL, 5, NULL);
	BLE_task();

	//vTaskSuspend( BLE_task_Handle );
	// 2.2 Install and run the State_Machine




	//==============================================================
	// Step 3. Interrupt Routines install, include Timers Init IRQ
	//==============================================================
	gpio_install_isr_service(0);
	gpio_isr_handler_add(BootBtn, gpio_IRQ_handler,(void *)BootBtn);

	Init_Acq_Timer(1000 * 1000);	// Acquisition period : 1 s
	Init_Display_Timer(100 * 1000); // Display update     : 100 ms
	Init_RTC_SYNC_Timer(1200 * 1000 * 1000); // RTC update  : 20 min

	//==============================================================
	// end of the app_main()
	//==============================================================
	//vTaskStartScheduler();

	ESP_LOGI(TAG, "App_main is finished");
}
