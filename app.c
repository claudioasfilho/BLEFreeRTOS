/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "em_common.h"
#include "sl_app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"

#include "sl_app_log.h"


// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;


/**************************************************************************//**
 * @brief  FreeRTOS Required Includes and definitions
 *****************************************************************************/


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define SL_BT_RTOS_APPLICATION_PRIORITY         10u

/**************************************************************************//**
 * @brief  Simple LED required includes
 *****************************************************************************/


#include "sl_led.h"
#include "sl_simple_led_instances.h"


/**************************************************************************//**
 * @brief  IADC Required Includes and definitions
 *****************************************************************************/

#include "em_iadc.h"
#include "em_cmu.h"
volatile IADC_Result_t sample;
volatile uint32_t millivolts;

// Set HFRCODLL clock to 80MHz
#define HFRCODPLL_FREQ            cmuHFRCODPLLFreq_80M0Hz

// Set CLK_ADC to 40MHz
#define CLK_SRC_ADC_FREQ          40000000 // CLK_SRC_ADC
#define CLK_ADC_FREQ              10000000 // CLK_ADC - 10MHz max in normal mode


// When changing GPIO port/pins above, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_BUS            CDBUSALLOC
#define IADC_INPUT_BUSALLOC       GPIO_CDBUSALLOC_CDEVEN0_ADC0

// IADC input GPIO port/pin configration
#define IADC_INPUT_POS            iadcPosInputPortCPin6
#define IADC_INPUT_NEG            iadcNegInputGnd

/**************************************************************************//**
 * @brief  Task and Queues Definitions
 *****************************************************************************/

//LED Task

#define LED_TASK_NAME          "led_task"
#define LED_TASK_STACK_SIZE    200
StackType_t led_task_stack[LED_TASK_STACK_SIZE];
//StaticTask_t led_task_handle;
TaskHandle_t led_task_handle;

//IADC Task

#define IADC_TASK_NAME          "led_task"
#define IADC_TASK_STACK_SIZE    200
StackType_t iadc_task_stack[IADC_TASK_STACK_SIZE];
StaticTask_t iadc_task_handle;




/**************************************************************************//**
 * @brief  Queues
 *****************************************************************************/

//This Queue will be used to trigger the LED once a measurement is done on the ADC

xQueueHandle ADC_to_LED_Queue_Handle = 0;



/**************************************************************************//**
 * @brief  IADC related function
 *****************************************************************************/

//This Function initializes the IADC to do a sincgle
void my_adc_init (void)
{
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

  // Enable IADC clock
  CMU_ClockEnable(cmuClock_IADC0, true);

  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);

  // Configure IADC clock source for use while in EM2
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_FSRCO);

  // Modify init structs and initialize
  init.warmup = iadcWarmupKeepWarm;

  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;

  // Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                    CLK_ADC_FREQ,
                                                                    0,
                                                                    iadcCfgModeNormal,
                                                                    init.srcClkPrescale);

  // Single initialization
  initSingle.dataValidLevel = _IADC_SINGLEFIFOCFG_DVL_VALID1;

  // Set conversions to run once
  initSingle.triggerAction = iadcTriggerActionOnce;

  // Configure Input sources for single ended conversion GPIO port C pin 2

  initSingleInput.posInput = iadcPosInputPortCPin2;
  initSingleInput.negInput = iadcNegInputGnd;

  // Initialize IADC
  IADC_init(IADC0, &init, &initAllConfigs);

  // Initialize Scan
  IADC_initSingle(IADC0, &initSingle, &initSingleInput);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_BUS |= IADC_INPUT_BUSALLOC;

}

void my_adc_start_measurement(void)
{

  IADC_command(IADC0, iadcCmdStartSingle);

}

int my_adc_measurement_get(void)
{


  // Wait for conversion to be complete
       while((IADC0->STATUS & (_IADC_STATUS_CONVERTING_MASK
                   | _IADC_STATUS_SINGLEFIFODV_MASK)) != IADC_STATUS_SINGLEFIFODV); //while combined status bits 8 & 6 don't equal 1 and 0 respectively
       sample = IADC_pullSingleFifoResult(IADC0);

  // Calculate input voltage in mV
  millivolts = (sample.data * 2500) / 4096;
  return millivolts;
}



/**************************************************************************//**
 * @brief  IADC Task
 *****************************************************************************/

void iadc_task(void *p_arg)

{

  (void)p_arg;
  while (1) {
     // Put your application code here!
     int milivolts =0;
     sl_app_log("ADC Task\r\n");
     my_adc_start_measurement();
     milivolts = my_adc_measurement_get();

     //It Updates the ADCData variable on the GattDB database with the most current value
     sl_bt_gatt_server_write_attribute_value(gattdb_ADCData,
                                              0,
                                               sizeof(milivolts),
                                               (uint8_t)&milivolts
                                               );

     if (! xQueueSend(ADC_to_LED_Queue_Handle,&milivolts,1000))
       {
         printf("Failed to send to the queue\r\n");
       }
     vTaskDelay(1000);
    // sl_led_toggle(&sl_led_led0);
   }

}

/**************************************************************************//**
 * @brief  LED task
 *****************************************************************************/

void led_task(void *p_arg)
{

  (void)p_arg;
  while (1)
    {
      int milivolts = 0;
        // Put your application code here!
      sl_app_log("LED Task\r\n");

      if(xQueueReceive(ADC_to_LED_Queue_Handle, &milivolts, 1000))
      {
          sl_led_toggle(&sl_led_led0);
      }
      else
        {
          printf("Failed to receive from the queue\r\n");
        }


      vTaskDelay(500);
    }

}


/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////

  //It Initializes the IADC module
  my_adc_init();


  //It Creates a Queue with the possibility of adding 3 items and each item will have the size of an int
  ADC_to_LED_Queue_Handle = xQueueCreate(3,sizeof(int));

#if 0

  xTaskCreateStatic(led_task,
                    LED_TASK_NAME,
                    LED_TASK_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY,
                    led_task_stack,
                    &led_task_handle);
#endif
  xTaskCreate(led_task, LED_TASK_NAME, LED_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, &led_task_handle);

  xTaskCreateStatic(iadc_task,
                    IADC_TASK_NAME,
                    IADC_TASK_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY,
                    iadc_task_stack,
                    &iadc_task_handle);


}

/**************************************************************************//**
 * Application Process Action.
 **********************************************************************`*******/
SL_WEAK void app_process_action(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];



  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:

      printf("Hello world\r\n");

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to get Bluetooth address\n",
                    (int)sc);

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(gattdb_system_id,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to write attribute\n",
                    (int)sc);

      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to create advertising set\n",
                    (int)sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to set advertising timing\n",
                    (int)sc);
      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      break;

#if 0
    case sl_bt_evt_gatt_server_characteristic_status_id:
              if (evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_ADCData)
                {
                // client characteristic configuration changed by remote GATT client
                if ((gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags == 1)
                  {
                    //Starting a Soft-Timer to send the data back to Central
                    sl_app_log("Central Subscribed to Characteristic\r\n");
                    //sl_bt_system_set_soft_timer ( 32768, 0, 0);
                    sl_bt_gatt_server_read_attribute_value  ( gattdb_ADCData,
                                                                           0,
                                                                            1,
                                                                            sizeof(gattdb_ADCData),
                                                                            &gattdb_ADCData
                                                                            );


                  }
                // confirmation of indication received from remove GATT client
                else if (gatt_server_confirmation == (gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags)
                  {
                    sl_app_log("Gatt_server_confirmation\r\n");
                  }
                else {
                  sl_app_assert(false,
                                "[E: 0x%04x] Unexpected status flag in evt_gatt_server_characteristic_status\n",
                                (int)evt->data.evt_gatt_server_characteristic_status.status_flags);
                }
              }
              break;


    case sl_bt_evt_gatt_server_user_write_request_id:

     if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_LED0) {
       if (evt->data.evt_gatt_server_user_write_request.value.data[0]) {
         sl_led_turn_on(&sl_led_led0);
   } else {
         sl_led_turn_off(&sl_led_led0);
       }
       sl_bt_gatt_server_send_user_write_response(
         evt->data.evt_gatt_server_user_write_request.connection,
         evt->data.evt_gatt_server_user_write_request.characteristic,
         0);


   } break;

    case sl_bt_evt_gatt_server_user_read_request_id:
     if(evt->data.evt_gatt_server_user_read_request.characteristic == gattdb_LED0) {
       led0_state = sl_led_get_state(&sl_led_led0);
       sl_bt_gatt_server_send_user_read_response(
         evt->data.evt_gatt_server_user_read_request.connection,
         evt->data.evt_gatt_server_user_read_request.characteristic,
         0,
         1, &led0_state, &sent_len);
   } break;

#endif
    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        advertising_set_handle,
        advertiser_general_discoverable,
        advertiser_connectable_scannable);
      sl_app_assert(sc == SL_STATUS_OK,
                    "[E: 0x%04x] Failed to start advertising\n",
                    (int)sc);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
