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


// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;


//FreeRTOS Required Includes and definitions

#include "FreeRTOS.h"
#include "task.h"

#define SL_BT_RTOS_APPLICATION_PRIORITY         10u

//Simple LED required includes

#include "sl_led.h"
#include "sl_simple_led_instances.h"

//IADC Required Includes and definitions

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

//Task Definitions

//LED Task

#define LED_TASK_NAME          "led_task"
#define LED_TASK_STACK_SIZE    200
StackType_t led_task_stack[LED_TASK_STACK_SIZE];
StaticTask_t led_task_handle;

//IADC Task

#define IADC_TASK_NAME          "led_task"
#define IADC_TASK_STACK_SIZE    200
StackType_t iadc_task_stack[IADC_TASK_STACK_SIZE];
StaticTask_t iadc_task_handle;



//LED task
void led_task(void *p_arg)

{

  (void)p_arg;
  while (1)
    {
        // Put your application code here!
        vTaskDelay(1000);
        sl_led_toggle(&sl_led_led0);

    }

}


//IADC Task

//Application task
void iadc_task(void *p_arg)

{

  (void)p_arg;
  while (1)
    {
        // Put your application code here!
        vTaskDelay(1000);
        sl_led_toggle(&sl_led_led0);

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
  ///


  xTaskCreateStatic(led_task,
                    LED_TASK_NAME,
                    LED_TASK_STACK_SIZE,
                    NULL,
                    tskIDLE_PRIORITY,
                    led_task_stack,
                    &led_task_handle);

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
