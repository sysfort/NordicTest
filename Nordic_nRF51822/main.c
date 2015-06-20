

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "sensorsim.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "cmsis_os.h"
#include "ble_advertising.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT      0                                       

#define DEVICE_NAME                          "Nordic_NRF"                           
#define MANUFACTURER_NAME                    "NordicSemiconductor"                     
#define APP_ADV_INTERVAL                     40                                        
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                        

#define APP_TIMER_PRESCALER                  0                                         
#define APP_TIMER_MAX_TIMERS                 (6+BSP_APP_TIMERS_NUMBER)                 
#define APP_TIMER_OP_QUEUE_SIZE              4                                         

#define LEVEL_INTERVAL          2000                                     
#define LEVEL1                    81                                      
#define LEVEL2                    10                                      
#define INCREMENT              1                                          

#define RATE_INTERVAL             1000                                     
#define MIN_RATE                       14                                       
#define MAX_RATE                       30                                       
#define RATE_INCREMENT                 1                                        


#define SENSOR_CONTACT_DETECTED_INTERVAL     5000                                      

#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)          
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)         
#define SLAVE_LATENCY                        0                                         
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)           

#define FIRST_CONN_PARAMS_UPDATE_DELAY       5000                                      
#define NEXT_CONN_PARAMS_UPDATE_DELAY        30000                                     
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                         

#define SEC_PARAM_BOND                       1                                         
#define SEC_PARAM_MITM                       0                                         
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       
#define SEC_PARAM_OOB                        0                                         
#define SEC_PARAM_MIN_KEY_SIZE               7                                          
#define SEC_PARAM_MAX_KEY_SIZE               16                                         

#define DEAD_BEEF                            0xDEADBEEF                               

static uint8_t test_data[4] = {0x01,0x02,0x03,0x04};
static uint8_t ps_flag = 0;

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   
static ble_bas_t                             m_bas;                                     
static ble_hrs_t                             m_hrs;                                    
static bool                                  m_rr_interval_enabled = true;             

static sensorsim_cfg_t                       m_battery_sim_cfg;                        
static sensorsim_state_t                     m_battery_sim_state;                       
static sensorsim_cfg_t                       m_timr_rate_sim_cfg;                    
static sensorsim_state_t                     m_timr_rate_sim_state;                   
static sensorsim_cfg_t                       m_rr_interval_sim_cfg;                   
static sensorsim_state_t                     m_rr_interval_sim_state;                 

static osTimerId                             m_battery_timer_id;                       
static osTimerId                             m_timr_rate_timer_id;                     
static osTimerId                             m_rr_interval_timer_id;                    
static osTimerId                             m_sensor_contact_timer_id;                 

static dm_application_instance_t             m_app_handle;                           

static ble_uuid_t                            m_adv_uuids[] =                           
{
    {BLE_UUID_HEART_RATE_SERVICE,         BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

static void battery_level_meas_timeout_handler(void const * p_context);                 /**< Battery timeout handler. */
static void timr_rate_meas_timeout_handler(void const * p_context);                    /**< timr rate measurement timeout handler. */
static void rr_interval_timeout_handler(void const * p_context);                        /**< RR interval timeout handler. */
static void sensor_contact_detected_timeout_handler(void const * p_context);            /**< Sensor contact detected timeout handler. */
static int b=1;

osTimerDef(battery_timer, battery_level_meas_timeout_handler);                          /**< Definition of battery timer. */
osTimerDef(timr_rate_timer, timr_rate_meas_timeout_handler);                          /**< Definition of timr rate timer. */
osTimerDef(rr_interval_timer, rr_interval_timeout_handler);                             /**< Definition of RR interval timer. */
osTimerDef(sensor_contact_timer, sensor_contact_detected_timeout_handler);              /**< Definition of sensor contact detected timer. */

osPoolDef(ble_evt_pool, 8, ble_evt_t);                                                  /**< Definition of memory pool for ble_stack_thread */
osPoolId ble_evt_pool;                                                                  /**< Memory pool for ble_stack_thread */
osMessageQDef(ble_stack_msg_box, 8, ble_evt_t);                                         /**< Definition of message box for ble_stack_thread */
osMessageQId ble_stack_msg_box;                                                         /**< Message box for ble_stack_thread */

osThreadId ble_stack_thread_id;                                                         /**< BLE Stack thread */

void ble_stack_thread(void const * arg);                                                
osThreadDef(ble_stack_thread, osPriorityAboveNormal, 1, 0);                             /**< Definition of BLE Stack thread */

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level);

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void const * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}


/**@brief Function for handling the timr rate measurement timer timeout.
 *
 * @details This function will be called each time the timr rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void timr_rate_meas_timeout_handler(void const * p_context)
{
    static uint32_t cnt = 0;
    uint32_t        err_code;
    uint16_t        timr_rate;

    UNUSED_PARAMETER(p_context);

    timr_rate = (uint16_t)sensorsim_measure(&m_timr_rate_sim_state, &m_timr_rate_sim_cfg);

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, timr_rate);

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_BUFFERS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third timr rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    //       of messages without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}


/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void rr_interval_timeout_handler(void const * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                      &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void sensor_contact_detected_timeout_handler(void const * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    m_battery_timer_id = osTimerCreate(osTimer(battery_timer), osTimerPeriodic, NULL);

    m_timr_rate_timer_id = osTimerCreate(osTimer(timr_rate_timer), osTimerPeriodic, NULL);

   
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

  //  err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_heart_RATE_SENSOR_heart_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the timr Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_hrs_init_t hrs_init;
    ble_bas_init_t bas_init;
    ble_dis_init_t dis_init;
    uint8_t        body_sensor_location;

    // Initialize timr Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the timr Rate Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_hrm_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_hrm_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hrs_init.hrs_bsl_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hrs_init.hrs_bsl_attr_md.write_perm);

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = LEVEL1;
    m_battery_sim_cfg.max          = LEVEL2;
    m_battery_sim_cfg.incr         = INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    m_timr_rate_sim_cfg.min          = MIN_RATE;
    m_timr_rate_sim_cfg.max          = MAX_RATE;
    m_timr_rate_sim_cfg.incr         = RATE_INCREMENT;
    m_timr_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&m_timr_rate_sim_state, &m_timr_rate_sim_cfg);

    
    m_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    // Start application timers.
    UNUSED_VARIABLE(osTimerStart(m_battery_timer_id, LEVEL_INTERVAL));

   // UNUSED_VARIABLE(osTimerStart(m_timr_rate_timer_id, RATE_INTERVAL));

 
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for receiving the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    ble_evt_t * mptr;

    mptr  = osPoolAlloc(ble_evt_pool);
    *mptr = *p_ble_evt;
    (void)osMessagePut(ble_stack_msg_box, (uint32_t)mptr, 0);
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

#if defined(S110) || defined(S130)
    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#ifdef S130
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}



/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Thread for handling the Application's BLE Stack events.
 *
 * @details This thread is responsible for handling BLE Stack events sent from on_ble_evt().
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
void ble_stack_thread(void const * arg)
{
    uint32_t    err_code;
    osEvent     evt;
    ble_evt_t * p_ble_evt;

    UNUSED_PARAMETER(arg);

    while (1)
    {
        evt = osMessageGet(ble_stack_msg_box, osWaitForever); // wait for message

        if (evt.status == osEventMessage)
        {
            p_ble_evt = evt.value.p;

            switch (p_ble_evt->header.evt_id)
            {
                case BLE_GAP_EVT_CONNECTED:

                    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
                    APP_ERROR_CHECK(err_code);
                    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
                    break;

                default:
                    // No implementation needed.
                    break;
            }
            (void)osPoolFree(ble_evt_pool, p_ble_evt);
        }
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}
static void example_cb_handler(pstorage_handle_t  * handle,
                               uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
    switch(op_code)
    {
                     case PSTORAGE_UPDATE_OP_CODE:
           if (result == NRF_SUCCESS)
           {
               // Update operation successful.
           }
           else
           {
               // Update operation failed.
           }
           break;
       
       
       
    }
}



static void pstorage_test(void)
{
	
	
	 
	
	uint32_t retval;
	retval = pstorage_init();
	pstorage_handle_t       handle;
	pstorage_module_param_t param;

	
	param.block_size  = 100;
param.block_count = 10;
param.cb          = example_cb_handler;
	
	
if(retval == NRF_SUCCESS)
{
    // Module initialization successful.
	
}
else
{
   // Initialization failed, take corrective action.

}
retval = pstorage_register(&param, &handle);
if (retval == NRF_SUCCESS)
{
    // Registration successful.
		
	
}
else
{
	
    // Failed to register, take corrective action.
}

pstorage_handle_t base_handle;
pstorage_handle_t block_handle;
retval = pstorage_block_identifier_get(&handle, 2, &block_handle);
if (retval == NRF_SUCCESS)
{
	//application_timers_start();
    // Get Block Identifier successful.
		//application_timers_start();
//	application_timers_start();
}
else
{
	//application_timers_start();
    // Failed to get block id, take corrective action.
}








   
// Request to write 8 bytes to block at an offset of 20 bytes.
retval = pstorage_store(&block_handle, test_data, 4, 0);
if (retval == NRF_SUCCESS)
{
//application_timers_start();
    // Store successfully requested. Wait for operation result.
}
else
{
    // Failed to request store, take corrective action.
}

retval = pstorage_block_identifier_get(&handle, 2, &block_handle);
if (retval == NRF_SUCCESS)
{
	
}
else
{
	//application_timers_start();
    // Failed to get block id, take corrective action.
}

uint8_t           load_data[4];
   
// Request to read 4 bytes from block at an offset of 12 bytes.
retval = pstorage_load(load_data, &block_handle, 4, 0);
if (retval == NRF_SUCCESS)
{
	//application_timers_start();
    // Load successful. Consume data.
		application_timers_start();

	 if ((load_data[0] == 0x01)&(load_data[1] == 0x02)&(load_data[2] == 0x03)&(load_data[3] == 0x04))
        {
            ps_flag = 100;
				//	application_timers_start();
        }
				
				else
				{ 
					 
					UNUSED_VARIABLE(osTimerStart(m_timr_rate_timer_id, RATE_INTERVAL));

						//application_timers_start();
				}
}
else
{
	
	//application_timers_start();
    // Failed to load, take corrective action.
}


}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
printf("%d\n",b);
    // Initialize.
    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();

    ble_evt_pool      = osPoolCreate(osPool(ble_evt_pool));
    ble_stack_msg_box = osMessageCreate(osMessageQ(ble_stack_msg_box), NULL);
pstorage_test();
    // Start execution.
    ble_stack_thread_id = osThreadCreate(osThread(ble_stack_thread), NULL);
    UNUSED_VARIABLE(ble_stack_thread_id);
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;; )
    {
        UNUSED_VARIABLE(osDelay(1000));
    }
}


