#include "scale.h"

// Define BLE variables
static BLE *            ble_node = nullptr; // Main variable to handle all the ble tasks such as create service, characteristic
static QueueHandle_t    is_scale_sending_mutex = xSemaphoreCreateMutex(); // Mutex lock for scale update flag
static QueueHandle_t    is_scale_updated_mutex = xSemaphoreCreateMutex(); // Mutex lock for scale update flag
static QueueHandle_t    is_qr_updated_mutex = xSemaphoreCreateMutex();
static ScaleValue       scale_value; // The decode data that is parsed from the scale (Contains sample date time and scale weight, scale unit)
static bool             is_scale_sending = false; // This flag is used to trigger if the scale is currently notify using BLE
static bool             is_scale_updated = false; // This flag is used to trigger if the scale is currently notify using BLE
static bool             is_qr_updated = false;
static String           scanner_data = "";

static int ForcePinMap [] = {PIN_L1, PIN_L2, PIN_L3, PIN_L4, PIN_L5, PIN_L6, PIN_L7, PIN_L8, PIN_L9, PIN_L10,
                             PIN_R1, PIN_R2, PIN_R3, PIN_R4, PIN_R5, PIN_R6, PIN_R7, PIN_R8, PIN_R9, PIN_R10};

static float lfoot_force_values [NB_OF_FORCE_SENSORS / 2] = {0.0},
             rfoot_force_values [NB_OF_FORCE_SENSORS / 2] = {0.0};
static QueueHandle_t force_value_mutex = xSemaphoreCreateMutex();

static const char * ForceCharUUID [] = {
        LEFT_CHAR_UUID, 
        RIGHT_CHAR_UUID
}; 

// Define QR Module serial
static HardwareSerial   GM65_serial (GM65_UART);
static GM65_Scanner     scanner(&GM65_serial);

// Define ESP Slave Communication
static HardwareSerial   master_serial (ESP_SLAVE_UART);

void initScaleBLEService()
{
    ESP_LOGI(SCALE_TAG, "Initializing Scale BLE Service...");
    ble_node = new BLE(); 
    // Initialize BLE for both TabletApp (Server) and MiScale2 (Client)
    ble_node->init(ESP_MODEL);
    BLEClient * ble_client = nullptr;
    
    // ===== PRIORITY 1: Setup TabletApp BLE Server (must be ready immediately) =====
    // Create services for TabletApp Server
    ble_node->createService(WEIGHT_SERVICE_UUID);
    ble_node->createCharacteristic(WEIGHT_SERVICE_UUID, WEIGHT_CHAR_UUID, BLE_NOTIFY, BLE_FORMAT_FLOAT32);
    
    ble_node->createService(FORCE_SERVICE_UUID);
    ble_node->createCharacteristic(FORCE_SERVICE_UUID, LEFT_CHAR_UUID, BLE_NOTIFY, BLE_FORMAT_UINT8);
    ble_node->createCharacteristic(FORCE_SERVICE_UUID, RIGHT_CHAR_UUID, BLE_NOTIFY, BLE_FORMAT_UINT8);

    ble_node->createService(QR_SERVICE_UUID);
    ble_node->createCharacteristic(QR_SERVICE_UUID, QR_CHAR_UUID, BLE_NOTIFY, BLE_FORMAT_STRING);
    
    ble_node->startService(WEIGHT_SERVICE_UUID);
    ble_node->startService(FORCE_SERVICE_UUID);
    ble_node->startService(QR_SERVICE_UUID);
    if (!ble_node->advertise())
    {
        ESP_LOGE(SCALE_TAG, "[TabletApp] Failed to advertise services to tablet app");
        while (1) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } else {
        ESP_LOGI(SCALE_TAG, "[TabletApp] Services advertised successfully, waiting for tablet app...");
        String myMacAddress = BLEDevice::getAddress().toString().c_str();
        ESP_LOGI(SCALE_TAG, "ESP32 BLE MAC Address: %s", myMacAddress.c_str());
    }
    
    // ===== PRIORITY 2: MiScale2 scan starts ASYNC in reconnectDevicesService task =====
    // This allows TabletApp to connect and use QR scanning even if MiScale2 is not ready
    ESP_LOGI(SCALE_TAG, "[MiScale2] MiScale2 connection will be handled by reconnectDevicesService task");
}

float massInterpolate(int pin, int analog_val, int sensor_idx){
    // Assign index of sensor if it is sent from slave
    int index = (pin == -1) ? sensor_idx : 0;
    if (pin != -1)
    {
        for (; index < NB_OF_FORCE_SENSORS; index++) {
            if (ForcePinMap[index] == pin)
                break;
        }
    
        if (index >= NB_OF_FORCE_SENSORS)
            return 0.0;
    
        if (ForcePinMap[index] == -1)
            return 0.0;
    }

    float analog_value = (pin == -1)? analog_val : analogRead((uint8_t)ForcePinMap[index]); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095

    if (analog_value <= 1000) return 0.0; // Lower than threshold, consider as zero mass
    if (analog_value >= 3750) return 10000.0; // Upper than threshold, consider as maximum mass (10 kg)

    return coefficients[index][0] * exp(coefficients[index][1] * analog_value) + coefficients[index][2];
}

static uint32_t timestamp = 0;
void parseScaleData (String UUID, unsigned char * data, size_t length)
{
    // Callback function to handle MiScale2 weight data notifications
    char uuid[37];
    if (UUID.length() > 2)
    {
        strcpy(uuid, UUID.c_str() + 2);
    }

    if (strcmp(uuid, WEIGHT_CHAR_UUID) != 0)
    return;
    
    if (data == nullptr || length <= 0)
    return;
    
    if (xSemaphoreTake(is_scale_sending_mutex, portMAX_DELAY) == pdTRUE) {
        is_scale_sending = true;
        xSemaphoreGive(is_scale_sending_mutex);
    }

    scale_value.is_lbs           = (data[0] & 0x01);
    scale_value.weight           = ((data[2] << 8) | data[1]) / ((scale_value.is_lbs) ? 100.0 : 200.0);
    scale_value.timestamp.year   = (data[4] << 8) | data[3];
    scale_value.timestamp.month  = data[5];
    scale_value.timestamp.day    = data[6];
    scale_value.timestamp.hour   = data[7];
    scale_value.timestamp.minute = data[8];
    scale_value.timestamp.second = data[9];

    ESP_LOGI(SCALE_TAG, "[MiScale2] Weight received: %.4f", scale_value.weight);
    if (xSemaphoreTake(is_scale_updated_mutex, portMAX_DELAY) == pdTRUE) {
        is_scale_updated = true;
        xSemaphoreGive(is_scale_updated_mutex);
    }
    timestamp = xTaskGetTickCountFromISR();
}

bool verifyForceData (uint8_t * buffer, int buffer_len)
{
    if (buffer == nullptr || buffer_len < 1)
        return false;

    // Verify if start byte is 0xA5
    if (buffer[0] != 0xA5) {
        ESP_LOGW(SCALE_TAG, "[ESP Slave] Invalid start byte in force data frame");
        return false;
    }

    int crc = buffer[buffer_len - 1],
        crc_calculated = 0;

    for (int i = 1; i < buffer_len - 1; i++) 
    {
        crc_calculated ^= buffer[i];
    }

    return (crc == crc_calculated);
}

void readForceSensors(void * pvParameters)
{
    TickType_t latest_force_read_time = xTaskGetTickCount(), test = millis();
    master_serial.begin(ESP_SLAVE_BAUD, SERIAL_8N1, ESP_SLAVE_RX_PIN, ESP_SLAVE_TX_PIN);

    while (1)
    {
        if (xSemaphoreTake(force_value_mutex, portMAX_DELAY) == pdTRUE)
        {
            // Parse right sensors' value received from slave
            for (int i = 0; i < NB_OF_FORCE_SENSORS / 2; i++) 
            {
                if (ForcePinMap[i] == -1)
                    continue;

                // Left side force sensors
                float force_value = massInterpolate(ForcePinMap[i]);
                lfoot_force_values[i] = force_value;    
            }
            xSemaphoreGive(force_value_mutex);
        }

        
        /*
            The workflow of updating right foot data is descibed below:
            Step 1: Send start byte to slave
            Step 2: Receive and validate the response
            Step 3: Put new values (If valid)
        */
        bool success = false;
        uint8_t data_frame [22] = {0xA5};
        
        for (int retry = 0; retry < 3; retry++)
        {
            master_serial.write(data_frame, 1); // Send start byte
            master_serial.flush();

            // Wait till data is available
            while (master_serial.available() == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            // Expect to receive full frame size: 1 start byte followed by 10 float sensors' data and 1 CRC byte
            if (master_serial.readBytes(data_frame, FORCE_FRAME_LEN) < FORCE_FRAME_LEN)
            {
                ESP_LOGW(SCALE_TAG, "ESP Slave - Incomplete force data frame received");
                vTaskDelay(pdMS_TO_TICKS(20)); // Retry after 20ms
                continue;
            }
            if (!verifyForceData(data_frame, FORCE_FRAME_LEN))
            {
                ESP_LOGW(SCALE_TAG, "ESP Slave - Mismatch CRC or passing nullptr into verify function");
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            else
            {
                success = true;
                break;
            }
        }

        // If failed to read from slave, print it out
        if (!success)
        {
            ESP_LOGW(SCALE_TAG, "Reached max retry when read force sensors from slave!");
            vTaskDelayUntil(&latest_force_read_time, pdMS_TO_TICKS(SCALE_FORCE_SENSOR_SAMPLE_FREQUENCY));
            continue;
        }

        if (xSemaphoreTake(force_value_mutex, portMAX_DELAY) == pdTRUE)
        {
            for (int i = 0; i < NB_OF_FORCE_SENSORS / 2; i++)
            {
                // Because of receiving data from slave, so we need to shift the index to be started at NUMBER OF SENSORS / 2 - sensors' index of right foot to get the coefficients correctly
                // Data frame layout from slave:
                // [0] = START_BYTE
                // [1..20] = 10 x (high, low) ADC bytes
                // [21] = CRC
                int hi_index = 1 + i * 2;
                int lo_index = hi_index + 1;
                int adc_raw = (data_frame[hi_index] << 8) | data_frame[lo_index];
                float force_value = massInterpolate(-1, adc_raw, i + NB_OF_FORCE_SENSORS / 2);
                rfoot_force_values[i] = force_value;
            }
            xSemaphoreGive(force_value_mutex);
            // if (millis() - test >= 1000){
            //     test = millis();
            //     Serial.printf("\n[%.4f", lfoot_force_values[0]);
            //     for (int i = 1; i < NB_OF_FORCE_SENSORS; i++)
            //     {
            //         Serial.printf(",%.4f", (i < NB_OF_FORCE_SENSORS / 2) ? lfoot_force_values[i] : rfoot_force_values[i - NB_OF_FORCE_SENSORS / 2]);
            //     }
            //     Serial.println("]");
            // }
        }
        vTaskDelayUntil(&latest_force_read_time, pdMS_TO_TICKS(SCALE_FORCE_SENSOR_SAMPLE_FREQUENCY));
    }
    vTaskDelete(NULL);
}

void updateValuesService(void * pvParameters) {
    // Task to send weight and force data to TabletApp via BLE notifications
    TickType_t last_scale_update_time = xTaskGetTickCount();
    TickType_t last_force_sensor_update_time = xTaskGetTickCount();
    while(1)
    {
        last_scale_update_time = xTaskGetTickCount();
        // Check if scale_update_flag is created
        while (ble_node == nullptr)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        /* QR Processing */
        // Send QR code data to TabletApp when GM65 scanner provides new data
        if (xSemaphoreTake(is_qr_updated_mutex, portMAX_DELAY) == pdTRUE) {
            if (is_qr_updated) {  // Only send when GM65 actually returned data
                is_qr_updated = false;
                ble_node->setValue(QR_SERVICE_UUID, QR_CHAR_UUID, scanner_data);
                ESP_LOGI(SCALE_TAG, "[TabletApp] Updated QR characteristic: %s", scanner_data.c_str());
            }
            xSemaphoreGive(is_qr_updated_mutex);
        }

        /* Force Sensor Processing */
        if (xTaskGetTickCount() - last_force_sensor_update_time > 1000.0 / SCALE_FORCE_SENSOR_SAMPLE_FREQUENCY)
        {
            if (xSemaphoreTake(force_value_mutex, portMAX_DELAY) == pdTRUE)
            {
                last_force_sensor_update_time = xTaskGetTickCount ();
                ble_node->setValue(FORCE_SERVICE_UUID, LEFT_CHAR_UUID, (uint8_t *)lfoot_force_values, sizeof(lfoot_force_values));
                ble_node->setValue(FORCE_SERVICE_UUID, RIGHT_CHAR_UUID, (uint8_t *)rfoot_force_values, sizeof(rfoot_force_values));
                xSemaphoreGive(force_value_mutex);
                ESP_LOGI(SCALE_TAG, "[TabletApp] Updated 2 Force characteristic (20 values)");
            }
        }

        /* Scale Weight Processing*/
        // Wait for MiScale2 weight data to arrive
        if (xSemaphoreTake(is_scale_sending_mutex, portMAX_DELAY) == pdTRUE)
        {   
            // If scale is not in sending period, keep waiting
            if (is_scale_sending == false) {
                xSemaphoreGive(is_scale_sending_mutex);
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }

            // No new MiScale2 data received in the last 2 seconds
            if (xTaskGetTickCount() - timestamp > pdMS_TO_TICKS(2000)) {
                if (is_scale_sending == 1) {
                    is_scale_sending = false;
                }
                
                xSemaphoreGive(is_scale_sending_mutex);
                // Stop updating if no data received from MiScale2
                vTaskDelay(pdMS_TO_TICKS(20));
                continue; 
            }
            xSemaphoreGive(is_scale_sending_mutex);
        }
        
        // If it is still in sending period and 
        // Send weight value to TabletApp BLE characteristic
        if (xSemaphoreTake(is_scale_sending_mutex, portMAX_DELAY) == pdTRUE) {
            is_scale_sending = false;
            xSemaphoreGive(is_scale_sending_mutex);
            ble_node->setValue(WEIGHT_SERVICE_UUID, WEIGHT_CHAR_UUID, scale_value.weight);
            ESP_LOGI(SCALE_TAG, "[TabletApp] Updated Weight characteristic: %.4f kg", scale_value.weight);
        }

        vTaskDelayUntil(&last_scale_update_time, pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

bool verifyCitizenID (String buffer) {
    if (buffer == "")   return false;
    
    int number_of_vertical_bar = 0;
    int index_of_vertical_bar[12] = {-1};  // Increased array size to handle up to 12 fields
    
    /*
    * Valid Vietnamese Citizen IDs:
    * - CCCD (Căn Cước Công Dân): 6 bars (7 fields)
    *   Format: 051204013352|212872341|Name|DOB|Gender|Address|IssueDate
    *   First bar at index 12
    * 
    * - CC (Căn Cước): 11 bars (12 fields: 7 data fields + 5 empty fields)
    *   Format: 051077006679|211996303|Name|DOB|Gender|Address|IssueDate||||
    *   First bar at index 10 or 12
    */
    for (int i = 0; i < buffer.length(); i++)
    {
        if (buffer[i] == '|')
        {
            if (number_of_vertical_bar < 12) {  // Prevent array overflow
                index_of_vertical_bar[number_of_vertical_bar] = i;
            }
            number_of_vertical_bar++;
        }
    }
    
    // Check for CCCD format: 6 bars, first bar at index 12
    if (number_of_vertical_bar == 6 && index_of_vertical_bar[0] == 12) {
        return true;
    }
    
    // Check for CC format: 10 bars, first bar at index 10 or 12
    if (number_of_vertical_bar == 10 && index_of_vertical_bar[0] == 12) {
        return true;
    }
    
    return false;
}

void qrScannerService(void *pvParameters) {
    GM65_serial.begin(GM65_BAUD, SERIAL_8N1, GM65_RX_PIN, GM65_TX_PIN);
    while (1)
    {
        if (GM65_serial.available() > 0){
            String raw_data = scanner.read();
            if (verifyCitizenID(raw_data)){
                scanner_data = raw_data;  // Only update if data is valid
                ESP_LOGI(SCALE_TAG, "[QR Scanner] Valid QR scanned: %s", scanner_data.c_str());
                if (xSemaphoreTake(is_qr_updated_mutex, portMAX_DELAY) == pdTRUE) {
                    is_qr_updated = true;
                    xSemaphoreGive(is_qr_updated_mutex);
                }
            } else {
                ESP_LOGD(SCALE_TAG, "[QR Scanner] Invalid QR format received (ignored)");
            }
        }
        vTaskDelay(50);
    }
    vTaskDelete(NULL);
}

void reconnectDevicesService(void * pvParameters) {
    // Task to maintain MiScale2 connection and handle initial scan/connect
    // This task runs ASYNCHRONOUSLY so TabletApp server is available immediately
    // MiScale2 may auto-poweroff when no one is standing on it
    
    // Wait for initial setup to complete (TabletApp server already advertising)
    vTaskDelay(pdMS_TO_TICKS(10000));  // Wait 10 seconds for initial setup
    ESP_LOGI(SCALE_TAG, "[MiScale2] Reconnect service started, beginning MiScale2 scan...");
    
    uint32_t failed_reconnect_count = 0;
    uint32_t max_reconnect_attempts = 3;  // Reduce to 3 attempts (9 seconds) before rescan
    bool is_rescanning = false;
    
    // ===== PHASE 1: Initial MiScale2 scan and connection =====
    if (!ble_node->isConnected())
    {
        ESP_LOGI(SCALE_TAG, "[MiScale2] PHASE 1: Starting initial scan for MiScale2 address '%s'...", SCALE_MAC);
        if (ble_node->scan(SCALE_MAC))
        {
            uint32_t scan_timeout = 0;
            while (!ble_node->isScanFound() && scan_timeout < 10000) {
                vTaskDelay(pdMS_TO_TICKS(100));
                scan_timeout += 100;
            }
            
            ESP_LOGI(SCALE_TAG, "[MiScale2] Scan timeout after %.1f seconds, found=%d", scan_timeout/1000.0f, ble_node->isScanFound());
            
            if (ble_node->isScanFound())
            {
                ESP_LOGI(SCALE_TAG, "[MiScale2] Device found! Attempting to connect...");
                if (ble_node->connect())
                {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    ESP_LOGI(SCALE_TAG, "[MiScale2] Connected! Registering for weight notifications...");
                    if (ble_node->registerForIndicate(WEIGHT_SERVICE_UUID, WEIGHT_CHAR_UUID, parseScaleData))
                    {
                        ESP_LOGI(SCALE_TAG, "[MiScale2] SUCCESS: Registered for notifications");
                        failed_reconnect_count = 0;
                    } else {
                        ESP_LOGW(SCALE_TAG, "[MiScale2] Failed to register notifications");
                    }
                } else {
                    ESP_LOGW(SCALE_TAG, "[MiScale2] Connection failed");
                }
            } else {
                    ESP_LOGW(SCALE_TAG, "[MiScale2] Scan timeout - device not found");
            }
        } else {
            ESP_LOGE(SCALE_TAG, "[MiScale2] Cannot start scan");
        }
    }
    
    // ===== PHASE 2: Continuous reconnection and monitoring =====
    while (1)
    {
        if (!ble_node->isConnected())
        {
            // If we're already rescanning and haven't found device yet, wait for scan to complete
            if (is_rescanning && !ble_node->isScanFound())
            {
                vTaskDelay(pdMS_TO_TICKS(1000));  // Check more frequently (1s vs 3s)
                continue;
            }
            
            is_rescanning = false;
            ESP_LOGW(SCALE_TAG, "[MiScale2] Not connected (attempt %lu), attempting to reconnect...", failed_reconnect_count);
            
            if (ble_node->connect())
            {
                // Successfully reconnected
                ESP_LOGI(SCALE_TAG, "[MiScale2] Successfully reconnected after device power-on");

                // Re-register for weight notifications after reconnect
                vTaskDelay(pdMS_TO_TICKS(500));  // Small delay before registering
                ESP_LOGI(SCALE_TAG, "[MiScale2] Re-registering for weight notifications...");
                if (!ble_node->registerForIndicate(WEIGHT_SERVICE_UUID, WEIGHT_CHAR_UUID, parseScaleData))
                {
                    ESP_LOGW(SCALE_TAG, "[MiScale2] Failed to re-register for weight notifications");
                } else {
                    ESP_LOGI(SCALE_TAG, "[MiScale2] Successfully re-registered for weight notifications");
                }

                failed_reconnect_count = 0;
            } else
            {
                failed_reconnect_count++;
                
                // If MiScale2 failed to reconnect multiple times, it might be powered off
                // Start a fresh scan to find it when it powers back on
                if (failed_reconnect_count >= max_reconnect_attempts)
                {
                    ESP_LOGW(SCALE_TAG, "[MiScale2] %lu reconnection failures - starting full rescan (device may have powered off)", max_reconnect_attempts);
                    ble_node->resetScan();
                    
                    // Start scanning again for MiScale2 (5s scan with 1s check = faster response)
                    if (ble_node->scan(SCALE_MAC))
                    {
                        ESP_LOGI(SCALE_TAG, "[MiScale2] Rescan started for %s, will check every 1s for device to power on...", SCALE_MAC);
                        is_rescanning = true;
                    }
                    failed_reconnect_count = 0;
                }
            }
        } else
        {
            // Connection is good, reset counter
            failed_reconnect_count = 0;
            is_rescanning = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Check every 1 second (faster than 3s)
    }
    vTaskDelete(NULL);
}


void runScaleService(void *pvParameters) {
    ESP_LOGI(SCALE_TAG, "===== Starting Scale BLE Service =====");
    ESP_LOGI(SCALE_TAG, "Initializing dual BLE connections: MiScale2 (Client) + TabletApp (Server)");
    initScaleBLEService();
    xTaskCreate(qrScannerService, "QR Scanner Service", 4096, NULL, 2, NULL);
    xTaskCreate(readForceSensors, "Read Force Sensor Task", 4096, NULL, 1, NULL);
    xTaskCreate(updateValuesService, "Update Scale Values Service", 8192, NULL, 1, NULL);
    xTaskCreate(reconnectDevicesService, "Reconnect MiScale2 Service", 8192, NULL, 1, NULL);
    ESP_LOGI(SCALE_TAG, "===== Scale BLE Service Started =====");
    vTaskDelete(NULL);
}
