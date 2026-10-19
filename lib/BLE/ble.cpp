#include "ble.h"

bool BLE::_do_connect = false;
String BLE::_target_address = "";
const NimBLEAdvertisedDevice * BLE::_target_advertise_device = nullptr;
BLE::BLECallbackFunction BLE::_upperlayer_callback_fp = nullptr;

/*Callback definitions*/
/*Client Callback (MiScale2 Connection)*/
void BLE::ClientCallbacks::onConnect(NimBLEClient * pClient)
{
    ESP_LOGI(BLE_TAG, "[MiScale2] Connected to MiScale2 device: %s", pClient->getPeerAddress().toString().c_str());
}

void BLE::ClientCallbacks::onDisconnect(NimBLEClient * pClient, int reason)
{
    // ESP_LOGI(BLE_TAG, "[MiScale2] Disconnected from MiScale2 device: %s", pClient->getPeerAddress());
}
/*Server Callback (TabletApp Connection)*/
void BLE::ServerCallbacks::onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) {
    ESP_LOGI(BLE_TAG, "[TabletApp] Server received connection from tablet app: %s\n", connInfo.getAddress().toString().c_str());

    /**
     *  We can use the connection handle here to ask for different connection parameters.
     *  Args: connection handle, min connection interval, max connection interval
     *  latency, supervision timeout.
     *  Units; Min/Max Intervals: 1.25 millisecond increments.
     *  Latency: number of intervals allowed to skip.
     *  Timeout: 10 millisecond increments.
     */
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
}

void BLE::ServerCallbacks::onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) {
    ESP_LOGI(BLE_TAG, "[TabletApp] Server disconnected from tablet app: %s", connInfo.getAddress().toString().c_str());
}

/*Scan Callback (MiScale2 Scan)*/
void BLE::ScanCallbacks::onResult(const NimBLEAdvertisedDevice* advertisedDevice)
{
    // Debug: Print discovered device (name may be empty)
    String advName = String(advertisedDevice->getName().c_str());
    String advAddr = String(advertisedDevice->getAddress().toString().c_str());
    ESP_LOGD(BLE_TAG, "[MiScale2] Scanned device: name='%s' RSSI=%d dBm addr=%s",
             advName.c_str(), advertisedDevice->getRSSI(), advAddr.c_str());

    // Match by address (case-insensitive) for robust identification
    String targetAddr = BLE::_target_address;
    advAddr.toLowerCase();
    targetAddr.toLowerCase();
    if (advAddr != targetAddr) {
        ESP_LOGD(BLE_TAG, "[MiScale2] Address mismatch: looking for '%s', got '%s'",
                 targetAddr.c_str(), advAddr.c_str());
        return; // Not the device we're looking for
    }
    
    // Check RSSI signal strength
    if (advertisedDevice->getRSSI() <= -99)
    {
        ESP_LOGW(BLE_TAG, "[MiScale2] Signal too weak: RSSI %d dBm (threshold: -75 dBm)",
                 advertisedDevice->getRSSI());
        return;  // Signal too weak
    }
    
    // Found matching device with good signal!
    BLE::_do_connect = true;
    BLE::_target_advertise_device = advertisedDevice;
    ESP_LOGI(BLE_TAG, "[MiScale2] ✓ Found device at %s with RSSI: %d dBm",
             advertisedDevice->getAddress().toString().c_str(),
             advertisedDevice->getRSSI());
    NimBLEDevice::getScan()->stop();
}

void BLE::ScanCallbacks::onScanEnd(const NimBLEScanResults& results, int reason)
{
    // Log scan result
    ESP_LOGI(BLE_TAG, "[MiScale2] Scan ended. Devices found: %d, Reason: %d", results.getCount(), reason);
    // Don't auto-restart scan here - let reconnectDevicesService task manage scan timing
    // This prevents callback from blocking and allows for better control flow
    // Task will restart scan if device not found
}

/*Notify callback function (MiScale2 Data Callback)*/
void BLE::indicateCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
    if (_upperlayer_callback_fp == nullptr)
    {
        return;
    }
    _upperlayer_callback_fp(String(pRemoteCharacteristic->getUUID().toString().c_str()), pData, length);
}
/*End of Callback definition*/

/*BLE Methods*/
bool BLE::init(String name)
{
    // Check if BLE server (TabletApp) has been initialized
    if (_ble_server != nullptr)
        return 0;
    
    NimBLEDevice::init(name.c_str());

    // Create BLE Server for TabletApp connection
    _ble_server = NimBLEDevice::createServer();
    _ble_server->setCallbacks(&server_callback);
    _ble_server->advertiseOnDisconnect(true);
    NimBLEAdvertising * ble_advertise = NimBLEDevice::getAdvertising();
    ble_advertise->setName(name.c_str()); 
    return true;
}

bool BLE::scan(String device_name)
{
    // Scan for MiScale2 device by address
    _target_address = device_name;
    NimBLEScan * scan_object = NimBLEDevice::getScan();

    ESP_LOGI(BLE_TAG, "[MiScale2] Starting scan for address: '%s' (Duration: 15s, Interval: 1000ms, Passive mode)", device_name.c_str());
    
    scan_object->setScanCallbacks(&scan_callback, false);
    scan_object->setInterval(1000);    // 1000ms interval to reduce CPU
    scan_object->setWindow(1000);       // 1000ms window
    scan_object->setActiveScan(false);  // Use passive scan
    
    bool started = scan_object->start(15000);  // 15s scan duration
    if (!started) {
        ESP_LOGE(BLE_TAG, "[MiScale2] Failed to start scan!");
    }
    return started;
}

bool BLE::isScanFound()
{
    // Check if MiScale2 device has been found during scan
    return _do_connect;
}

void BLE::resetScan()
{
    // Reset scan state to allow finding MiScale2 device again after it powers off/on
    _do_connect = false;
    _target_advertise_device = nullptr;
    ESP_LOGI(BLE_TAG, "[MiScale2] Scan state reset, ready for new scan");
}

bool BLE::connect()
{
    // If MiScale2 device has not been found during scan
    if (_target_advertise_device == nullptr) {
        ESP_LOGE(BLE_TAG, "[MiScale2] ✗ Cannot connect: _target_advertise_device is NULL");
        return false;
    }

    ESP_LOGI(BLE_TAG, "[MiScale2] Connection attempt to %s (%s)",
             _target_advertise_device->getName().c_str(),
             _target_advertise_device->getAddress().toString().c_str());

    // If there are BLE clients (MiScale2 connections) already created
    if (NimBLEDevice::getCreatedClientCount())
    {
        ESP_LOGD(BLE_TAG, "[MiScale2] Found %d existing client(s), checking status...", NimBLEDevice::getCreatedClientCount());
        
        _ble_client = NimBLEDevice::getClientByPeerAddress(_target_advertise_device->getAddress());
        // If MiScale2 device already exists from previous connection
        if (_ble_client != nullptr)
        {
            // Check if client is already connected to avoid duplicate connect attempt
            if (_ble_client->isConnected())
            {
                ESP_LOGI(BLE_TAG, "[MiScale2] ✓ Already connected to MiScale2");
                return true;
            }
            
            // Try to reconnect to MiScale2 if it's disconnected
            ESP_LOGI(BLE_TAG, "[MiScale2] Reconnecting to previous client...");
            if (!_ble_client->connect(_target_advertise_device, false))
            {
                ESP_LOGW(BLE_TAG, "[MiScale2] ✗ Reconnection failed");
                return false;
            }
            ESP_LOGI(BLE_TAG, "[MiScale2] ✓ Reconnected successfully");
            return true;
        } else
        {
            /* 
            There will be 2 cases for MiScale2 client:
            case 1: There is at least one disconnected MiScale2 client
            case 2: No disconnected client available
            */
            _ble_client = NimBLEDevice::getDisconnectedClient();
            if (_ble_client) {
                ESP_LOGD(BLE_TAG, "[MiScale2] Using disconnected client for reconnection");
            }
        }
    }

    // If there isn't any disconnected MiScale2 client, then create a new one
    if (_ble_client == nullptr)
    {
        if (NimBLEDevice::getCreatedClientCount() >= MYNEWT_VAL(BLE_MAX_CONNECTIONS))
        {
            ESP_LOGE(BLE_TAG, "[MiScale2] ✗ Cannot create client: max connections reached");
            return false;
        }
        
        ESP_LOGI(BLE_TAG, "[MiScale2] Creating new BLE client (client count: %d)...", NimBLEDevice::getCreatedClientCount());
        _ble_client = NimBLEDevice::createClient();
        if (!_ble_client) {
            ESP_LOGE(BLE_TAG, "[MiScale2] ✗ Failed to create client object");
            return false;
        }
        
        _ble_client->setClientCallbacks(&client_callback);
        _ble_client->setConnectionParams(12, 12, 0, 150);
        _ble_client->setConnectTimeout(5000);
        
        ESP_LOGI(BLE_TAG, "[MiScale2] Connecting new client to %s...", _target_advertise_device->getAddress().toString().c_str());
        if (!_ble_client->connect(_target_advertise_device, false))
        {
            ESP_LOGW(BLE_TAG, "[MiScale2] ✗ New client connection failed");
            return false;
        }
    } else if (!_ble_client->isConnected())
    {
        // We have a disconnected client, try to reconnect it
        ESP_LOGI(BLE_TAG, "[MiScale2] Reconnecting existing client...");
        if (!_ble_client->connect(_target_advertise_device, false))
        {
            ESP_LOGW(BLE_TAG, "[MiScale2] ✗ Failed to reconnect existing client");
            return false;
        }
    }

    // Discover MiScale2 attributes and services
    ESP_LOGI(BLE_TAG, "[MiScale2] Discovering attributes and services...");
    _ble_client->discoverAttributes();
    
    ESP_LOGI(BLE_TAG, "[MiScale2] ✓ Connect completed successfully");
    return true;
}

bool BLE::isConnected()
{
    // Check if MiScale2 client is connected
    if (_ble_client == nullptr)     return false;
    return _ble_client->isConnected();
}

bool BLE::registerForIndicate(String service_uuid, String characteristic_uuid, BLECallbackFunction callback_func)
{
    // Register to receive weight notifications from MiScale2
    if (_ble_client == nullptr)         return false;
    if (!_ble_client->isConnected())    return false;
    
    // Get MiScale2 service
    NimBLERemoteService * ble_service = _ble_client->getService(NimBLEUUID(service_uuid.c_str()));
    if (ble_service == nullptr)
    {
        ESP_LOGW(BLE_TAG, "[MiScale2] Cannot find service %s in MiScale2 device", service_uuid.c_str());
        return false;
    }

    // Check if MiScale2 service contains the wanted characteristic
    NimBLERemoteCharacteristic * ble_characteristic = ble_service->getCharacteristic(NimBLEUUID(characteristic_uuid.c_str()));

    if (ble_characteristic == nullptr)
    {
        ESP_LOGW(BLE_TAG, "[MiScale2] Cannot find characteristic %s in MiScale2 device", characteristic_uuid.c_str());
        return false;
    }

    // If MiScale2 characteristic can notify, subscribe to it
    if (ble_characteristic->canIndicate())
    {
        _upperlayer_callback_fp = callback_func;
        return ble_characteristic->subscribe(false, indicateCB);
    }
    ESP_LOGW(BLE_TAG, "[MiScale2] Cannot subscribe: service %s, characteristic %s does not support notifications", ble_service->getUUID().toString().c_str(), ble_characteristic->getUUID().toString().c_str());
    return false;
}

bool BLE::createService (String service_uuid)
{
    // Create service for TabletApp connection
    if (_ble_server == nullptr)     return false;   
    if (_ble_server->getServiceByUUID(service_uuid.c_str()) != nullptr)     
    {
        ESP_LOGW(BLE_TAG, "[TabletApp] Service already exists for TabletApp");
        return false;
    }

    return _ble_server->createService(NimBLEUUID(service_uuid.c_str())) != nullptr;
}

bool BLE::createCharacteristic(String service_uuid, String characteristic_uuid, BLECharMode mode, BLEDataType type)
{
    // Create characteristic for TabletApp to read/receive
    if (_ble_server == nullptr)         return false;
    
    NimBLEService * ble_service = _ble_server->getServiceByUUID(NimBLEUUID(service_uuid.c_str()));
    if (ble_service == nullptr)         return false;
    
    NimBLECharacteristic * ble_characteristic = ble_service->getCharacteristic(NimBLEUUID(characteristic_uuid.c_str()));
    if (ble_characteristic != nullptr)  return false;

    ble_characteristic = ble_service->createCharacteristic(NimBLEUUID(characteristic_uuid.c_str()), mode);
    if (ble_characteristic == nullptr)  return false;

    if (type != BLE_FORMAT_UNKNOWN)
    {
        NimBLE2904 *ble_2904 = ble_characteristic->create2904();
        switch (type){
            case BLE_FORMAT_FLOAT32:
                ble_2904->setFormat(NimBLE2904::FORMAT_FLOAT32);
                break;
            case BLE_FORMAT_FLOAT64:
                ble_2904->setFormat(NimBLE2904::FORMAT_FLOAT64);
                break;
            case BLE_FORMAT_SINT32:
                ble_2904->setFormat(NimBLE2904::FORMAT_SINT32);
                break;
            case BLE_FORMAT_UINT8:
                ble_2904->setFormat(NimBLE2904::FORMAT_UINT8);
                break;
            case BLE_FORMAT_UINT32:
                ble_2904->setFormat(NimBLE2904::FORMAT_UINT32);
                break;
            case BLE_FORMAT_STRING:
                ble_2904->setFormat(NimBLE2904::FORMAT_UTF8);
                break;
            default:
                break;
        }
    }
    return true;
}

bool BLE::startService(String service_uuid)
{
    // Start TabletApp service and make it discoverable
    if (_ble_server == nullptr)     return false;
    
    NimBLEService * ble_service = _ble_server->getServiceByUUID(NimBLEUUID(service_uuid.c_str()));
    if (ble_service == nullptr)     return false;

    ble_service->start();
    NimBLEAdvertising* ble_advertise = NimBLEDevice::getAdvertising();
    // To avoid "data length exceeded" errors, only add short (16-bit) UUIDs
    // to the advertising payload; 128-bit UUIDs consume too much space.
    String uuidStr = ble_service->getUUID().toString().c_str();
    if (uuidStr.length() <= 4) {
        return ble_advertise->addServiceUUID(ble_service->getUUID());
    }
    // Skip adding long UUIDs to adv; they remain discoverable after connection.
    return true;
}

bool BLE::advertise()
{
    // Start advertising TabletApp services and wait for app to connect
    if (_ble_server == nullptr)     return false;

    NimBLEAdvertising* ble_advertise = NimBLEDevice::getAdvertising();
    ble_advertise->enableScanResponse(true);
    return ble_advertise->start();
}
/*End of BLE Methods*/