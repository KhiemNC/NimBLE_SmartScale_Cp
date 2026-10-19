#include <Arduino.h>

#include "esp_log.h"
#include <NimBLEDevice.h>

#define BLE_TAG "Scale BLE"

#define BLE_MAX_CONNECTIONS     3

#define MAX_SERVICES            20
#define MAX_CHARACTERISTICS     30

enum BLEError {BLE_OK};
enum BLEDataType {BLE_FORMAT_UNKNOWN, BLE_FORMAT_SINT32, BLE_FORMAT_UINT8, BLE_FORMAT_UINT32, BLE_FORMAT_FLOAT32, BLE_FORMAT_FLOAT64, BLE_FORMAT_STRING};
enum BLECharMode {BLE_INDICATE = INDICATE, BLE_NOTIFY = NOTIFY};

class BLE {
private:
    NimBLEClient    * _ble_client;
    NimBLEServer    * _ble_server;

    static bool      _do_connect;
    static String    _target_address;
    static const NimBLEAdvertisedDevice * _target_advertise_device;

    typedef void (* BLECallbackFunction) (String, unsigned char *, size_t length);
    static BLECallbackFunction _upperlayer_callback_fp;

    NimBLEAdvertising * _server_advertise_object = nullptr;
public:
    class ClientCallbacks : public NimBLEClientCallbacks {
        void onConnect(NimBLEClient * pClient) override;
        void onDisconnect(NimBLEClient * pClient, int reason) override;  
    } client_callback;

    class ServerCallbacks : public NimBLEServerCallbacks {
        void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override;
        void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override;
    } server_callback;

    class ScanCallbacks : public NimBLEScanCallbacks {
        void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override;
        void onScanEnd(const NimBLEScanResults& results, int reason) override;
    } scan_callback;

    static void indicateCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);

    bool init (String name);         
    bool scan (String device_address);
    bool isScanFound();
    void resetScan();  // Reset scan state to prepare for new scan
    bool connect();
    bool isConnected();
    bool registerForIndicate(String service_uuid, String characteristic_uuid, BLECallbackFunction callback_func);
    
    bool createService(String service_uuid);
    bool createCharacteristic(String service_uuid, String characteristic_uuid, BLECharMode mode = BLE_INDICATE, BLEDataType type = BLE_FORMAT_UNKNOWN);
    bool startService(String service_uuid);
    bool advertise();

    template <typename T>
    bool setValue(String service_uuid, String characteristic_uuid, T value, int size = -1)
    {
        if (_ble_server == nullptr) return false;

        NimBLEService * ble_service = _ble_server->getServiceByUUID(NimBLEUUID(service_uuid.c_str()));
        if (ble_service == nullptr)
            return false;

        NimBLECharacteristic * ble_characteristic = ble_service->getCharacteristic(characteristic_uuid.c_str());
        if (ble_characteristic == nullptr)
            return false;

        if constexpr (std::is_same<T, String>::value) {
            ble_characteristic->setValue(value.c_str());
        } else if constexpr (std::is_same<T, std::string>::value) {
            ble_characteristic->setValue(value);
        } else if constexpr (std::is_same<T, int>::value) {
            ble_characteristic->setValue(value);
        } else if constexpr (std::is_same<T, float>::value) {
            ble_characteristic->setValue(value);
        } else if constexpr (std::is_same<T, double>::value) {
            ble_characteristic->setValue(value);
        } else if constexpr (std::is_same<T, uint8_t>::value) {
            if (size == -1)
                ble_characteristic->setValue(value);
            else
                ble_characteristic->setValue(value, size);
        } else if constexpr (std::is_same<T, uint8_t *>::value) {
            ble_characteristic->setValue(value, size);
        } else if constexpr (std::is_same<T, uint16_t>::value) {
            ble_characteristic->setValue(value);
        } else if constexpr (std::is_same<T, uint32_t>::value) {
            ble_characteristic->setValue(value);
        } else {
            return false;
        }
        
        ble_characteristic->notify();
        return true;
    }
};