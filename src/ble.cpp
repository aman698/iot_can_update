#include "ble.h"
#include "vehiclecontrol.h"

#define SERVICE_UUID "751e2651-42eb-46a1-be00-069dddaf953e"
#define CHARACTERISTIC_UUID "752e2651-42eb-46a1-be00-069dddaf953e"

#define COMMAND 0x01
#define PAYLOAD 0x02

#define IGNITION_COMMAND 0x01

BLEServer *pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        Serial.println("Connected");
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("Disconnected");
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        uint8_t *data;
        if (pCharacteristic->getLength() == 4)
        {
            data = pCharacteristic->getData();

            switch (data[COMMAND])
            {
            case IGNITION_COMMAND:
                setIgnitionLock(!data[PAYLOAD]);
                pCharacteristic->indicate();
                pCharacteristic->notify();
                break;
            }
        }
    }
};

void BLEInit()
{
    BLEDevice::init("MZ-01");
    BLEDevice::setMTU(256);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);

    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->setValue("Waiting for a command....");
    pCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEOTA.begin(pServer);
    BLEOTA.init();

    BLEAdvertising *pAdvertising = pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLEOTA.getBLEOTAuuid());
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
}

bool BLELoop()
{
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500);                  // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        oldDeviceConnected = deviceConnected;
    }
    BLEOTA.process();

    return BLEOTA.isRunning();
}