#ifndef SCALE_CONFIG_H
#define SCALE_CONFIG_H

#define SCALE_FORCE_SENSOR_SAMPLE_FREQUENCY 1 // The esp will sample data with frequency of 1Hz

// Define ESP model
#define ESP_MODEL   "Smart Scale"

// Define scale model
#define SCALE_MODEL "MI SCALE2"

// Define MiScale2 MAC address (use this for reliable scanning/connection)
#define SCALE_MAC "70:87:9e:ea:3a:e0"

// Define scale service and characteristic UUIDs
#define WEIGHT_SERVICE_UUID     "181d"
#define WEIGHT_CHAR_UUID        "2a9d"

#define FORCE_SERVICE_UUID      "08a932dd-8b9e-45a6-b306-d0bd0986b9f6"
#define LEFT_CHAR_UUID          "ae5405a1-0e4b-4233-8df7-2f1a2e8793d1"
#define RIGHT_CHAR_UUID         "ae5405a1-0e4b-4233-8df7-2f1a2e8793d2"

#define QR_SERVICE_UUID         "d0f407df-ac96-440e-a5a4-ff86531775cf"
#define QR_CHAR_UUID            "317cbcaa-4207-4f9f-8f7f-1c14e7dcb9a7"

// Define the force sensor characteristic

// Number of force sensors to be used
#define NB_OF_FORCE_SENSORS 20
// Force-frame length: [START_BYTE][10 x 2-byte ADC][CRC] - Frame to be passed between master and slave, each handle 10 sensors
#define FORCE_FRAME_LEN     (1 + (NB_OF_FORCE_SENSORS / 2) * 2 + 1)

// Left foot sensors' pin
#define PIN_L1  1
#define PIN_L2  2
#define PIN_L3  3
#define PIN_L4  4
#define PIN_L5  6
#define PIN_L6  7
#define PIN_L7  8
#define PIN_L8  9
#define PIN_L9  10
#define PIN_L10 17
// Right foot sensors' pin
#define PIN_R1  -1
#define PIN_R2  -1
#define PIN_R3  -1
#define PIN_R4  -1
#define PIN_R5  -1
#define PIN_R6  -1
#define PIN_R7  -1
#define PIN_R8  -1
#define PIN_R9  -1
#define PIN_R10 -1

// Define LED pin
#define PIN_LED 48

// Define GM65 QR Scanner UART configuration
#define GM65_UART   2
#define GM65_RX_PIN 18
#define GM65_TX_PIN 21
#define GM65_BAUD   115200

// Define ESP Slave Communication UART configuration
#define ESP_SLAVE_UART   1
#define ESP_SLAVE_RX_PIN 38
#define ESP_SLAVE_TX_PIN 47
#define ESP_SLAVE_BAUD   115200

// static const double coefficients[8][3] = 
// {
//     {0.000000000399184, 0.0075004896, 0.6611337472},
//     {0.00000168757255, 0.0048290435, 0.5135031397},
//     {0.000000808534972, 0.0051556504, 0.5685645867},
//     {0.000042874426074, 0.0037457318, 0.4377670331},
//     {0.00000000348171, 0.0070260404, 0.6290049283},
//     {0.000000006663683, 0.0065223838, 0.6159364189},
//     {0.000002798378231, 0.0046492665, 0.4557127059},
//     {0.000000014241345, 0.0065633556, 0.6326569395}
// }; Old 220 Ohm

static const double coefficients[20][3] = 
{
    {0.000021371697445, 0.0053481007, 445.3563285368}, // L1
    {0.000000248176611, 0.0065162229, 757.2816045619},
    {0.000719860201035, 0.0044510120, 498.9688407556},
    {0.000000601681908, 0.0062887706, 668.9779842416},
    {0.000098152819274, 0.0052662315, 554.6524258285},
    {0.000056030506155, 0.0058924149, 704.0026281307},
    {0.000024852325814, 0.0053380905, 594.1884855287},
    {0.000291992545113, 0.0054068015, 543.0428109926},
    {0.011317313305444, 0.0043003174, 531.7289600894},
    {0.011034009785699, 0.0042794907, 499.6047240035}, // L10
    {0.000006084444986, 0.0057797375, 586.2208666368}, // R1
    {0.000007054436425, 0.0056012102, 721.7916753847},
    {0.000000041044454, 0.0070507735, 827.1322905522},
    {0.000000159442609, 0.0066791395, 769.9240003012},
    {0.000415937520261, 0.0051158558, 559.4077975833},
    {0.000236172500451, 0.0050267324, 470.6430723455},
    {0.000440398820481, 0.0052668556, 589.6309825653},
    {0.000285834229796, 0.0049295444, 583.6073306037},
    {0.000219138754404, 0.0051670381, 498.3750881804},
    {0.000561571962786, 0.0049850783, 497.1891494923} // R10
}; // New 470 Ohm

#endif /* SCALE_CONFIG_H */