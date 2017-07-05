#ifndef __XLI_NODECONFIG_H
#define __XLI_NODECONFIG_H

// Node config protocol
/// MySensor Cmd = C_INTERNAL
/// MySensor Type = I_CONFIG

/// Use MySensor Sensor as Node Config Field (NCF), and payload as config data
#define NCF_QUERY                       0       // Query NCF, payload length = 0 (query) or n (ack)
#define NCF_MAP_SENSOR                  1       // Sensor Bitmap, payload length = 2
#define NCF_MAP_FUNC                    2       // Function Bitmap, payload length = 2

#define NCF_DEV_ASSOCIATE               10      // Associate node to device(s), payload length = 2 to 8, a device per uint16_t
#define NCF_DEV_EN_SDTM                 11      // Simple Direct Test Mode flag, payload length = 2
#define NCF_DEV_MAX_NMRT                12      // Max. Node Message Repeat Times, payload length = 2
#define NCF_DEV_SET_SUBID               13      // Set device subid, payload length = 2
#define NCF_DEV_CONFIG_MODE             14      // Put Device into Config Mode, payload length = 2
#define NCF_DEV_SET_RELAY_NODE          15      // Set relay node id & subID, payload length = 2
#define NCF_DEV_SET_RELAY_KEYS          16      // Set relay keys, payload length = 2 to 4

#define NCF_PAN_SET_BTN_1               20      // Set Panel Button Action, payload length = 2
#define NCF_PAN_SET_BTN_2               21      // Set Panel Button Action, payload length = 2
#define NCF_PAN_SET_BTN_3               22      // Set Panel Button Action, payload length = 2
#define NCF_PAN_SET_BTN_4               23      // Set Panel Button Action, payload length = 2

#define NCF_DATA_ALS_RANGE              50      // Lower and upper threshholds of ALS, payload length = 2
#define NCF_DATA_TEMP_RANGE             51      // Tempreture threshholds, payload length = 2
#define NCF_DATA_HUM_RANGE              52      // Humidity threshholds, payload length = 2
#define NCF_DATA_PM25_RANGE             53      // PM2.5 threshholds, payload length = 2
#define NCF_DATA_PIR_RANGE              54      // PIR control brightness (off br, on br), payload length = 2
#define NCF_DATA_FN_SCENARIO            60      // Scenario ID for Remote Fn keys (b1=fn_id, b2=scenario_id), payload length = 2
#define NCF_DATA_FN_HUE                 61      // Hue for Remote Fn keys (b1_7-4=bmDevice, b1_3-0=fn_id, b2-11=Hue), payload length = 12

#define NCF_LEN_DATA_FN_HUE             12

typedef enum
{
  sensorDHT             = ((uint16_t)0x0001),
  sensorALS             = ((uint16_t)0x0002),
  sensorMIC             = ((uint16_t)0x0004),
  sensorVIBRATION       = ((uint16_t)0x0008),
  sensorPIR             = ((uint16_t)0x0010),
  sensorSMOKE           = ((uint16_t)0x0020),
  sensorGAS             = ((uint16_t)0x0040),
  sensorDUST            = ((uint16_t)0x0080),
  sensorLEAK            = ((uint16_t)0x0100),
  sensorBEAT            = ((uint16_t)0x0200),
  sensor_All            = ((uint8_t)0xFFFF)
}sensors_bit_t;

typedef enum
{
  controlPIR            = ((uint16_t)0x0001),   // Use PIR to control device on / off
  controlALS            = ((uint16_t)0x0002),   // Use ALS threshold to control device on / off
  constALS              = ((uint16_t)0x0004),   // Constant brightness level
  constTEMP             = ((uint16_t)0x0008),   // Constant tempreture
  constHUM              = ((uint16_t)0x0010),   // Constant humidity
  constPM25             = ((uint16_t)0x0020),   // Constant air quality
  constMIC              = ((uint16_t)0x0040),   // Use MIC threshold to control device on / off
  func_All              = ((uint8_t)0xFFFF)
}functions_bit_t;

#endif /* __XLI_NODECONFIG_H */
