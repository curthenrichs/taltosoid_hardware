/**
 * Supernumerary Robotic Finger
 * Version 1.1.5
 * @author Curt Henrichs
 * @date 4-19-19
 *
 * Firmware (main) file.
 */

//==============================================================================
//  Libraries
//==============================================================================

#include "HardwareConfig.h"
#include "FingerDriver.h"
#include "FlexSensors.h"

#include <ArduinoJson.h>
#include <elapsedMillis.h>

//==============================================================================
//  Constants and Macro Definitions
//==============================================================================

#define API_RX_BUFFER_SIZE (250)

#define API_MSG_DELIMITER '\0'

#define FLEX_SENSOR_SAMPLE_TIMER_THRESHOLD (50)
#define API_DATA_PUSH_TIMER (100)

#define SEND_JSON(json, device) do { serializeJson(json, device); device.print(API_MSG_DELIMITER); } while(0)

//==============================================================================
//  Private Data Members
//==============================================================================

static elapsedMillis flexSensorSampleTimer;
static elapsedMillis apiDataPushTimer;

static char _rx_buffer[API_RX_BUFFER_SIZE+1];
static int _rx_buffer_index = 0;
static bool _rx_buffer_contains_msg = false;

//==============================================================================
//  Private Function Prototypes
//==============================================================================

static void _api_push_msg(void);
static void _api_handle_set_joint_request(int id, int angle);
static void _api_handle_get_joint_request(int id);
static void _api_handle_flex_sensor_request(int id);
static void _api_handle_metadata_request(void);

static char* _sensor_index_to_id_str(int index);
static int _sensor_id_str_to_index(const char* str);
static char* _joint_index_to_id_str(int index);
static int _joint_id_str_to_index(const char* str);

static bool firstIteration = true;

//==============================================================================
//  Main
//==============================================================================

void setup(void) {
  flexSensorSampleTimer = 0;
  apiDataPushTimer = 0;

  _rx_buffer[API_RX_BUFFER_SIZE] = '\0';
  _rx_buffer_index = 0;
  _rx_buffer_contains_msg = false;

  hw_init();
  fd_init();
}

void loop(void) {

  // On first iteration, correct joints of finger
  if (firstIteration) {
    firstIteration = false;
    fd_reset_all_joint_states();
  }

  // process character string from serial, connected to external device
  while (API_SERIAL.available() && !_rx_buffer_contains_msg) {

    char c = API_SERIAL.read();

    if (c == API_MSG_DELIMITER) {
      _rx_buffer[_rx_buffer_index] = '\0';
      _rx_buffer_contains_msg = true;
    } else {
      _rx_buffer[_rx_buffer_index] = c;
    }
    _rx_buffer_index++;

    if (_rx_buffer_index >= API_RX_BUFFER_SIZE && !_rx_buffer_contains_msg) {
      _rx_buffer_index = 0;
    }
  }

  // handle API requests from external device
  if (_rx_buffer_contains_msg) {
    _rx_buffer_contains_msg = false;
    _rx_buffer_index = 0;

    // convert to JSON document
    StaticJsonDocument<300> rxDoc;
    deserializeJson(rxDoc,(const char*)(_rx_buffer));

    const char* requestType = rxDoc["request_type"];
    if (requestType != NULL) {
      if (strcmp(requestType,"joint_get") == 0) {
        int id = _joint_id_str_to_index(rxDoc["joint_id"]);
        _api_handle_get_joint_request(id);
      } else if (strcmp(rxDoc["request_type"],"joint_set") == 0) {
        int id = _joint_id_str_to_index(rxDoc["joint_id"]);
        if (rxDoc.containsKey("angle")) {
          int angle = rxDoc["angle"];
          _api_handle_set_joint_request(id,angle);
        } else {
          StaticJsonDocument<500> txDoc;
          txDoc["error"] = true;
          txDoc["error_msg"] = "missing 'angle' attribute";
          txDoc["buffer"] = _rx_buffer;
          SEND_JSON(txDoc, API_SERIAL);
        }
      } else if (strcmp(rxDoc["request_type"],"flex_sensor_get") == 0) {
        int id = _sensor_id_str_to_index(rxDoc["flex_id"]);
        _api_handle_flex_sensor_request(id);
      } else if (strcmp(rxDoc["request_type"],"metadata_get") == 0) {
        _api_handle_metadata_request();
      } else if (strcmp(rxDoc["request_type"],"force_push_msg") == 0) {
        _api_push_msg();
      } else {
        StaticJsonDocument<500> txDoc;
        txDoc["error"] = true;
        txDoc["error_msg"] = "invalid type";
        txDoc["buffer"] = _rx_buffer;
        SEND_JSON(txDoc, API_SERIAL);
      }
    } else{
      StaticJsonDocument<500> txDoc;
        txDoc["error"] = true;
        txDoc["error_msg"] = "request type not specified";
        txDoc["buffer"] = _rx_buffer;
        SEND_JSON(txDoc, API_SERIAL);
    }
  }

  // Sample sensors
  if (flexSensorSampleTimer >= FLEX_SENSOR_SAMPLE_TIMER_THRESHOLD) {
    flexSensorSampleTimer -= FLEX_SENSOR_SAMPLE_TIMER_THRESHOLD;
    fx_update();
  }

  // Push data to external device
  if (apiDataPushTimer >= API_DATA_PUSH_TIMER) {
    apiDataPushTimer -= API_DATA_PUSH_TIMER;
    _api_push_msg();
  }

  delay(10);
}

//==============================================================================
//  Private Function Implementation
//==============================================================================

static void _api_push_msg(void) {

  StaticJsonDocument<1000> doc;

  doc["message_type"] = "state_push";

  // read flex sensors
  for (int i=0; i<__NUM_FLEX_SENSORS__; i++) {
    doc[_sensor_index_to_id_str(i)] = fx_get((FlexSensorId_t)(i));
  }

  // read current joint states
  for (int i=0; i<__NUM_JOINTS__; i++) {
    doc[_joint_index_to_id_str(i)] = fd_get_joint_state((JointId_t)(i));
  }

  SEND_JSON(doc, API_SERIAL);
}

static void _api_handle_set_joint_request(int id, int angle) {

  StaticJsonDocument<100> doc;

  doc["joint_id"] = _joint_index_to_id_str(id);
  doc["result"] = fd_set_joint_state(((JointId_t)(id)),angle);

  SEND_JSON(doc, API_SERIAL);
}

static void _api_handle_get_joint_request(int id) {

  StaticJsonDocument<100> doc;

  doc["joint_id"] = _joint_index_to_id_str(id);
  doc["result"] = fd_get_joint_state((JointId_t)(id));

  SEND_JSON(doc, API_SERIAL);
}

static void _api_handle_flex_sensor_request(int id) {

  StaticJsonDocument<100> doc;

  doc["flex_id"] = _sensor_index_to_id_str(id);
  doc["result"] = fx_get(((FlexSensorId_t)(id)));

  SEND_JSON(doc, API_SERIAL);

}

static void _api_handle_metadata_request(void) {

  char chipIdStr[64];
  uint64_t chipid = ESP.getEfuseMac();
  sprintf(chipIdStr,"%04X%08X",(uint16_t)(chipid>>32),(uint32_t)(chipid));

  StaticJsonDocument<300> doc;

  doc["msg_type"] = "metadata";
  doc["chip_id"] = chipIdStr;
  doc["software_id"] = VERSION_STR;

  SEND_JSON(doc, API_SERIAL);
}

static char* _sensor_index_to_id_str(int index) {
  if (index == FLEX_0_0) {
    return "flex_0_0";
  } else if (index == FLEX_0_1) {
    return "flex_0_1";
  } else if (index == FLEX_1_0) {
    return "flex_1_0";
  } else if (index == FLEX_1_1) {
    return "flex_1_1";
  } else if (index == FLEX_2_0) {
    return "flex_2_0";
  } else if (index == FLEX_2_1) {
    return "flex_2_1";
  } else if (index == FLEX_3_0) {
    return "flex_3_0";
  } else if (index == FLEX_3_1) {
    return "flex_3_1";
  } else if (index == FLEX_4_0) {
    return "flex_4_0";
  } else if (index == FLEX_4_1) {
    return "flex_4_1";
  } else if (index == FLEX_5_0) {
    return "flex_5_0";
  } else if (index == FLEX_6_0) {
    return "flex_6_0";
  } else {
    return "invalid";
  }
}

static int _sensor_id_str_to_index(const char* str) {
  if (str != NULL) {
    if (strcmp(str,"flex_0_0") == 0) {
      return FLEX_0_0;
    } else if (strcmp(str,"flex_0_1") == 0) {
      return FLEX_0_1;
    } else if (strcmp(str,"flex_1_0") == 0) {
      return FLEX_1_0;
    } else if (strcmp(str,"flex_1_1") == 0) {
      return FLEX_1_1;
    } else if (strcmp(str,"flex_2_0") == 0) {
      return FLEX_2_0;
    } else if (strcmp(str,"flex_2_1") == 0) {
      return FLEX_2_1;
    } else if (strcmp(str,"flex_3_0") == 0) {
      return FLEX_3_0;
    } else if (strcmp(str,"flex_3_1") == 0) {
      return FLEX_3_1;
    } else if (strcmp(str,"flex_4_0") == 0) {
      return FLEX_4_0;
    } else if (strcmp(str,"flex_4_1") == 0) {
      return FLEX_4_1;
    } else if (strcmp(str,"flex_5_0") == 0) {
      return FLEX_5_0;
    } else if (strcmp(str,"flex_6_0") == 0) {
      return FLEX_6_0;
    } else {
      return -1;
    }
  } else {
    return -1;
  }
}

static char* _joint_index_to_id_str(int index) {
  if (index == JOINT_0) {
    return "joint_0";
  } else if (index == JOINT_1) {
    return "joint_1";
  } else if (index == JOINT_2) {
    return "joint_2";
  } else if (index == JOINT_3) {
    return "joint_3";
  } else {
    return "invalid";
  }
}

static int _joint_id_str_to_index(const char* str) {
  if (str != NULL) {
    if (strcmp(str,"joint_0") == 0) {
      return JOINT_0;
    } else if (strcmp(str,"joint_1") == 0) {
      return JOINT_1;
    } else if (strcmp(str,"joint_2") == 0) {
      return JOINT_2;
    } else if (strcmp(str,"joint_3") == 0) {
      return JOINT_3;
    } else {
      return -1;
    }
  } else {
    return -1;
  }
}
