#include <Arduino.h>
#include <common/mavlink.h>

#define HEARTBEAT_TIMEOUT 1000 // Heartbeat timeout in milliseconds

unsigned long lastHeartbeat = 0;
void checkForMavlinkMessages();
void setMessageRate(uint32_t msg_id, uint16_t message_rate_hz);
void requestMessageNow(uint32_t msg_id);


void setup()
{
  Serial.begin(9600);
  Serial2.begin(500000);
  setMessageRate(MAVLINK_MSG_ID_SCALED_IMU, 100);
  // Initialize other necessary setups
}

void loop()
{
  checkForMavlinkMessages();
}

void checkForMavlinkMessages()
{
  mavlink_message_t msg;
  mavlink_status_t status;

  if (Serial2.available() > 0)
  {
    uint8_t c = Serial2.read();
    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      // Handle a new message
      if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
      {
        Serial.println("hb");
        // Update last heartbeat time
        // Handle heartbeat message if necessary
      }
      else if (msg.msgid == MAVLINK_MSG_ID_SCALED_IMU)
      {
        // Handle SCALED_IMU message if necessary
        mavlink_scaled_imu_t imu;
        mavlink_msg_scaled_imu_decode(&msg, &imu);
        Serial.println(imu.zgyro);
      }
    }
  }
}

void requestMessageNow(uint32_t msg_id)
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the COMMAND_LONG message
    mavlink_msg_command_long_pack(
        1, // Your system ID
        0, // Your component ID
        &msg,
        1, // Target system ID
        0, // Target component ID
        MAV_CMD_REQUEST_MESSAGE, // Command ID
        0, // Confirmation
        msg_id, // Parameter 1: Message ID to request
        0, 0, 0, 0, 0, 0 // Unused parameters
    );

    // Serialize the message
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message over the serial interface
    Serial2.write(buf, len);
}

void setMessageRate(uint32_t msg_id, uint16_t message_rate_hz)
{
    uint32_t interval_us = 1000000 / message_rate_hz;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Pack the COMMAND_LONG message
    mavlink_msg_command_long_pack(
        1, // Your system ID
        0, // Your component ID
        &msg,
        1, // Target system ID
        0, // Target component ID
        MAV_CMD_SET_MESSAGE_INTERVAL , // Command ID
        0, // Confirmation
        msg_id, // Parameter 1: Message ID to request
        interval_us,  //interval
        0, 0, 0, 0, 0 // Unused parameters
    );

    // Serialize the message
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    // Send the message over the serial interface
    Serial2.write(buf, len);
}