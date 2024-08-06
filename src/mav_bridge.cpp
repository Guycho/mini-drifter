#include "mav_bridge.h"

MavBridge::MavBridge()
{
}
MavBridge::~MavBridge()
{
}
void MavBridge::init(HardwareSerial *serial, uint32_t baudrate, uint8_t system_id, uint8_t component_id, uint8_t steering_channel, uint8_t throttle_channel, uint8_t message_rate)
{
  m_serial = serial;
  m_baudrate = baudrate;
  m_system_id = system_id;
  m_component_id = component_id;
  m_steering_channel = steering_channel;
  m_throttle_channel = throttle_channel;
  m_message_rate = message_rate;

  m_serial->begin(m_baudrate);
  set_messages_rates();
  // Initialize other necessary setups
}

void MavBridge::run()
{
  mavlink_message_t msg;
  mavlink_status_t status;


  while (m_serial->available() > 0)
  {
    uint8_t c = Serial2.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      if (msg.msgid == MAVLINK_MSG_ID_SCALED_IMU)
      {
        mavlink_scaled_imu_t imu;
        mavlink_msg_scaled_imu_decode(&msg, &imu);
        m_gyro_data = utils::calcs::milli_to_single(imu.zgyro);
      }
    }
  }
}

void MavBridge::set_steering(float steering_pct)
{
    m_steering_pct = steering_pct;
    // Convert the steering percentage to a PWM value
    uint16_t pwm = map(steering_pct, -100, 100, 1000, 2000);
    // Set the steering servo
    set_servo(m_steering_channel, pwm);

}

void MavBridge::set_throttle(float throttle_pct)
{
  m_throttle_pct = throttle_pct;
  // Convert the throttle percentage to a PWM value
  uint16_t pwm = map(throttle_pct, -100, 100, 1000, 2000);

  // Set the throttle servo
  set_servo(m_throttle_channel, pwm);
}

float MavBridge::get_gyro_data()
{
  return m_gyro_data;
}

float MavBridge::get_steering_pct()
{
  return m_steering_pct;
}
float MavBridge::get_throttle_pct()
{
  return m_throttle_pct;
}
void MavBridge::set_messages_rates()
{
  set_message_rate(MAVLINK_MSG_ID_SCALED_IMU, m_message_rate);
}

void MavBridge::set_message_rate(uint32_t msg_id, uint16_t message_rate_hz)
{
  uint32_t interval_us = 1e6 / message_rate_hz;
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the COMMAND_LONG message
  mavlink_msg_command_long_pack(
      m_system_id,    // Your system ID
      m_component_id, // Your component ID
      &msg,
      m_system_id,                  // Target system ID
      m_component_id,               // Target component ID
      MAV_CMD_SET_MESSAGE_INTERVAL, // Command ID
      0,                            // Confirmation
      msg_id,                       // Parameter 1: Message ID to request
      interval_us,                  // interval
      0, 0, 0, 0, 0                 // Unused parameters
  );

  // Serialize the message
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message over the serial interface
  m_serial->write(buf, len);
}

void MavBridge::set_servo(uint8_t channel, uint16_t pwm)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the COMMAND_LONG message
  mavlink_msg_command_long_pack(
      m_system_id,    // Your system ID
      m_component_id, // Your component ID
      &msg,
      m_system_id,          // Target system ID
      m_component_id,       // Target component ID
      MAV_CMD_DO_SET_SERVO, // Command ID
      0,                    // Confirmation
      channel,              // Parameter 1: Message ID to request
      pwm,                  // interval
      0, 0, 0, 0, 0         // Unused parameters
  );
  // Serialize the message
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // Send the message over the serial interface
  m_serial->write(buf, len);
}
