void DFRobot_BloodOxygen_S::getHeartbeatSPO2(void)
{
  uint8_t SPO2Valid;
  int8_t HeartbeatValid;
  uint8_t rbuf[8];
  readReg(0x0C,rbuf,8);
  static uint8_t heartbeat_count;
  uint32_t SPO2_all_val=0;
  uint32_t heartbeat_all_val=0;
  _sHeartbeatSPO2.SPO2 = rbuf[0];
  if(_sHeartbeatSPO2.SPO2 == 0)
  {
    _sHeartbeatSPO2.SPO2 = -1;
  }
  _sHeartbeatSPO2.Heartbeat = ((uint32_t)rbuf[2] << 24) | ((uint32_t)rbuf[3] << 16) | ((uint32_t)rbuf[4] << 8) | ((uint32_t)rbuf[5]);
  if (_sHeartbeatSPO2.Heartbeat == 0)
  {
    _sHeartbeatSPO2.Heartbeat = -1;
  }
}

float DFRobot_BloodOxygen_S::getTemperature_C(void)
{
  uint8_t temp_buf[2];
  readReg(0x14, temp_buf, 2);
  float Temperature = temp_buf[0] * 1.0 + temp_buf[1] / 100.0;
  return Temperature;
}

void DFRobot_BloodOxygen_S::setBautrate(ebautrate bautrate)
{
  uint8_t w_buf[2];
  w_buf[0] = (uint8_t)(bautrate >> 8);
  w_buf[1] = (uint8_t)bautrate;
  writeReg(0x06, w_buf, sizeof(w_buf));
  delay(100);
  w_buf[0] = 0x00;
  w_buf[1] = 0x01;
  writeReg(0x1A, w_buf, sizeof(w_buf));
  delay(1000);
}

uint32_t DFRobot_BloodOxygen_S::getBautrate(void)
{
  uint8_t r_buf[2];
  readReg(0x06, r_buf, sizeof(r_buf));
  uint16_t baudrate_type = (uint16_t)r_buf[0] << 8 | (uint16_t)r_buf[1];
  switch (baudrate_type)
  {
    case 0:
      return 1200;
      break;
    case 1:
      return 2400;
      break;
    case 3:
      return 9600;
      break;
    case 5:
      return 19200;
      break;
    case 6:
      return 38400;
      break;
    case 7:
      return 57600;
      break;
    case 8:
      return 115200;
      break;
    default:
      return 9600;
      break;
  }
}
