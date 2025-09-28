#include "system/sensord/sensors/lsm6ds3_gyro.h"

#include <cassert>
#include <cmath>
#include <cstring>

#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"

#define DEG2RAD(x) ((x) * M_PI / 180.0)

LSM6DS3_Gyro::LSM6DS3_Gyro(I2CBus *bus) :
  I2CSensor(bus, 0, false) {}

void LSM6DS3_Gyro::wait_for_data_ready() {
  uint8_t drdy = 0;
  uint8_t buffer[6];

  do {
    read_register(LSM6DS3_GYRO_I2C_REG_STAT_REG, &drdy, sizeof(drdy));
    drdy &= LSM6DS3_GYRO_DRDY_GDA;
  } while (drdy == 0);

  read_register(LSM6DS3_GYRO_I2C_REG_OUTX_L_G, buffer, sizeof(buffer));
}

void LSM6DS3_Gyro::read_and_avg_data(float* out_buf) {
  uint8_t drdy = 0;
  uint8_t buffer[6];

  for (int i = 0; i < 5; i++) {
    do {
      read_register(LSM6DS3_GYRO_I2C_REG_STAT_REG, &drdy, sizeof(drdy));
      drdy &= LSM6DS3_GYRO_DRDY_GDA;
    } while (drdy == 0);

    int len = read_register(LSM6DS3_GYRO_I2C_REG_OUTX_L_G, buffer, sizeof(buffer));
    assert(len == sizeof(buffer));

    for (int j = 0; j < 3; j++) {
      out_buf[j] += (float)read_16_bit(buffer[j*2], buffer[j*2+1]) * 70.0f;
    }
  }

  // calculate the mg average values
  for (int i = 0; i < 3; i++) {
    out_buf[i] /= 5.0f;
  }
}

int LSM6DS3_Gyro::self_test(int test_type) {
  float val_st_off[3] = {0};
  float val_st_on[3] = {0};
  float test_val[3] = {0};

  // prepare sensor for self-test

  // full scale: 2000dps, ODR: 208Hz
  int ret = set_register(LSM6DS3_GYRO_I2C_REG_CTRL2_G, LSM6DS3_GYRO_ODR_208HZ | LSM6DS3_GYRO_FS_2000dps);
  if (ret < 0) {
    return ret;
  }

  // wait for stable output, and discard first values
  util::sleep_for(150);
  wait_for_data_ready();
  read_and_avg_data(val_st_off);

  // enable Self Test positive (or negative)
  ret = set_register(LSM6DS3_GYRO_I2C_REG_CTRL5_C, test_type);
  if (ret < 0) {
    return ret;
  }

  // wait for stable output, and discard first values
  util::sleep_for(50);
  wait_for_data_ready();
  read_and_avg_data(val_st_on);

  // disable sensor
  ret = set_register(LSM6DS3_GYRO_I2C_REG_CTRL2_G, 0);
  if (ret < 0) {
    return ret;
  }

  // disable self test
  ret = set_register(LSM6DS3_GYRO_I2C_REG_CTRL5_C, 0);
  if (ret < 0) {
    return ret;
  }

  // calculate the mg values for self test
  for (int i = 0; i < 3; i++) {
    test_val[i] = fabs(val_st_on[i] - val_st_off[i]);
  }

  // verify test result
  for (int i = 0; i < 3; i++) {
    if ((LSM6DS3_GYRO_MIN_ST_LIMIT_mdps > test_val[i]) ||
        (test_val[i] > LSM6DS3_GYRO_MAX_ST_LIMIT_mdps)) {
      return -1;
    }
  }

  return ret;
}

int LSM6DS3_Gyro::init() {
  bool do_self_test = false;

  const char* env_lsm_selftest = std::getenv("LSM_SELF_TEST");
  if (env_lsm_selftest != nullptr && strncmp(env_lsm_selftest, "1", 1) == 0) {
    do_self_test = true;
  }

  int ret = verify_chip_id(LSM6DS3_GYRO_I2C_REG_ID, {LSM6DS3_GYRO_CHIP_ID, LSM6DS3TRC_GYRO_CHIP_ID});
  if (ret == -1) return -1;

  if (ret == LSM6DS3TRC_GYRO_CHIP_ID) {
    source = cereal::SensorEventData::SensorSource::LSM6DS3TRC;
  }

  ret = self_test(LSM6DS3_GYRO_POSITIVE_TEST);
  if (ret < 0) {
    LOGE("LSM6DS3 gyro positive self-test failed!");
    if (do_self_test) goto fail;
  }

  ret = self_test(LSM6DS3_GYRO_NEGATIVE_TEST);
  if (ret < 0) {
    LOGE("LSM6DS3 gyro negative self-test failed!");
    if (do_self_test) goto fail;
  }

  // Disable interrupts for polling mode (since GPIO is damaged)
  ret = set_register(LSM6DS3_GYRO_I2C_REG_INT1_CTRL, 0x00);
  if (ret < 0) {
    goto fail;
  }

  // Enable BDU and IF_INC like accelerometer for consistency
  ret = set_register(0x12, 0x44);  // CTRL3_C: BDU=1, IF_INC=1
  if (ret < 0) {
    goto fail;
  }

  // Optimized configuration: 208Hz ODR with ±500dps scale for better performance
  // CTRL2_G: ODR=208Hz (0101), FS=±500dps (01)
  ret = set_register(LSM6DS3_GYRO_I2C_REG_CTRL2_G, 0x54);  // 208Hz, ±500dps
  if (ret < 0) {
    goto fail;
  }

  // Enable gyroscope low-pass filter for noise reduction
  // CTRL7_G: HPM=00, HP_EN=0, LP_EN=1
  ret = set_register(0x16, 0x80);  // Enable LPF
  if (ret < 0) {
    goto fail;
  }

fail:
  return ret;
}

int LSM6DS3_Gyro::shutdown() {
  int ret = 0;

  // enable power-down mode
  uint8_t value = 0;
  ret = read_register(LSM6DS3_GYRO_I2C_REG_CTRL2_G, &value, 1);
  if (ret < 0) {
    goto fail;
  }

  value &= 0x0F;
  ret = set_register(LSM6DS3_GYRO_I2C_REG_CTRL2_G, value);
  if (ret < 0) {
    LOGE("Could not power-down lsm6ds3 gyroscope!");
    goto fail;
  }

fail:
  return ret;
}

bool LSM6DS3_Gyro::get_event(MessageBuilder &msg, uint64_t ts) {

  uint8_t status = 0;
  int len = read_register(LSM6DS3_GYRO_I2C_REG_STAT_REG, &status, sizeof(status));
  if (len < 0 || !(status & LSM6DS3_GYRO_DRDY_GDA)) {
    return false; // No new data available
  }

  uint8_t buffer[6];
  len = read_register(LSM6DS3_GYRO_I2C_REG_OUTX_L_G, buffer, sizeof(buffer));
  assert(len == sizeof(buffer));

  // For ±500 deg/s: sensitivity is 17.50 mdps/LSB
  float scale = 17.50 / 1000.0; // Convert mdps to deg/s
  float x = DEG2RAD(read_16_bit(buffer[0], buffer[1]) * scale);
  float y = DEG2RAD(read_16_bit(buffer[2], buffer[3]) * scale);
  float z = DEG2RAD(read_16_bit(buffer[4], buffer[5]) * scale);

  auto event = msg.initEvent().initGyroscope();
  event.setSource(source);
  event.setVersion(2);
  event.setSensor(SENSOR_GYRO_UNCALIBRATED);
  event.setType(SENSOR_TYPE_GYROSCOPE_UNCALIBRATED);
  event.setTimestamp(ts);

  float xyz[] = {y, -x, z};
  auto svec = event.initGyroUncalibrated();
  svec.setV(xyz);
  svec.setStatus(true);

  return true;
}
