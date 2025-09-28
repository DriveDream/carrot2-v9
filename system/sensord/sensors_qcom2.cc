#include <sys/resource.h>

#include <chrono>
#include <thread>
#include <vector>
#include <map>
#include <poll.h>
#include <linux/gpio.h>

#include "cereal/services.h"
#include "cereal/messaging/messaging.h"
#include "common/i2c.h"
#include "common/ratekeeper.h"
#include "common/swaglog.h"
#include "common/timing.h"
#include "common/util.h"
#include "system/sensord/sensors/bmx055_accel.h"
#include "system/sensord/sensors/bmx055_gyro.h"
#include "system/sensord/sensors/bmx055_magn.h"
#include "system/sensord/sensors/bmx055_temp.h"
#include "system/sensord/sensors/constants.h"
#include "system/sensord/sensors/lsm6ds3_accel.h"
#include "system/sensord/sensors/lsm6ds3_gyro.h"
#include "system/sensord/sensors/lsm6ds3_temp.h"
#include "system/sensord/sensors/mmc5603nj_magn.h"

#define I2C_BUS_IMU 1

ExitHandler do_exit;

void interrupt_loop(std::vector<std::tuple<Sensor *, std::string>> sensors) {
  // Collect message names for interrupt-enabled sensors only
  std::vector<const char *> interrupt_msg_names;
  for (auto &[sensor, msg_name] : sensors) {
    if (sensor->has_interrupt_enabled()) {
      interrupt_msg_names.push_back(msg_name.c_str());
    }
  }

  if (interrupt_msg_names.empty()) {
    return; // No interrupt sensors to handle
  }

  PubMaster pm(interrupt_msg_names);

  int fd = -1;
  for (auto &[sensor, msg_name] : sensors) {
    if (sensor->has_interrupt_enabled()) {
      fd = sensor->gpio_fd;
      break;
    }
  }

  uint64_t offset = nanos_since_epoch() - nanos_since_boot();
  struct pollfd fd_list[1] = {0};
  fd_list[0].fd = fd;
  fd_list[0].events = POLLIN | POLLPRI;

  while (!do_exit) {
    int err = poll(fd_list, 1, 100);
    if (err == -1) {
      if (errno == EINTR) {
        continue;
      }
      return;
    } else if (err == 0) {
      LOGE("poll timed out");
      continue;
    }

    if ((fd_list[0].revents & (POLLIN | POLLPRI)) == 0) {
      LOGE("no poll events set");
      continue;
    }

    // Read all events
    struct gpioevent_data evdata[16];
    err = HANDLE_EINTR(read(fd, evdata, sizeof(evdata)));
    if (err < 0 || err % sizeof(*evdata) != 0) {
      LOGE("error reading event data %d", err);
      continue;
    }

    uint64_t cur_offset = nanos_since_epoch() - nanos_since_boot();
    uint64_t diff = cur_offset > offset ? cur_offset - offset : offset - cur_offset;
    if (diff > 10*1e6) { // 10ms
      LOGW("time jumped: %lu %lu", cur_offset, offset);
      offset = cur_offset;

      // we don't have a valid timestamp since the
      // time jumped, so throw out this measurement.
      continue;
    }

    int num_events = err / sizeof(*evdata);
    uint64_t ts = evdata[num_events - 1].timestamp - cur_offset;

    for (auto &[sensor, msg_name] : sensors) {
      if (!sensor->has_interrupt_enabled()) {
        continue;
      }

      MessageBuilder msg;
      if (!sensor->get_event(msg, ts)) {
        continue;
      }

      if (!sensor->is_data_valid(ts)) {
        continue;
      }

      pm.send(msg_name.c_str(), msg);
    }
  }
}

void polling_loop(Sensor *sensor, std::string msg_name) {
  PubMaster pm({msg_name.c_str()});
  RateKeeper rk(msg_name, services.at(msg_name).frequency);
  while (!do_exit) {
    MessageBuilder msg;
    uint64_t current_ts = nanos_since_boot();
    if (sensor->get_event(msg, current_ts) && sensor->is_data_valid(current_ts)) {
      pm.send(msg_name.c_str(), msg);
    }
    rk.keepTime();
  }
}

int sensor_loop(I2CBus *i2c_bus_imu) {
  // Sensor init - try LSM6DS3 first, fallback to BMX055 if failed
  std::vector<std::tuple<Sensor *, std::string>> sensors_init;

  // Try LSM6DS3 sensors first
  LSM6DS3_Accel *lsm_accel = new LSM6DS3_Accel(i2c_bus_imu);
  LSM6DS3_Gyro *lsm_gyro = new LSM6DS3_Gyro(i2c_bus_imu);
  LSM6DS3_Temp *lsm_temp = new LSM6DS3_Temp(i2c_bus_imu);

  bool lsm_accel_ok = (lsm_accel->init() >= 0);
  bool lsm_gyro_ok = (lsm_gyro->init() >= 0);
  bool lsm_temp_ok = (lsm_temp->init() >= 0);

  if (lsm_accel_ok) {
    sensors_init.push_back({lsm_accel, "accelerometer"});
  } else {
    delete lsm_accel;
    // Fallback to BMX055 accelerometer
    BMX055_Accel *bmx_accel = new BMX055_Accel(i2c_bus_imu);
    if (bmx_accel->init() >= 0) {
      sensors_init.push_back({bmx_accel, "accelerometer2"});
    } else {
      delete bmx_accel;
    }
  }

  if (lsm_gyro_ok) {
    sensors_init.push_back({lsm_gyro, "gyroscope"});
  } else {
    delete lsm_gyro;
    // Fallback to BMX055 gyroscope
    BMX055_Gyro *bmx_gyro = new BMX055_Gyro(i2c_bus_imu);
    if (bmx_gyro->init() >= 0) {
      sensors_init.push_back({bmx_gyro, "gyroscope2"});
    } else {
      delete bmx_gyro;
    }
  }

  if (lsm_temp_ok) {
    sensors_init.push_back({lsm_temp, "temperatureSensor"});
  } else {
    delete lsm_temp;
    // Fallback to BMX055 temperature
    BMX055_Temp *bmx_temp = new BMX055_Temp(i2c_bus_imu);
    if (bmx_temp->init() >= 0) {
      sensors_init.push_back({bmx_temp, "temperatureSensor2"});
    } else {
      delete bmx_temp;
    }
  }

  // Always try magnetometer (only available in BMX055 and MMC5603NJ)
  BMX055_Magn *bmx_magn = new BMX055_Magn(i2c_bus_imu);
  if (bmx_magn->init() >= 0) {
    sensors_init.push_back({bmx_magn, "magnetometer"});
  } else {
    delete bmx_magn;
    // Try MMC5603NJ magnetometer
    MMC5603NJ_Magn *mmc_magn = new MMC5603NJ_Magn(i2c_bus_imu);
    if (mmc_magn->init() >= 0) {
      sensors_init.push_back({mmc_magn, "magnetometer"});
    } else {
      delete mmc_magn;
    }
  }

  // Start polling threads for sensors (already initialized above)
  std::vector<std::thread> threads;
  for (auto &[sensor, msg_name] : sensors_init) {
    if (!sensor->has_interrupt_enabled()) {
      threads.emplace_back(polling_loop, sensor, msg_name);
    }
  }

  // increase interrupt quality by pinning interrupt and process to core 1
  setpriority(PRIO_PROCESS, 0, -18);
  util::set_core_affinity({1});

  // TODO: get the IRQ number from gpiochip
  std::string irq_path = "/proc/irq/336/smp_affinity_list";
  if (!util::file_exists(irq_path)) {
    irq_path = "/proc/irq/335/smp_affinity_list";
  }
  std::system(util::string_format("sudo su -c 'echo 1 > %s'", irq_path.c_str()).c_str());

  // thread for reading events via interrupts (only if there are interrupt-enabled sensors)
  bool has_interrupt_sensors = false;
  for (auto &[sensor, msg_name] : sensors_init) {
    if (sensor->has_interrupt_enabled()) {
      has_interrupt_sensors = true;
      break;
    }
  }

  if (has_interrupt_sensors) {
    threads.emplace_back(&interrupt_loop, std::ref(sensors_init));
  }

  // wait for all threads to finish
  for (auto &t : threads) {
    t.join();
  }

  for (auto &[sensor, msg_name] : sensors_init) {
    sensor->shutdown();
    delete sensor;
  }
  return 0;
}

int main(int argc, char *argv[]) {
  try {
    auto i2c_bus_imu = std::make_unique<I2CBus>(I2C_BUS_IMU);
    return sensor_loop(i2c_bus_imu.get());
  } catch (std::exception &e) {
    LOGE("I2CBus init failed");
    return -1;
  }
}
