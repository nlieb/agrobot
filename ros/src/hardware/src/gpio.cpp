#include <fcntl.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"

// Set the value of the requested GPIO pin
// Return: Success = 0 ; otherwise open file error
int gpio_set_value(int gpio, unsigned char* value) {
  int fd;
  char commandBuffer[64];
  char ch;

  snprintf(commandBuffer, sizeof(commandBuffer),
          SYSFS_GPIO_DIR "/gpio%d/value",
          gpio);

  // O_RDONLY, O_WRONLY, or O_RDWR.
  fd = open(commandBuffer, O_WRONLY);
  if (fd < 0) {
    char errorBuffer[128];
    snprintf(errorBuffer, sizeof(errorBuffer),
             "gpioGetValue unable to open gpio%d", gpio);
    perror(errorBuffer);
    return fd;
  }

  if (read(fd, &ch, 1) != 1) {
    perror("gpio_set_value");
    return fd;
  }

  if (ch != '0') {
    *value = 1;
  } else {
    *value = 0;
  }

  close(fd);
  return 0;
}


