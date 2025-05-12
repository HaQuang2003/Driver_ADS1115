#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <string.h>
#include <errno.h>

#define ADS1115_DEVICE "/dev/ADS1115"

#define ADS1115_IOCTL_READ_CONFIG _IOWR('a', 1, struct ADS1115_read_config)

/* Cấu trúc cấu hình ADC giống với bên kernel */
struct ADS1115_read_config {
    uint8_t channel;     // 0–7
    uint16_t pga;        // pgapga
    uint16_t mode;       // liên tục hoặc single-shot
    uint16_t datarate;   // tốc độ mẫu
    int16_t result;      // giá trị ADC raw
};

// Chuyển ADC raw → Volt 
float convert_to_voltage(int16_t raw_adc, float pga_voltage) {
    return ((float)raw_adc / 32767.0f) * pga_voltage;
}

// LM35: 10mV/°C = 0.01V/°C 
float voltage_to_temperature(float voltage) {
    return voltage / 0.01f;
}

int main() {
    int fd;
    struct ADS1115_read_config cfg;
    float voltage, temperature;

    // Mở thiết bị 
    fd = open(ADS1115_DEVICE, O_RDWR);
    if (fd < 0) {
        perror("Failed to open ADS1115 device");
        return 1;
    }

    // Cấu hình 
    cfg.channel = 1;                          // AIN0
    cfg.pga = 0x0400;                         // ±2.048V (ADS1115_PGA_2_048V)
    cfg.mode = 0x0000;                        // Chế độ liên tục (ADS1115_MODE_CONTINUOUS)
    cfg.datarate = 0x0080;                    // 128SPS (ADS1115_DR_128SPS)

    while (1) {
        if (ioctl(fd, ADS1115_IOCTL_READ_CONFIG, &cfg) < 0) {
            perror("IOCTL read failed");
            break;
        }

        voltage = convert_to_voltage(cfg.result, 2.048f);  // PGA là ±2.048V
        temperature = voltage_to_temperature(voltage);

        printf("ADC Raw: %d | Voltage: %.3f V | Temp: %.2f °C\n", cfg.result, voltage, temperature);

        usleep(500000); // delay 500ms
    }

    close(fd);
    return 0;
}
