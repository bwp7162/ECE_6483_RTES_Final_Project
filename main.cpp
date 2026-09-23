#include "mbed.h"
#include "arm_math.h"

BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int) {
    return &serial_port;
}

DigitalOut led1(LED1); // Onboard LEDs for indication
DigitalOut led2(LED2); 
I2C i2c(PB_11, PB_10);

#define LSM6DSL_ADDR (0x6A << 1)
#define WHO_AM_I    0x0F
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define OUTX_L_XL   0x28
#define OUTX_H_XL   0x29
#define OUTY_L_XL   0x2A
#define OUTY_H_XL   0x2B
#define OUTZ_L_XL   0x2C
#define OUTZ_H_XL   0x2D

#define FFT_SIZE 512
#define SAMPLE_RATE 104.0f
#define RESOLUTION (SAMPLE_RATE / FFT_SIZE)

float32_t input_data[FFT_SIZE];
float32_t fft_out[FFT_SIZE];
float32_t magnitude[FFT_SIZE / 2];

arm_rfft_fast_instance_f32 FFT_Instance; 

// Write a single register
void write_register(uint8_t reg, uint8_t value) {
    char data[2] = {(char)reg, (char)value};
    i2c.write(LSM6DSL_ADDR, data, 2);
}

// Read a single register
uint8_t read_register(uint8_t reg) {
    char data = reg;
    i2c.write(LSM6DSL_ADDR, &data, 1, true);
    i2c.read(LSM6DSL_ADDR, &data, 1);
    return (uint8_t)data;
}

// Read a 16-bit value from two registers
int16_t read_16bit_value(uint8_t low_reg, uint8_t high_reg) {
    char low_byte = read_register(low_reg);
    char high_byte = read_register(high_reg);
    return (int16_t)((high_byte << 8) | (uint8_t)low_byte);
}

// Collect magnitude data from the accelerometer
// and store it in the input_data array
void collect_magnitude_data() {
    for (int i = 0; i < FFT_SIZE; i++) { // Collect data for FFT_SIZE samples
        int16_t ax_raw = read_16bit_value(OUTX_L_XL, OUTX_H_XL); // Read X-axis data
        int16_t ay_raw = read_16bit_value(OUTY_L_XL, OUTY_H_XL); // Read Y-axis data
        int16_t az_raw = read_16bit_value(OUTZ_L_XL, OUTZ_H_XL); // Read Z-axis data

        // Convert raw data to g
        float ax_g = ax_raw * 0.061f / 1000.0f; 
        float ay_g = ay_raw * 0.061f / 1000.0f; 
        float az_g = az_raw * 0.061f / 1000.0f;

        input_data[i] = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g); // Calculate magnitude of acceleration vector[3]

        ThisThread::sleep_for(10ms);  // ~100Hz
    }

    float32_t mean = 0;
    for (int i = 0; i < FFT_SIZE; i++) mean += input_data[i]; // Calculate mean
    mean /= FFT_SIZE;
    for (int i = 0; i < FFT_SIZE; i++) input_data[i] -= mean; // Remove DC offset by subtracting mean
}

// Perform FFT and show results using LEDs
// as well as serial output for debugging
void run_fft_and_show() {
    arm_rfft_fast_init_f32(&FFT_Instance, FFT_SIZE); // Initialize FFT instance
    arm_rfft_fast_f32(&FFT_Instance, input_data, fft_out, 0); // Perform FFT
    arm_cmplx_mag_f32(fft_out, magnitude, FFT_SIZE / 2); // Calculate magnitude

    float32_t maxValue; // Maximum value in the magnitude array
    uint32_t maxIndex; // Index of the maximum value

    arm_max_f32(magnitude, 50, &maxValue, &maxIndex); // compute maximum frequency bin value (magnitude) and index

    float freq = maxIndex * RESOLUTION; // Convert frequency bin index to frequency in Hz

    if (freq > 10.0f && maxValue < 0.1f) { // Disregard noisy signals
        printf(">> Noise detected (%.2f Hz, weak signal %.3f), skipped\n", freq, maxValue);
        return;
    }

    printf("Max magnitude: %.3f at %.2f Hz\n", maxValue, freq); // Debugging output

    const char* level;
    if (freq > 3 && freq <= 5) {                    // If tremor detected (3-5 Hz)
        if (maxValue < 125.0f && maxValue > 5.0f) { // Mild tremor magnitude threshold
            level = "Mild Tremor";                  // Indicate mild tremor with solid LED
            led1 = 1; 
            led2 = 0;
            ThisThread::sleep_for(3000ms);
            led1 = 0;
            led2 = 0;
        }
        else if (maxValue >= 125.0f) {           // Severe tremor magnitude threshold
            level = "Severe Tremor";
            for (int i = 0; i < 3; i++) {        // Blink LED to indicate severe tremor
                led1 = 1;
                ThisThread::sleep_for(500ms);
                led1 = 0;
                ThisThread::sleep_for(500ms);
            }
        }
        else {                                   // If no tremor detected (magnitude too low)
            level = "Normal or other movement";
            led1 = 0;
            led2 = 0;
        }
    }
    else if (freq > 5 && freq <= 7) {               // If dyskinesia detected (5-7 Hz)
        if (maxValue < 150.0f && maxValue > 5.0f) { // Mild dyskinesia magnitude threshold
            level = "Mild Dyskinesia";
            led1 = 1;                               // Indicate mild dyskinesia with solid LED
            led2 = 1;
            ThisThread::sleep_for(3000ms);
            led1 = 0;
            led2 = 0;
        }
        else if (maxValue >= 150.0f) {             // Severe dyskinesia magnitude threshold
            level = "Severe Dyskinesia";
            for (int i = 0; i < 3; i++) {          // Blink LED to indicate severe dyskinesia
                led1 = 1;
                led2 = 1;
                ThisThread::sleep_for(500ms);
                led1 = 0;
                led2 = 0;
                ThisThread::sleep_for(500ms);
            }
        }
        else {                                    // If no dyskinesia detected (magnitude too low)
            level = "Normal or other movement";
            led1 = 0;
            led2 = 0;
        }
    }
    else
        level = "Normal or other movement";

    printf(">> Detected status: %s\n", level);
}

int main() {
    i2c.frequency(400000);

    uint8_t id = read_register(WHO_AM_I); // Read WHO_AM_I register
    printf("WHO_AM_I = 0x%02X (Expected: 0x6A)\r\n", id); // Board ID check
    if (id != 0x6A) { // Check if the i2c device is LSM6DSL
        printf("Error: LSM6DSL not found\n");
        while (1);
    }

    write_register(CTRL1_XL, 0x40);

    printf("LSM6DSL Ready. Starting loop...\n");

    while (true) { // Main loop
        collect_magnitude_data(); // Collect data from accelerometer
        run_fft_and_show(); // Perform FFT and show results
        ThisThread::sleep_for(3000ms); // Wait before next iteration
    }
}
