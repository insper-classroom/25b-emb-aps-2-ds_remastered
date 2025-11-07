#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

// Includes do FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Include da Biblioteca de Fusão
#include "Fusion.h"



const int I2C_SDA_PIN = 4;
const int I2C_SCL_PIN = 5;

const int I2C_PORT_NUM = 0; // i2c0
#define I2C_PORT i2c0      // Mantemos o #define para a instância do I2C

const int MPU_ADDRESS = 0x68;

#define SAMPLE_PERIOD (0.01f)

// --- Parâmetros do Mouse ---
#define MOUSE_DEAD_ZONE_ANGLE   2.0f
#define MOUSE_SENSITIVITY       0.8f
#define MOUSE_MAX_SPEED         8

// --- Parâmetros do Clique por Gesto ---
#define CLICK_ACCEL_THRESHOLD_Y 20000
#define CLICK_RESET_THRESHOLD_Y 1000

// =================================================================================
// === IMPLEMENTAÇÃO DO DRIVER MPU6050 =============================================
// =================================================================================

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t val;

    val = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x43;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    val = 0x41;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 2, false);
    *temp = buffer[0] << 8 | buffer[1];
}

// =================================================================================
// === LÓGICA DO PROGRAMA COM FREERTOS =============================================
// =================================================================================

typedef struct {
    int16_t x_mov;
    int16_t y_mov;
    bool click_event;
} mouse_data_t;

QueueHandle_t xQueuePos;

void mpu_task(void *p) {
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    bool can_click = true;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = { .axis = { .x = gyro[0] / 131.0f, .y = gyro[1] / 131.0f, .z = gyro[2] / 131.0f } };
        FusionVector accelerometer = { .axis = { .x = acceleration[0] / 16384.0f, .y = acceleration[1] / 16384.0f, .z = acceleration[2] / 16384.0f } };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        mouse_data_t data_to_send = {0};

        if (euler.angle.roll > MOUSE_DEAD_ZONE_ANGLE || euler.angle.roll < -MOUSE_DEAD_ZONE_ANGLE) {
            data_to_send.y_mov = (int16_t)(euler.angle.roll * MOUSE_SENSITIVITY);
            if (data_to_send.y_mov > MOUSE_MAX_SPEED) data_to_send.y_mov = MOUSE_MAX_SPEED;
            if (data_to_send.y_mov < -MOUSE_MAX_SPEED) data_to_send.y_mov = -MOUSE_MAX_SPEED;
        }

        if (euler.angle.pitch > MOUSE_DEAD_ZONE_ANGLE || euler.angle.pitch < -MOUSE_DEAD_ZONE_ANGLE) {
            data_to_send.x_mov = (int16_t)(euler.angle.pitch * -MOUSE_SENSITIVITY);
            if (data_to_send.x_mov > MOUSE_MAX_SPEED) data_to_send.x_mov = MOUSE_MAX_SPEED;
            if (data_to_send.x_mov < -MOUSE_MAX_SPEED) data_to_send.x_mov = -MOUSE_MAX_SPEED;
        }

        if (acceleration[1] > CLICK_ACCEL_THRESHOLD_Y && can_click) {
            data_to_send.click_event = true;
            can_click = false;
        } else if (acceleration[1] < CLICK_RESET_THRESHOLD_Y) {
            can_click = true;
        }

        xQueueSend(xQueuePos, &data_to_send, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p) {
    mouse_data_t received_data;
    while (1) {
        if (xQueueReceive(xQueuePos, &received_data, portMAX_DELAY)) {

            putchar_raw(0xFF);
            putchar_raw(0);
            putchar_raw(received_data.x_mov & 0xFF);
            putchar_raw((received_data.x_mov >> 8) & 0xFF);

            putchar_raw(0xFF);
            putchar_raw(1);
            putchar_raw(received_data.y_mov & 0xFF);
            putchar_raw((received_data.y_mov >> 8) & 0xFF);

            if (received_data.click_event) {
                putchar_raw(0xFF);
                putchar_raw(2);
                putchar_raw(1);
                putchar_raw(0);
            }
        }
    }
}

int main() {
    stdio_init_all();
    xQueuePos = xQueueCreate(10, sizeof(mouse_data_t));
    xTaskCreate(mpu_task, "MPU_Task", 4096, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART_Task", 2048, NULL, 1, NULL);
    vTaskStartScheduler();
    while (true);
}