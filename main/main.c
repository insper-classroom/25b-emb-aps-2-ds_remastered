// main.c — correção de mapeamento joystick e flags de inversão
// X -> A/D ; Y -> W/S
// Para inverter eixos, ajuste INVERT_X / INVERT_Y abaixo.

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Fusion.h"
#include "mpu6050.h"

// ================= CONFIGURAÇÃO RÁPIDA (REFATOR B) =================
#define DEBUG_JOYSTICK 0
#define JOY_DEADZONE_RAW 120  // deadzone pós escala (-255..255)
#define JOY_HYST_PRESS 140    // limiar para pressionar tecla
#define JOY_HYST_RELEASE 90   // limiar para soltar tecla
#define MOUSE_GAIN_PITCH 3.5f // ganho reduzido para menor sensibilidade eixo X
#define MOUSE_GAIN_ROLL 3.5f  // ganho reduzido para menor sensibilidade eixo Y
#define MOUSE_DEADZONE 3.5f   // zona morta angular aumentada para ignorar micro movimentos
#define MOUSE_MOTION_ALPHA 0.2f // fator EMA para suavizar pitch/roll
#define MOUSE_MOTION_REQUIRED_SAMPLES 2 // exige N amostras consecutivas acima da deadzone
// ======================================================

#define IMU_SDA 4
#define IMU_SCL 5
#define JOY_X_PIN 26 // ADC0
#define JOY_Y_PIN 27 // ADC1

#define BTN_ATACK 12
#define BTN_CURA 13
#define BTN_ROLL 14
#define BTN_ESC 15

#define LED_CONNECTED 16

#define I2C_PORT i2c0
#define MPU_ADDRESS 0x68

#define SAMPLE_PERIOD_MS 10
#define SAMPLE_PERIOD (SAMPLE_PERIOD_MS / 1000.0f)

// IMU click (refinado)
#define CLICK_ACCEL_THRESHOLD_Y 34000
#define CLICK_RESET_THRESHOLD_Y 2500
#define CLICK_GYRO_THRESHOLD 4000
#define IMU_CLICK_DEBOUNCE_MS 450
#define IMU_REQUIRED_SAMPLES 4

// média móvel ADC
#define ADC_WINDOW 8

// Protocolo mouse lab: AXIS (0=x,1=y), LSB, MSB, EOP(0xFF)
#define MOUSE_EOP 0xFF
// Eventos extras (WASD, clique) usando prefixo diferente
#define EXTRA_SYNC 0xFE
#define EXTRA_ID_CLICK 0xF0
#define EXTRA_ID_W 0xF1
#define EXTRA_ID_A 0xF2
#define EXTRA_ID_S 0xF3
#define EXTRA_ID_D 0xF4

// Estruturas
typedef struct {
    uint8_t axis; // 0=X,1=Y
    int16_t val;  // -255..255
} adc_t;

typedef struct {
    uint8_t id;    // EXTRA_ID_*
    uint8_t value; // 0 ou 1
} extra_evt_t;

static QueueHandle_t qExtra; // eventos (WASD, clique)

// helpers MPU (igual aos anteriores)
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, buf, sizeof(buf), false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg;
    reg = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
    reg = 0x43;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        gyro[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
    reg = 0x41;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 2, false);
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
}

// uart send
static inline void uart_send_raw_byte(uint8_t b) { putchar_raw(b); }
static inline void uart_send_mouse(uint8_t axis, int16_t v) {
    uart_send_raw_byte(axis);
    uart_send_raw_byte((uint8_t)(v & 0xFF));
    uart_send_raw_byte((uint8_t)((v >> 8) & 0xFF));
    uart_send_raw_byte(MOUSE_EOP);
}
static inline void uart_send_extra(uint8_t id, uint8_t val) {
    uart_send_raw_byte(EXTRA_SYNC);
    uart_send_raw_byte(id);
    uart_send_raw_byte(val);
    uart_send_raw_byte(0x00);
}

// TASK: buttons (opcional – eventos extras)
void task_buttons(void *p) {
    const uint btn_pins[] = {BTN_ATACK, BTN_CURA, BTN_ROLL, BTN_ESC};
    const uint8_t btn_ids[] = {EXTRA_ID_CLICK, EXTRA_ID_W, EXTRA_ID_A, EXTRA_ID_D}; // exemplo de mapeamento
    const int NUM = 4;
    for (int i = 0; i < NUM; i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }
    bool last[NUM], read[NUM];
    uint32_t stable[NUM];
    memset(stable, 0, sizeof(stable));
    for (int i = 0; i < NUM; i++) {
        last[i] = gpio_get(btn_pins[i]);
        read[i] = last[i];
    }
    const uint32_t DEBOUNCE_MS = 20, POLL_MS = 10;
    while (1) {
        for (int i = 0; i < NUM; i++) {
            bool raw = gpio_get(btn_pins[i]);
            if (raw == read[i]) {
                stable[i] += POLL_MS;
                if (stable[i] >= DEBOUNCE_MS && raw != last[i]) {
                    last[i] = raw;
                    extra_evt_t ev = {btn_ids[i], (uint8_t)(raw == 0)};
                    xQueueSend(qExtra, &ev, 0);
                }
            } else
                stable[i] = 0;
            read[i] = raw;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

// Média móvel simples
static int16_t moving_average_update(uint16_t raw, uint16_t *buffer, uint8_t *idx, uint8_t *count) {
    buffer[*idx] = raw;
    *idx = (*idx + 1) % ADC_WINDOW;
    if (*count < ADC_WINDOW)
        (*count)++;
    uint32_t sum = 0;
    for (uint8_t i = 0; i < *count; i++)
        sum += buffer[i];
    return (int16_t)(sum / *count);
}

static int16_t scale_adc(int16_t avg) {
    int16_t centered = avg - 2048;
    int16_t scaled = centered / 8; // ~ -256..255
    if (abs(scaled) < JOY_DEADZONE_RAW)
        scaled = 0;
    if (scaled > 255)
        scaled = 255;
    if (scaled < -255)
        scaled = -255;
    return scaled;
}

void task_joy_x(void *p) {
    adc_gpio_init(JOY_X_PIN);
    uint16_t buf[ADC_WINDOW] = {0};
    uint8_t idx = 0, count = 0;
    bool key_a = false, key_d = false;
    while (1) {
        adc_select_input(0);
        uint16_t raw = adc_read();
        int16_t avg = moving_average_update(raw, buf, &idx, &count);
        int16_t val = scale_adc(avg);
        bool next_d = key_d;
        if (!key_d && val > JOY_HYST_PRESS)
            next_d = true;
        if (key_d && val < JOY_HYST_RELEASE)
            next_d = false;
        if (next_d != key_d) {
            key_d = next_d;
            extra_evt_t ev = {EXTRA_ID_D, (uint8_t)(key_d ? 1 : 0)};
            xQueueSend(qExtra, &ev, 0);
            if (DEBUG_JOYSTICK)
                printf("KEY D -> %d val=%d\r\n", key_d, val);
        }
        bool next_a = key_a;
        if (!key_a && val < -JOY_HYST_PRESS)
            next_a = true;
        if (key_a && val > -JOY_HYST_RELEASE)
            next_a = false;
        if (next_a != key_a) {
            key_a = next_a;
            extra_evt_t ev = {EXTRA_ID_A, (uint8_t)(key_a ? 1 : 0)};
            xQueueSend(qExtra, &ev, 0);
            if (DEBUG_JOYSTICK)
                printf("KEY A -> %d val=%d\r\n", key_a, val);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_joy_y(void *p) {
    adc_gpio_init(JOY_Y_PIN);
    uint16_t buf[ADC_WINDOW] = {0};
    uint8_t idx = 0, count = 0;
    bool key_w = false, key_s = false;
    while (1) {
        adc_select_input(1);
        uint16_t raw = adc_read();
        int16_t avg = moving_average_update(raw, buf, &idx, &count);
        int16_t val = scale_adc(avg);
        bool next_w = key_w;
        if (!key_w && val > JOY_HYST_PRESS)
            next_w = true;
        if (key_w && val < JOY_HYST_RELEASE)
            next_w = false;
        if (next_w != key_w) {
            key_w = next_w;
            extra_evt_t ev = {EXTRA_ID_W, (uint8_t)(key_w ? 1 : 0)};
            xQueueSend(qExtra, &ev, 0);
            if (DEBUG_JOYSTICK)
                printf("KEY W -> %d val=%d\r\n", key_w, val);
        }
        bool next_s = key_s;
        if (!key_s && val < -JOY_HYST_PRESS)
            next_s = true;
        if (key_s && val > -JOY_HYST_RELEASE)
            next_s = false;
        if (next_s != key_s) {
            key_s = next_s;
            extra_evt_t ev = {EXTRA_ID_S, (uint8_t)(key_s ? 1 : 0)};
            xQueueSend(qExtra, &ev, 0);
            if (DEBUG_JOYSTICK)
                printf("KEY S -> %d val=%d\r\n", key_s, val);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task_imu_mouse(void *p) {
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
    mpu6050_reset();
    int16_t accel[3], gyro[3], temp;
    bool can_click = true;
    uint64_t last_click_ts = 0;
    static int accel_over = 0, gyro_over = 0;
    static float filt_pitch = 0.0f;
    static float filt_roll  = 0.0f;
    static uint8_t motion_x = 0;
    static uint8_t motion_y = 0;
    while (1) {
        mpu6050_read_raw(accel, gyro, &temp);
        const float SCALE = 1.0f / 16384.0f;
        float ax = accel[0] * SCALE, ay = accel[1] * SCALE, az = accel[2] * SCALE;
        float raw_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * 57.29578f;
        float raw_roll  = atan2f(ay, az) * 57.29578f;
        // filtro EMA
        filt_pitch += MOUSE_MOTION_ALPHA * (raw_pitch - filt_pitch);
        filt_roll  += MOUSE_MOTION_ALPHA * (raw_roll  - filt_roll);
        int16_t mx = 0, my = 0;
        if (fabsf(filt_pitch) > MOUSE_DEADZONE) {
            if (motion_x < MOUSE_MOTION_REQUIRED_SAMPLES) motion_x++;
            if (motion_x >= MOUSE_MOTION_REQUIRED_SAMPLES)
                mx = (int16_t)(filt_pitch * MOUSE_GAIN_PITCH);
        } else motion_x = 0;
        if (fabsf(filt_roll) > MOUSE_DEADZONE) {
            if (motion_y < MOUSE_MOTION_REQUIRED_SAMPLES) motion_y++;
            if (motion_y >= MOUSE_MOTION_REQUIRED_SAMPLES)
                my = (int16_t)(filt_roll * MOUSE_GAIN_ROLL);
        } else motion_y = 0;
        if (mx > 255)
            mx = 255;
        if (mx < -255)
            mx = -255;
        if (my > 255)
            my = 255;
        if (my < -255)
            my = -255;
        if (accel[1] > CLICK_ACCEL_THRESHOLD_Y) {
            if (accel_over < IMU_REQUIRED_SAMPLES)
                accel_over++;
        } else
            accel_over = 0;
        int32_t gz = abs(gyro[2]), gx = abs(gyro[0]);
        if ((gx > CLICK_GYRO_THRESHOLD || gz > CLICK_GYRO_THRESHOLD)) {
            if (gyro_over < IMU_REQUIRED_SAMPLES)
                gyro_over++;
        } else
            gyro_over = 0;
        bool click_cond = (accel_over >= IMU_REQUIRED_SAMPLES) || (gyro_over >= IMU_REQUIRED_SAMPLES);
        if (click_cond) {
            uint64_t now = to_us_since_boot(get_absolute_time()) / 1000ULL;
            if (can_click && (now - last_click_ts) > IMU_CLICK_DEBOUNCE_MS) {
                extra_evt_t ev = {EXTRA_ID_CLICK, 1};
                xQueueSend(qExtra, &ev, 0);
                last_click_ts = now;
                can_click = false;
                if (DEBUG_JOYSTICK)
                    printf("IMU CLICK pitch_f=%.1f roll_f=%.1f\r\n", filt_pitch, filt_roll);
            }
        }
        if (accel[1] < CLICK_RESET_THRESHOLD_Y)
            can_click = true;
        uart_send_mouse(0, mx);
        uart_send_mouse(1, my);
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

// TASK UART: envia eventos extras (mouse já sai direto da task IMU)
void task_uart(void *p) {
    extra_evt_t ev;
    while (1) {
        if (xQueueReceive(qExtra, &ev, pdMS_TO_TICKS(50))) {
            uart_send_extra(ev.id, ev.value);
        }
    }
}

// main
int main() {
    stdio_init_all();
    gpio_init(LED_CONNECTED);
    gpio_set_dir(LED_CONNECTED, GPIO_OUT);
    gpio_put(LED_CONNECTED, 0);

    qExtra = xQueueCreate(20, sizeof(extra_evt_t));
    if (!qExtra) {
        while (1) {
            gpio_xor_mask(1u << LED_CONNECTED);
            sleep_ms(200);
        }
    }
    adc_init();
    xTaskCreate(task_imu_mouse, "IMU_MOUSE", 4096, NULL, 2, NULL);
    xTaskCreate(task_joy_x, "JOY_X", 1024, NULL, 2, NULL);
    xTaskCreate(task_joy_y, "JOY_Y", 1024, NULL, 2, NULL);
    xTaskCreate(task_buttons, "BTN_TASK", 1024, NULL, 2, NULL);
    xTaskCreate(task_uart, "UART_TASK", 2048, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1)
        tight_loop_contents();
    return 0;
}
