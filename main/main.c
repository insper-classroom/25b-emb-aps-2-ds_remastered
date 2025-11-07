// main.c — versão ajustada: IMU NÃO move o cursor; só envia clique direito por gesto.

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
#include "mpu6050.h" // seu header (assuma compatível)

// --------------------------- PINOS / CONSTANTES -------------------------------
#define IMU_SDA         4
#define IMU_SCL         5
#define JOY_X_PIN       26 // ADC0
#define JOY_Y_PIN       27 // ADC1

#define BTN_ATACK       12 // hardware button -> attack
#define BTN_CURA        13
#define BTN_ROLL        14
#define BTN_ESC         15

#define LED_CONNECTED   16

#define I2C_PORT        i2c0
#define MPU_ADDRESS     0x68

#define SAMPLE_PERIOD_MS   10
#define SAMPLE_PERIOD      (SAMPLE_PERIOD_MS / 1000.0f)

// ----- clique por gesto -----
#define CLICK_ACCEL_THRESHOLD_Y 20000
#define CLICK_RESET_THRESHOLD_Y 1000
#define IMU_CLICK_DEBOUNCE_MS   200  // debounce local

// ----- joystick thresholds -----
#define JOY_DEADZONE            1500
#define JOY_KEY_THRESHOLD       12000
#define DIAGONAL_E_THRESHOLD    20000
#define JOY_SEND_THRESHOLD      2000
#define JOY_MIN_CHANGE          800

// --------------------------- UART IDs (packets) ------------------------------
#define UART_PKT_SYNC      0xFF

// IMU click
#define ID_IMU_CLICK       2   // value=1 -> click event

// physical buttons
#define ID_BTN_ATACK       3
#define ID_BTN_CURA        4
#define ID_BTN_ROLL        5
#define ID_BTN_ESC         6

// joystick raw (kept for compatibility)
#define ID_JOY_X           7
#define ID_JOY_Y           8

// KEY events generated on MCU (PC only executes)
#define ID_KEY_W           9
#define ID_KEY_A           10
#define ID_KEY_S           11
#define ID_KEY_D           12
#define ID_KEY_E           13

// --------------------------- STRUCTS / QUEUES --------------------------------
typedef struct {
    bool click;
    int16_t joy_x;
    int16_t joy_y;
} mdata_t;

typedef struct {
    uint8_t id;
    bool pressed;
} bevent_t;

static QueueHandle_t q_mdata;
static QueueHandle_t q_bevent;

// --------------------------- MPU helper (simplified) --------------------------
static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00}; // PWR_MGMT_1 = 0
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, buf, sizeof(buf), false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg;
    reg = 0x3B;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) accel[i] = (int16_t)((buffer[i*2] << 8) | buffer[i*2 + 1]);
    reg = 0x43;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) gyro[i] = (int16_t)((buffer[i*2] << 8) | buffer[i*2 + 1]);
    reg = 0x41;
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 2, false);
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
}

// --------------------------- UART send helpers --------------------------------
static inline void uart_send_raw_byte(uint8_t b) {
    putchar_raw(b);
}
static inline void uart_send_packet_int16(uint8_t id, int16_t v) {
    uart_send_raw_byte(UART_PKT_SYNC);
    uart_send_raw_byte(id);
    uart_send_raw_byte((uint8_t)(v & 0xFF));
    uart_send_raw_byte((uint8_t)((v >> 8) & 0xFF));
}
static inline void uart_send_packet_flag(uint8_t id, uint8_t flag) {
    uart_send_raw_byte(UART_PKT_SYNC);
    uart_send_raw_byte(id);
    uart_send_raw_byte(flag);
    uart_send_raw_byte(0x00);
}

// --------------------------- TASK: MPU + lógica (faz decisões) ----------------
void task_mpu(void *p) {
    // I2C
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
    mpu6050_reset();

    // ADC joystick
    adc_init();
    adc_gpio_init(JOY_X_PIN);
    adc_gpio_init(JOY_Y_PIN);

    int16_t accel[3], gyro[3], temp;
    bool can_click = true;
    uint64_t last_click_ts = 0;

    // joystick state -> we will send key press/release events based on thresholds
    bool key_w = false, key_a = false, key_s = false, key_d = false, key_e = false;
    int16_t last_sent_joy_x = 0, last_sent_joy_y = 0;

    while (1) {
        mpu6050_read_raw(accel, gyro, &temp);

        // Apenas lógica de clique por aceleração (não calculamos/mapeamos movimento do cursor)
        mdata_t out;
        out.click = false;
        out.joy_x = 0;
        out.joy_y = 0;
        bool any = false;

        // IMU click por aceleração (eixo Y)
        if (accel[1] > CLICK_ACCEL_THRESHOLD_Y) {
            uint64_t now = to_us_since_boot(get_absolute_time()) / 1000ULL;
            if (can_click && (now - last_click_ts) > IMU_CLICK_DEBOUNCE_MS) {
                out.click = true;
                any = true;
                last_click_ts = now;
                can_click = false;
            }
        } else if (accel[1] < CLICK_RESET_THRESHOLD_Y) {
            can_click = true;
        }

        // JOYSTICK -> decide quais teclas (w/a/s/d/e) devem estar pressionadas
        adc_select_input(0);
        uint16_t raw_x = adc_read();
        int16_t joy_x = (int16_t)(((int32_t)raw_x - 2048) * 16);
        adc_select_input(1);
        uint16_t raw_y = adc_read();
        int16_t joy_y = (int16_t)(((int32_t)raw_y - 2048) * 16);

        int16_t adj_x = (abs(joy_x) < JOY_DEADZONE) ? 0 : joy_x;
        int16_t adj_y = (abs(joy_y) < JOY_DEADZONE) ? 0 : joy_y;

        bool should_e = (abs(adj_x) > DIAGONAL_E_THRESHOLD && abs(adj_y) > DIAGONAL_E_THRESHOLD);
        if (should_e != key_e) {
            key_e = should_e;
            uart_send_packet_flag(ID_KEY_E, key_e ? 1 : 0);
        }
        bool should_w = (adj_y > JOY_KEY_THRESHOLD);
        bool should_s = (adj_y < -JOY_KEY_THRESHOLD);
        if (should_w != key_w) { key_w = should_w; uart_send_packet_flag(ID_KEY_W, key_w ? 1 : 0); }
        if (should_s != key_s) { key_s = should_s; uart_send_packet_flag(ID_KEY_S, key_s ? 1 : 0); }

        bool should_d = (adj_x > JOY_KEY_THRESHOLD);
        bool should_a = (adj_x < -JOY_KEY_THRESHOLD);
        if (should_d != key_d) { key_d = should_d; uart_send_packet_flag(ID_KEY_D, key_d ? 1 : 0); }
        if (should_a != key_a) { key_a = should_a; uart_send_packet_flag(ID_KEY_A, key_a ? 1 : 0); }

        // enviar joystick raw quando mudar bastante (compatibilidade)
        if (abs(joy_x) > JOY_SEND_THRESHOLD) {
            if (abs(joy_x - last_sent_joy_x) > JOY_MIN_CHANGE) {
                out.joy_x = joy_x;
                last_sent_joy_x = joy_x;
                any = true;
            }
        } else {
            if (abs(last_sent_joy_x) > JOY_SEND_THRESHOLD) {
                out.joy_x = 0;
                last_sent_joy_x = 0;
                any = true;
            }
        }
        if (abs(joy_y) > JOY_SEND_THRESHOLD) {
            if (abs(joy_y - last_sent_joy_y) > JOY_MIN_CHANGE) {
                out.joy_y = joy_y;
                last_sent_joy_y = joy_y;
                any = true;
            }
        } else {
            if (abs(last_sent_joy_y) > JOY_SEND_THRESHOLD) {
                out.joy_y = 0;
                last_sent_joy_y = 0;
                any = true;
            }
        }

        if (any) {
            xQueueSend(q_mdata, &out, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

// --------------------------- TASK: buttons (debounced) -------------------------
void task_buttons(void *p) {
    const uint btn_pins[] = { BTN_ATACK, BTN_CURA, BTN_ROLL, BTN_ESC };
    const uint8_t btn_ids[] = { ID_BTN_ATACK, ID_BTN_CURA, ID_BTN_ROLL, ID_BTN_ESC };
    const int NUM = 4;
    for (int i = 0; i < NUM; i++) { gpio_init(btn_pins[i]); gpio_set_dir(btn_pins[i], GPIO_IN); gpio_pull_up(btn_pins[i]); }
    bool last[NUM], read[NUM];
    uint32_t stable[NUM];
    memset(stable, 0, sizeof(stable));
    for (int i = 0; i < NUM; i++) { last[i] = gpio_get(btn_pins[i]); read[i] = last[i]; }
    const uint32_t DEBOUNCE_MS = 20;
    const uint32_t POLL_MS = 10;
    while (1) {
        for (int i = 0; i < NUM; i++) {
            bool raw = gpio_get(btn_pins[i]);
            if (raw == read[i]) {
                stable[i] += POLL_MS;
                if (stable[i] >= DEBOUNCE_MS && raw != last[i]) {
                    last[i] = raw;
                    bevent_t ev;
                    ev.id = btn_ids[i];
                    ev.pressed = (raw == 0); // pull-up: 0 pressed
                    xQueueSend(q_bevent, &ev, 0);
                }
            } else {
                stable[i] = 0;
            }
            read[i] = raw;
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

// --------------------------- TASK: UART sender --------------------------------
void task_uart(void *p) {
    mdata_t md;
    bevent_t be;
    while (1) {
        // flush buttons queue (non-blocking)
        while (xQueueReceive(q_bevent, &be, 0) == pdTRUE) {
            uart_send_packet_flag(be.id, be.pressed ? 1 : 0);
        }
        // wait for mdata (blocking with timeout)
        if (xQueueReceive(q_mdata, &md, pdMS_TO_TICKS(200))) {
            if (md.click) uart_send_packet_flag(ID_IMU_CLICK, 1);
            // send joystick X/Y packets only for compatibility (no IMU movement)
            uart_send_packet_int16(ID_JOY_X, md.joy_x);
            uart_send_packet_int16(ID_JOY_Y, md.joy_y);
        }
    }
}

// --------------------------- main --------------------------------------------

int main() {
    stdio_init_all();
    gpio_init(LED_CONNECTED);
    gpio_set_dir(LED_CONNECTED, GPIO_OUT);
    gpio_put(LED_CONNECTED, 0);

    q_mdata = xQueueCreate(10, sizeof(mdata_t));
    q_bevent = xQueueCreate(10, sizeof(bevent_t));
    if (!q_mdata || !q_bevent) {
        while (1) { gpio_xor_mask(1u << LED_CONNECTED); sleep_ms(200); }
    }

    xTaskCreate(task_mpu, "MPU_TASK", 4096, NULL, 2, NULL);
    xTaskCreate(task_buttons, "BTN_TASK", 1024, NULL, 2, NULL);
    xTaskCreate(task_uart, "UART_TASK", 2048, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) tight_loop_contents();
    return 0;
}
