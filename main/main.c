// main.c
// Firmware para RP2040 (Pico) + FreeRTOS
// - Lê MPU6050 (Fusion para Euler)
// - Lê joystick via ADC
// - Debounce de 4 botões
// - Envia pacotes UART/CDC no formato:
//     0xFF <id:1> <value_low:1> <value_high:1>
//   (ex.: id 0 = IMU X, id 1 = IMU Y, id 2 = click IMU, id 3-6 = botões, id 7 = joy_x, id 8 = joy_y)
//
// Observações:
// - Ajuste constantes (thresholds, sensibilidade) conforme necessário.
// - Usa filas FreeRTOS: xQueuePos (mouse/joystick) e xQueueBtn (eventos de botão).
// - Mantive putchar_raw(...) para envio que mapeia para stdio/USB CDC (stdio_init_all()).

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Fusion.h"
#include "mpu6050.h" // seu header já enviado

// =================================================================================
// === DEFINIÇÕES DE PINOS / CONSTANTES ===========================================
// =================================================================================

#define IMU_SDA         4
#define IMU_SCL         5
#define JOY_X           26 // ADC0
#define JOY_Y           27 // ADC1

#define BTN_ATACK       12 // F
#define BTN_CURA        13 // Q
#define BTN_ROLL        14 // R
#define BTN_ESC         15 // Esc

#define LED_CONNECTED   16

#define I2C_PORT        i2c0
#define MPU_ADDRESS     0x68

#define SAMPLE_PERIOD   (0.01f) // 10ms ~ 100 Hz

// --- Parâmetros do Mouse / IMU ---
#define MOUSE_DEAD_ZONE_ANGLE   2.0f
#define MOUSE_SENSITIVITY       0.8f
#define MOUSE_MAX_SPEED         8

// --- Clique por Gesto (ajuste conforme necessidade) ---
#define CLICK_ACCEL_THRESHOLD_Y 20000
#define CLICK_RESET_THRESHOLD_Y 1000

// --- UART IDs para botões e eixos ---
#define UART_ID_BTN_ATACK 3
#define UART_ID_BTN_CURA  4
#define UART_ID_BTN_ROLL  5
#define UART_ID_BTN_ESC   6

#define UART_ID_JOY_X 7
#define UART_ID_JOY_Y 8

// =================================================================================
// === DRIVER MPU6050 (funções mínimas) ==========================================
// =================================================================================

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00}; // PWR_MGMT_1 = 0
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, buf, sizeof(buf), false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg;

    // accel
    reg = 0x3B; // ACCEL_XOUT_H
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) accel[i] = (int16_t)((buffer[i*2] << 8) | buffer[i*2 + 1]);

    // gyro
    reg = 0x43; // GYRO_XOUT_H
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) gyro[i] = (int16_t)((buffer[i*2] << 8) | buffer[i*2 + 1]);

    // temp
    reg = 0x41; // TEMP_OUT_H
    i2c_write_blocking(I2C_PORT, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU_ADDRESS, buffer, 2, false);
    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
}

// =================================================================================
// === ESTRUTURAS E FILAS =========================================================
// =================================================================================

typedef struct {
    int16_t x_mov;
    int16_t y_mov;
    bool    click_event;
    int16_t joy_x;
    int16_t joy_y;
} mouse_data_t;

typedef struct {
    uint8_t id;
    bool    pressed;
} button_event_t;

static QueueHandle_t xQueuePos;
static QueueHandle_t xQueueBtn;

// =================================================================================
// === TASK MPU / IMU + ADC JOYSTICK ==============================================
// =================================================================================

void mpu_task(void *p) {
    // I2C IMU
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
    mpu6050_reset();

    // ADC joystick init
    adc_init();
    adc_gpio_init(JOY_X); // GPIO26 -> ADC0
    adc_gpio_init(JOY_Y); // GPIO27 -> ADC1

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

        // roll -> Y movement (inverter/escala conforme necessidade)
        if (fabsf(euler.angle.roll) > MOUSE_DEAD_ZONE_ANGLE) {
            data_to_send.y_mov = (int16_t)(euler.angle.roll * MOUSE_SENSITIVITY * 10.0f); // *10 por resolução
            if (data_to_send.y_mov > MOUSE_MAX_SPEED * 10) data_to_send.y_mov = MOUSE_MAX_SPEED * 10;
            if (data_to_send.y_mov < -MOUSE_MAX_SPEED * 10) data_to_send.y_mov = -MOUSE_MAX_SPEED * 10;
        }

        // pitch -> X movement (note o sinal negativo para ajustar orientação)
        if (fabsf(euler.angle.pitch) > MOUSE_DEAD_ZONE_ANGLE) {
            data_to_send.x_mov = (int16_t)(euler.angle.pitch * -MOUSE_SENSITIVITY * 10.0f);
            if (data_to_send.x_mov > MOUSE_MAX_SPEED * 10) data_to_send.x_mov = MOUSE_MAX_SPEED * 10;
            if (data_to_send.x_mov < -MOUSE_MAX_SPEED * 10) data_to_send.x_mov = -MOUSE_MAX_SPEED * 10;
        }

        // clique por aceleração (pode gerar falsos positivos -> calibrar)
        if (acceleration[1] > CLICK_ACCEL_THRESHOLD_Y && can_click) {
            data_to_send.click_event = true;
            can_click = false;
        } else if (acceleration[1] < CLICK_RESET_THRESHOLD_Y) {
            can_click = true;
        }

        // Leitura joystick ADC0 (JOY_X)
        adc_select_input(0); // ADC0 -> GPIO26
        uint16_t raw_x = adc_read();
        // converte 0..4095 para valor signed int16 aproximado
        data_to_send.joy_x = (int16_t)(((int32_t)raw_x - 2048) * 16);

        // Leitura joystick ADC1 (JOY_Y)
        adc_select_input(1); // ADC1 -> GPIO27
        uint16_t raw_y = adc_read();
        data_to_send.joy_y = (int16_t)(((int32_t)raw_y - 2048) * 16);

        // Envia para fila (sem bloqueio)
        xQueueSend(xQueuePos, &data_to_send, 0);

        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms loop ~100Hz
    }
}

// =================================================================================
// === TASK DOS BOTÕES =============================================================
// =================================================================================

void button_task(void *p) {
    const uint btn_pins[] = { BTN_ATACK, BTN_CURA, BTN_ROLL, BTN_ESC };
    const uint8_t btn_ids[] = { UART_ID_BTN_ATACK, UART_ID_BTN_CURA, UART_ID_BTN_ROLL, UART_ID_BTN_ESC };
    const int NUM_BTNS = 4;

    // Inicializa botões com pull-ups
    for (int i = 0; i < NUM_BTNS; i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }

    bool last_state[NUM_BTNS];
    bool read_state[NUM_BTNS];
    uint32_t stable_time[NUM_BTNS];
    memset(stable_time, 0, sizeof(stable_time)); // zera tempo de estabilidade

    for (int i = 0; i < NUM_BTNS; i++) {
        last_state[i] = gpio_get(btn_pins[i]);
        read_state[i] = last_state[i];
    }

    const uint32_t DEBOUNCE_MS = 20;
    const uint32_t POLL_MS = 10;

    while (1) {
        for (int i = 0; i < NUM_BTNS; i++) {
            bool raw = gpio_get(btn_pins[i]); // leitura (pull-up -> 1 quando não pressionado)
            if (raw == read_state[i]) {
                stable_time[i] += POLL_MS;
                if (stable_time[i] >= DEBOUNCE_MS && raw != last_state[i]) {
                    last_state[i] = raw;

                    button_event_t ev;
                    ev.id = btn_ids[i];
                    ev.pressed = (raw == 0); // 0 = pressionado (pull-up)
                    xQueueSend(xQueueBtn, &ev, 0);
                }
            } else {
                stable_time[i] = 0;
            }
            read_state[i] = raw;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

// =================================================================================
// === TASK UART (envio) ==========================================================
// =================================================================================

/*
 * Observação:
 * - usamos putchar_raw(...) para enviar bytes para stdio/CDC.
 *   Se você preferir usar outra interface UART (hardware uart), substitua por uart_write_blocking().
 */

void uart_task(void *p) {
    mouse_data_t mdata;
    button_event_t bevent;

    while (1) {
        // Envia eventos de botão (esgotando a fila de botões)
        while (xQueueReceive(xQueueBtn, &bevent, 0) == pdTRUE) {
            putchar_raw(0xFF);
            putchar_raw(bevent.id);
            putchar_raw(bevent.pressed ? 1 : 0);
            putchar_raw(0x00); // padding
        }

        // Envia dados do IMU + joystick (espera até 100ms por novo pacote)
        if (xQueueReceive(xQueuePos, &mdata, pdMS_TO_TICKS(100))) {
            // IMU X (id 0) - little endian
            putchar_raw(0xFF);
            putchar_raw(0);
            putchar_raw(mdata.x_mov & 0xFF);
            putchar_raw((mdata.x_mov >> 8) & 0xFF);

            // IMU Y (id 1)
            putchar_raw(0xFF);
            putchar_raw(1);
            putchar_raw(mdata.y_mov & 0xFF);
            putchar_raw((mdata.y_mov >> 8) & 0xFF);

            // Clique IMU (id 2)
            if (mdata.click_event) {
                putchar_raw(0xFF);
                putchar_raw(2);
                putchar_raw(1);
                putchar_raw(0);
            }

            // Joystick X (id 7)
            putchar_raw(0xFF);
            putchar_raw(UART_ID_JOY_X);
            putchar_raw(mdata.joy_x & 0xFF);
            putchar_raw((mdata.joy_x >> 8) & 0xFF);

            // Joystick Y (id 8)
            putchar_raw(0xFF);
            putchar_raw(UART_ID_JOY_Y);
            putchar_raw(mdata.joy_y & 0xFF);
            putchar_raw((mdata.joy_y >> 8) & 0xFF);
        }
    }
}

// =================================================================================
// === MAIN ========================================================================
// =================================================================================

int main() {
    stdio_init_all();
    gpio_init(LED_CONNECTED);
    gpio_set_dir(LED_CONNECTED, GPIO_OUT);
    gpio_put(LED_CONNECTED, 0); // LED desligado

    // Cria filas
    xQueuePos = xQueueCreate(10, sizeof(mouse_data_t));
    xQueueBtn = xQueueCreate(10, sizeof(button_event_t));

    // Checa alocação de filas
    if (xQueuePos == NULL || xQueueBtn == NULL) {
        // falha crítica: não conseguimos criar filas
        while (1) {
            gpio_xor_mask(1u << LED_CONNECTED);
            sleep_ms(200);
        }
    }

    // Cria tarefas (tamanhos de stack arbitrários; ajuste se overflow ocorrer)
    xTaskCreate(mpu_task, "MPU", 4096, NULL, 1, NULL);
    xTaskCreate(button_task, "BTNs", 1024, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART", 2048, NULL, 1, NULL);

    // Inicia scheduler
    vTaskStartScheduler();

    // Nunca deve chegar aqui
    while (true) {
        tight_loop_contents();
    }
    return 0;
}
