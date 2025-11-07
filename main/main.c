// main.c (versão otimizada para não enviar dados desnecessários)
// Substitua o seu main.c por este arquivo.

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
#include "mpu6050.h" // seu header

// =================================================================================
// === PINOS / CONSTANTES =========================================================
// =================================================================================

#define IMU_SDA         4
#define IMU_SCL         5
#define JOY_X_PIN       26 // ADC0
#define JOY_Y_PIN       27 // ADC1

#define BTN_ATACK       12 // F
#define BTN_CURA        13 // Q
#define BTN_ROLL        14 // R
#define BTN_ESC         15 // Esc

#define LED_CONNECTED   16

#define I2C_PORT        i2c0
#define MPU_ADDRESS     0x68

#define SAMPLE_PERIOD   (0.01f) // 10ms ~ 100 Hz

// --- Mouse / IMU ---
#define MOUSE_DEAD_ZONE_ANGLE   2.0f
#define MOUSE_SENSITIVITY       0.8f
#define MOUSE_MAX_SPEED         8

// --- Clique por Gesto ---
#define CLICK_ACCEL_THRESHOLD_Y 20000
#define CLICK_RESET_THRESHOLD_Y 1000

// --- UART IDs ---
#define UART_ID_IMU_X 0
#define UART_ID_IMU_Y 1
#define UART_ID_IMU_CLICK 2

#define UART_ID_BTN_ATACK 3
#define UART_ID_BTN_CURA  4
#define UART_ID_BTN_ROLL  5
#define UART_ID_BTN_ESC   6

#define UART_ID_JOY_X 7
#define UART_ID_JOY_Y 8

// --- Joystick send policy (evita spam) ---
#define JOY_SEND_THRESHOLD 2000   // só considera movimento significativo se |joy| > threshold
#define JOY_MIN_CHANGE     800    // só envia se mudou mais que isso desde último envio

// =================================================================================
// === MPU6050 DRIVER (simplificado) ==============================================
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
// === TIPOS E FILAS =============================================================
// =================================================================================

typedef struct {
    int16_t x_mov;      // IMU X movement (signed)
    int16_t y_mov;      // IMU Y movement (signed)
    bool    click_event; // IMU click gesture
    int16_t joy_x;      // joystick X mapped signed
    int16_t joy_y;      // joystick Y mapped signed
} mouse_data_t;

typedef struct {
    uint8_t id;
    bool    pressed;
} button_event_t;

static QueueHandle_t xQueuePos;
static QueueHandle_t xQueueBtn;

// =================================================================================
// === TASK MPU / JOYSTICK (envia apenas quando necessário) =======================
// =================================================================================

void mpu_task(void *p) {
    // init I2C
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
    mpu6050_reset();

    // ADC joystick init
    adc_init();
    adc_gpio_init(JOY_X_PIN); // ADC0
    adc_gpio_init(JOY_Y_PIN); // ADC1

    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    bool can_click = true;

    // variáveis para política de envio do joystick
    int16_t last_sent_joy_x = 0;
    int16_t last_sent_joy_y = 0;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        FusionVector gyroscope = { .axis = { .x = gyro[0] / 131.0f, .y = gyro[1] / 131.0f, .z = gyro[2] / 131.0f } };
        FusionVector accelerometer = { .axis = { .x = acceleration[0] / 16384.0f, .y = acceleration[1] / 16384.0f, .z = acceleration[2] / 16384.0f } };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // Leitura joystick ADC
        adc_select_input(0); // ADC0 -> JOY_X_PIN
        uint16_t raw_x = adc_read(); // 0..4095
        int16_t joy_x = (int16_t)(((int32_t)raw_x - 2048) * 16);

        adc_select_input(1); // ADC1 -> JOY_Y_PIN
        uint16_t raw_y = adc_read();
        int16_t joy_y = (int16_t)(((int32_t)raw_y - 2048) * 16);

        // Prepara estrutura, mas só envia pra fila se houver algo útil
        mouse_data_t data_to_send;
        data_to_send.x_mov = 0;
        data_to_send.y_mov = 0;
        data_to_send.click_event = false;
        data_to_send.joy_x = 0;
        data_to_send.joy_y = 0;

        bool any_to_send = false;

        // IMU X/Y (ângulos) -> só preencher se além da deadzone
        if (fabsf(euler.angle.roll) > MOUSE_DEAD_ZONE_ANGLE) {
            int16_t v = (int16_t)(euler.angle.roll * MOUSE_SENSITIVITY * 10.0f);
            if (v > (MOUSE_MAX_SPEED * 10)) v = MOUSE_MAX_SPEED * 10;
            if (v < -(MOUSE_MAX_SPEED * 10)) v = -MOUSE_MAX_SPEED * 10;
            data_to_send.y_mov = v; // roll -> Y
            any_to_send = true;
        } else {
            data_to_send.y_mov = 0;
        }

        if (fabsf(euler.angle.pitch) > MOUSE_DEAD_ZONE_ANGLE) {
            int16_t v = (int16_t)(euler.angle.pitch * -MOUSE_SENSITIVITY * 10.0f);
            if (v > (MOUSE_MAX_SPEED * 10)) v = MOUSE_MAX_SPEED * 10;
            if (v < -(MOUSE_MAX_SPEED * 10)) v = -MOUSE_MAX_SPEED * 10;
            data_to_send.x_mov = v; // pitch -> X
            any_to_send = true;
        } else {
            data_to_send.x_mov = 0;
        }

        // Clique IMU por aceleração
        if (acceleration[1] > CLICK_ACCEL_THRESHOLD_Y && can_click) {
            data_to_send.click_event = true;
            can_click = false;
            any_to_send = true;
        } else if (acceleration[1] < CLICK_RESET_THRESHOLD_Y) {
            can_click = true;
        }

        // Joystick: política de envio para X
        if ( (abs(joy_x) > JOY_SEND_THRESHOLD) ) {
            // enviável se mudou suficientemente desde último envio
            if (abs(joy_x - last_sent_joy_x) > JOY_MIN_CHANGE) {
                data_to_send.joy_x = joy_x;
                last_sent_joy_x = joy_x;
                any_to_send = true;
            }
        } else {
            // se joystick voltou para centro e antes tinha sido enviado valor significativo,
            // é útil enviar zero para sinalizar "parado" no PC — enviar zero apenas se último enviado
            // era fora do threshold (evita spam de zeros)
            if (abs(last_sent_joy_x) > JOY_SEND_THRESHOLD) {
                data_to_send.joy_x = 0;
                last_sent_joy_x = 0;
                any_to_send = true;
            }
        }

        // Joystick: política de envio para Y (mesma lógica)
        if ( (abs(joy_y) > JOY_SEND_THRESHOLD) ) {
            if (abs(joy_y - last_sent_joy_y) > JOY_MIN_CHANGE) {
                data_to_send.joy_y = joy_y;
                last_sent_joy_y = joy_y;
                any_to_send = true;
            }
        } else {
            if (abs(last_sent_joy_y) > JOY_SEND_THRESHOLD) {
                data_to_send.joy_y = 0;
                last_sent_joy_y = 0;
                any_to_send = true;
            }
        }

        // Se houver algo útil, envia para a fila. Caso contrário ignora (sem enviar zeros inúteis).
        if (any_to_send) {
            // tentativa não bloqueante: se fila cheia, descartamos (ou você pode bloquear com timeout)
            xQueueSend(xQueuePos, &data_to_send, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms ~ 100Hz
    }
}

// =================================================================================
// === TASK BOTÕES (sem mudanças significativas) =================================
// =================================================================================

void button_task(void *p) {
    const uint btn_pins[] = { BTN_ATACK, BTN_CURA, BTN_ROLL, BTN_ESC };
    const uint8_t btn_ids[] = { UART_ID_BTN_ATACK, UART_ID_BTN_CURA, UART_ID_BTN_ROLL, UART_ID_BTN_ESC };
    const int NUM_BTNS = 4;

    for (int i = 0; i < NUM_BTNS; i++) {
        gpio_init(btn_pins[i]);
        gpio_set_dir(btn_pins[i], GPIO_IN);
        gpio_pull_up(btn_pins[i]);
    }

    bool last_state[NUM_BTNS];
    bool read_state[NUM_BTNS];
    uint32_t stable_time[NUM_BTNS];
    memset(stable_time, 0, sizeof(stable_time));

    for (int i = 0; i < NUM_BTNS; i++) {
        last_state[i] = gpio_get(btn_pins[i]);
        read_state[i] = last_state[i];
    }

    const uint32_t DEBOUNCE_MS = 20;
    const uint32_t POLL_MS = 10;

    while (1) {
        for (int i = 0; i < NUM_BTNS; i++) {
            bool raw = gpio_get(btn_pins[i]);
            if (raw == read_state[i]) {
                stable_time[i] += POLL_MS;
                if (stable_time[i] >= DEBOUNCE_MS && raw != last_state[i]) {
                    last_state[i] = raw;
                    button_event_t ev;
                    ev.id = btn_ids[i];
                    ev.pressed = (raw == 0); // pull-up logic: 0 pressed
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
// === TASK UART (envio) - mantida ===============================================
// =================================================================================

void uart_task(void *p) {
    mouse_data_t mdata;
    button_event_t bevent;

    while (1) {
        // esvaziar fila de botões
        while (xQueueReceive(xQueueBtn, &bevent, 0) == pdTRUE) {
            putchar_raw(0xFF);
            putchar_raw(bevent.id);
            putchar_raw(bevent.pressed ? 1 : 0);
            putchar_raw(0x00);
        }

        // aguarda por pacote do mpu_task
        if (xQueueReceive(xQueuePos, &mdata, pdMS_TO_TICKS(200))) {
            // IMU X
            if (mdata.x_mov != 0) {
                putchar_raw(0xFF);
                putchar_raw(UART_ID_IMU_X);
                putchar_raw(mdata.x_mov & 0xFF);
                putchar_raw((mdata.x_mov >> 8) & 0xFF);
            }
            // IMU Y
            if (mdata.y_mov != 0) {
                putchar_raw(0xFF);
                putchar_raw(UART_ID_IMU_Y);
                putchar_raw(mdata.y_mov & 0xFF);
                putchar_raw((mdata.y_mov >> 8) & 0xFF);
            }
            // Clique IMU
            if (mdata.click_event) {
                putchar_raw(0xFF);
                putchar_raw(UART_ID_IMU_CLICK);
                putchar_raw(1);
                putchar_raw(0x00);
            }
            // Joystick X
            // Se mdata.joy_x==0 significa "parou" e é útil enviar para liberar teclas no PC;
            // se mdata.joy_x != 0 então envia valor.
            if (mdata.joy_x != 0) {
                putchar_raw(0xFF);
                putchar_raw(UART_ID_JOY_X);
                putchar_raw(mdata.joy_x & 0xFF);
                putchar_raw((mdata.joy_x >> 8) & 0xFF);
            } else {
                // se zero foi marcado explicitamente (volta ao centro), envie zero para liberar teclas
                // precisamos distinguir "campo não presente" de "campo zero": no design acima,
                // quando decidimos enviar zero (retorno ao centro), mdata.joy_x foi ajustado para 0 e any_to_send==true.
                // portanto se chegarmos aqui com mdata.joy_x==0 e (mdata.joy_y || outras flags) então o zero será enviado abaixo.
                // Para simplificar, se fila enviou esta struct com joy_x == 0 e nenhuma outra flag para X, envie zero:
                // (no código atual, só chegamos ao xQueueReceive quando any_to_send==true).
                // Então enviamos também o zero:
                if (mdata.joy_x == 0) {
                    putchar_raw(0xFF);
                    putchar_raw(UART_ID_JOY_X);
                    putchar_raw(0 & 0xFF);
                    putchar_raw(0 >> 8);
                }
            }
            // Joystick Y (mesma lógica)
            if (mdata.joy_y != 0) {
                putchar_raw(0xFF);
                putchar_raw(UART_ID_JOY_Y);
                putchar_raw(mdata.joy_y & 0xFF);
                putchar_raw((mdata.joy_y >> 8) & 0xFF);
            } else {
                if (mdata.joy_y == 0) {
                    putchar_raw(0xFF);
                    putchar_raw(UART_ID_JOY_Y);
                    putchar_raw(0 & 0xFF);
                    putchar_raw(0 >> 8);
                }
            }
        }
        // sem vTaskDelay aqui: o xQueueReceive bloqueia
    }
}

// =================================================================================
// === MAIN =======================================================================
// =================================================================================

int main() {
    stdio_init_all();
    gpio_init(LED_CONNECTED);
    gpio_set_dir(LED_CONNECTED, GPIO_OUT);
    gpio_put(LED_CONNECTED, 0);

    xQueuePos = xQueueCreate(10, sizeof(mouse_data_t));
    xQueueBtn = xQueueCreate(10, sizeof(button_event_t));

    if (xQueuePos == NULL || xQueueBtn == NULL) {
        while (1) {
            gpio_xor_mask(1u << LED_CONNECTED);
            sleep_ms(200);
        }
    }

    xTaskCreate(mpu_task, "MPU", 4096, NULL, 2, NULL);
    xTaskCreate(button_task, "BTNs", 1024, NULL, 2, NULL);
    xTaskCreate(uart_task, "UART", 2048, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true) {
        tight_loop_contents();
    }
    return 0;
}
