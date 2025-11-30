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

// ================= CONFIGURAÇÃO RÁPIDA =================
#define INVERT_X 0   // 0 = normal, 1 = inverte sinal X
#define INVERT_Y 0   // 0 = normal, 1 = inverte sinal Y
#define DEBUG_JOYSTICK 1
// ======================================================

// Habilita um scan I2C rápido no boot (0 = off, 1 = on)
#define RUN_I2C_SCAN 0

#define IMU_SDA         4
#define IMU_SCL         5
#define JOY_X_PIN       26 // ADC0
#define JOY_Y_PIN       27 // ADC1

#define BTN_ATACK       12
#define BTN_CURA        13
#define BTN_ROLL        14
#define BTN_ESC         15

#define LED_CONNECTED   16

#define I2C_PORT        i2c0
#define MPU_ADDRESS     0x68

#define SAMPLE_PERIOD_MS   10
#define SAMPLE_PERIOD      (SAMPLE_PERIOD_MS / 1000.0f)

#define CLICK_ACCEL_THRESHOLD_Y   32000   
#define CLICK_RESET_THRESHOLD_Y   2000    
#define CLICK_GYRO_THRESHOLD      3500   
#define IMU_CLICK_DEBOUNCE_MS     400     
#define IMU_REQUIRED_SAMPLES      3


#define JOY_DEADZONE 2200   
#define JOY_KEY_PRESS_THRESHOLD   5000
#define JOY_KEY_RELEASE_THRESHOLD 3000
#define JOY_SEND_THRESHOLD      2000
#define JOY_MIN_CHANGE          800

#define UART_PKT_SYNC      0xFF
#define ID_IMU_CLICK       2
#define ID_BTN_ATACK       3
#define ID_BTN_CURA        4
#define ID_BTN_ROLL        5
#define ID_BTN_ESC         6
#define ID_JOY_X           7
#define ID_JOY_Y           8
#define ID_KEY_W           9
#define ID_KEY_A           10
#define ID_KEY_S           11
#define ID_KEY_D           12

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

static int i2c_write_checked(i2c_inst_t *i2c, uint8_t addr, uint8_t *buf, size_t len, bool nostop, int timeout_us) {
    int res = i2c_write_timeout_us(i2c, addr, buf, len, nostop, timeout_us);
    if (res != (int)len) {
        printf("I2C WRITE ERR: addr=0x%02x wrote=%d expected=%d\n", addr, res, (int)len);
    }
    return res;
}

static int i2c_read_checked(i2c_inst_t *i2c, uint8_t addr, uint8_t *buf, size_t len, bool nostop, int timeout_us) {
    int res = i2c_read_timeout_us(i2c, addr, buf, len, nostop, timeout_us);
    if (res != (int)len) {
        printf("I2C READ ERR: addr=0x%02x read=%d expected=%d\n", addr, res, (int)len);
    }
    return res;
}

static void i2c_scan_demo() {
    printf("I2C scan (100kHz):\n");
    for (int addr = 1; addr < 127; ++addr) {
        int res = i2c_write_timeout_us(I2C_PORT, addr, NULL, 0, true, 10000);
        if (res >= 0) {
            printf("  Found 0x%02x\n", addr);
        }
    }
    printf("I2C scan done\n");
}

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    // tenta algumas vezes com timeout
    for (int attempt = 0; attempt < 3; ++attempt) {
        int r = i2c_write_checked(I2C_PORT, MPU_ADDRESS, buf, sizeof(buf), false, 50000); // 50 ms timeout
        if (r == (int)sizeof(buf)) return;
        sleep_ms(10);
    }
    printf("MPU6050 reset failed after retries\n");
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg;

    reg = 0x3B;
    if (i2c_write_checked(I2C_PORT, MPU_ADDRESS, &reg, 1, true, 20000) != 1) return;
    if (i2c_read_checked(I2C_PORT, MPU_ADDRESS, buffer, 6, false, 20000) == 6) {
        for (int i = 0; i < 3; i++) accel[i] = (int16_t)((buffer[i*2] << 8) | buffer[i*2 + 1]);
    } else {
        return;
    }

    reg = 0x43;
    if (i2c_write_checked(I2C_PORT, MPU_ADDRESS, &reg, 1, true, 20000) != 1) return;
    if (i2c_read_checked(I2C_PORT, MPU_ADDRESS, buffer, 6, false, 20000) == 6) {
        for (int i = 0; i < 3; i++) gyro[i] = (int16_t)((buffer[i*2] << 8) | buffer[i*2 + 1]);
    } else {
        // falha ao ler giroscópio
        return;
    }

    reg = 0x41;
    if (i2c_write_checked(I2C_PORT, MPU_ADDRESS, &reg, 1, true, 20000) != 1) return;
    if (i2c_read_checked(I2C_PORT, MPU_ADDRESS, buffer, 2, false, 20000) == 2) {
        *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    } else {
        return;
    }
}

// uart send
static inline void uart_send_raw_byte(uint8_t b) { putchar_raw(b); }
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
                    ev.pressed = (raw == 0);
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

void task_mpu_and_joy(void *p) {
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
    mpu6050_reset();

    adc_init();
    adc_gpio_init(JOY_X_PIN);
    adc_gpio_init(JOY_Y_PIN);

    int16_t accel[3] = {0}, gyro[3] = {0}, temp = 0;
    bool can_click = true;
    uint64_t last_click_ts = 0;

    bool key_w = false, key_a = false, key_s = false, key_d = false;
    int16_t last_sent_joy_x = 0, last_sent_joy_y = 0;

    // filtros EMA inteiros para reduzir ruído do ADC (alpha = 1/4)
    static int32_t filt_raw_x = 2048;
    static int32_t filt_raw_y = 2048;

    static int imu_spike_count = 0;
    static int32_t last_ax = 0, last_ay = 0, last_az = 0;

    while (1) {
        mpu6050_read_raw(accel, gyro, &temp);

        mdata_t out;
        out.click = false;
        out.joy_x = 0;
        out.joy_y = 0;
        bool any = false;

        // --- calcula ângulos (pitch/roll) a partir do acelerômetro ---
        // escala para g (MPU6050 em ±2g -> 16384 LSB/g)
        const float AX_SCALE = 1.0f / 16384.0f;
        float ax = accel[0] * AX_SCALE;
        float ay = accel[1] * AX_SCALE;
        float az = accel[2] * AX_SCALE;
        // pitch: rotação em torno do eixo Y (aprox), roll: em torno do eixo X
        float pitch = atan2f(ax, sqrtf(ay*ay + az*az)) * 57.295779513f; 
        float roll  = atan2f(ay, az) * 57.295779513f; // graus

        // ---Detecção de clique IMU melhorada: baseia-se em spike (delta) ---
        // thresholds (ajuste conforme necessário)
        const int32_t ACCEL_DELTA_THRESHOLD = 8000; // delta do vetor accel que conta como "movimento forte"
        const int32_t GYRO_DELTA_THRESHOLD  = 4000; // soma simples de |gx|+|gz|
        const int CLICK_RESET_MAG = 1500; // magnitude abaixo da qual re-armamos o clique

        int32_t dax = accel[0] - last_ax;
        int32_t day = accel[1] - last_ay;
        int32_t daz = accel[2] - last_az;
        int64_t accel_delta_mag = (int64_t)dax*dax + (int64_t)day*day + (int64_t)daz*daz; // quadrado da mag
        const int64_t ACCEL_DELTA_TH_SQR = (int64_t)ACCEL_DELTA_THRESHOLD * (int64_t)ACCEL_DELTA_THRESHOLD;

        int32_t dg = abs((int)gyro[2]) + abs((int)gyro[0]); 
        bool spike = (accel_delta_mag >= ACCEL_DELTA_TH_SQR) || (dg >= GYRO_DELTA_THRESHOLD);

        if (spike) {
            if (imu_spike_count < IMU_REQUIRED_SAMPLES) imu_spike_count++;
        } else {
            imu_spike_count = 0;
        }

        bool tilted_enough = (fabsf(pitch) > 45.0f) || (fabsf(roll) > 45.0f); // ajuste entre 45..80 
        bool click_condition = (imu_spike_count >= IMU_REQUIRED_SAMPLES) && tilted_enough;

        if (click_condition) {
            uint64_t now = to_us_since_boot(get_absolute_time()) / 1000ULL;
            if (can_click && (now - last_click_ts) > IMU_CLICK_DEBOUNCE_MS) {
                out.click = true; any = true; last_click_ts = now; can_click = false;
                imu_spike_count = 0; // reset contador
                if (DEBUG_JOYSTICK) printf("IMU CLICK (pitch=%.1f roll=%.1f accel_delta=%lld dg=%d)\r\n",
                                           pitch, roll, (long long)accel_delta_mag, dg);
            }
        }

        int64_t accel_mag_sq = (int64_t)accel[0]*accel[0] + (int64_t)accel[1]*accel[1] + (int64_t)accel[2]*accel[2];
        if (accel_mag_sq < (int64_t)CLICK_RESET_MAG * CLICK_RESET_MAG) {
            can_click = true;
        }

        last_ax = accel[0];
        last_ay = accel[1];
        last_az = accel[2];

        // JOYSTICK read
        adc_select_input(0);
        uint16_t raw_x = adc_read();
        adc_select_input(1);
        uint16_t raw_y = adc_read();

        // Filtro EMA simples (inteiro): new = old + (measured - old)/4
        filt_raw_x += ((int32_t)raw_x - filt_raw_x) >> 2;
        filt_raw_y += ((int32_t)raw_y - filt_raw_y) >> 2;

        int32_t tmpx = (filt_raw_x - 2048) * 16;
        int32_t tmpy = (filt_raw_y - 2048) * 16;

        // aplicar inversão se necessário
        if (INVERT_X) tmpx = -tmpx;
        if (INVERT_Y) tmpy = -tmpy;

        int16_t joy_x = (int16_t)tmpx;
        int16_t joy_y = (int16_t)tmpy;

        // DEBUG: print valores brutos para calibrar e estados das teclas
        if (DEBUG_JOYSTICK) {
            static int16_t last_dbg_x = 0, last_dbg_y = 0;
            if (abs(joy_x - last_dbg_x) > 200 || abs(joy_y - last_dbg_y) > 200) {
                printf("JOY raw_x=%u raw_y=%u filt_x=%ld filt_y=%ld -> joy_x=%d joy_y=%d (adj)\r\n",
                       raw_x, raw_y, (long)filt_raw_x, (long)filt_raw_y, joy_x, joy_y);
                last_dbg_x = joy_x; last_dbg_y = joy_y;
            }
        }

        // deadzone
        int16_t adj_x = (abs(joy_x) < JOY_DEADZONE) ? 0 : joy_x;
        int16_t adj_y = (abs(joy_y) < JOY_DEADZONE) ? 0 : joy_y;

        // W / S (eixo Y)
        bool next_w = key_w;
        if (!key_w && (adj_y >= JOY_KEY_PRESS_THRESHOLD)) next_w = true;   // sobe -> press W
        if ( key_w && (adj_y <= JOY_KEY_RELEASE_THRESHOLD)) next_w = false; // volta -> release W
        if (next_w != key_w) {
            key_w = next_w;
            uart_send_packet_flag(ID_KEY_W, key_w ? 1 : 0);
            if (DEBUG_JOYSTICK) printf("KEY W -> %d (adj_y=%d, press=%d release=%d)\r\n",
                                       key_w, adj_y, JOY_KEY_PRESS_THRESHOLD, JOY_KEY_RELEASE_THRESHOLD);
        }

        bool next_s = key_s;
        if (!key_s && (adj_y <= -JOY_KEY_PRESS_THRESHOLD)) next_s = true;   // desce -> press S
        if ( key_s && (adj_y >= -JOY_KEY_RELEASE_THRESHOLD)) next_s = false; // volta -> release S
        if (next_s != key_s) {
            key_s = next_s;
            uart_send_packet_flag(ID_KEY_S, key_s ? 1 : 0);
            if (DEBUG_JOYSTICK) printf("KEY S -> %d (adj_y=%d)\r\n", key_s, adj_y);
        }

        // D / A (eixo X)
        bool next_d = key_d;
        if (!key_d && (adj_x >= JOY_KEY_PRESS_THRESHOLD)) next_d = true;   // direita -> press D
        if ( key_d && (adj_x <= JOY_KEY_RELEASE_THRESHOLD)) next_d = false; // volta -> release D
        if (next_d != key_d) {
            key_d = next_d;
            uart_send_packet_flag(ID_KEY_D, key_d ? 1 : 0);
            if (DEBUG_JOYSTICK) printf("KEY D -> %d (adj_x=%d, press=%d release=%d)\r\n",
                                       key_d, adj_x, JOY_KEY_PRESS_THRESHOLD, JOY_KEY_RELEASE_THRESHOLD);
        }

        bool next_a = key_a;
        if (!key_a && (adj_x <= -JOY_KEY_PRESS_THRESHOLD)) next_a = true;   // esquerda -> press A
        if ( key_a && (adj_x >= -JOY_KEY_RELEASE_THRESHOLD)) next_a = false; // volta -> release A
        if (next_a != key_a) {
            key_a = next_a;
            uart_send_packet_flag(ID_KEY_A, key_a ? 1 : 0);
            if (DEBUG_JOYSTICK) printf("KEY A -> %d (adj_x=%d)\r\n", key_a, adj_x);
        }

        if (abs(joy_x) > JOY_SEND_THRESHOLD) {
            if (abs(joy_x - last_sent_joy_x) > JOY_MIN_CHANGE) { out.joy_x = joy_x; last_sent_joy_x = joy_x; any = true; }
        } else {
            if (abs(last_sent_joy_x) > JOY_SEND_THRESHOLD) { out.joy_x = 0; last_sent_joy_x = 0; any = true; }
        }
        if (abs(joy_y) > JOY_SEND_THRESHOLD) {
            if (abs(joy_y - last_sent_joy_y) > JOY_MIN_CHANGE) { out.joy_y = joy_y; last_sent_joy_y = joy_y; any = true; }
        } else {
            if (abs(last_sent_joy_y) > JOY_SEND_THRESHOLD) { out.joy_y = 0; last_sent_joy_y = 0; any = true; }
        }

        // imprime resumo por loop (quando DEBUG_JOYSTICK && any)
        if (DEBUG_JOYSTICK && any) {
            printf("SENT -> joy_x=%d joy_y=%d | W=%d A=%d S=%d D=%d | pitch=%.1f roll=%.1f\r\n",
                   out.joy_x, out.joy_y, key_w, key_a, key_s, key_d, pitch, roll);
        }

        if (any) xQueueSend(q_mdata, &out, 0);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

void task_uart(void *p) {
    mdata_t md;
    bevent_t be;
    while (1) {
        while (xQueueReceive(q_bevent, &be, 0) == pdTRUE) {
            uart_send_packet_flag(be.id, be.pressed ? 1 : 0);
        }
        if (xQueueReceive(q_mdata, &md, pdMS_TO_TICKS(200))) {
            if (md.click) uart_send_packet_flag(ID_IMU_CLICK, 1);
            uart_send_packet_int16(ID_JOY_X, md.joy_x);
            uart_send_packet_int16(ID_JOY_Y, md.joy_y);
        }
    }
}

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

    // Opcional debug
    i2c_init(I2C_PORT, 100000);
    gpio_set_function(IMU_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_SDA);
    gpio_pull_up(IMU_SCL);
#if RUN_I2C_SCAN
    i2c_scan_demo();
#endif

    xTaskCreate(task_mpu_and_joy, "MPUJOY", 4096, NULL, 2, NULL);
    xTaskCreate(task_buttons, "BTN_TASK", 1024, NULL, 2, NULL);
    xTaskCreate(task_uart, "UART_TASK", 2048, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1) tight_loop_contents();
    return 0;
}
