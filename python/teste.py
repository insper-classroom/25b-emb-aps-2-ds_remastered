# teste.py — leitor simples: recebe pacotes 0xFF id low high
# Joystick: X -> A/D ; Y -> W/S. IMU -> right click.

import serial
import time
import pyautogui

pyautogui.PAUSE = 0
pyautogui.FAILSAFE = False

# IDs (devem bater com main.c)
ID_IMU_CLICK = 2

ID_BTN_ATACK = 3
ID_BTN_CURA  = 4
ID_BTN_ROLL  = 5
ID_BTN_ESC   = 6

ID_JOY_X = 7
ID_JOY_Y = 8

ID_KEY_W = 9
ID_KEY_A = 10
ID_KEY_S = 11
ID_KEY_D = 12
# ID_KEY_E REMOVED

BUTTON_KEYMAP = {
    ID_BTN_ATACK: 'f',
    ID_BTN_CURA:  'q',
    ID_BTN_ROLL:  'r',
    ID_BTN_ESC:   'esc'
}

KEY_ID_MAP = {
    ID_KEY_W: 'w',
    ID_KEY_A: 'a',
    ID_KEY_S: 's',
    ID_KEY_D: 'd'
}

def parse_int16(low, high):
    v = int.from_bytes(bytes([low, high]), byteorder='little', signed=True)
    return v

def handle_packet(id_, low, high):
    if id_ == ID_IMU_CLICK:
        # clique direito (IMU)
        try:
            pyautogui.click(button='right')
        except Exception as e:
            print("click err", e)
        return

    if id_ in (ID_JOY_X, ID_JOY_Y):
        # joystick raw: ignoramos no PC (MCU já envia W/A/S/D)
        return

    if id_ in BUTTON_KEYMAP:
        key = BUTTON_KEYMAP[id_]
        if low == 1:
            pyautogui.keyDown(key)
        else:
            pyautogui.keyUp(key)
        return
    if id_ in KEY_ID_MAP:
        key = KEY_ID_MAP[id_]
        if low == 1:
            pyautogui.keyDown(key)
        else:
            pyautogui.keyUp(key)
        return

def main():
    port = input("Serial port (ex: COM3 or /dev/ttyACM0): ").strip()
    ser = serial.Serial(port, 115200, timeout=0.1)
    print("Conectado:", port)
    try:
        while True:
            sync = ser.read(1)
            if not sync:
                continue
            if sync[0] != 0xFF:
                continue
            hdr = ser.read(3)
            if len(hdr) < 3:
                continue
            id_ = hdr[0]
            low = hdr[1]
            high = hdr[2]
            handle_packet(id_, low, high)
    except KeyboardInterrupt:
        print("Saindo")
    finally:
        ser.close()

if __name__ == "__main__":
    main()