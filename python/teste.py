# teste.py - versão sem calibração do joystick, botões corrigidos
# IMU -> clique direito, Joystick -> WASD (+ E para diagonal)
# Não faz calibração que bloqueie; joystick usa deadzone no cliente.

import sys
import glob
import serial
from serial.tools import list_ports
import pyautogui
pyautogui.PAUSE = 0
pyautogui.FAILSAFE = False
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time

# ----------------- CONFIG / MAPEAMENTO DE IDs -----------------
AXIS_IMU_X = 0
AXIS_IMU_Y = 1
AXIS_IMU_CLICK = 2

BTN_ATACK_ID = 3
BTN_CURA_ID  = 4
BTN_ROLL_ID  = 5
BTN_ESC_ID   = 6

JOY_X_ID = 7
JOY_Y_ID = 8

BUTTON_KEYMAP = {
    BTN_ATACK_ID: 'f',
    BTN_CURA_ID:  'q',
    BTN_ROLL_ID:  'r',
    BTN_ESC_ID:   'esc'
}

# ----------------- SENSIBILIDADES / THRESHOLDS -----------------
IMU_MOVE_SCALE = 4          # escala para movimento do mouse vindo da IMU
JOY_KEY_THRESHOLD = 12000   # valor absoluto necessário para considerar direcional
DIAGONAL_E_THRESHOLD = 20000

# Deadzone/filters on PC (no calibration)
JOY_DEADZONE = 1500         # if |value| < deadzone => treat as zero
JOY_CHANGE_MIN = 800        # minimal change to trigger processing (rate-limit)

# ----------------- Estado de teclas (para enviar keyDown/keyUp uma vez) ----------
key_state = {
    'w': False,
    'a': False,
    's': False,
    'd': False,
    'e': False
}

last_raw_x = 0
last_raw_y = 0
last_adj_x = 0
last_adj_y = 0

# Verbose -> True para debug completo (muitos prints). False para uso normal.
VERBOSE = False

# ----------------- HELPERS -----------------
def key_down_if_needed(key):
    if not key_state.get(key, False):
        try:
            pyautogui.keyDown(key)
        except Exception as e:
            print("Erro keyDown:", e)
        key_state[key] = True
        print(f"[KEY DOWN] {key}")

def key_up_if_needed(key):
    if key_state.get(key, False):
        try:
            pyautogui.keyUp(key)
        except Exception as e:
            print("Erro keyUp:", e)
        key_state[key] = False
        print(f"[KEY UP] {key}")

def release_all_movement_keys():
    for k in ('w','a','s','d','e'):
        key_up_if_needed(k)

def move_mouse(axis, value):
    # value is signed int16 from firmware
    if axis == AXIS_IMU_X:
        dx = int(value * IMU_MOVE_SCALE)
        if dx != 0:
            pyautogui.moveRel(dx, 0)
        if VERBOSE: print(f"[MOUSE] IMU X move: raw={value} scaled={dx}")
    elif axis == AXIS_IMU_Y:
        dy = int(value * IMU_MOVE_SCALE)
        if dy != 0:
            pyautogui.moveRel(0, dy)
        if VERBOSE: print(f"[MOUSE] IMU Y move: raw={value} scaled={dy}")

def handle_click_right(value):
    # firmware sends 1 when gesture detected
    if value != 0:
        print(f"[CLICK] IMU click detected value={value} -> right click")
        try:
            pyautogui.click(button='right')
        except Exception as e:
            print("Erro em right-click:", e)

def handle_button(id_, value):
    # value: 1 = pressed, 0 = released (as firmware sends)
    key = BUTTON_KEYMAP.get(id_)
    if not key:
        if VERBOSE: print(f"[BTN] Botão com ID {id_} não mapeado. value={value}")
        return
    if value != 0:
        # press
        print(f"[BTN] ID {id_} PRESSED -> {key}")
        try:
            pyautogui.keyDown(key)
        except Exception as e:
            print("Erro em keyDown:", e)
    else:
        # release
        print(f"[BTN] ID {id_} RELEASED -> {key}")
        try:
            pyautogui.keyUp(key)
        except Exception as e:
            print("Erro em keyUp:", e)

# ----------------- PARSE -----------------
def parse_data(data):
    """Recebe 3 bytes: id (1 byte), value low, value high (little-endian signed)"""
    if len(data) < 3:
        return None, None
    id_ = data[0]
    value = int.from_bytes(data[1:3], byteorder='little', signed=True)
    return id_, value

# ----------------- JOYSTICK HANDLER (sem calibração) ----------
def handle_joystick(raw_x, raw_y):
    """
    raw_x/raw_y are signed int16 from firmware.
    Apply deadzone and only act on significant changes.
    """
    global last_adj_x, last_adj_y

    # adjust by deadzone: treat small values as zero
    adj_x = 0 if abs(raw_x) < JOY_DEADZONE else raw_x
    adj_y = 0 if abs(raw_y) < JOY_DEADZONE else raw_y

    # quick exit: if both adj are zero, release keys if any pressed
    if adj_x == 0 and adj_y == 0:
        if any(key_state[k] for k in ('w','a','s','d','e')):
            release_all_movement_keys()
        # update lasts
        last_adj_x = 0
        last_adj_y = 0
        return

    # rate-limit: avoid handling very small oscillations
    if abs(adj_x - last_adj_x) < JOY_CHANGE_MIN and abs(adj_y - last_adj_y) < JOY_CHANGE_MIN:
        return

    last_adj_x = adj_x
    last_adj_y = adj_y

    # Diagonal strong -> 'e'
    if abs(adj_x) > DIAGONAL_E_THRESHOLD and abs(adj_y) > DIAGONAL_E_THRESHOLD:
        key_down_if_needed('e')
    else:
        key_up_if_needed('e')

    # Vertical (assumes positive y = up; invert if needed)
    if adj_y > JOY_KEY_THRESHOLD:
        key_down_if_needed('w')
        key_up_if_needed('s')
    elif adj_y < -JOY_KEY_THRESHOLD:
        key_down_if_needed('s')
        key_up_if_needed('w')
    else:
        key_up_if_needed('w')
        key_up_if_needed('s')

    # Horizontal
    if adj_x > JOY_KEY_THRESHOLD:
        key_down_if_needed('d')
        key_up_if_needed('a')
    elif adj_x < -JOY_KEY_THRESHOLD:
        key_down_if_needed('a')
        key_up_if_needed('d')
    else:
        key_up_if_needed('a')
        key_up_if_needed('d')

    # print brief state
    print(f"[JOY STATE] adj_x={adj_x} adj_y={adj_y} -> keys: "
          f"{'w' if key_state['w'] else ''}{'a' if key_state['a'] else ''}"
          f"{'s' if key_state['s'] else ''}{'d' if key_state['d'] else ''}"
          f"{' e' if key_state['e'] else ''}".strip())

# ----------------- SERIAL / LOOP PRINCIPAL -----------------
def controle(ser):
    # no calibration: start reading right away
    print("Loop pronto. Sem calibração de joystick. Pressione botões ou mova joystick.")
    global last_raw_x, last_raw_y
    try:
        while True:
            sync = ser.read(size=1)
            if not sync:
                continue
            if sync[0] != 0xFF:
                # resync quickly
                if VERBOSE: print(f"[SYNC] ignorando byte {sync[0]:02X}")
                continue
            data = ser.read(size=3)
            if len(data) < 3:
                continue
            id_, value = parse_data(data)
            if id_ is None:
                continue

            if VERBOSE:
                print(f"[RX] id={id_} value={value}")

            if id_ == AXIS_IMU_X or id_ == AXIS_IMU_Y:
                move_mouse(id_, value)
            elif id_ == AXIS_IMU_CLICK:
                handle_click_right(value)
            elif id_ in BUTTON_KEYMAP:
                # value: 1 pressed, 0 released (firmware sends that)
                handle_button(id_, value)
            elif id_ == JOY_X_ID:
                last_raw_x = value
                # combine with last_raw_y to decide
                handle_joystick(last_raw_x, last_raw_y)
            elif id_ == JOY_Y_ID:
                last_raw_y = value
                handle_joystick(last_raw_x, last_raw_y)
            else:
                if VERBOSE:
                    print(f"[WARN] ID desconhecido: {id_} value={value}")

    except serial.SerialException as e:
        print("SerialException:", e)
    except KeyboardInterrupt:
        print("Ctrl-C recebido, saindo...")
    except Exception as e:
        print("Erro no loop de leitura:", e)
    finally:
        try:
            ser.close()
        except:
            pass
        print("Conexão serial fechada.")
        release_all_movement_keys()

# ----------------- GUI / PORTAS -----------------
def serial_ports():
    """Lista portas sem abrir (rápido)."""
    try:
        return [p.device for p in list_ports.comports()]
    except Exception:
        # fallback simples
        if sys.platform.startswith('win'):
            return [f'COM{i}' for i in range(1, 21)]
        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            return glob.glob('/dev/tty[A-Za-z0-9]*')
        elif sys.platform.startswith('darwin'):
            return glob.glob('/dev/tty.*')
        return []

def conectar_porta(port_name, root, botao_conectar, status_label, mudar_cor_circulo):
    if not port_name:
        messagebox.showwarning("Aviso", "Selecione uma porta serial antes de conectar.")
        return
    try:
        ser = serial.Serial(port_name, 115200, timeout=0.1)
        status_label.config(text=f"Conectado em {port_name}", foreground="green")
        mudar_cor_circulo("green")
        botao_conectar.config(text="Conectado")
        root.update()
        t = threading.Thread(target=controle, args=(ser,), daemon=True)
        t.start()
    except Exception as e:
        messagebox.showerror("Erro de Conexão", f"Não foi possível conectar em {port_name}.\nErro: {e}")
        mudar_cor_circulo("red")

def criar_janela():
    root = tk.Tk()
    root.title("Controle")
    root.geometry("420x200")
    root.resizable(False, False)

    frame = ttk.Frame(root, padding=12)
    frame.pack(expand=True, fill="both")

    titulo = ttk.Label(frame, text="Controle (IMU/Joystick)", font=("Segoe UI", 13, "bold"))
    titulo.pack(pady=(0,10))

    porta_var = tk.StringVar(value="")

    botao_conectar = ttk.Button(
        frame,
        text="Conectar (sem calibração)",
        command=lambda: conectar_porta(porta_var.get(), root, botao_conectar, status_label, mudar_cor_circulo)
    )
    botao_conectar.pack(pady=10)

    footer = tk.Frame(root)
    footer.pack(side="bottom", fill="x", padx=10, pady=(6,10))

    status_label = tk.Label(footer, text="Aguardando seleção de porta...", font=("Segoe UI", 10))
    status_label.grid(row=0, column=0, sticky="w")

    portas = serial_ports()
    if portas:
        porta_var.set(portas[0])
    port_dropdown = ttk.Combobox(footer, textvariable=porta_var, values=portas, state="readonly", width=14)
    port_dropdown.grid(row=0, column=1, padx=8)

    circle = tk.Canvas(footer, width=18, height=18, highlightthickness=0)
    circle_item = circle.create_oval(2,2,16,16, fill="red", outline="")
    circle.grid(row=0, column=2, padx=6)

    def mudar_cor_circulo(c):
        circle.itemconfig(circle_item, fill=c)

    footer.columnconfigure(1, weight=1)
    root.mainloop()

if __name__ == "__main__":
    print("Iniciando GUI. Logs aparecerão no terminal. Sem calibração de joystick.")
    criar_janela()
