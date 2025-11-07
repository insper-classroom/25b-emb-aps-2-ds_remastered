# teste.py - atualizado com prints de debug e ajustes
# baseado na sua versão original (IMU -> clique direito; Joystick -> WASDE)
import sys
import glob
import serial
import pyautogui
pyautogui.PAUSE = 0
pyautogui.FAILSAFE = False
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
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

# Mapeamento de botão -> tecla (mantive para seus botões físicos)
BUTTON_KEYMAP = {
    BTN_ATACK_ID: 'f',
    BTN_CURA_ID:  'q',
    BTN_ROLL_ID:  'r',
    BTN_ESC_ID:   'esc'
}

# ----------------- SENSIBILIDADES / THRESHOLDS -----------------
IMU_MOVE_SCALE = 4          # escala para movimento do mouse vindo da IMU
# ADC no firmware: raw-> ((raw-2048)*16) => ~ -32768..+32767
JOY_KEY_THRESHOLD = 12000    # ajuste: limiar para considerar direção (faça tuning)
DIAGONAL_E_THRESHOLD = 20000 # se |x|>this e |y|>this -> dispara 'e'
# se achar que está muito sensível, aumente JOY_KEY_THRESHOLD ou DIAGONAL_E_THRESHOLD

# ----------------- Estado de teclas (para enviar keyDown/keyUp uma vez) ----------
key_state = {
    'w': False,
    'a': False,
    's': False,
    'd': False,
    'e': False
}

# ----------------- UTILS -----------------
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
    """IMU movimento: axis==AXIS_IMU_X => movimento X, AXIS_IMU_Y => Y.
       value já chega como signed int16 escalado pelo firmware."""
    if axis == AXIS_IMU_X:
        dx = int(value * IMU_MOVE_SCALE)
        if dx != 0:
            pyautogui.moveRel(dx, 0)
        print(f"[MOUSE] IMU X move: raw={value} scaled={dx}")
    elif axis == AXIS_IMU_Y:
        dy = int(value * IMU_MOVE_SCALE)
        if dy != 0:
            pyautogui.moveRel(0, dy)
        print(f"[MOUSE] IMU Y move: raw={value} scaled={dy}")

def handle_click_right(value):
    """IMU click (ID 2) -> clique direito único quando value != 0."""
    if value != 0:
        print(f"[CLICK] IMU click detected value={value} -> right click")
        try:
            pyautogui.click(button='right')
        except Exception as e:
            print("Erro em right-click:", e)

def handle_button(id_, value):
    """Trata botões físicos com keyDown/keyUp."""
    key = BUTTON_KEYMAP.get(id_)
    if not key:
        print(f"[BTN] Botão com ID {id_} não mapeado. value={value}")
        return
    if value != 0:
        print(f"[BTN] ID {id_} PRESSED -> keyDown {key}")
        try:
            pyautogui.keyDown(key)
        except Exception as e:
            print("Erro em keyDown:", e)
        # opcional: não setamos key_state aqui, pois são botões momentâneos
    else:
        print(f"[BTN] ID {id_} RELEASED -> keyUp {key}")
        try:
            pyautogui.keyUp(key)
        except Exception as e:
            print("Erro em keyUp:", e)

def handle_joystick(x_value, y_value):
    """
    x_value, y_value são signed int16 de -32768..+32767 (conforme firmware).
    Mapeia para W A S D e E (diagonal forte).
    Usa histerese implícita via key_state (só envia keyDown uma vez).
    """
    # DEBUG
    print(f"[JOY] raw x={x_value} y={y_value}")

    # Diagonal forte -> 'e'
    if abs(x_value) > DIAGONAL_E_THRESHOLD and abs(y_value) > DIAGONAL_E_THRESHOLD:
        key_down_if_needed('e')
    else:
        key_up_if_needed('e')

    # Vertical: assumo joystick pra cima => y positivo no firmware.
    # Se na prática estiver invertido, troque os sinais aqui.
    if y_value > JOY_KEY_THRESHOLD:
        key_down_if_needed('w')
        key_up_if_needed('s')
    elif y_value < -JOY_KEY_THRESHOLD:
        key_down_if_needed('s')
        key_up_if_needed('w')
    else:
        key_up_if_needed('w')
        key_up_if_needed('s')

    # Horizontal: right = D, left = A
    if x_value > JOY_KEY_THRESHOLD:
        key_down_if_needed('d')
        key_up_if_needed('a')
    elif x_value < -JOY_KEY_THRESHOLD:
        key_down_if_needed('a')
        key_up_if_needed('d')
    else:
        key_up_if_needed('a')
        key_up_if_needed('d')

# ----------------- SERIAL / PARSE -----------------
def parse_data(data):
    """Recebe 3 bytes: id (1 byte), value low, value high (little-endian signed)"""
    if len(data) < 3:
        return None, None
    id_ = data[0]
    # little-endian signed 16-bit
    value = int.from_bytes(data[1:3], byteorder='little', signed=True)
    return id_, value

# We keep last read joystick values to combine X and Y when they arrive individually
last_joy_x = 0
last_joy_y = 0

def controle(ser):
    global last_joy_x, last_joy_y
    print("Iniciando loop serial (IMU->right-click; Joystick->WASDE)...")
    try:
        while True:
            sync = ser.read(size=1)
            if not sync:
                continue
            if sync[0] != 0xFF:
                # ressincronizar se byte inválido
                print(f"[SYNC] Byte inesperado: {sync[0]:02X}, aguardando 0xFF...")
                continue
            data = ser.read(size=3)
            if len(data) < 3:
                # timeout parcial
                continue

            id_, value = parse_data(data)
            if id_ is None:
                continue

            # DEBUG: print recebido cru
            print(f"[RX] id={id_} value={value}")

            # Tratamento por ID
            if id_ == AXIS_IMU_X or id_ == AXIS_IMU_Y:
                move_mouse(id_, value)
            elif id_ == AXIS_IMU_CLICK:
                handle_click_right(value)
            elif id_ in BUTTON_KEYMAP:
                handle_button(id_, value)
            elif id_ == JOY_X_ID:
                last_joy_x = value
                # processa combinando com last_joy_y (se ambos estiverem em movimento, 'e' pode disparar)
                handle_joystick(last_joy_x, last_joy_y)
            elif id_ == JOY_Y_ID:
                last_joy_y = value
                handle_joystick(last_joy_x, last_joy_y)
            else:
                print(f"[WARN] ID desconhecido recebido: {id_} value={value}")

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
        # soltar todas as teclas caso saia
        release_all_movement_keys()

# ----------------- GUI / PORTA -----------------
def serial_ports():
    ports = []
    if sys.platform.startswith('win'):
        for i in range(1, 256):
            port = f'COM{i}'
            try:
                s = serial.Serial(port)
                s.close()
                ports.append(port)
            except (OSError, serial.SerialException):
                pass
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        ports = glob.glob('/dev/tty[A-Za-z0-9]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Plataforma não suportada.')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

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
    except KeyboardInterrupt:
        print("Encerrando via KeyboardInterrupt.")
    except Exception as e:
        messagebox.showerror("Erro de Conexão", f"Não foi possível conectar em {port_name}.\nErro: {e}")
        mudar_cor_circulo("red")

def criar_janela():
    root = tk.Tk()
    root.title("Controle de Mouse")
    root.geometry("400x250")
    root.resizable(False, False)
    dark_bg = "#2e2e2e"
    dark_fg = "#ffffff"
    accent_color = "#007acc"
    root.configure(bg=dark_bg)

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TFrame", background=dark_bg)
    style.configure("TLabel", background=dark_bg, foreground=dark_fg, font=("Segoe UI", 11))
    style.configure("TButton", font=("Segoe UI", 10, "bold"),
                    foreground=dark_fg, background="#444444", borderwidth=0)
    style.map("TButton", background=[("active", "#555555")])
    style.configure("Accent.TButton", font=("Segoe UI", 12, "bold"),
                    foreground=dark_fg, background=accent_color, padding=6)
    style.map("Accent.TButton", background=[("active", "#005f9e")])
    style.configure("TCombobox",
                    fieldbackground=dark_bg,
                    background=dark_bg,
                    foreground=dark_fg,
                    padding=4)
    style.map("TCombobox", fieldbackground=[("readonly", dark_bg)])

    frame_principal = ttk.Frame(root, padding="20")
    frame_principal.pack(expand=True, fill="both")

    titulo_label = ttk.Label(frame_principal, text="Controle de Mouse", font=("Segoe UI", 14, "bold"))
    titulo_label.pack(pady=(0, 10))

    porta_var = tk.StringVar(value="")

    botao_conectar = ttk.Button(
        frame_principal,
        text="Conectar e Iniciar Leitura",
        style="Accent.TButton",
        command=lambda: conectar_porta(porta_var.get(), root, botao_conectar, status_label, mudar_cor_circulo)
    )
    botao_conectar.pack(pady=10)

    footer_frame = tk.Frame(root, bg=dark_bg)
    footer_frame.pack(side="bottom", fill="x", padx=10, pady=(10, 0))

    status_label = tk.Label(footer_frame, text="Aguardando seleção de porta...", font=("Segoe UI", 11),
                            bg=dark_bg, fg=dark_fg)
    status_label.grid(row=0, column=0, sticky="w")

    portas_disponiveis = serial_ports()
    if portas_disponiveis:
        porta_var.set(portas_disponiveis[0])
    port_dropdown = ttk.Combobox(footer_frame, textvariable=porta_var,
                                 values=portas_disponiveis, state="readonly", width=10)
    port_dropdown.grid(row=0, column=1, padx=10)

    circle_canvas = tk.Canvas(footer_frame, width=20, height=20, highlightthickness=0, bg=dark_bg)
    circle_item = circle_canvas.create_oval(2, 2, 18, 18, fill="red", outline="")
    circle_canvas.grid(row=0, column=2, sticky="e")

    footer_frame.columnconfigure(1, weight=1)

    def mudar_cor_circulo(cor):
        circle_canvas.itemconfig(circle_item, fill=cor)

    root.mainloop()

if __name__ == "__main__":
    print("Iniciando GUI do controle. Abra o terminal para ver logs de debug.")
    criar_janela()
