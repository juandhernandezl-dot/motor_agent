#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
motor_gui.py  -  INTERFAZ GRAFICA (un cliente mas del bus)
==========================================================

Cuatro zonas:

  1) MANDO        : consigna (rpm), PERIODO DE MUESTREO (Ts) en caliente,
                    habilitar/parar, PWM manual, medidas.
  2) SENALES      : senal medida (rpm) contra la consigna y senal de control
                    (PWM), en ventana movil.
  3) CONTROLADORES: lista DESCUBIERTA SOLA de la carpeta (cualquier .py con un
                    dict CONTROLLER; ver controller_registry.py), formulario de
                    parametros construido al vuelo, y lanzamiento como proceso
                    aparte. Cada uno abre su CONSOLA (salida + entrada de
                    teclado, asi funcionan tambien los menus interactivos).
  4) CEREBRO      : mapa de calor de la tabla del agente RL que este corriendo,
                    en directo, con la celda del estado actual resaltada.

La GUI no ejecuta ningun lazo de control: lanza procesos y escucha el bus. Al
cerrarla, manda Ctrl+C a los controladores para que aparquen el motor ellos.

    python3 motor_bus.py --sim     # 1) bus
    python3 motor_gui.py           # 2) esto; los controladores se lanzan desde aqui

Solo libreria estandar (tkinter).
"""

import argparse
import math
import os
import queue
import signal
import subprocess
import threading
import time
from collections import deque

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:
    raise SystemExit("Falta tkinter.  Debian/Ubuntu:  sudo apt install python3-tk")

from motor_client import BusClient, BusError, BusRechazado
from controller_registry import descubrir, construir_cmd

VENTANA_S = 30.0
REFRESCO_MS = 80
MAX_LINEAS_CONSOLA = 1200

BG = "#12151c"
PANEL = "#1b1f2a"
FG = "#e6e9ef"
SUAVE = "#8b93a7"
ACENTO = "#4da3ff"
VERDE = "#3ddc84"
ROJO = "#ff5f56"
AMBAR = "#ffbd2e"
REJILLA = "#2a3040"


# =============================================================================
# PALETAS
# =============================================================================

def _mix(c1, c2, t):
    a = tuple(int(c1[i:i + 2], 16) for i in (1, 3, 5))
    b = tuple(int(c2[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def color_valor(t):
    t = max(0.0, min(1.0, t))
    if t < 0.5:
        return _mix("#1b2a6b", "#1a9e6f", t / 0.5)
    return _mix("#1a9e6f", "#f5d020", (t - 0.5) / 0.5)


def color_accion(i, n):
    if n <= 1:
        return "#666666"
    t = i / (n - 1.0)
    if t < 0.5:
        return _mix("#2f6fed", "#3a4152", t / 0.5)
    return _mix("#3a4152", "#ff5f56", (t - 0.5) / 0.5)


# =============================================================================
# PROCESOS DE CONTROL
# =============================================================================

class Proceso:
    """
    Un controlador lanzado desde la GUI. Se comunica por tuberias:
      * su stdout/stderr -> cola -> consola de su pestana
      * lo que escribas en la consola -> su stdin (para los menus interactivos)

    'detener()' manda Ctrl+C (SIGINT, o CTRL_BREAK en Windows), NO un kill: asi
    el controlador ejecuta su bloque 'finally', para el motor y suelta el mando.
    Matarlo a lo bruto dejaria el motor girando hasta que saltara el watchdog.
    """

    def __init__(self, ctl, cmd, cola):
        self.ctl, self.cmd, self.cola = ctl, cmd, cola
        kw = {}
        if os.name == "nt":
            kw["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        self.p = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            stdin=subprocess.PIPE, text=True, bufsize=1,
            cwd=os.path.dirname(os.path.abspath(ctl.ruta)),
            errors="replace", **kw)
        self.t0 = time.time()
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        for linea in self.p.stdout:
            self.cola.put((self.ctl.clave, "out", linea.rstrip("\n")))
        rc = self.p.wait()
        self.cola.put((self.ctl.clave, "fin", rc))

    def vivo(self):
        return self.p.poll() is None

    def enviar(self, txt):
        if not self.vivo():
            return False
        try:
            self.p.stdin.write(txt + "\n")
            self.p.stdin.flush()
            return True
        except (OSError, ValueError):
            return False

    def detener(self):
        if not self.vivo():
            return
        try:
            if os.name == "nt":
                self.p.send_signal(signal.CTRL_BREAK_EVENT)
            else:
                self.p.send_signal(signal.SIGINT)
        except Exception:
            pass

    def matar(self):
        try:
            self.p.kill()
        except Exception:
            pass


# =============================================================================
# UTILIDAD: marco con scroll (para el formulario de parametros)
# =============================================================================

class MarcoScroll(tk.Frame):
    def __init__(self, padre, alto=190, **kw):
        super().__init__(padre, bg=PANEL, **kw)
        self.canvas = tk.Canvas(self, bg=PANEL, highlightthickness=0, height=alto)
        sb = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.interior = tk.Frame(self.canvas, bg=PANEL)
        self.interior.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self._win = self.canvas.create_window((0, 0), window=self.interior,
                                              anchor="nw")
        self.canvas.bind("<Configure>",
                         lambda e: self.canvas.itemconfigure(self._win,
                                                             width=e.width))
        self.canvas.configure(yscrollcommand=sb.set)
        self.canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

    def limpiar(self):
        for w in self.interior.winfo_children():
            w.destroy()


# =============================================================================
# APP
# =============================================================================

class MotorGUI:
    def __init__(self, root, bus: BusClient):
        self.root, self.bus = root, bus
        self.host, self.port = bus.host, bus.port
        self.eventos = queue.Queue()
        self.cola_proc = queue.Queue()
        # Historial acotado por TIEMPO en _on_tel (con tope duro de seguridad).
        # El maxlen fijo de antes (VENTANA_S*60) asumia 60 Hz: con Ts=10 ms
        # (100 Hz) la ventana de 30 s se quedaba en 18 s reales.
        self.hist = deque(maxlen=12000)
        self.tabla = None
        self.status = {}
        self.owner = "manual"
        self.enabled = False
        self.err_prev = None
        self.err_actual = 0.0
        self.derr_actual = 0.0
        self.banda = deque(maxlen=200)
        self.err2 = deque(maxlen=200)
        self.t0 = time.time()
        self.rpm_fs = float(bus.info.get("rpm_fs", 3000))
        self.ts_ms = int(bus.info.get("ts_ms", 50))
        self.controladores = []
        self.ctl_actual = None
        self.widgets_param = []
        self.procesos = {}          # clave -> Proceso
        self.consolas = {}          # clave -> dict(frame, text, entry)
        # --- caches de dibujo (dibujo INCREMENTAL, ver _pintar_tabla/_strip) ---
        self._strips = {}           # canvas -> items reutilizables de la grafica
        self._tab_cache = None      # rectangulos del mapa de calor ya creados
        self._leg_firma = None      # que leyenda esta dibujada (evita repintarla)
        self._leg_lbls = None       # ids de las etiquetas vmin/vmax de la leyenda
        self._disc_cache = None     # Discretizer cacheado para la celda actual
        self._tel_nueva = True      # hay telemetria sin dibujar
        self._tabla_nueva = False   # hay tabla sin dibujar (coalescing por tick)
        # --- estado de conexion (bus TCP y enlace serie del Arduino) ---
        self.conectado = True
        self.link_ok = True
        self.link_msg = ""
        self._reconectando = False
        self._parar_hilos = threading.Event()
        # --- publicador asincrono: NADA de lo que teclees debe congelar la
        # ventana. Las publicaciones se guardan aqui y un hilo aparte las
        # envia; si el bus tarda o esta caido, la GUI sigue respondiendo. Se
        # queda solo con el ULTIMO valor por topico (coalescing), asi arrastrar
        # un slider no acumula cientos de envios pendientes. ---
        self._bus_lock = threading.Lock()
        self._pub_pendientes = {}
        # Cola APARTE, en FIFO (sin coalescing), para eventos de una sola vez
        # que NO deben pisarse entre si -- ej. "controller/ack": si llegan dos
        # pedidos de cambio de modo casi al mismo tiempo (dos mensajes de
        # Telegram seguidos), cada uno necesita SU PROPIO ack; coalescing por
        # topico (como hace _pub_pendientes) perderia el primero si el
        # segundo lo pisa antes de que el hilo alcance a enviarlo, y ese
        # pedido se quedaria esperando hasta el timeout del lado del bot
        # aunque la GUI si lo haya resuelto.
        self._pub_unicos = deque()
        self._pub_lock = threading.Lock()
        self._pub_evento = threading.Event()
        threading.Thread(target=self._pub_worker, daemon=True).start()
        self._construir()
        self._suscribir()
        self._recargar_controladores()
        self._on_config(bus.get("config") or {"ts_ms": self.ts_ms})
        self.root.after(REFRESCO_MS, self._tick)

    def _suscribir(self):
        for t in ("telemetry", "agent/table", "agent/status", "mode", "log",
                  "setpoint", "enable", "config", "link",
                  "controller/launch", "controller/stop"):
            self.bus.subscribe(t, lambda d, t=t: self.eventos.put((t, d)))
        self.bus.on_close = lambda: self.eventos.put(("__cerrado__", None))

    # =========================================================================
    # PUBLICADOR ASINCRONO Y RECONEXION
    # =========================================================================
    def _pub_worker(self):
        """Hilo aparte: vacia lo pendiente y lo envia. Un bus.publish() lento
        (timeout de hasta 5s) bloquea SOLO a este hilo, nunca a Tkinter."""
        while not self._parar_hilos.is_set():
            if not self._pub_evento.wait(timeout=0.2):
                continue
            self._pub_evento.clear()
            with self._pub_lock:
                pendientes = self._pub_pendientes
                self._pub_pendientes = {}
                unicos = list(self._pub_unicos)
                self._pub_unicos.clear()
            for topico, data in pendientes.items():
                self._publicar_uno(topico, data)
            for topico, data in unicos:
                self._publicar_uno(topico, data)

    def _publicar_uno(self, topico, data):
        """Un solo intento de publish, distinguiendo RECHAZO de protocolo
        (BusRechazado: el bus respondio 'ok:false', ej. arbitraje de
        'control' cuando quien publica no tiene el mando) de una caida REAL
        de conexion (BusError pelada / cualquier otra excepcion). Solo la
        segunda debe disparar reconexion -- un rechazo es una respuesta
        normal del protocolo, no significa que el bus este caido."""
        try:
            with self._bus_lock:
                bus = self.bus
            bus.publish(topico, data)
        except BusRechazado as e:
            # El bus esta vivo y respondio; solo rechazo el comando.
            self.eventos.put(("__pub_error__", f"{topico}: {e}"))
        except Exception as e:
            self.eventos.put(("__pub_error__", f"{topico}: {e}"))
            self._quiza_reconectar()

    def _quiza_reconectar(self):
        if self._reconectando or self._parar_hilos.is_set():
            return
        self._reconectando = True
        threading.Thread(target=self._reconectar_hilo, daemon=True).start()

    def _reconectar_hilo(self):
        intento = 0
        while not self._parar_hilos.is_set():
            intento += 1
            self.eventos.put(("__reconectando__", intento))
            try:
                nuevo = BusClient("gui", self.host, self.port, timeout=3.0).connect()
            except OSError:
                time.sleep(min(2.0 * intento, 10.0))
                continue
            with self._bus_lock:
                try:
                    self.bus.close()
                except Exception:
                    pass
                self.bus = nuevo
            self._suscribir()
            self.eventos.put(("__reconectado__", None))
            self._reconectando = False
            return

    # =========================================================================
    # CONSTRUCCION
    # =========================================================================
    def _construir(self):
        r = self.root
        r.title("Control de velocidad por RL  -  Arduino Mega")
        r.configure(bg=BG)
        r.geometry("1420x880")
        r.minsize(1180, 720)

        st = ttk.Style()
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        st.configure(".", background=PANEL, foreground=FG, fieldbackground="#262c3a")
        st.configure("TFrame", background=PANEL)
        st.configure("TLabel", background=PANEL, foreground=FG)
        st.configure("Suave.TLabel", foreground=SUAVE)
        st.configure("Titulo.TLabel", font=("TkDefaultFont", 11, "bold"))
        st.configure("TRadiobutton", background=PANEL, foreground=FG)
        st.configure("TCheckbutton", background=PANEL, foreground=FG)
        st.configure("TNotebook", background=BG, borderwidth=0)
        st.configure("TNotebook.Tab", background=PANEL, foreground=SUAVE,
                     padding=(10, 4))
        st.map("TNotebook.Tab", background=[("selected", "#2b3242")],
               foreground=[("selected", FG)])
        st.configure("TCombobox", fieldbackground="#262c3a", background="#262c3a")

        top = tk.Frame(r, bg=PANEL, padx=10, pady=6)
        top.pack(fill="x", side="top")
        self.lb_estado = tk.Label(top, text="conectando...", bg=PANEL, fg=SUAVE,
                                  font=("TkDefaultFont", 10, "bold"))
        self.lb_estado.pack(side="left")
        tk.Button(top, text="PARADA", bg=ROJO, fg="white", relief="flat",
                  padx=18, pady=4, font=("TkDefaultFont", 10, "bold"),
                  command=self._parada).pack(side="right")
        self.btn_en = tk.Button(top, text="Habilitar motor", bg=VERDE,
                                fg="#0b1410", relief="flat", padx=14, pady=4,
                                command=self._toggle_enable)
        self.btn_en.pack(side="right", padx=8)

        cuerpo = tk.Frame(r, bg=BG)
        cuerpo.pack(fill="both", expand=True, padx=8, pady=8)
        self._col_mando(cuerpo)
        self._col_centro(cuerpo)
        self._col_derecha(cuerpo)
        r.protocol("WM_DELETE_WINDOW", self._cerrar)

    # ---- columna 1: mando ----------------------------------------------------
    def _col_mando(self, padre):
        izq = tk.Frame(padre, bg=PANEL, padx=10, pady=10, width=290)
        izq.pack(side="left", fill="y")
        izq.pack_propagate(False)

        ttk.Label(izq, text="CONSIGNA", style="Titulo.TLabel").pack(anchor="w")
        self.sp_var = tk.DoubleVar(value=1200.0)
        self.lb_sp = tk.Label(izq, text="1200 rpm", bg=PANEL, fg=ACENTO,
                              font=("TkDefaultFont", 20, "bold"))
        self.lb_sp.pack(anchor="w")
        tk.Scale(izq, from_=0, to=self.rpm_fs * 0.9, orient="horizontal",
                 variable=self.sp_var, resolution=10, showvalue=False, bg=PANEL,
                 fg=FG, troughcolor=REJILLA, highlightthickness=0,
                 command=self._sp_cambia, length=250).pack(fill="x")
        f = tk.Frame(izq, bg=PANEL)
        f.pack(fill="x", pady=(2, 10))
        for v in (300, 800, 1200, 1800, 2400):
            tk.Button(f, text=str(v), bg=REJILLA, fg=FG, relief="flat", padx=3,
                      command=lambda v=v: self._sp_set(v)).pack(side="left", padx=2)

        ttk.Label(izq, text="MUESTREO (Ts)", style="Titulo.TLabel").pack(anchor="w")
        fm = tk.Frame(izq, bg=PANEL)
        fm.pack(fill="x")
        self.ts_var = tk.IntVar(value=self.ts_ms)
        tk.Spinbox(fm, from_=10, to=1000, increment=5, textvariable=self.ts_var,
                   width=6, bg="#262c3a", fg=FG, buttonbackground=REJILLA,
                   relief="flat", insertbackground=FG).pack(side="left")
        tk.Label(fm, text="ms", bg=PANEL, fg=SUAVE).pack(side="left", padx=(4, 8))
        tk.Button(fm, text="Aplicar", bg=ACENTO, fg="#04101d", relief="flat",
                  padx=10, command=self._aplicar_ts).pack(side="left")
        self.lb_ts = tk.Label(izq, text="", bg=PANEL, fg=SUAVE, justify="left",
                              wraplength=265, font=("TkFixedFont", 8))
        self.lb_ts.pack(anchor="w", pady=(2, 10))

        ttk.Label(izq, text="MANDO", style="Titulo.TLabel").pack(anchor="w")
        self.lb_owner = tk.Label(izq, text="manual", bg=PANEL, fg=AMBAR,
                                 font=("TkDefaultFont", 12, "bold"))
        self.lb_owner.pack(anchor="w")
        tk.Button(izq, text="Recuperar mando manual", bg=REJILLA, fg=FG,
                  relief="flat", pady=3, command=self._mando_manual).pack(fill="x")

        ttk.Label(izq, text="PWM MANUAL", style="Titulo.TLabel").pack(anchor="w",
                                                                     pady=(10, 0))
        self.pwm_var = tk.IntVar(value=0)
        self.sc_pwm = tk.Scale(izq, from_=0, to=255, orient="horizontal",
                               variable=self.pwm_var, showvalue=True, bg=PANEL,
                               fg=FG, troughcolor=REJILLA, highlightthickness=0,
                               command=self._pwm_cambia, length=250)
        self.sc_pwm.pack(fill="x")

        ttk.Label(izq, text="MEDIDAS", style="Titulo.TLabel").pack(anchor="w",
                                                                   pady=(10, 0))
        self.lb_med = tk.Label(izq, text="-", bg=PANEL, fg=FG, justify="left",
                               font=("TkFixedFont", 10))
        self.lb_med.pack(anchor="w")

        ttk.Label(izq, text="CONTROLADOR ACTIVO",
                  style="Titulo.TLabel").pack(anchor="w", pady=(10, 0))
        self.lb_algo = tk.Label(izq, text="ninguno", bg=PANEL, fg=SUAVE,
                                justify="left", font=("TkFixedFont", 10))
        self.lb_algo.pack(anchor="w")

    # ---- columna 2: graficas + consolas --------------------------------------
    def _col_centro(self, padre):
        centro = tk.Frame(padre, bg=BG)
        centro.pack(side="left", fill="both", expand=True, padx=8)
        graf = tk.Frame(centro, bg=BG)
        graf.pack(fill="both", expand=True)
        self.c_rpm = tk.Canvas(graf, bg=PANEL, highlightthickness=0, height=250)
        self.c_rpm.pack(fill="both", expand=True)
        self.c_pwm = tk.Canvas(graf, bg=PANEL, highlightthickness=0, height=120)
        self.c_pwm.pack(fill="x", pady=(6, 0))

        cons = tk.Frame(centro, bg=PANEL)
        cons.pack(fill="both", expand=True, pady=(8, 0))
        cab = tk.Frame(cons, bg=PANEL)
        cab.pack(fill="x", padx=6, pady=(4, 0))
        ttk.Label(cab, text="CONSOLAS", style="Titulo.TLabel").pack(side="left")
        ttk.Label(cab, text="   escribe y pulsa Enter para responder al controlador",
                  style="Suave.TLabel").pack(side="left")
        self.nb = ttk.Notebook(cons)
        self.nb.pack(fill="both", expand=True, padx=4, pady=4)
        self._consola_bus()

    def _consola_bus(self):
        f = tk.Frame(self.nb, bg=PANEL)
        self.txt_bus = tk.Text(f, bg="#0e1118", fg=SUAVE, relief="flat",
                               font=("TkFixedFont", 9), height=8, wrap="none")
        sb = ttk.Scrollbar(f, orient="vertical", command=self.txt_bus.yview)
        self.txt_bus.configure(yscrollcommand=sb.set)
        self.txt_bus.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.nb.add(f, text="bus / arduino")

    # ---- columna 3: lanzador + tabla -----------------------------------------
    def _col_derecha(self, padre):
        der = tk.Frame(padre, bg=PANEL, padx=10, pady=10, width=440)
        der.pack(side="left", fill="y")
        der.pack_propagate(False)

        cab = tk.Frame(der, bg=PANEL)
        cab.pack(fill="x")
        ttk.Label(cab, text="CONTROLADORES", style="Titulo.TLabel").pack(side="left")
        tk.Button(cab, text="recargar", bg=REJILLA, fg=SUAVE, relief="flat",
                  font=("TkDefaultFont", 8),
                  command=self._recargar_controladores).pack(side="right")
        self.cb_ctl = ttk.Combobox(der, state="readonly", values=[])
        self.cb_ctl.pack(fill="x", pady=(4, 2))
        self.cb_ctl.bind("<<ComboboxSelected>>", self._elegir_controlador)
        self.lb_desc = tk.Label(der, text="", bg=PANEL, fg=SUAVE, justify="left",
                                wraplength=415, font=("TkDefaultFont", 8))
        self.lb_desc.pack(anchor="w")
        self.form = MarcoScroll(der, alto=170)
        self.form.pack(fill="x", pady=4)
        fb = tk.Frame(der, bg=PANEL)
        fb.pack(fill="x")
        tk.Button(fb, text="Lanzar", bg=VERDE, fg="#0b1410", relief="flat",
                  pady=4, command=self._lanzar).pack(side="left", fill="x",
                                                     expand=True)
        tk.Button(fb, text="Detener (Ctrl+C)", bg=AMBAR, fg="#1a1206",
                  relief="flat", pady=4,
                  command=self._detener_actual).pack(side="left", fill="x",
                                                     expand=True, padx=(6, 0))

        ttk.Label(der, text="TABLA DEL AGENTE (en vivo)",
                  style="Titulo.TLabel").pack(anchor="w", pady=(10, 0))
        self.lb_tabla = tk.Label(der, text="esperando a un algoritmo...", bg=PANEL,
                                 fg=SUAVE, justify="left", font=("TkFixedFont", 9))
        self.lb_tabla.pack(anchor="w")
        self.vista = tk.StringVar(value="accion")
        vf = tk.Frame(der, bg=PANEL)
        vf.pack(anchor="w")
        for txt, val in (("por accion", "accion"), ("por valor", "valor")):
            ttk.Radiobutton(vf, text=txt, value=val, variable=self.vista,
                            command=self._pintar_tabla).pack(side="left", padx=4)
        self.c_tab = tk.Canvas(der, bg=PANEL, highlightthickness=0)
        self.c_tab.pack(fill="both", expand=True, pady=4)
        self.c_leg = tk.Canvas(der, bg=PANEL, highlightthickness=0, height=54)
        self.c_leg.pack(fill="x")
        self.c_tab.bind("<Configure>", lambda e: self._pintar_tabla())

    # =========================================================================
    # LANZADOR DINAMICO
    # =========================================================================
    def _recargar_controladores(self):
        aqui = os.path.dirname(os.path.abspath(__file__))
        self.controladores = descubrir([aqui, os.path.join(aqui, "controladores")])
        etiquetas = [f"[{c.familia}] {c.nombre}" for c in self.controladores]
        self.cb_ctl.configure(values=etiquetas)
        self._log(f"[gui] {len(etiquetas)} controladores detectados")
        if etiquetas:
            i = 0
            if self.ctl_actual:
                for j, c in enumerate(self.controladores):
                    if c.clave == self.ctl_actual.clave:
                        i = j
                        break
            self.cb_ctl.current(i)
            self._elegir_controlador()
        else:
            self.lb_desc.config(
                text="Ningun .py de la carpeta declara un dict CONTROLLER.\n"
                     "Mira la cabecera de controller_registry.py.")

    def _elegir_controlador(self, _e=None):
        i = self.cb_ctl.current()
        if i < 0 or i >= len(self.controladores):
            return
        c = self.controladores[i]
        self.ctl_actual = c
        self.lb_desc.config(text=f"{c.fichero}   ·   clave '{c.clave}'\n"
                                 f"{c.descripcion.strip()}")
        self._construir_form(c)

    def _construir_form(self, c):
        """Formulario construido AL VUELO desde el manifiesto del controlador."""
        self.form.limpiar()
        self.widgets_param = []
        for p in c.params:
            fila = tk.Frame(self.form.interior, bg=PANEL)
            fila.pack(fill="x", pady=1)
            tipo = p.get("tipo", "str")
            etiqueta = p.get("etiqueta", p.get("flag", "?"))
            if tipo == "bool":
                var = tk.BooleanVar(value=bool(p.get("default")))
                ttk.Checkbutton(fila, text=etiqueta, variable=var).pack(anchor="w")
            elif tipo == "choice":
                tk.Label(fila, text=etiqueta, bg=PANEL, fg=SUAVE, width=15,
                         anchor="w", font=("TkDefaultFont", 8)).pack(side="left")
                opciones = p.get("opciones", [])
                cb = ttk.Combobox(fila, state="readonly", width=28,
                                  values=[o[0] for o in opciones])
                cb.pack(side="left", fill="x", expand=True)
                idx = next((k for k, o in enumerate(opciones)
                            if o[1] == p.get("default", "")), 0)
                if opciones:
                    cb.current(idx)
                var = ("choice", cb, opciones)
            else:
                tk.Label(fila, text=etiqueta, bg=PANEL, fg=SUAVE, width=15,
                         anchor="w", font=("TkDefaultFont", 8)).pack(side="left")
                var = tk.StringVar(value="" if p.get("default") is None
                                   else str(p["default"]))
                tk.Entry(fila, textvariable=var, bg="#262c3a", fg=FG,
                         relief="flat", insertbackground=FG,
                         width=10).pack(side="left")
                if p.get("ayuda"):
                    tk.Label(fila, text=p["ayuda"], bg=PANEL, fg="#5f6982",
                             font=("TkDefaultFont", 7)).pack(side="left", padx=4)
            self.widgets_param.append(var)

    def _valores_form(self):
        vals = {}
        for i, v in enumerate(self.widgets_param):
            if isinstance(v, tuple) and v[0] == "choice":
                _, cb, opciones = v
                j = cb.current()
                vals[i] = opciones[j][1] if 0 <= j < len(opciones) else ""
            else:
                vals[i] = v.get()
        return vals

    def _lanzar(self):
        c = self.ctl_actual
        if not c:
            return
        if c.clave in self.procesos and self.procesos[c.clave].vivo():
            self._log(f"[gui] '{c.clave}' ya esta corriendo; detenlo primero.")
            self._foco_consola(c.clave)
            return
        try:
            cmd = construir_cmd(c, self._valores_form())
        except Exception as e:
            self._log(f"[gui] parametros invalidos: {e}")
            return
        self._consola(c)
        self._escribir(c.clave, "$ " + " ".join(cmd), "gui")
        try:
            self.procesos[c.clave] = Proceso(c, cmd, self.cola_proc)
        except Exception as e:
            self._escribir(c.clave, f"[gui] no se pudo lanzar: {e}", "gui")
            return
        self._log(f"[gui] lanzado {c.fichero} (pid {self.procesos[c.clave].p.pid})")
        self._foco_consola(c.clave)

    def _detener_actual(self):
        clave = self._clave_pestana() or (self.ctl_actual.clave
                                          if self.ctl_actual else None)
        p = self.procesos.get(clave)
        if not p or not p.vivo():
            self._log("[gui] no hay ningun proceso vivo en esta pestana.")
            return
        p.detener()
        self._escribir(clave, "[gui] Ctrl+C enviado: parando el motor y soltando "
                              "el mando...", "gui")

    # =========================================================================
    # CONTROL REMOTO (ej. desde bot.py / el agente de Telegram, via el bus)
    # =========================================================================
    # Nadie mas que esta GUI toca subprocesos (Popen/Ctrl+C): asi evitamos que
    # el bot lance su propia copia de qlearning peleando por el mismo
    # qtable_motor_qlearning.json. El bot solo publica "quiero que el motor
    # pase a X" en el topico "controller/launch"; esta seccion es la que de
    # verdad detiene lo que este vivo y lanza lo pedido (o solo cambia el
    # modo, si X es manual/free, que no tienen script propio).
    #
    # Para RL, el default de fabrica de "Modo"/"Ritmo" es "" (menu
    # interactivo en la consola). Un lanzamiento remoto no tiene a nadie
    # sentado en esa consola para contestar el menu, asi que aqui se fuerza
    # "--controlar --directo" (usar la politica ya aprendida, sin
    # pre-entrenar) en vez de heredar ese default a ciegas.
    _REMOTO_OVERRIDES = {
        "qlearning": {"Modo": "--controlar", "Ritmo": "--directo"},
        "sarsa_lambda": {"Modo": "--controlar", "Ritmo": "--directo"},
        "reinforce": {"Modo": "--controlar", "Ritmo": "--directo"},
    }

    def _valores_remotos(self, c, data=None):
        """'data' es el payload crudo de "controller/launch". Si trae
        'entrenar'/'ritmo' (ver bot.py::_cambiar_controlador_sync), pisan el
        default de _REMOTO_OVERRIDES; si no vienen, el comportamiento es
        IDÉNTICO al de siempre (--controlar --directo)."""
        overrides = dict(self._REMOTO_OVERRIDES.get(c.clave, {}))
        if data:
            if data.get("entrenar"):
                overrides["Modo"] = data["entrenar"]
            if data.get("ritmo"):
                overrides["Ritmo"] = data["ritmo"]
        return {i: overrides.get(p.get("etiqueta", ""), p.get("default"))
                for i, p in enumerate(c.params)}

    def _publicar_ack(self, clave, ok, error=None):
        data = {"clave": clave, "ok": ok}
        if error:
            data["error"] = error
        self._pub_unico("controller/ack", data)

    def _on_remote_launch(self, data):
        clave = data.get("clave")
        src = data.get("src", "remoto")

        vivos = [k for k, p in self.procesos.items() if p.vivo()]
        for k in vivos:
            self.procesos[k].detener()
            self._escribir(k, f"[gui] Ctrl+C enviado (pedido remoto de {src}, "
                              f"cambio a '{clave}').", "gui")

        if clave in ("manual", "free"):
            self._pub("mode", {"owner": clave})
            self._log(f"[remoto] {src} pidio modo '{clave}'" +
                      (f" (detuvo {', '.join(vivos)})" if vivos else ""))
            self._publicar_ack(clave, True)
            return

        c = next((ctl for ctl in self.controladores if ctl.clave == clave), None)
        if c is None:
            self._log(f"[remoto] {src} pidio '{clave}': no esta entre los "
                      f"controladores descubiertos (¿existe el .py? ¿tiene "
                      f"'recargar'?).")
            self._publicar_ack(clave, False, "controlador no encontrado")
            return

        try:
            cmd = construir_cmd(c, self._valores_remotos(c, data))
        except Exception as e:
            self._log(f"[remoto] parametros invalidos para '{clave}': {e}")
            self._publicar_ack(clave, False, f"parametros invalidos: {e}")
            return

        self._consola(c)
        self._escribir(c.clave, "$ " + " ".join(cmd) +
                       f"   (lanzado remotamente por {src})", "gui")
        try:
            self.procesos[c.clave] = Proceso(c, cmd, self.cola_proc)
        except Exception as e:
            self._escribir(c.clave, f"[gui] no se pudo lanzar: {e}", "gui")
            self._publicar_ack(clave, False, f"no se pudo lanzar: {e}")
            return

        self.ctl_actual = c
        self._log(f"[remoto] {src} lanzo {c.fichero} "
                  f"(pid {self.procesos[c.clave].p.pid})" +
                  (f", detuvo {', '.join(vivos)}" if vivos else ""))
        self._foco_consola(c.clave)
        self._publicar_ack(clave, True)

    def _on_remote_stop(self, data):
        clave = data.get("clave")  # None = detener todo lo que este vivo
        src = data.get("src", "remoto")
        objetivo = [clave] if clave else list(self.procesos.keys())
        detenidos = []
        for k in objetivo:
            p = self.procesos.get(k)
            if p and p.vivo():
                p.detener()
                self._escribir(k, f"[gui] Ctrl+C enviado (pedido remoto de "
                                  f"{src}).", "gui")
                detenidos.append(k)
        self._log(f"[remoto] {src} pidio detener" +
                  (f" {', '.join(detenidos)}" if detenidos
                   else ": nada estaba corriendo."))
        self._publicar_ack(clave or "todos", True)

    def _clave_pestana(self):
        try:
            actual = self.nb.select()
            for clave, c in self.consolas.items():
                if str(c["frame"]) == actual:
                    return clave
        except tk.TclError:
            pass
        return None

    # ---- consolas ------------------------------------------------------------
    def _consola(self, c):
        if c.clave in self.consolas:
            return
        f = tk.Frame(self.nb, bg=PANEL)
        txt = tk.Text(f, bg="#0e1118", fg="#cfd6e6", relief="flat",
                      font=("TkFixedFont", 9), height=8, wrap="none")
        sb = ttk.Scrollbar(f, orient="vertical", command=txt.yview)
        txt.configure(yscrollcommand=sb.set)
        txt.tag_configure("gui", foreground=ACENTO)
        txt.tag_configure("fin", foreground=AMBAR)
        txt.tag_configure("tu", foreground=VERDE)
        barra = tk.Frame(f, bg=PANEL)
        tk.Label(barra, text="stdin >", bg=PANEL, fg=SUAVE,
                 font=("TkFixedFont", 9)).pack(side="left", padx=(4, 4))
        ent = tk.Entry(barra, bg="#262c3a", fg=FG, relief="flat",
                       insertbackground=FG)
        ent.bind("<Return>", lambda e, k=c.clave: self._enviar_stdin(k))
        ent.pack(side="left", fill="x", expand=True, pady=3)
        tk.Button(barra, text="enviar", bg=REJILLA, fg=FG, relief="flat",
                  command=lambda k=c.clave: self._enviar_stdin(k)).pack(side="left",
                                                                        padx=4)
        tk.Button(barra, text="limpiar", bg=REJILLA, fg=SUAVE, relief="flat",
                  command=lambda k=c.clave: self.consolas[k]["text"].delete(
                      "1.0", "end")).pack(side="left")
        barra.pack(side="bottom", fill="x")
        txt.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.nb.add(f, text=c.clave)
        self.consolas[c.clave] = {"frame": f, "text": txt, "entry": ent}

    def _foco_consola(self, clave):
        if clave in self.consolas:
            self.nb.select(self.consolas[clave]["frame"])

    def _escribir(self, clave, txt, tag=None):
        c = self.consolas.get(clave)
        if not c:
            return
        t = c["text"]
        t.insert("end", txt + "\n", tag)
        if int(t.index("end-1c").split(".")[0]) > MAX_LINEAS_CONSOLA:
            t.delete("1.0", "200.0")
        t.see("end")

    def _enviar_stdin(self, clave):
        c = self.consolas.get(clave)
        p = self.procesos.get(clave)
        if not c:
            return
        txt = c["entry"].get()
        c["entry"].delete(0, "end")
        if not p or not p.vivo():
            self._escribir(clave, "[gui] el proceso ya no esta vivo.", "gui")
            return
        self._escribir(clave, f"> {txt}", "tu")
        p.enviar(txt)

    # =========================================================================
    # MANDO
    # =========================================================================
    def _pub(self, topico, data):
        """Encola la publicacion (no bloquea Tkinter). Si dos llamadas seguidas
        tocan el mismo topico antes de que el hilo envie la primera (p. ej.
        arrastrar el slider), solo se manda la mas reciente."""
        with self._pub_lock:
            self._pub_pendientes[topico] = data
        self._pub_evento.set()

    def _pub_unico(self, topico, data):
        """Como _pub(), pero para eventos de una sola vez que NO deben
        pisarse entre si (ej. "controller/ack" ante dos pedidos casi
        simultaneos). Cada llamada se entrega, en orden, sin coalescing."""
        with self._pub_lock:
            self._pub_unicos.append((topico, data))
        self._pub_evento.set()

    def _sp_cambia(self, _v=None):
        v = float(self.sp_var.get())
        self.lb_sp.config(text=f"{v:.0f} rpm")
        self._pub("setpoint", {"rpm": v})

    def _sp_set(self, v):
        self.sp_var.set(v)
        self._sp_cambia()

    def _aplicar_ts(self):
        try:
            ts = int(self.ts_var.get())
        except (tk.TclError, ValueError):
            self._log("[gui] Ts debe ser un entero de ms.")
            return
        self._pub("config", {"ts_ms": ts})

    def _pwm_cambia(self, _v=None):
        if self.owner != "manual":
            return
        # 'src' explicito: el cliente TCP de la GUI se llama "gui", pero el
        # 'owner' que autoriza 'control' en modo manual es literalmente la
        # cadena "manual" (ver arbitraje en motor_bus.py: publicar_desde()).
        # Sin este 'src', el bus pondria src="gui" por defecto y rechazaria
        # SIEMPRE el PWM manual (owner="manual" != src="gui"), aunque el
        # modo sea manual de verdad -- ese era el bug real, no una caida
        # de conexion.
        self._pub("control", {"pwm": int(self.pwm_var.get()), "src": "manual"})

    def _toggle_enable(self):
        self._pub("enable", {"on": not self.enabled})

    def _parada(self):
        for clave, p in self.procesos.items():
            if p.vivo():
                p.detener()
                self._escribir(clave, "[gui] PARADA general: Ctrl+C enviado.", "gui")
        self._pub("mode", {"owner": "manual"})
        self._pub("control", {"pwm": 0, "src": "manual"})
        self._pub("enable", {"on": False})
        self.pwm_var.set(0)
        self._log("[gui] PARADA: mando a manual, salida a cero.")

    def _mando_manual(self):
        self._pub("mode", {"owner": "manual"})
        self.pwm_var.set(0)

    def _log(self, txt):
        self.txt_bus.insert("end", txt + "\n")
        if int(self.txt_bus.index("end-1c").split(".")[0]) > MAX_LINEAS_CONSOLA:
            self.txt_bus.delete("1.0", "200.0")
        self.txt_bus.see("end")

    def _cerrar(self):
        self._parar_hilos.set()             # detiene publicador y reconexion
        vivos = [c for c, p in self.procesos.items() if p.vivo()]
        for c in vivos:
            self.procesos[c].detener()      # que aparquen el motor ellos mismos
        if vivos:
            self._log(f"[gui] esperando a {', '.join(vivos)} ...")
            self.root.update()
            t0 = time.time()
            while time.time() - t0 < 3.0 and any(self.procesos[c].vivo()
                                                 for c in vivos):
                time.sleep(0.1)
            for c in vivos:
                if self.procesos[c].vivo():
                    self.procesos[c].matar()
        if self.conectado:
            try:
                self.bus.publish("control", {"pwm": 0, "src": "manual"})
                self.bus.publish("enable", {"on": False})
            except Exception:
                pass          # si el bus ya no responde, no vale la pena esperar
        try:
            self.bus.close()
        except Exception:
            pass
        self.root.destroy()

    # =========================================================================
    # BUCLE DE REFRESCO
    # =========================================================================
    def _tick(self):
        while True:
            try:
                topico, data = self.eventos.get_nowait()
            except queue.Empty:
                break
            if topico == "__cerrado__":
                if self.conectado:      # evita spamear si ya se sabia
                    self.conectado = False
                    self._log("[gui] se perdio la conexion con el bus; "
                             "reconectando solo...")
                self._quiza_reconectar()
            elif topico == "__pub_error__":
                self._log(f"[gui] no se pudo publicar ({data})")
            elif topico == "__reconectando__":
                self.conectado = False
                if data == 1 or data % 5 == 0:
                    self._log(f"[gui] intento de reconexion #{data} a "
                             f"{self.host}:{self.port} ...")
            elif topico == "__reconectado__":
                self.conectado = True
                self._log("[gui] bus reconectado.")
            elif topico == "telemetry":
                self.conectado = True
                self._on_tel(data)
            elif topico == "agent/table":
                self.tabla = data
                self._tabla_nueva = True   # se pinta UNA vez al final del tick
            elif topico == "agent/status":
                self.status = data
            elif topico == "mode":
                self.owner = data.get("owner", "manual")
            elif topico == "enable":
                self.enabled = bool(data.get("on"))
            elif topico == "config":
                self._on_config(data)
            elif topico == "link":
                self._on_link(data)
            elif topico == "setpoint":
                if abs(float(data.get("rpm", 0)) - self.sp_var.get()) > 1:
                    self.sp_var.set(float(data.get("rpm", 0)))
                    self.lb_sp.config(text=f"{self.sp_var.get():.0f} rpm")
            elif topico == "log":
                self._log(f"[arduino] {data.get('msg')}")
            elif topico == "controller/launch":
                self._on_remote_launch(data)
            elif topico == "controller/stop":
                self._on_remote_stop(data)

        while True:                       # salida de los procesos lanzados
            try:
                clave, tipo, dato = self.cola_proc.get_nowait()
            except queue.Empty:
                break
            if tipo == "out":
                self._escribir(clave, dato)
            else:
                self._escribir(clave, f"[gui] proceso terminado (codigo {dato}).",
                               "fin")
                self._log(f"[gui] '{clave}' termino (codigo {dato})")

        if self._tabla_nueva:
            self._tabla_nueva = False
            self._pintar_tabla()
        elif self.tabla:
            self._marcar_celda_actual()    # el borde blanco sigue al estado
        self._pintar_graficas()
        self._pintar_panel()
        self.root.after(REFRESCO_MS, self._tick)   # la GUI NUNCA deja de latir

    def _on_link(self, data):
        ok = bool(data.get("ok", True))
        if ok != self.link_ok:
            if ok:
                self._log("[gui] enlace serie con el Arduino restablecido.")
            else:
                self._log(f"[gui] Arduino desconectado ({data.get('msg','')}); "
                         f"el bus sigue vivo y reintenta solo. El motor esta a "
                         f"salvo (watchdog del firmware).")
        self.link_ok = ok
        self.link_msg = data.get("msg", "")

    def _on_config(self, data):
        ts = int(data.get("ts_ms", self.ts_ms))
        cambio = ts != self.ts_ms
        self.ts_ms = ts
        self.ts_var.set(ts)
        aviso = ""
        if any(p.vivo() for p in self.procesos.values()):
            aviso = ("\nOJO: hay un controlador en marcha. Lo aprendido con otro "
                     "Ts deja de ser exacto: reentrena si el cambio es grande.")
        self.lb_ts.config(
            text=f"{1000.0/ts:.1f} decisiones/s. Regla: Ts entre tau/10 y tau/5 "
                 f"(tau~0.3s -> 30-60 ms)." + aviso,
            fg=AMBAR if aviso else SUAVE)
        if cambio:
            self._log(f"[gui] Ts = {ts} ms ({1000.0/ts:.1f} Hz)")

    def _on_tel(self, m):
        sp = float(m.get("setpoint") or 0.0)
        err = sp - m["rpm"]
        self.derr_actual = err - (self.err_prev if self.err_prev is not None else err)
        self.err_prev = err
        self.err_actual = err
        self.enabled = bool(m.get("enabled"))
        self.owner = m.get("owner", self.owner)
        self.hist.append((m["t_pc"] - self.t0, m["rpm"], sp, m["pwm"]))
        t = self.hist[-1][0]
        while self.hist and t - self.hist[0][0] > VENTANA_S + 1.0:
            self.hist.popleft()            # recorte por tiempo (ver __init__)
        self._tel_nueva = True
        if sp > 0:
            self.banda.append(1 if abs(err) <= 40 else 0)
            self.err2.append(err * err)

    # ---- panel ---------------------------------------------------------------
    def _pintar_panel(self):
        if not self.conectado:
            self.lb_estado.config(
                text=f"BUS DESCONECTADO - reconectando solo a "
                     f"{self.host}:{self.port} ... (motor a salvo: watchdog "
                     f"del firmware corta el PWM si nadie habla)", fg=ROJO)
            return
        ult = self.hist[-1] if self.hist else (0, 0, 0, 0)
        n_vivos = sum(1 for p in self.procesos.values() if p.vivo())
        estado_link = "" if self.link_ok else "  |  ARDUINO DESCONECTADO (bus ok, reintentando)"
        self.lb_estado.config(
            text=f"bus ok  |  Ts {self.ts_ms} ms  |  mando: {self.owner}  |  "
                 f"motor: {'HABILITADO' if self.enabled else 'parado'}  |  "
                 f"{n_vivos} controlador(es) en marcha{estado_link}",
            fg=AMBAR if not self.link_ok else (VERDE if self.enabled else SUAVE))
        self.btn_en.config(text="Deshabilitar motor" if self.enabled
                           else "Habilitar motor",
                           bg=AMBAR if self.enabled else VERDE)
        self.lb_owner.config(text=self.owner,
                             fg=AMBAR if self.owner == "manual" else ACENTO)
        pct = 100.0 * sum(self.banda) / len(self.banda) if self.banda else float("nan")
        rmse = math.sqrt(sum(self.err2) / len(self.err2)) if self.err2 else float("nan")
        self.lb_med.config(text=f"rpm   {ult[1]:8.1f}\nerror {self.err_actual:+8.1f}"
                                f"\npwm   {ult[3]:8d}  ({ult[3]/2.55:.0f}%)"
                                f"\nbanda {pct:7.1f} %\nrmse  {rmse:8.1f}")
        s = self.status
        if s:
            txt = (f"{s.get('algo','?')}\nfase   {s.get('phase','?')}\n"
                   f"pasos  {s.get('pasos','-')}")
            if s.get("epsilon") is not None:
                txt += f"\neps    {s.get('epsilon')}"
            if s.get("estados") is not None:
                txt += f"\nestados {s.get('estados')}"
            if "p" in s:      # PID
                txt += f"\nP {s['p']:+7.1f}\nI {s['i']:+7.1f}\nD {s['d']:+7.1f}"
            self.lb_algo.config(text=txt, fg=FG)
        self.sc_pwm.config(state="normal" if self.owner == "manual" else "disabled",
                           fg=FG if self.owner == "manual" else SUAVE)

    # ---- graficas ------------------------------------------------------------
    def _pintar_graficas(self):
        """Antes: delete("all") y ~2000 items nuevos 12 veces/segundo aunque no
        hubiera datos nuevos. Ahora la rejilla/titulos se crean UNA vez (y solo
        se rehacen al cambiar de tamanno), las curvas son items fijos a los que
        se les mueven los puntos con coords(), y si no llego telemetria nueva
        no se toca nada."""
        nueva = self._tel_nueva
        self._tel_nueva = False
        self._strip(self.c_rpm, ("rpm medido", "consigna"), 0, self.rpm_fs,
                    [(1, VERDE), (2, ACENTO)], "Senal medida: velocidad (rpm)",
                    nueva)
        self._strip(self.c_pwm, ("pwm",), 0, 255, [(3, AMBAR)],
                    "Senal de control: PWM (0-255)", nueva)

    def _strip(self, c, nombres, ymin, ymax, series, titulo, nueva):
        w = c.winfo_width() or 600
        h = c.winfo_height() or 200
        cache = self._strips.get(c)
        if cache and cache["firma"] == (w, h) and not nueva:
            return                       # nada que hacer: ni datos ni resize
        ml, mr, mt, mb = 52, 10, 22, 20
        x0, y0, x1, y1 = ml, mt, w - mr, h - mb
        if x1 <= x0 or y1 <= y0:
            return
        if not cache or cache["firma"] != (w, h):
            # ---- capa ESTATICA: solo al crear o al redimensionar ----
            c.delete("all")
            c.create_text(ml, 10, text=titulo, fill=SUAVE, anchor="w",
                          font=("TkDefaultFont", 9))
            for i in range(5):
                y = y0 + (y1 - y0) * i / 4
                c.create_line(x0, y, x1, y, fill=REJILLA)
                c.create_text(ml - 6, y, text=f"{ymax - (ymax-ymin)*i/4:.0f}",
                              fill=SUAVE, anchor="e", font=("TkFixedFont", 8))
            c.create_text(x0, y1 + 10, text=f"-{VENTANA_S:.0f}s", fill=SUAVE,
                          anchor="w", font=("TkFixedFont", 8))
            c.create_text(x1, y1 + 10, text="ahora", fill=SUAVE, anchor="e",
                          font=("TkFixedFont", 8))
            for i, n in enumerate(nombres):
                c.create_text(x1 - 8, mt + 4 + i * 13, text=n, anchor="e",
                              fill=series[i][1], font=("TkFixedFont", 8))
            lineas = [c.create_line(0, 0, 0, 0, fill=color, width=2,
                                    state="hidden") for _, color in series]
            cache = {"firma": (w, h), "lineas": lineas}
            self._strips[c] = cache

        datos = self.hist                # ya viene recortado por tiempo
        lineas = cache["lineas"]
        if len(datos) < 2:
            for lid in lineas:
                c.itemconfigure(lid, state="hidden")
            return
        t_fin = datos[-1][0]
        t_ini = t_fin - VENTANA_S
        # Submuestreo: no tiene sentido dibujar mas puntos que pixeles. Con
        # Ts=10 ms son 3000 muestras en 30 s para ~600 px de ancho.
        paso = max(1, len(datos) // max(1, int(x1 - x0)))
        sx = (x1 - x0) / VENTANA_S
        sy = (y1 - y0) / (ymax - ymin)
        for (idx, _), lid in zip(series, lineas):
            pts = []
            k = 0
            for d in datos:
                td = d[0]
                if td < t_ini:
                    continue
                if k % paso == 0:
                    v = d[idx]
                    if v < ymin:
                        v = ymin
                    elif v > ymax:
                        v = ymax
                    pts.append(x0 + (td - t_ini) * sx)
                    pts.append(y1 - (v - ymin) * sy)
                k += 1
            if len(pts) >= 4:
                c.coords(lid, *pts)      # mover la curva, no recrearla
                c.itemconfigure(lid, state="normal")
            else:
                c.itemconfigure(lid, state="hidden")

    # ---- mapa de calor -------------------------------------------------------
    def _pintar_tabla(self):
        """Mapa de calor INCREMENTAL. Antes cada actualizacion del agente (4/s)
        y cada resize destruia y recreaba ~500 rectangulos + textos + ejes.
        Ahora los items se crean UNA vez (o al cambiar tamanno/rejilla) y las
        actualizaciones solo cambian el color con itemconfigure: el coste por
        refresco cae un orden de magnitud y la GUI no parpadea."""
        c = self.c_tab
        t = self.tabla
        w = c.winfo_width() or 420
        h = c.winfo_height() or 300
        if not t:
            c.delete("all")
            self._tab_cache = None
            msg = "sin datos:\nlanza un controlador de RL"
            if self.status.get("kind") == "none":
                msg = (f"{self.status.get('algo','')} no tiene tabla"
                       f"\n(controlador clasico)")
            c.create_text(w / 2, h / 2, text=msg, fill=SUAVE, justify="center")
            self.c_leg.delete("all")
            self._leg_firma = None
            return
        nx, ny = t["err_bins"], t["derr_bins"]
        acciones = t["acciones"]
        firma = (nx, ny, w, h, tuple(acciones))
        cache = self._tab_cache
        if cache is None or cache["firma"] != firma:
            cache = self._crear_tabla_items(t, nx, ny, w, h, firma)
            self._tab_cache = cache

        # ---- actualizacion: SOLO colores y textos ----
        vals = [v for fila in t["value"] for v in fila if v is not None]
        vmin, vmax = (min(vals), max(vals)) if vals else (0.0, 1.0)
        rango = (vmax - vmin) or 1.0
        por_accion = self.vista.get() == "accion"
        rects, textos = cache["rects"], cache["textos"]
        n_acc = len(acciones)
        n_vis = 0
        cfg = c.itemconfigure
        for db in range(ny):
            fv, fa, fr = t["value"][db], t["action"][db], rects[db]
            ft = textos[db] if textos else None
            for eb in range(nx):
                v, a = fv[eb], fa[eb]
                if v is None or a is None:
                    col = "#232838"
                else:
                    n_vis += 1
                    col = (color_accion(a, n_acc) if por_accion
                           else color_valor((v - vmin) / rango))
                cfg(fr[eb], fill=col)
                if ft:
                    cfg(ft[eb], text="" if a is None else str(acciones[a]))
        self._marcar_celda_actual()
        self.lb_tabla.config(text=f"{t['algo']} ({t['kind']})  celdas "
                                  f"{n_vis}/{nx*ny}   valor {vmin:+.2f}..{vmax:+.2f}")
        self._pintar_leyenda(acciones, vmin, vmax, por_accion)

    def _crear_tabla_items(self, t, nx, ny, w, h, firma):
        """Crea (una sola vez) rectangulos, textos, ejes y la marca del estado."""
        c = self.c_tab
        c.delete("all")
        ml, mt, mr, mb = 44, 14, 8, 28
        cw = max(4, (w - ml - mr) / nx)
        ch = max(4, (h - mt - mb) / ny)
        con_txt = cw >= 26 and ch >= 18
        rects, textos = [], ([] if con_txt else None)
        for db in range(ny):
            y = mt + (ny - 1 - db) * ch
            fr, ft = [], []
            for eb in range(nx):
                x = ml + eb * cw
                fr.append(c.create_rectangle(x, y, x + cw, y + ch,
                                             fill="#232838", outline=""))
                if con_txt:
                    ft.append(c.create_text(x + cw / 2, y + ch / 2, text="",
                                            fill="#e8ecf5",
                                            font=("TkFixedFont", 7)))
            rects.append(fr)
            if con_txt:
                textos.append(ft)
        ee, de = t["err_edges"], t["derr_edges"]
        c.create_text(ml + (w - ml - mr) / 2, h - 6,
                      text="error de velocidad (consigna - medida) [rpm]",
                      fill=SUAVE, font=("TkFixedFont", 8))
        for i in (0, nx // 2, nx):
            c.create_text(ml + i * cw, mt + ny * ch + 8,
                          text=f"{ee[min(i, len(ee)-1)]:.0f}", fill=SUAVE,
                          font=("TkFixedFont", 7))
        for i in (0, ny // 2, ny):
            c.create_text(ml - 4, mt + (ny - i) * ch,
                          text=f"{de[min(i, len(de)-1)]:.0f}", fill=SUAVE,
                          anchor="e", font=("TkFixedFont", 7))
        c.create_text(10, mt + ny * ch / 2, text="d(error)", fill=SUAVE,
                      angle=90, font=("TkFixedFont", 8))
        marca = c.create_rectangle(0, 0, 0, 0, outline="#ffffff", width=2,
                                   state="hidden")
        return {"firma": firma, "rects": rects, "textos": textos,
                "marca": marca, "geom": (ml, mt, cw, ch, nx, ny)}

    def _marcar_celda_actual(self):
        """Mueve el borde blanco del estado actual SIN repintar el mapa: asi la
        marca sigue a la telemetria en cada tick, no solo cuando el agente
        publica su tabla (antes iba a saltos)."""
        cache, t = self._tab_cache, self.tabla
        if not cache or not t:
            return
        cel = self._celda_actual(t)
        c = self.c_tab
        if not cel:
            c.itemconfigure(cache["marca"], state="hidden")
            return
        ml, mt, cw, ch, nx, ny = cache["geom"]
        eb, db = cel
        c.coords(cache["marca"], ml + eb * cw, mt + (ny - 1 - db) * ch,
                 ml + (eb + 1) * cw, mt + (ny - db) * ch)
        c.itemconfigure(cache["marca"], state="normal")
        c.tag_raise(cache["marca"])

    def _celda_actual(self, t):
        try:
            bins = (t["err_bins"], t["derr_bins"])
            if self._disc_cache is None or self._disc_cache[0] != bins:
                from rl_motor_common import Discretizer
                self._disc_cache = (bins, Discretizer(err_bins=bins[0],
                                                      derr_bins=bins[1]))
            return self._disc_cache[1].key({"err": self.err_actual,
                                            "derr": self.derr_actual})
        except Exception:
            return None

    def _pintar_leyenda(self, acciones, vmin, vmax, por_accion):
        """La leyenda solo se redibuja al cambiar de modo o de tamanno; en modo
        'valor' las etiquetas vmin/vmax se actualizan con itemconfigure (antes
        se redibujaba el degradado entero, ~400 lineas, en cada refresco)."""
        c = self.c_leg
        w = c.winfo_width() or 420
        firma = (por_accion, w, tuple(acciones))
        if firma != self._leg_firma:
            self._leg_firma = firma
            self._leg_lbls = None
            c.delete("all")
            if por_accion:
                c.create_text(4, 8, text="accion: incremento de PWM por decision",
                              fill=SUAVE, anchor="w", font=("TkFixedFont", 8))
                n = len(acciones)
                bw = (w - 16) / n
                for i, a in enumerate(acciones):
                    x = 8 + i * bw
                    c.create_rectangle(x, 20, x + bw - 2, 38,
                                       fill=color_accion(i, n), outline="")
                    c.create_text(x + bw / 2, 29, text=f"{a:+d}", fill="#e8ecf5",
                                  font=("TkFixedFont", 8))
                c.create_text(8, 47, text="frenar", fill=SUAVE, anchor="w",
                              font=("TkFixedFont", 7))
                c.create_text(w - 8, 47, text="acelerar", fill=SUAVE, anchor="e",
                              font=("TkFixedFont", 7))
            else:
                c.create_text(4, 8, text="valor del estado (max_a Q)", fill=SUAVE,
                              anchor="w", font=("TkFixedFont", 8))
                for i in range(max(1, w - 16)):
                    c.create_line(8 + i, 20, 8 + i, 38,
                                  fill=color_valor(i / max(1, w - 17)))
                lb_min = c.create_text(8, 47, text="", fill=SUAVE, anchor="w",
                                       font=("TkFixedFont", 7))
                lb_max = c.create_text(w - 8, 47, text="", fill=SUAVE, anchor="e",
                                       font=("TkFixedFont", 7))
                self._leg_lbls = (lb_min, lb_max)
            c.create_text(w - 8, 8, text="borde blanco = estado actual",
                          fill=SUAVE, anchor="e", font=("TkFixedFont", 7))
        if not por_accion and self._leg_lbls:
            c.itemconfigure(self._leg_lbls[0], text=f"{vmin:+.2f}")
            c.itemconfigure(self._leg_lbls[1], text=f"{vmax:+.2f}")


def main():
    ap = argparse.ArgumentParser(description="GUI del control de velocidad por RL.")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8770)
    args = ap.parse_args()
    try:
        bus = BusClient("gui", args.host, args.port).connect()
    except OSError as e:
        raise SystemExit(f"No hay bus en {args.host}:{args.port} ({e}).\n"
                         f"Arranca primero:  python3 motor_bus.py --sim")
    root = tk.Tk()
    MotorGUI(root, bus)
    root.mainloop()


if __name__ == "__main__":
    main()