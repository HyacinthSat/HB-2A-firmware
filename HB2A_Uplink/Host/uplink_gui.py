"""HB-2A USB ground transmitter. Never sends automatically or retries RF commands."""
import argparse
import queue
import threading
import time
import tkinter as tk
from datetime import datetime
from tkinter import ttk, messagebox

from protocol import command_data, usb_command

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


class SerialWorker(threading.Thread):
    def __init__(self, events):
        super().__init__(daemon=True)
        self.events = events
        self.jobs = queue.Queue()
        self.port = None

    def exchange(self, payload):
        self.port.write(payload)
        deadline = time.monotonic() + 3.0
        response = bytearray()
        while time.monotonic() < deadline:
            chunk = self.port.read(1)
            if chunk == b"\n":
                return response.decode("ascii", errors="replace").strip()
            response.extend(chunk)
            if len(response) > 256:
                raise ValueError("Invalid board response")
        raise TimeoutError("Board response timed out")

    def close_port(self):
        if self.port is not None:
            try:
                self.port.close()
            finally:
                self.port = None

    def run(self):
        while True:
            action, data = self.jobs.get()
            try:
                if action in ("close", "quit"):
                    self.close_port()
                    if action == "quit":
                        return
                    self.events.put(("closed", ""))
                    continue
                if action == "open":
                    self.close_port()
                    if serial is None:
                        raise RuntimeError("pyserial missing: python -m pip install -r requirements.txt")
                    self.port = serial.Serial(data, 115200, timeout=0.1, write_timeout=1)
                    self.port.reset_input_buffer()
                    # Discard any partial request retained from a previous USB session.
                    self.port.write(b"\x18\n")
                    time.sleep(0.1)
                    self.port.reset_input_buffer()
                    reply = self.exchange(b"STATUS\n")
                    if not reply.startswith("OK HB2A-UPLINK/1 "):
                        raise ValueError("Board is not running HB2A-UPLINK/1 firmware: " + reply)
                    self.events.put(("connected", (data, reply)))
                else:
                    if self.port is None:
                        raise RuntimeError("Not connected")
                    reply = self.exchange(data)
                    self.events.put(("sent" if action == "send" else "status", reply))
            except Exception as exc:
                self.close_port()
                self.events.put(("error", (action, str(exc))))


class App:
    def __init__(self, root):
        self.root = root
        self.events = queue.Queue()
        self.worker = SerialWorker(self.events)
        self.worker.start()
        self.connected = False
        self.ready = False
        self.busy = False
        self.port_names = {}
        root.title("HB-2A 上行控制台")
        root.geometry("900x710")
        root.minsize(740, 660)
        root.configure(bg="#f3f5f6")
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", font=("Microsoft YaHei UI", 10))
        style.configure("TFrame", background="#f3f5f6")
        style.configure("TLabel", background="#f3f5f6", foreground="#263238")
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 21, "bold"))
        style.configure("TButton", padding=(10, 6))
        style.configure("Send.TButton", background="#15745b", foreground="white", padding=(20, 10))
        style.map("Send.TButton", background=[("disabled", "#dde3e2"), ("active", "#12644e")],
                  foreground=[("disabled", "#697574")])
        outer = ttk.Frame(root, padding=22)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(7, weight=1)
        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 14))
        ttk.Label(header, text="HB-2A", style="Title.TLabel").pack(side="left")
        ttk.Label(header, text="上行控制台", font=("Microsoft YaHei UI", 12)).pack(side="left", padx=14)
        ttk.Label(header, text="435.400 MHz  /  GMSK  /  4800 bps").pack(side="right")

        connection = ttk.Frame(outer)
        connection.grid(row=1, column=0, sticky="ew")
        connection.columnconfigure(1, weight=1)
        ttk.Label(connection, text="USB 串口").grid(row=0, column=0, padx=(0, 10))
        self.port_var = tk.StringVar()
        self.ports = ttk.Combobox(connection, textvariable=self.port_var, state="readonly")
        self.ports.grid(row=0, column=1, sticky="ew")
        self.refresh_button = ttk.Button(connection, text="刷新", command=self.refresh)
        self.refresh_button.grid(row=0, column=2, padx=8)
        self.connect_button = ttk.Button(connection, text="连接", command=self.connect)
        self.connect_button.grid(row=0, column=3)
        self.query_button = ttk.Button(connection, text="查询状态", command=self.query)
        self.query_button.grid(row=0, column=4, padx=(8, 0))
        self.connection_text = tk.StringVar(value="未连接")
        ttk.Label(outer, textvariable=self.connection_text).grid(row=2, column=0, sticky="w", pady=(9, 13))
        ttk.Separator(outer).grid(row=3, column=0, sticky="ew", pady=(0, 14))

        command = ttk.Frame(outer)
        command.grid(row=4, column=0, sticky="nsew")
        command.columnconfigure(1, weight=1)
        ttk.Label(command, text="指令").grid(row=0, column=0, sticky="w", pady=5, padx=(0, 16))
        self.kind = tk.StringVar(value="TRANPD")
        self.command_select = ttk.Combobox(command, textvariable=self.kind, state="readonly",
                                           values=("TRANPD", "EXPR", "CAPTR", "REQF", "HSPAR", "CHPRAM", "CFRM", "CANCEL"), width=15)
        self.command_select.grid(row=0, column=1, sticky="w")
        self.command_select.bind("<<ComboboxSelected>>", lambda _: self.change_command())
        ttk.Label(command, text="功率 (dBm)").grid(row=0, column=2, padx=(16, 8))
        self.power = tk.StringVar(value="10")
        ttk.Spinbox(command, from_=2, to=17, textvariable=self.power, width=5).grid(row=0, column=3)
        self.command_status = tk.StringVar()
        self.notice = ttk.Label(command, textvariable=self.command_status, foreground="#925619", wraplength=640)
        self.notice.grid(row=1, column=0, columnspan=4, sticky="w", pady=(6, 10))
        self.fields_frame = ttk.Frame(command)
        self.fields_frame.grid(row=2, column=0, columnspan=4, sticky="ew")
        self.fields_frame.columnconfigure(1, weight=1)
        self.fields = []
        self.saved = {}
        self.current = None
        self.change_command()

        action = ttk.Frame(outer)
        action.grid(row=5, column=0, sticky="ew", pady=(14, 10))
        self.send_button = ttk.Button(action, text="发送一次", style="Send.TButton", command=self.send)
        self.send_button.pack(side="right")
        self.result_text = tk.StringVar(value="气球执行状态：未知（无上行确认）")
        ttk.Label(action, textvariable=self.result_text, wraplength=480).pack(side="left")
        ttk.Label(outer, text="通信记录", font=("Microsoft YaHei UI", 10, "bold")).grid(
            row=6, column=0, sticky="w", pady=(4, 6))
        log_frame = ttk.Frame(outer)
        log_frame.grid(row=7, column=0, sticky="nsew")
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        self.log = tk.Text(log_frame, height=7, font=("Consolas", 10), bg="#ffffff", fg="#263238",
                           relief="solid", bd=1, wrap="word", state="disabled", padx=10, pady=8)
        self.log.grid(row=0, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(log_frame, command=self.log.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log.configure(yscrollcommand=scrollbar.set)
        self.refresh()
        self.controls()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(80, self.poll)

    def add_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", f"{datetime.now():%H:%M:%S}  {text}\n")
        if int(self.log.index("end-1c").split(".")[0]) > 600:
            self.log.delete("1.0", "101.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def change_command(self):
        if self.current:
            self.saved[self.current] = [v.get() for v in self.fields]
        self.current = self.kind.get()
        for widget in self.fields_frame.winfo_children():
            widget.destroy()
        labels, defaults = {
            "TRANPD": (["源呼号", "目的呼号", "转发路径", "消息 (≤7 ASCII)"], ["BG7ZDQ", "APZDQ7", "HB-2A", "HELLO"]),
            "EXPR": ([], []),
            "CAPTR": (["相机 (A/B)", "宽度", "高度", "质量 (0..7)"], ["A", "255", "250", "7"]),
            "REQF": (["文件索引"], ["0"]),
            "HSPAR": (["速率 ID", "编码 ID", "交织 ID"], ["0", "0", "1"]),
            "CHPRAM": (["速率 ID", "编码 ID", "交织 ID"], ["0", "0", "1"]),
            "CFRM": ([], []),
            "CANCEL": ([], []),
        }[self.current]
        self.command_status.set({
            "TRANPD": "气球端：已接入转发队列，转发频率 144.925 MHz",
            "EXPR": "气球端：正常模式下进入 5 分钟试验模式",
            "CAPTR": "气球端：试验模式下启动图像拍摄协商；参数为相机、宽度、高度、质量",
            "REQF": "气球端：试验模式下启动文件发送协商",
            "HSPAR": "气球端：等待参数阶段接受高速图传参数",
            "CHPRAM": "气球端：当前协议可编码；请按气球端协商状态使用",
            "CFRM": "气球端：确认后开始高速图传",
            "CANCEL": "气球端：取消当前高速图传",
        }[self.current])
        self.fields = []
        for row, (label, default) in enumerate(zip(labels, self.saved.get(self.current, defaults))):
            ttk.Label(self.fields_frame, text=label, width=20).grid(row=row, column=0, sticky="w", pady=4)
            value = tk.StringVar(value=default)
            self.fields.append(value)
            entry = ttk.Entry(self.fields_frame, textvariable=value, width=30)
            entry.grid(row=row, column=1, sticky="ew", pady=4)
            if self.current == "TRANPD" and row == 2:
                entry.configure(state="readonly")
        # Keep the action row stable when switching among commands.
        self.fields_frame.configure(height=152)
        self.fields_frame.grid_propagate(False)

    def refresh(self):
        items = list(list_ports.comports()) if list_ports else []
        self.port_names = {f"{p.device}  |  {p.description}": p.device for p in items}
        self.ports.configure(values=list(self.port_names))
        if self.port_var.get() not in self.port_names:
            self.port_var.set(next(iter(self.port_names), ""))
        if not serial:
            self.connection_text.set("缺少 pyserial：请运行 Host/start.ps1")

    def controls(self):
        self.connect_button.configure(text="断开" if self.connected else "连接",
                                      state="disabled" if self.busy else "normal")
        self.query_button.configure(state="normal" if self.connected and not self.busy else "disabled")
        self.send_button.configure(state="normal" if self.connected and self.ready and not self.busy else "disabled")
        self.ports.configure(state="disabled" if self.connected or self.busy else "readonly")
        self.refresh_button.configure(state="disabled" if self.busy or self.connected else "normal")

    def submit(self, action, data=None):
        self.busy = True
        self.controls()
        self.worker.jobs.put((action, data))

    def connect(self):
        if self.connected:
            self.submit("close")
        elif self.port_var.get() in self.port_names:
            self.submit("open", self.port_names[self.port_var.get()])
        else:
            messagebox.showerror("串口", "没有可用串口，请连接板卡并刷新。")

    def query(self):
        self.submit("status", b"STATUS\n")

    def send(self):
        try:
            fields = [v.get() for v in self.fields]
            payload = usb_command(self.kind.get(), fields, int(self.power.get()))
            data = command_data(self.kind.get(), fields)
        except ValueError as exc:
            messagebox.showerror("指令参数", str(exc))
            return
        self.add_log(f"TX {self.kind.get()}  {self.power.get()} dBm  DATA={data.hex().upper()}")
        self.result_text.set("板卡发射中…")
        self.submit("send", payload)

    def poll(self):
        while True:
            try:
                event, data = self.events.get_nowait()
            except queue.Empty:
                break
            self.busy = False
            if event == "connected":
                port, reply = data
                self.connected = True
                self.ready = "RADIO=READY" in reply
                self.connection_text.set(f"{port} 已连接  |  射频{'就绪' if self.ready else '初始化失败'}")
                self.add_log(reply)
            elif event == "closed":
                self.connected = self.ready = False
                self.connection_text.set("未连接")
            elif event == "error":
                action, error = data
                self.connected = self.ready = False
                self.connection_text.set("连接中断")
                self.add_log("ERROR " + error)
                if action == "send":
                    self.result_text.set("发送结果未知；未自动重发")
                else:
                    self.result_text.set("连接失败，详见通信记录")
            else:
                self.add_log(data)
                if event == "sent":
                    if data.startswith("OK SENT "):
                        self.result_text.set("板卡发射完成；气球执行状态未知")
                    else:
                        self.result_text.set("板卡返回错误；气球执行状态未知")
                        if data.startswith("ERR RADIO"):
                            self.ready = False
                            self.connection_text.set("射频错误；可查询状态以重新初始化")
                else:
                    self.ready = "RADIO=READY" in data
                    self.connection_text.set("USB 已连接  |  射频" + ("就绪" if self.ready else "错误"))
            self.controls()
        self.root.after(80, self.poll)

    def close(self):
        self.worker.jobs.put(("quit", None))
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--smoke-test", action="store_true", help="Render, exercise command views and exit; no serial access")
    args = parser.parse_args()
    root = tk.Tk()
    app = App(root)
    if args.smoke_test:
        for kind in ("EXPR", "CAPTR", "REQF", "TRANPD"):
            app.kind.set(kind)
            app.change_command()
            root.update()
        root.after(500, app.close)
    root.mainloop()


if __name__ == "__main__":
    main()
