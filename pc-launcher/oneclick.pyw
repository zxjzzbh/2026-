"""Double-click launcher: wait for the car, relay bytes locally, open its dashboard."""
import argparse
import json
import os
from pathlib import Path
import select
import socket
import socketserver
import threading
import time
import urllib.request
import webbrowser

HOST = os.environ.get('SMARTCAR_HOST', '').strip()
CAR_PORT = 8080
LOCAL_PORT = 18080
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def car_status(host=HOST, port=CAR_PORT):
    with OPENER.open(f'http://{host}:{port}/api/drive/status', timeout=3) as response:
        return json.load(response)


class Forward(socketserver.BaseRequestHandler):
    def handle(self):
        try:
            with socket.create_connection(self.server.upstream, timeout=3) as upstream:
                # Control headers and bodies can arrive as separate small writes.
                # Do not wait for a cellular ACK before forwarding the next one.
                upstream.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                upstream.settimeout(2)
                self.request.settimeout(2)
                while True:
                    for source in select.select([self.request, upstream], [], [], 1)[0]:
                        body = source.recv(32768)
                        if not body:
                            return
                        (upstream if source is self.request else self.request).sendall(body)
        except OSError:
            return  # Never retry or replay a control message.


class Relay(socketserver.ThreadingTCPServer):
    # Windows SO_REUSEADDR can let multiple launchers bind the same port.
    allow_reuse_address = os.name != 'nt'
    daemon_threads = True

    def server_bind(self):
        if os.name == 'nt':
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

    def __init__(self, port=LOCAL_PORT, upstream=(HOST, CAR_PORT)):
        self.upstream = upstream
        super().__init__(('127.0.0.1', port), Forward)


def existing_relay_matches(state, port=LOCAL_PORT):
    try:
        local = car_status('127.0.0.1', port)
        return bool(state.get('control_session_id')) and local.get('control_session_id') == state['control_session_id']
    except (OSError, ValueError):
        return False


def connect_dashboard():
    import tkinter as tk
    from tkinter import messagebox
    window = tk.Tk()
    window.title('智能车 5G 一键测试')
    window.geometry('460x165')
    window.resizable(False, False)
    label = tk.Label(window, text='正在等待树莓派通过 5G 联网……', font=('Microsoft YaHei', 12), wraplength=425)
    label.pack(padx=16, pady=18)
    tk.Label(window, text='请保持电脑联网，Tailscale 在线。\n连接成功后会自动打开测试页面。', font=('Microsoft YaHei', 10)).pack()
    finished = threading.Event()
    results = []
    relay = None

    def wait_for_car():
        deadline = time.monotonic()+180
        while not finished.is_set() and time.monotonic()<deadline:
            try:
                state = car_status()
                if state.get('boot_test') is not True:
                    results.append(RuntimeError('树莓派当前没有运行一键测试服务，请先完成服务部署。'))
                else:
                    results.append(state)
                return
            except (OSError, ValueError):
                finished.wait(1)
        if not finished.is_set():
            results.append(RuntimeError('未连上树莓派。请确认车辆开机、5G 模块联网、电脑 Tailscale 在线，然后再次双击入口。'))

    def close():
        finished.set()
        if relay is not None:
            relay.shutdown()
            relay.server_close()
        window.destroy()

    def poll():
        nonlocal relay
        if not results:
            window.after(200, poll)
            return
        state = results[0]
        if isinstance(state, Exception):
            messagebox.showerror('智能车连接', str(state))
            close()
            return
        try:
            relay = Relay()
        except OSError:
            if not existing_relay_matches(state):
                messagebox.showerror('智能车连接', '本机 18080 端口被其他程序占用，未连接到这辆车。请关闭占用程序后再试。')
                close()
                return
        if relay is not None:
            threading.Thread(target=relay.serve_forever, daemon=True).start()
        if not webbrowser.open(f'http://127.0.0.1:{LOCAL_PORT}/', new=2):
            messagebox.showinfo('智能车已连接', f'请在浏览器打开 http://127.0.0.1:{LOCAL_PORT}/')
        if relay is None:
            close()
        else:
            window.withdraw()  # Own the local relay while the browser remains usable.

    window.protocol('WM_DELETE_WINDOW', close)
    threading.Thread(target=wait_for_car, daemon=True).start()
    window.after(100, poll)
    window.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='read-only connectivity check; no browser or control action')
    args = parser.parse_args()
    if not HOST:
        message = '请先设置 SMARTCAR_HOST 为已授权访问的树莓派地址，见 pc-launcher/使用说明.txt。'
        if args.check:
            parser.error(message)
        from tkinter import messagebox
        messagebox.showerror('智能车连接配置', message)
        return
    if args.check:
        state = car_status()
        print(json.dumps({'connected': True, 'boot_test_installed': state.get('boot_test') is True,
                          'phase': state.get('boot_test_phase'), 'mode': state.get('mode'),
                          'actuator_commands_sent': False}))
    else:
        connect_dashboard()


if __name__ == '__main__':
    main()
