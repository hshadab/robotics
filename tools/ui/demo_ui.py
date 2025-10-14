#!/usr/bin/env python3
import os
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import messagebox


def bash_run(cmd: str, env=None):
    setup = os.environ.get("ROS_SETUP_BASH")
    if not setup:
        # default to workspace-relative install
        here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        setup = os.path.join(here, "install", "setup.bash")
    shell_cmd = f"set -e; source '{setup}' >/dev/null 2>&1 || true; {cmd}"
    return subprocess.Popen(["/bin/bash", "-lc", shell_cmd], env=env)


class DemoUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("JOLT zkML Guard Demo")
        self.geometry("560x360")
        self.teleop = None
        self.camera = None
        self.guard = None
        self.bag = None
        self.verifier = None

        # Toggles
        self.var_proof = tk.BooleanVar(value=False)
        self.var_burger = tk.BooleanVar(value=False)
        self.var_mode = tk.StringVar(value="http")  # http or cli
        self.var_record = tk.BooleanVar(value=False)

        row = 0
        tk.Label(self, text="ROS 2 Controls", font=("Arial", 12, "bold")).grid(row=row, column=0, sticky="w", padx=8, pady=6)
        row += 1

        self.btn_teleop = tk.Button(self, text="Start Teleop", width=20, command=self.toggle_teleop)
        self.btn_teleop.grid(row=row, column=0, padx=8, pady=4)
        self.btn_camera = tk.Button(self, text="Start Camera", width=20, command=self.toggle_camera)
        self.btn_camera.grid(row=row, column=1, padx=8, pady=4)
        self.btn_guard = tk.Button(self, text="Start Proof Guard", width=20, command=self.toggle_guard)
        self.btn_guard.grid(row=row, column=2, padx=8, pady=4)
        row += 1

        self.btn_bag = tk.Button(self, text="Start MCAP Record", width=20, command=self.toggle_bag)
        self.btn_bag.grid(row=row, column=0, padx=8, pady=4)
        self.btn_build = tk.Button(self, text="Build Prover (Docker/Podman)", width=24, command=self.build_prover)
        self.btn_build.grid(row=row, column=1, padx=8, pady=4)
        self.btn_quit = tk.Button(self, text="Stop All", width=20, command=self.stop_all)
        self.btn_quit.grid(row=row, column=2, padx=8, pady=4)
        row += 1

        # Options
        tk.Checkbutton(self, text="Proof On (requires prover binary)", variable=self.var_proof).grid(row=row, column=0, sticky="w", padx=8)
        tk.Checkbutton(self, text="Burger Mode (synthetic camera)", variable=self.var_burger).grid(row=row, column=1, sticky="w", padx=8)
        row += 1
        # Verifier mode selector
        tk.Label(self, text="Verifier Mode:").grid(row=row, column=0, sticky="e", padx=8)
        mode_menu = tk.OptionMenu(self, self.var_mode, "http", "cli")
        mode_menu.config(width=10)
        mode_menu.grid(row=row, column=1, sticky="w", padx=8)
        row += 1
        tk.Checkbutton(self, text="Record MCAP (one-shot)", variable=self.var_record).grid(row=row, column=0, sticky="w", padx=8)
        self.btn_full = tk.Button(self, text="Start Full Demo", width=20, command=self.start_full_demo)
        self.btn_full.grid(row=row, column=1, padx=8, pady=4)
        row += 1
        # External verifier server control
        self.btn_verifier = tk.Button(self, text="Start /onnx-verifier Server", width=26, command=self.toggle_verifier)
        self.btn_verifier.grid(row=row, column=0, padx=8, pady=4)
        row += 1

        tk.Label(self, text="Status", font=("Arial", 12, "bold")).grid(row=row, column=0, sticky="w", padx=8, pady=6)
        row += 1
        self.status = tk.Text(self, height=12)
        self.status.grid(row=row, column=0, columnspan=3, sticky="nsew", padx=8, pady=4)
        self.grid_rowconfigure(row, weight=1)
        self.grid_columnconfigure(2, weight=1)

        # Ensure tools/bin is on PATH for atlas_argmax_prover
        repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        tools_bin = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'bin')
        os.environ['PATH'] = tools_bin + os.pathsep + os.environ.get('PATH', '')
        self.log("UI ready. Set ROS_SETUP_BASH env if needed.")

    def log(self, msg: str):
        self.status.insert(tk.END, msg + "\n")
        self.status.see(tk.END)

    def toggle_teleop(self):
        if self.teleop and self.teleop.poll() is None:
            self.teleop.terminate(); self.teleop = None
            self.btn_teleop.config(text="Start Teleop"); self.log("Teleop stopped")
        else:
            self.teleop = bash_run("ros2 run teleop_twist_keyboard teleop_twist_keyboard")
            self.btn_teleop.config(text="Stop Teleop"); self.log("Teleop started")

    def toggle_camera(self):
        if self.camera and self.camera.poll() is None:
            self.camera.terminate(); self.camera = None
            self.btn_camera.config(text="Start Camera"); self.log("Camera stopped")
        else:
            if self.var_burger.get():
                self.camera = bash_run("ros2 run image_tools cam2image --ros-args -p burger_mode:=true")
                self.log("Camera started (burger mode)")
            else:
                self.camera = bash_run("ros2 run image_tools cam2image")
                self.log("Camera started (webcam)")
            self.btn_camera.config(text="Stop Camera"); self.log("Camera started")

    def toggle_guard(self):
        if self.guard and self.guard.poll() is None:
            self.guard.terminate(); self.guard = None
            self.btn_guard.config(text="Start Proof Guard"); self.log("Guard stopped")
        else:
            if self.var_proof.get():
                # Proof-on: use selected verifier mode
                if self.var_mode.get().lower() == "cli":
                    cmd = "ros2 launch zkml_guard zkml_guard_proof.launch.py --ros-args -p verifier_mode:=cli"
                else:
                    cmd = "ros2 launch zkml_guard zkml_guard_proof.launch.py"
                self.guard = bash_run(cmd)
                self.log(f"Guard started (proof-on, mode={self.var_mode.get()})")
            else:
                # No-proof: run demo launch with require_proof=false
                self.guard = bash_run("ros2 launch zkml_guard zkml_guard_demo.launch.py gating_mode:=argmax require_proof:=false")
                self.log("Guard started (no-proof)")
            self.btn_guard.config(text="Stop Proof Guard")

    def toggle_bag(self):
        if self.bag and self.bag.poll() is None:
            self.bag.terminate(); self.bag = None
            self.btn_bag.config(text="Start MCAP Record"); self.log("Recording stopped")
        else:
            self.bag = bash_run("ros2 bag record -a -s mcap")
            self.btn_bag.config(text="Stop MCAP Record"); self.log("Recording started")

    def build_prover(self):
        def task():
            here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            script = os.path.join(here, "build_helper.sh")
            if not os.path.exists(script):
                self.log("build_helper.sh not found")
                return
            env = os.environ.copy()
            proc = subprocess.Popen(["/bin/bash", script], cwd=here, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            for line in proc.stdout:
                self.log(line.rstrip())
            ret = proc.wait()
            if ret == 0:
                self.log("Prover built. Add tools/bin to PATH or restart guard.")
            else:
                self.log(f"Prover build failed (exit {ret}). Ensure docker/podman is installed.")
        threading.Thread(target=task, daemon=True).start()

    def stop_all(self):
        for proc_name in ["teleop", "camera", "guard", "bag"]:
            proc = getattr(self, proc_name)
            if proc and proc.poll() is None:
                proc.terminate()
                setattr(self, proc_name, None)
        self.btn_teleop.config(text="Start Teleop")
        self.btn_camera.config(text="Start Camera")
        self.btn_guard.config(text="Start Proof Guard")
        self.btn_bag.config(text="Start MCAP Record")
        if self.verifier and self.verifier.poll() is None:
            self.verifier.terminate(); self.verifier=None
            self.btn_verifier.config(text="Start /onnx-verifier Server")
        self.log("All processes stopped")

    def toggle_verifier(self):
        if self.verifier and self.verifier.poll() is None:
            self.verifier.terminate(); self.verifier=None
            self.btn_verifier.config(text="Start /onnx-verifier Server")
            self.log("onnx-verifier server stopped")
        else:
            # Start node server.js from consolidated tools/onnx-verifier
            repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            cmd = f"cd '{repo_root}/tools/onnx-verifier' && node server.js"
            self.verifier = bash_run(cmd)
            self.btn_verifier.config(text="Stop /onnx-verifier Server")
            self.log("onnx-verifier server started on http://localhost:9100")

    def start_full_demo(self):
        # Start verifier if in HTTP mode and not running
        if self.var_proof.get() and self.var_mode.get().lower() == "http":
            if not (self.verifier and self.verifier.poll() is None):
                self.toggle_verifier()
        # Camera
        if not (self.camera and self.camera.poll() is None):
            self.toggle_camera()
        # Proof Guard
        if not (self.guard and self.guard.poll() is None):
            # Ensure Proof On is consistent; if not, enable no-proof demo
            if not self.var_proof.get():
                self.log("Proof Off: starting guard without proofs")
            self.toggle_guard()
        # Teleop
        if not (self.teleop and self.teleop.poll() is None):
            self.toggle_teleop()
        # MCAP Record (optional)
        if self.var_record.get() and not (self.bag and self.bag.poll() is None):
            self.toggle_bag()
        self.log("Full demo started")


if __name__ == "__main__":
    try:
        app = DemoUI()
        app.mainloop()
    except KeyboardInterrupt:
        pass
