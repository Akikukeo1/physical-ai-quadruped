"""Windows operation window for the WSL MuJoCo search agent."""

import ast
import json
import os
import queue
import subprocess
import tempfile
import threading
import tkinter as tk
import uuid
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROMPT = "青を探して、見つからなくても出発地点へ戻って。全体240秒以内。"


def write_instruction(path: Path, message: str) -> None:
    message = message.strip()
    if not 1 <= len(message) <= 500:
        raise ValueError("指示は1〜500文字で入力してください。")
    payload = json.dumps({"id": uuid.uuid4().hex, "message": message}, ensure_ascii=False).encode("utf-8")
    if len(payload) > 4096:
        raise ValueError("指示が長すぎます。")
    # A single atomic replacement prevents the simulator from reading a partial JSON message.
    temporary = path.with_name(f".{uuid.uuid4().hex}.tmp")
    try:
        temporary.write_bytes(payload)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def describe_line(line: str) -> tuple[str | None, bool]:
    line = line.strip()
    if line.startswith("REPLY: "):
        return "ロボット: " + line.removeprefix("REPLY: "), False
    if line.startswith("INPUT READY: "):
        return line.removeprefix("INPUT READY: "), True
    if line.startswith("DIALOGUE INPUT: "):
        return "受信: " + line.removeprefix("DIALOGUE INPUT: "), False
    if line.startswith("CORRECTION: "):
        return "受信: " + line.removeprefix("CORRECTION: ").removesuffix("; holding position"), False
    if line.startswith("QUESTION: "):
        return "ロボットからの確認: " + line.removeprefix("QUESTION: "), True
    if line.startswith(("UPDATED: tasks=", "UNCHANGED: tasks=")):
        try:
            unchanged = line.startswith("UNCHANGED:")
            tasks = ast.literal_eval(line.split("tasks=", 1)[1])
            names = {
                "search:blue": "青を探索",
                "search:green": "緑を探索",
                "reobserve:blue": "青を再確認",
                "reobserve:green": "緑を再確認",
                "return_home:None": "帰還",
            }
            label = "現在の計画: " if unchanged else "計画を更新: "
            return label + " → ".join(names.get(task, str(task)) for task in tasks), True
        except (SyntaxError, ValueError, TypeError):
            return "計画を更新しました。", True
    if line.startswith("FRONTIER: "):
        return "探索先の判断: " + line.removeprefix("FRONTIER: "), False
    if line.startswith("TASK DONE:"):
        return "タスク完了: " + line.removeprefix("TASK DONE:").strip(), False
    if line.startswith("ERROR:"):
        return "エラー: " + line.removeprefix("ERROR:").strip(), False
    if line.startswith("CORRECTION INPUT:"):
        return "MuJoCoを開始しました。最初の指示を解釈しています。", False
    return (line or None), False


class InteractiveApp:
    def __init__(self, root):
        self.root = root
        root.title("四脚ロボット 操作画面")
        root.geometry("860x680")
        root.minsize(680, 500)
        self.process = None
        self.events = queue.Queue()
        self.session_dir = None
        self.pid_file = None
        self.ready = False
        self.area_path = tk.StringVar(value=str(ROOT / "examples" / "field-areas.json"))
        self.status = tk.StringVar(value="初期指示を確認して「開始」を押してください。")

        outer = ttk.Frame(root, padding=14)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="四脚ロボットと会話して探索", font=("Yu Gothic UI", 16, "bold")).pack(anchor="w")
        ttk.Label(outer, text="会話を始めます。探索を指示すると、別ウィンドウでMuJoCoが開きます。").pack(
            anchor="w", pady=(4, 12)
        )
        ttk.Label(outer, text="最初のメッセージ（会話、または青・緑の探索と帰還）").pack(anchor="w")
        self.initial = ttk.Entry(outer)
        self.initial.insert(0, DEFAULT_PROMPT)
        self.initial.pack(fill="x", pady=(3, 8))
        area = ttk.Frame(outer)
        area.pack(fill="x")
        ttk.Label(area, text="エリア:").pack(side="left")
        ttk.Entry(area, textvariable=self.area_path, state="readonly").pack(side="left", fill="x", expand=True, padx=6)
        self.choose = ttk.Button(area, text="JSONを選択", command=self.choose_areas)
        self.choose.pack(side="left")
        self.start_button = ttk.Button(outer, text="開始", command=self.start)
        self.start_button.pack(anchor="w", pady=8)
        ttk.Label(outer, textvariable=self.status).pack(anchor="w", pady=(0, 5))
        self.log = scrolledtext.ScrolledText(outer, height=15, wrap="word", state="disabled", font=("Yu Gothic UI", 10))
        self.log.pack(fill="both", expand=True)
        ttk.Label(outer, text="メッセージ・探索中の指示・確認への回答").pack(anchor="w", pady=(10, 3))
        send = ttk.Frame(outer)
        send.pack(fill="x")
        self.message = ttk.Entry(send)
        self.message.pack(side="left", fill="x", expand=True)
        self.message.bind("<Return>", lambda _event: self.send())
        self.send_button = ttk.Button(send, text="送信", command=self.send, state="disabled")
        self.send_button.pack(side="left", padx=(6, 0))
        ttk.Label(outer, text="例: エリア2を避けて、緑を先に探して。 ／ エリア2も通っていいよ。").pack(
            anchor="w", pady=(5, 0)
        )
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self.poll)

    def append(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def choose_areas(self):
        path = filedialog.askopenfilename(title="エリアJSON", filetypes=[("JSON", "*.json")])
        if path:
            self.area_path.set(path)

    def start(self):
        if self.process is not None:
            return
        prompt = self.initial.get().strip()
        if not 1 <= len(prompt) <= 500:
            messagebox.showerror("入力エラー", "最初の指示は1〜500文字で入力してください。")
            return
        try:
            areas = Path(self.area_path.get())
            if not areas.is_file():
                raise ValueError("エリアJSONが見つかりません。")
            out = ROOT / "outputs" / "interactive"
            out.mkdir(parents=True, exist_ok=True)
            self.session_dir = Path(tempfile.mkdtemp(prefix="run-", dir=out))
            correction = self.session_dir / "correction.txt"
            correction.write_text("", encoding="utf-8")
            self.pid_file = self.session_dir / "process.pid"
            command = [
                "wsl.exe",
                "--exec",
                "setsid",
                "--wait",
                "bash",
                "-c",
                'set -e; root=$(wslpath -a "$1"); area=$(wslpath -a "$2"); out=$(wslpath -a "$3"); '
                'cd "$root"; printf "%s\\n" "$$" > "$out/process.pid"; '
                "exec bash scripts/demo.sh --task search --planner codex --seed 305 --agent "
                '--interactive-dialogue --field-areas "$area" --correction-file "$out/correction.txt" '
                '--prompt "$4" --out-dir "$out"',
                "robot-session",
                str(ROOT),
                str(areas),
                str(self.session_dir),
                prompt,
            ]
            self.process = subprocess.Popen(
                command,
                cwd=ROOT,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                creationflags=subprocess.CREATE_NO_WINDOW,
            )
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            messagebox.showerror("開始できません", str(exc))
            return
        self.start_button.configure(state="disabled")
        self.choose.configure(state="disabled")
        self.initial.configure(state="disabled")
        self.status.set("起動中。最初の指示の処理を待っています。")
        self.append("開始: " + prompt)
        threading.Thread(target=self.read_output, daemon=True).start()

    def read_output(self):
        process = self.process
        for line in process.stdout:
            self.events.put(("line", line))
        process.wait()
        self.events.put(("exit", process.returncode))

    def send(self):
        if not self.ready or self.process is None:
            return
        try:
            write_instruction(self.session_dir / "correction.txt", self.message.get())
        except (OSError, ValueError) as exc:
            messagebox.showerror("送信できません", str(exc))
            return
        self.append("あなた: " + self.message.get().strip())
        self.message.delete(0, "end")
        self.ready = False
        self.send_button.configure(state="disabled")
        self.status.set("指示を処理中です。結果か確認質問を待ってください。")

    def poll(self):
        while True:
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "line":
                text, ready = describe_line(value)
                if text:
                    self.append(text)
                if ready:
                    self.ready = True
                    self.send_button.configure(state="normal")
                    self.status.set("指示を入力できます。処理中は次の送信を待ちます。")
            else:
                self.ready = False
                self.send_button.configure(state="disabled")
                self.start_button.configure(state="normal")
                self.choose.configure(state="normal")
                self.initial.configure(state="normal")
                self.status.set("終了しました。開始を押すと新しい試行を行います。")
                files = sorted(self.session_dir.glob("plan-*.json"))
                if files:
                    try:
                        data = json.loads(files[-1].read_text(encoding="utf-8"))
                        result = data.get("result", {})
                        self.append(f"結果: {result.get('outcome', data.get('status'))} ／ ログ: {files[-1]}")
                    except (OSError, ValueError):
                        self.append("終了ログを読み取れませんでした。")
                else:
                    self.append(f"終了コード: {value}")
                self.process = None
        self.root.after(100, self.poll)

    def close(self):
        if self.process is not None and self.process.poll() is None:
            try:
                pid = self.pid_file.read_text(encoding="ascii").strip()
                if pid.isdecimal() and int(pid) > 1:
                    subprocess.run(
                        ["wsl.exe", "-e", "/bin/kill", "-TERM", "--", f"-{pid}"],
                        capture_output=True,
                        timeout=5,
                        creationflags=subprocess.CREATE_NO_WINDOW,
                    )
            except (OSError, subprocess.SubprocessError):
                pass
            self.process.terminate()
        self.root.destroy()


if __name__ == "__main__":
    InteractiveApp(tk.Tk()).root.mainloop()
