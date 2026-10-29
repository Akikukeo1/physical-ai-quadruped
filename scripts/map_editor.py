"""Small top-down editor for a custom MuJoCo search map."""

import json
import sys
import tkinter as tk
from copy import deepcopy
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from pai_motion.custom_map import validate_map  # noqa: E402

EMPTY = {"start": [0, 0], "obstacles": [], "targets": {"blue": [3.7, 1], "green": [3.7, -1]}, "areas": [], "points": []}
SCALE = 100


class MapEditor:
    def __init__(self, window, on_saved=None, path=None):
        self.window, self.on_saved = window, on_saved
        self.map = deepcopy(EMPTY)
        if path:
            self.map = validate_map(json.loads(Path(path).read_text(encoding="utf-8")))
        self.tool = tk.StringVar(value="obstacle")
        self.first = None
        window.title("マップ作成")
        bar = ttk.Frame(window, padding=8)
        bar.pack(fill="x")
        for label, value in (
            ("壁", "obstacle"),
            ("エリア", "area"),
            ("出発", "start"),
            ("青", "blue"),
            ("緑", "green"),
            ("ポイント", "point"),
            ("削除", "delete"),
        ):
            ttk.Radiobutton(bar, text=label, variable=self.tool, value=value).pack(side="left")
        ttk.Button(bar, text="開く", command=self.load).pack(side="right")
        ttk.Button(bar, text="保存して選択", command=self.save).pack(side="right", padx=6)
        ttk.Label(
            window, text="壁・エリアはドラッグ。他はクリックで配置。削除は図形をクリック。橙色はポイント。", padding=6
        ).pack(anchor="w")
        self.canvas = tk.Canvas(window, width=600, height=560, bg="#f3f5f7", highlightthickness=0)
        self.canvas.pack(padx=10, pady=6)
        self.canvas.bind("<ButtonPress-1>", self.down)
        self.canvas.bind("<ButtonRelease-1>", self.up)
        self.draw()

    @staticmethod
    def screen(p):
        return (p[0] + 1.3) * SCALE, (2.8 - p[1]) * SCALE

    @staticmethod
    def world(x, y):
        return [round(max(-1.15, min(4.55, x / SCALE - 1.3)), 2), round(max(-2.65, min(2.65, 2.8 - y / SCALE)), 2)]

    def draw(self):
        c = self.canvas
        c.delete("all")
        c.create_rectangle(0, 0, 600, 560, outline="#263238", width=5)
        for i, box in enumerate(self.map["obstacles"]):
            self.rectangle(box, "#63727a", f"obstacle:{i}")
        for i, area in enumerate(self.map["areas"]):
            self.rectangle(area["bounds"], "#a5e3e8", f"area:{i}")
            x, y = self.screen(
                [(area["bounds"][0] + area["bounds"][2]) / 2, (area["bounds"][1] + area["bounds"][3]) / 2]
            )
            c.create_text(x, y, text=area["id"], tags=(f"area:{i}",))
        for i, point in enumerate(self.map["points"]):
            x, y = self.screen(point["position"])
            c.create_oval(x - 10, y - 10, x + 10, y + 10, fill="#ff9d2e", tags=(f"point:{i}",))
            c.create_text(x, y - 18, text=point["id"], tags=(f"point:{i}",))
        for name, point in self.map["targets"].items():
            x, y = self.screen(point)
            c.create_rectangle(
                x - 10, y - 10, x + 10, y + 10, fill="#2f65ed" if name == "blue" else "#28ad68", tags=(name,)
            )
        x, y = self.screen(self.map["start"])
        c.create_oval(x - 12, y - 12, x + 12, y + 12, fill="#f5d742", tags=("start",))
        c.create_text(x, y, text="S", tags=("start",))

    def rectangle(self, box, color, tag):
        x1, y1 = self.screen(box[:2])
        x2, y2 = self.screen(box[2:])
        self.canvas.create_rectangle(x1, y2, x2, y1, fill=color, outline="#334", tags=(tag,))

    def down(self, event):
        self.first = self.world(event.x, event.y)
        if self.tool.get() == "delete":
            items = self.canvas.find_overlapping(event.x - 3, event.y - 3, event.x + 3, event.y + 3)
            if items:
                tags = self.canvas.gettags(items[-1])
                if tags:
                    tag = tags[0]
                    if ":" in tag:
                        kind, index = tag.split(":")
                        self.map[{"point": "points", "area": "areas", "obstacle": "obstacles"}[kind]].pop(int(index))
                    elif tag in ("blue", "green"):
                        self.map["targets"].pop(tag, None)
            self.draw()

    def up(self, event):
        if self.first is None:
            return
        p, q = self.first, self.world(event.x, event.y)
        self.first = None
        tool = self.tool.get()
        if tool in ("obstacle", "area"):
            box = [min(p[0], q[0]), min(p[1], q[1]), max(p[0], q[0]), max(p[1], q[1])]
            if box[2] - box[0] < 0.1 or box[3] - box[1] < 0.1:
                return
            if tool == "obstacle":
                self.map["obstacles"].append(box)
            elif len(self.map["areas"]) < 8:
                names = {a["id"] for a in self.map["areas"]}
                name = next(f"area{i}" for i in range(1, 9) if f"area{i}" not in names)
                self.map["areas"].append({"id": name, "bounds": box})
        elif tool == "start":
            self.map["start"] = q
        elif tool in ("blue", "green"):
            self.map["targets"][tool] = q
        elif tool == "point" and len(self.map["points"]) < 8:
            names = {point["id"] for point in self.map["points"]}
            name = next(f"P{i}" for i in range(1, 9) if f"P{i}" not in names)
            self.map["points"].append({"id": name, "position": q})
        self.draw()

    def load(self):
        name = filedialog.askopenfilename(parent=self.window, filetypes=[("JSON", "*.json")])
        if name:
            try:
                self.map = validate_map(json.loads(Path(name).read_text(encoding="utf-8")))
                self.draw()
            except (ValueError, OSError) as exc:
                messagebox.showerror("開けません", str(exc), parent=self.window)

    def save(self):
        try:
            validate_map(self.map)
        except ValueError as exc:
            messagebox.showerror("保存できません", str(exc), parent=self.window)
            return
        name = filedialog.asksaveasfilename(
            parent=self.window, defaultextension=".json", filetypes=[("JSON", "*.json")]
        )
        if name:
            try:
                Path(name).write_text(
                    json.dumps(self.map, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n"
                )
                if self.on_saved:
                    self.on_saved(name)
                self.window.destroy()
            except OSError as exc:
                messagebox.showerror("保存できません", str(exc), parent=self.window)


if __name__ == "__main__":
    root = tk.Tk()
    MapEditor(root)
    root.mainloop()
