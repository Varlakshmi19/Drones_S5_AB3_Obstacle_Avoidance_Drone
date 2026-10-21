"""
Interactive 2D/3D Obstacle Placement & Arrangement Editor for MuJoCo Drone Simulation.
Provides full Mouse and Keyboard control for placing, moving, resizing, and configuring
static and dynamic obstacles.

Controls:
  Mouse:
    - Left Click on empty space: Place new obstacle
    - Left Click on obstacle: Select obstacle
    - Left Click & Drag: Move / reposition obstacle across the arena
    - Right Click on obstacle: Toggle Static <-> Dynamic (moving) obstacle
    - Mouse Wheel / Scroll: Resize selected obstacle
  Keyboard:
    - 'A' or 'B': Add Box obstacle at mouse pointer
    - 'C': Add Cylinder obstacle at mouse pointer
    - 'D': Toggle selected obstacle between Static and Dynamic
    - 'Delete' or 'Backspace': Remove selected obstacle
    - Arrow Keys or 'W'/'A'/'S'/'D': Nudge selected obstacle position
    - '+' / '-': Increase / decrease obstacle size
    - 'R': Rotate obstacle
    - 'Space' / 'Enter': Launch MuJoCo 3D Simulation with this custom layout
    - 'Ctrl+S': Save custom arrangement to JSON
    - 'Ctrl+O': Open saved custom arrangement JSON

Usage:
  python interactive_arena.py
  python run_sim.py --editor
"""

import sys
import os
import json
import subprocess
import tkinter as tk
from tkinter import messagebox, filedialog
import numpy as np

# Ensure workspace root in path
current_dir = os.path.dirname(os.path.abspath(__file__))
workspace_dir = os.path.abspath(os.path.join(current_dir, ".."))
for p in [workspace_dir, current_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)


class InteractiveArenaEditor(tk.Tk):
    """
    Tkinter desktop application for visual mouse & keyboard obstacle editing.
    """

    def __init__(self):
        super().__init__()
        self.title("MuJoCo Drone Arena Customizer | Mouse & Keyboard Editor")
        self.geometry("1180x820")
        self.minsize(980, 700)
        self.configure(bg="#0b0f19")

        # Arena coordinate bounds (meters)
        self.arena_x_min = -1.5
        self.arena_x_max = 8.0
        self.arena_y_min = -3.2
        self.arena_y_max = 3.2

        # Drone start and goal positions
        self.start_pos = [0.0, 0.0, 1.0]
        self.goal_pos = [6.0, 0.0, 1.0]

        # Obstacles list
        self.obstacles = []
        self.selected_idx = None
        self.drag_data = {"x": 0, "y": 0, "item": None}

        # Settings
        self.current_shape = "box"  # "box" or "cylinder"
        self.make_dynamic = False
        self.default_size = [0.18, 0.18, 0.50]
        self.grid_snap = True
        self.snap_size = 0.10  # 10cm snap

        # Build UI Layout
        self._build_header()
        self._build_main_layout()
        self._bind_events()

        # Load default obstacle preset (Forest / Dynamic)
        self.load_preset("single_dynamic")

    def _build_header(self):
        header_frame = tk.Frame(self, bg="#111827", padx=16, pady=10, relief=tk.FLAT)
        header_frame.pack(fill=tk.X, side=tk.TOP)

        title_lbl = tk.Label(
            header_frame,
            text="QUADCOPTER ARENA OBSTACLE BUILDER",
            font=("Segoe UI", 14, "bold"),
            fg="#00e5ff",
            bg="#111827"
        )
        title_lbl.pack(side=tk.LEFT)

        sub_lbl = tk.Label(
            header_frame,
            text=" | Mouse: Click & Drag to arrange | Keyboard: Keys A, C, D, Del, Arrows, Space to Run",
            font=("Segoe UI", 10),
            fg="#9ca3af",
            bg="#111827"
        )
        sub_lbl.pack(side=tk.LEFT, padx=10)

        # Quick Run Button in Header
        run_btn = tk.Button(
            header_frame,
            text="▶ RUN IN MUJOCO 3D",
            font=("Segoe UI", 10, "bold"),
            bg="#00e676",
            fg="#000000",
            activebackground="#69f0ae",
            padx=14,
            pady=4,
            relief=tk.FLAT,
            command=self.launch_mujoco_sim
        )
        run_btn.pack(side=tk.RIGHT)

    def _build_main_layout(self):
        container = tk.Frame(self, bg="#0b0f19")
        container.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)

        # Left Canvas (Arena View)
        self.canvas_frame = tk.Frame(container, bg="#161e2e", bd=1, relief=tk.SOLID)
        self.canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        self.canvas = tk.Canvas(self.canvas_frame, bg="#0e1422", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Coordinates info bar at bottom of canvas
        self.coord_bar = tk.Label(
            self.canvas_frame,
            text="World Pos: X=0.00m, Y=0.00m | Obstacles: 0 | Selected: None",
            bg="#111827",
            fg="#38bdf8",
            font=("Consolas", 10),
            anchor=tk.W,
            padx=10,
            pady=5
        )
        self.coord_bar.pack(side=tk.BOTTOM, fill=tk.X)

        # Right Control Panel
        panel = tk.Frame(container, bg="#111827", width=330, padx=14, pady=14)
        panel.pack(side=tk.RIGHT, fill=tk.Y)
        panel.pack_propagate(False)

        # Section: Place Obstacles
        self._create_section_label(panel, "1. PLACE OBSTACLES")

        btn_row1 = tk.Frame(panel, bg="#111827")
        btn_row1.pack(fill=tk.X, pady=4)
        tk.Button(
            btn_row1, text="+ Add Box (A)", font=("Segoe UI", 9, "bold"), bg="#1e293b", fg="#f3f4f6",
            activebackground="#334155", relief=tk.FLAT, pady=6, command=lambda: self.add_obstacle("box")
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        tk.Button(
            btn_row1, text="+ Add Cylinder (C)", font=("Segoe UI", 9, "bold"), bg="#1e293b", fg="#f3f4f6",
            activebackground="#334155", relief=tk.FLAT, pady=6, command=lambda: self.add_obstacle("cylinder")
        ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(4, 0))

        self.dynamic_var = tk.BooleanVar(value=False)
        dyn_chk = tk.Checkbutton(
            panel, text="Dynamic Moving Obstacle (D)", variable=self.dynamic_var,
            font=("Segoe UI", 9, "bold"), fg="#fbbf24", bg="#111827", selectcolor="#1f2937",
            activebackground="#111827", activeforeground="#fbbf24"
        )
        dyn_chk.pack(anchor=tk.W, pady=4)

        # Section: Selected Obstacle Controls
        self._create_section_label(panel, "2. EDIT SELECTED")

        edit_row = tk.Frame(panel, bg="#111827")
        edit_row.pack(fill=tk.X, pady=4)
        tk.Button(
            edit_row, text="Toggle Dynamic", font=("Segoe UI", 8, "bold"), bg="#374151", fg="#fbbf24",
            relief=tk.FLAT, pady=5, command=self.toggle_selected_dynamic
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3))
        tk.Button(
            edit_row, text="Delete (Del)", font=("Segoe UI", 8, "bold"), bg="#991b1b", fg="#ffffff",
            relief=tk.FLAT, pady=5, command=self.delete_selected
        ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(3, 0))

        size_row = tk.Frame(panel, bg="#111827")
        size_row.pack(fill=tk.X, pady=4)
        tk.Button(
            size_row, text="Size + (+)", font=("Segoe UI", 8), bg="#1e293b", fg="#f3f4f6",
            relief=tk.FLAT, pady=4, command=lambda: self.adjust_selected_size(0.04)
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3))
        tk.Button(
            size_row, text="Size - (-)", font=("Segoe UI", 8), bg="#1e293b", fg="#f3f4f6",
            relief=tk.FLAT, pady=4, command=lambda: self.adjust_selected_size(-0.04)
        ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(3, 0))

        # Section: Presets
        self._create_section_label(panel, "3. PRESET ARRANGEMENTS")
        preset_frame = tk.Frame(panel, bg="#111827")
        preset_frame.pack(fill=tk.X, pady=4)

        presets = [
            ("Dynamic Intruder", "single_dynamic"),
            ("Narrow Slit", "narrow_slit"),
            ("Forest Maze", "forest"),
            ("U-Trap", "dead_end"),
            ("Dual Dynamic", "dual_dynamic")
        ]
        for name, key in presets:
            tk.Button(
                preset_frame, text=name, font=("Segoe UI", 8), bg="#1e293b", fg="#93c5fd",
                relief=tk.FLAT, pady=4, command=lambda k=key: self.load_preset(k)
            ).pack(fill=tk.X, pady=2)

        # Section: File & Simulation
        self._create_section_label(panel, "4. SAVE & SIMULATE")

        file_row = tk.Frame(panel, bg="#111827")
        file_row.pack(fill=tk.X, pady=4)
        tk.Button(
            file_row, text="Save JSON", font=("Segoe UI", 9), bg="#1e293b", fg="#f3f4f6",
            relief=tk.FLAT, pady=5, command=self.save_to_json
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 3))
        tk.Button(
            file_row, text="Load JSON", font=("Segoe UI", 9), bg="#1e293b", fg="#f3f4f6",
            relief=tk.FLAT, pady=5, command=self.load_from_json
        ).pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(3, 0))

        tk.Button(
            panel, text="Clear Arena", font=("Segoe UI", 8), bg="#262626", fg="#9ca3af",
            relief=tk.FLAT, pady=4, command=self.clear_all
        ).pack(fill=tk.X, pady=(4, 10))

        # Launch Big Button
        launch_btn = tk.Button(
            panel,
            text="▶ LAUNCH MUJOCO 3D GUI\n(Spacebar)",
            font=("Segoe UI", 10, "bold"),
            bg="#00e676",
            fg="#000000",
            activebackground="#69f0ae",
            pady=8,
            relief=tk.FLAT,
            command=self.launch_mujoco_sim
        )
        launch_btn.pack(fill=tk.X, pady=6)

        # Analyze Button
        analyze_btn = tk.Button(
            panel,
            text="📊 Run & Show Analysis Plot",
            font=("Segoe UI", 9, "bold"),
            bg="#3b82f6",
            fg="#ffffff",
            activebackground="#60a5fa",
            pady=6,
            relief=tk.FLAT,
            command=self.launch_analysis
        )
        analyze_btn.pack(fill=tk.X, pady=4)

    def _create_section_label(self, parent, text):
        lbl = tk.Label(
            parent, text=text, font=("Segoe UI", 8, "bold"),
            fg="#00e5ff", bg="#111827", pady=6
        )
        lbl.pack(anchor=tk.W, pady=(8, 2))

    # ============================================================
    # Coordinate Transforms (World <-> Canvas)
    # ============================================================
    def world_to_canvas(self, wx, wy):
        w = self.canvas.winfo_width() or 800
        h = self.canvas.winfo_height() or 600
        cx = ((wx - self.arena_x_min) / (self.arena_x_max - self.arena_x_min)) * w
        cy = ((self.arena_y_max - wy) / (self.arena_y_max - self.arena_y_min)) * h
        return cx, cy

    def canvas_to_world(self, cx, cy):
        w = self.canvas.winfo_width() or 800
        h = self.canvas.winfo_height() or 600
        wx = self.arena_x_min + (cx / w) * (self.arena_x_max - self.arena_x_min)
        wy = self.arena_y_max - (cy / h) * (self.arena_y_max - self.arena_y_min)
        if self.grid_snap:
            wx = round(wx / self.snap_size) * self.snap_size
            wy = round(wy / self.snap_size) * self.snap_size
        return round(wx, 2), round(wy, 2)

    # ============================================================
    # Canvas Rendering
    # ============================================================
    def redraw(self):
        self.canvas.delete("all")
        w = self.canvas.winfo_width() or 800
        h = self.canvas.winfo_height() or 600

        # Draw Grid Lines
        for gx in np.arange(0.0, 7.5, 0.5):
            cx1, cy1 = self.world_to_canvas(gx, self.arena_y_min)
            cx2, cy2 = self.world_to_canvas(gx, self.arena_y_max)
            color = "#1e293b" if int(gx) == gx else "#141c2c"
            self.canvas.create_line(cx1, cy1, cx2, cy2, fill=color, width=1)
            if int(gx) == gx:
                self.canvas.create_text(cx1, h - 14, text=f"{int(gx)}m", fill="#475569", font=("Consolas", 8))

        for gy in np.arange(-3.0, 3.5, 0.5):
            cx1, cy1 = self.world_to_canvas(self.arena_x_min, gy)
            cx2, cy2 = self.world_to_canvas(self.arena_x_max, gy)
            color = "#1e293b" if int(gy) == gy else "#141c2c"
            self.canvas.create_line(cx1, cy1, cx2, cy2, fill=color, width=1)

        # Centerline
        c_x1, c_y = self.world_to_canvas(self.arena_x_min, 0.0)
        c_x2, _ = self.world_to_canvas(self.arena_x_max, 0.0)
        self.canvas.create_line(c_x1, c_y, c_x2, c_y, fill="#334155", dash=(4, 4), width=1)

        # Draw Flight Trajectory Line (Start to Goal)
        s_cx, s_cy = self.world_to_canvas(self.start_pos[0], self.start_pos[1])
        g_cx, g_cy = self.world_to_canvas(self.goal_pos[0], self.goal_pos[1])
        self.canvas.create_line(s_cx, s_cy, g_cx, g_cy, fill="#0284c7", dash=(2, 6), width=1.5)

        # Draw Start Marker
        self.canvas.create_oval(s_cx - 12, s_cy - 12, s_cx + 12, s_cy + 12, fill="#0284c7", outline="#38bdf8", width=2)
        self.canvas.create_text(s_cx, s_cy - 18, text="DRONE START (0,0)", fill="#38bdf8", font=("Segoe UI", 8, "bold"))

        # Draw Goal Marker
        self.canvas.create_oval(g_cx - 14, g_cy - 14, g_cx + 14, g_cy + 14, fill="#166534", outline="#22c55e", width=2)
        self.canvas.create_text(g_cx, g_cy, text="★", fill="#22c55e", font=("Segoe UI", 12, "bold"))
        self.canvas.create_text(g_cx, g_cy - 20, text="GOAL BEACON (6,0)", fill="#22c55e", font=("Segoe UI", 8, "bold"))

        # Draw Obstacles
        for idx, obs in enumerate(self.obstacles):
            ox, oy = obs["pos"][0], obs["pos"][1]
            ocx, ocy = self.world_to_canvas(ox, oy)
            half_w = obs["size"][0]
            half_h = obs["size"][1]

            # Convert half sizes to canvas pixels
            px_w = abs(self.world_to_canvas(ox + half_w, oy)[0] - ocx)
            px_h = abs(self.world_to_canvas(ox, oy + half_h)[1] - ocy)

            is_sel = (idx == self.selected_idx)
            is_dyn = obs.get("is_dynamic", False)

            # Colors
            if is_dyn:
                fill_color = "#ea580c"  # Orange for dynamic
                border_color = "#f97316"
            elif obs.get("type") == "cylinder":
                fill_color = "#9333ea"  # Purple for cylinder
                border_color = "#a855f7"
            else:
                fill_color = "#dc2626"  # Red for static box
                border_color = "#ef4444"

            if is_sel:
                border_color = "#00e5ff"
                border_width = 3
            else:
                border_width = 1.5

            # Dynamic motion slide track guide
            if is_dyn:
                t_top_x, t_top_y = self.world_to_canvas(ox, 1.2)
                t_bot_x, t_bot_y = self.world_to_canvas(ox, -1.2)
                self.canvas.create_line(t_top_x, t_top_y, t_bot_x, t_bot_y, fill="#fb923c", dash=(3, 3), width=1)
                self.canvas.create_text(ocx, t_top_y - 8, text="↕ Dynamic Slide Track", fill="#fb923c", font=("Consolas", 7))

            # Shape draw
            if obs.get("type") == "cylinder":
                tag = f"obs_{idx}"
                self.canvas.create_oval(
                    ocx - px_w, ocy - px_h, ocx + px_w, ocy + px_h,
                    fill=fill_color, outline=border_color, width=border_width, tags=(tag, "obstacle")
                )
            else:
                tag = f"obs_{idx}"
                self.canvas.create_rectangle(
                    ocx - px_w, ocy - px_h, ocx + px_w, ocy + px_h,
                    fill=fill_color, outline=border_color, width=border_width, tags=(tag, "obstacle")
                )

            # Label
            type_label = "DYN" if is_dyn else ("CYL" if obs.get("type") == "cylinder" else "BOX")
            self.canvas.create_text(
                ocx, ocy, text=f"#{idx+1} {type_label}", fill="#ffffff", font=("Segoe UI", 7, "bold")
            )

        # Update info bar
        sel_text = f"#{self.selected_idx+1} ({self.obstacles[self.selected_idx]['type']})" if self.selected_idx is not None else "None"
        self.coord_bar.config(
            text=f"Total Obstacles: {len(self.obstacles)} | Selected: {sel_text} | Snap: {self.snap_size}m"
        )

    # ============================================================
    # Mouse & Keyboard Event Handlers
    # ============================================================
    def _bind_events(self):
        self.canvas.bind("<Configure>", lambda e: self.redraw())
        self.canvas.bind("<Button-1>", self.on_canvas_left_click)
        self.canvas.bind("<B1-Motion>", self.on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)
        self.canvas.bind("<Button-3>", self.on_canvas_right_click)
        self.canvas.bind("<Motion>", self.on_mouse_move)
        self.canvas.bind("<MouseWheel>", self.on_mouse_wheel)

        # Keyboard Bindings
        self.bind("<Key-a>", lambda e: self.add_obstacle("box"))
        self.bind("<Key-A>", lambda e: self.add_obstacle("box"))
        self.bind("<Key-b>", lambda e: self.add_obstacle("box"))
        self.bind("<Key-B>", lambda e: self.add_obstacle("box"))
        self.bind("<Key-c>", lambda e: self.add_obstacle("cylinder"))
        self.bind("<Key-C>", lambda e: self.add_obstacle("cylinder"))
        self.bind("<Key-d>", lambda e: self.toggle_selected_dynamic())
        self.bind("<Key-D>", lambda e: self.toggle_selected_dynamic())
        self.bind("<Delete>", lambda e: self.delete_selected())
        self.bind("<BackSpace>", lambda e: self.delete_selected())

        # Arrow Keys Nudging
        self.bind("<Left>", lambda e: self.nudge_selected(-0.1, 0.0))
        self.bind("<Right>", lambda e: self.nudge_selected(0.1, 0.0))
        self.bind("<Up>", lambda e: self.nudge_selected(0.0, 0.1))
        self.bind("<Down>", lambda e: self.nudge_selected(0.0, -0.1))

        # Size Adjustments
        self.bind("<plus>", lambda e: self.adjust_selected_size(0.04))
        self.bind("<equal>", lambda e: self.adjust_selected_size(0.04))
        self.bind("<minus>", lambda e: self.adjust_selected_size(-0.04))
        self.bind("<underscore>", lambda e: self.adjust_selected_size(-0.04))

        # Spacebar / Enter to run
        self.bind("<space>", lambda e: self.launch_mujoco_sim())
        self.bind("<Return>", lambda e: self.launch_mujoco_sim())

        # File shortcuts
        self.bind("<Control-s>", lambda e: self.save_to_json())
        self.bind("<Control-o>", lambda e: self.load_from_json())

    def on_mouse_move(self, event):
        wx, wy = self.canvas_to_world(event.x, event.y)
        sel_text = f"#{self.selected_idx+1}" if self.selected_idx is not None else "None"
        self.coord_bar.config(
            text=f"Mouse Pos: X={wx:.2f}m, Y={wy:.2f}m | Obstacles: {len(self.obstacles)} | Selected: {sel_text}"
        )

    def on_canvas_left_click(self, event):
        wx, wy = self.canvas_to_world(event.x, event.y)
        clicked_idx = self.find_obstacle_at(wx, wy)

        if clicked_idx is not None:
            # Select existing obstacle
            self.selected_idx = clicked_idx
            self.drag_data["x"] = wx
            self.drag_data["y"] = wy
            self.drag_data["item"] = clicked_idx
        else:
            # Clicked on empty space: Place new obstacle!
            self.add_obstacle_at(wx, wy, shape=self.current_shape, is_dynamic=self.dynamic_var.get())

        self.redraw()

    def on_canvas_drag(self, event):
        if self.selected_idx is not None:
            wx, wy = self.canvas_to_world(event.x, event.y)
            # Bound inside arena
            wx = float(np.clip(wx, self.arena_x_min + 0.5, self.arena_x_max - 0.5))
            wy = float(np.clip(wy, self.arena_y_min + 0.3, self.arena_y_max - 0.3))

            self.obstacles[self.selected_idx]["pos"][0] = wx
            self.obstacles[self.selected_idx]["pos"][1] = wy
            self.redraw()

    def on_canvas_release(self, event):
        self.drag_data["item"] = None

    def on_canvas_right_click(self, event):
        wx, wy = self.canvas_to_world(event.x, event.y)
        idx = self.find_obstacle_at(wx, wy)
        if idx is not None:
            self.selected_idx = idx
            self.toggle_selected_dynamic()
        self.redraw()

    def on_mouse_wheel(self, event):
        if self.selected_idx is not None:
            delta = 0.03 if event.delta > 0 else -0.03
            self.adjust_selected_size(delta)

    def find_obstacle_at(self, wx, wy):
        """Find index of obstacle covering world coordinate (wx, wy)."""
        for idx, obs in enumerate(self.obstacles):
            ox, oy = obs["pos"][0], obs["pos"][1]
            hw, hh = obs["size"][0], obs["size"][1]
            if (ox - hw <= wx <= ox + hw) and (oy - hh <= wy <= oy + hh):
                return idx
        return None

    # ============================================================
    # Obstacle Editing Actions
    # ============================================================
    def add_obstacle_at(self, x, y, shape="box", is_dynamic=False):
        new_obs = {
            "name": f"obs_custom_{len(self.obstacles)+1}",
            "pos": [round(float(x), 2), round(float(y), 2), 1.0],
            "size": [0.18, 0.18, 0.50] if shape == "box" else [0.18, 0.50],
            "type": shape,
            "rgba": [0.95, 0.35, 0.1, 1.0] if is_dynamic else ([0.75, 0.25, 0.85, 1.0] if shape == "cylinder" else [0.85, 0.25, 0.25, 1.0]),
            "is_dynamic": is_dynamic
        }
        self.obstacles.append(new_obs)
        self.selected_idx = len(self.obstacles) - 1
        self.redraw()

    def add_obstacle(self, shape):
        self.current_shape = shape
        # Place in center between start and goal
        x = 3.0
        y = 0.0
        self.add_obstacle_at(x, y, shape=shape, is_dynamic=self.dynamic_var.get())

    def toggle_selected_dynamic(self):
        if self.selected_idx is not None and self.selected_idx < len(self.obstacles):
            obs = self.obstacles[self.selected_idx]
            obs["is_dynamic"] = not obs.get("is_dynamic", False)
            if obs["is_dynamic"]:
                obs["rgba"] = [0.95, 0.45, 0.1, 1.0]
            else:
                obs["rgba"] = [0.85, 0.25, 0.25, 1.0] if obs["type"] == "box" else [0.75, 0.25, 0.85, 1.0]
            self.redraw()

    def delete_selected(self):
        if self.selected_idx is not None and self.selected_idx < len(self.obstacles):
            del self.obstacles[self.selected_idx]
            self.selected_idx = None
            self.redraw()

    def nudge_selected(self, dx, dy):
        if self.selected_idx is not None and self.selected_idx < len(self.obstacles):
            obs = self.obstacles[self.selected_idx]
            obs["pos"][0] = round(obs["pos"][0] + dx, 2)
            obs["pos"][1] = round(obs["pos"][1] + dy, 2)
            self.redraw()

    def adjust_selected_size(self, delta):
        if self.selected_idx is not None and self.selected_idx < len(self.obstacles):
            obs = self.obstacles[self.selected_idx]
            obs["size"][0] = max(0.08, round(obs["size"][0] + delta, 2))
            if obs["type"] == "box":
                obs["size"][1] = max(0.08, round(obs["size"][1] + delta, 2))
            self.redraw()

    def clear_all(self):
        if messagebox.askyesno("Clear Arena", "Clear all obstacles from the arena?"):
            self.obstacles = []
            self.selected_idx = None
            self.redraw()

    # ============================================================
    # Presets
    # ============================================================
    def load_preset(self, preset_name):
        self.obstacles = []
        z = 1.0

        if preset_name == "single_dynamic":
            self.obstacles.append({
                "name": "obs_dynamic",
                "pos": [2.8, 0.0, z],
                "size": [0.18, 0.18, 0.50],
                "type": "box",
                "rgba": [0.95, 0.55, 0.1, 1.0],
                "is_dynamic": True
            })
        elif preset_name == "narrow_slit":
            self.obstacles.extend([
                {"name": "obs1_l", "pos": [2.5, 0.38, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.3, 0.2, 1.0]},
                {"name": "obs1_r", "pos": [2.5, -0.38, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.3, 0.2, 1.0]},
                {"name": "obs2_l", "pos": [3.5, 0.42, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.3, 0.2, 1.0]},
                {"name": "obs2_r", "pos": [3.5, -0.42, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.3, 0.2, 1.0]}
            ])
        elif preset_name == "forest":
            self.obstacles.extend([
                {"name": "obs1", "pos": [1.8, 0.0, z], "size": [0.15, 0.50], "type": "cylinder", "rgba": [0.8, 0.3, 0.2, 1.0]},
                {"name": "obs2", "pos": [3.0, 0.48, z], "size": [0.15, 0.50], "type": "cylinder", "rgba": [0.8, 0.3, 0.2, 1.0]},
                {"name": "obs3", "pos": [4.0, -0.42, z], "size": [0.15, 0.50], "type": "cylinder", "rgba": [0.8, 0.3, 0.2, 1.0]},
                {"name": "obs4", "pos": [4.8, 0.10, z], "size": [0.15, 0.50], "type": "cylinder", "rgba": [0.8, 0.3, 0.2, 1.0]}
            ])
        elif preset_name == "dead_end":
            self.obstacles.extend([
                {"name": "obs1", "pos": [2.5, 0.0, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.15, 0.15, 1.0]},
                {"name": "obs2_l", "pos": [2.35, 0.36, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.25, 0.25, 1.0]},
                {"name": "obs2_r", "pos": [2.35, -0.36, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.25, 0.25, 1.0]},
                {"name": "obs3_l", "pos": [2.0, 0.52, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.25, 0.25, 1.0]},
                {"name": "obs3_r", "pos": [2.0, -0.52, z], "size": [0.15, 0.15, 0.50], "type": "box", "rgba": [0.85, 0.25, 0.25, 1.0]}
            ])
        elif preset_name == "dual_dynamic":
            self.obstacles.extend([
                {"name": "obs_dyn1", "pos": [2.2, 0.0, z], "size": [0.18, 0.18, 0.50], "type": "box", "rgba": [0.95, 0.55, 0.1, 1.0], "is_dynamic": True},
                {"name": "obs_dyn2", "pos": [4.0, 0.0, z], "size": [0.18, 0.18, 0.50], "type": "box", "rgba": [0.95, 0.35, 0.1, 1.0], "is_dynamic": True}
            ])

        self.selected_idx = 0 if len(self.obstacles) > 0 else None
        self.redraw()

    # ============================================================
    # JSON Save & Load
    # ============================================================
    def get_custom_layout_path(self):
        return os.path.join(current_dir, "custom_arena.json")

    def save_to_json(self):
        filepath = self.get_custom_layout_path()
        data = {
            "start_pos": self.start_pos,
            "goal_pos": self.goal_pos,
            "obstacles": self.obstacles
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)
        messagebox.showinfo("Saved", f"Arena layout saved to:\n{filepath}")

    def load_from_json(self):
        filepath = filedialog.askopenfilename(
            initialdir=current_dir,
            title="Select Custom Arena JSON",
            filetypes=(("JSON Files", "*.json"), ("All Files", "*.*"))
        )
        if filepath and os.path.exists(filepath):
            with open(filepath, "r") as f:
                data = json.load(f)
            self.obstacles = data.get("obstacles", [])
            self.selected_idx = 0 if len(self.obstacles) > 0 else None
            self.redraw()
            messagebox.showinfo("Loaded", f"Loaded {len(self.obstacles)} obstacles from {os.path.basename(filepath)}")

    # ============================================================
    # Launch Simulation & Analysis
    # ============================================================
    def launch_mujoco_sim(self):
        # Auto-save current layout
        filepath = self.get_custom_layout_path()
        with open(filepath, "w") as f:
            json.dump({"obstacles": self.obstacles}, f, indent=2)

        # Launch command
        cmd = [
            sys.executable,
            os.path.join(current_dir, "run_sim.py"),
            "--custom-file", filepath,
            "--algo", "reactive",
            "--gui"
        ]
        print(f"[INFO] Launching MuJoCo 3D Simulation: {' '.join(cmd)}")
        subprocess.Popen(cmd)

    def launch_analysis(self):
        filepath = self.get_custom_layout_path()
        with open(filepath, "w") as f:
            json.dump({"obstacles": self.obstacles}, f, indent=2)

        cmd = [
            sys.executable,
            os.path.join(current_dir, "analyze_dynamic_obstacle.py"),
            "--scenario", "7",
            "--save-plot"
        ]
        subprocess.Popen(cmd)
        messagebox.showinfo("Running Analysis", "Running dynamic simulation analysis.\nCheck dynamic_obstacle_analysis.png when complete!")


def launch_arena_editor():
    """Launch the interactive arena builder application."""
    app = InteractiveArenaEditor()
    app.mainloop()


if __name__ == "__main__":
    launch_arena_editor()
