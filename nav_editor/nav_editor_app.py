from __future__ import annotations

import copy
import os
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Optional, Tuple, List, Set, Dict

from nav_editor_model import Project, EditorItem, load_project
from nav_export import export_all

HANDLE_SIZE = 8
MIN_SIZE = 24
POINT_DRAW_RADIUS = 8
POINT_HIT_RADIUS = 12
MARQUEE_DASH = (4, 3)


class NavEditorApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title('Graybox Nav Editor')
        self.root.geometry('1500x960')

        self.project = Project()
        self.project_path: Optional[str] = None

        self.script_dir = Path(__file__).resolve().parent
        self.output_dir = os.path.abspath(self.script_dir.parent / 'assets')

        self.next_id = 1

        self.tool = tk.StringVar(value='select')

        self.selected_ids: Set[int] = set()
        self.selected_id: Optional[int] = None

        self.drag_mode: Optional[str] = None
        self.drag_start = (0.0, 0.0)
        self.orig_geom = (0.0, 0.0, 0.0, 0.0)
        self.group_orig_pos: Dict[int, Tuple[float, float]] = {}

        self.marquee_start: Optional[Tuple[float, float]] = None
        self.marquee_end: Optional[Tuple[float, float]] = None

        self.undo_stack = []
        self.max_undo = 100

        self._build_ui()
        self._bind_shortcuts()
        self.redraw()

    # ---------------- UI -----------------
    def _build_ui(self) -> None:
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(1, weight=1)

        toolbar = ttk.Frame(self.root, padding=6)
        toolbar.grid(row=0, column=0, columnspan=2, sticky='ew')

        tool_buttons = [
            ('select', 'Select'),
            ('nav_rect', 'Nav Rect'),
            ('nav_circle', 'Nav Circle'),
            ('trigger_rect', 'Trigger Rect'),
            ('trigger_circle', 'Trigger Circle'),
            ('object_rect', 'Object Rect'),
            ('object_circle', 'Object Circle'),
            ('point', 'Point'),
            ('delete', 'Delete'),
        ]
        for value, text in tool_buttons:
            ttk.Radiobutton(toolbar, text=text, value=value, variable=self.tool).pack(side='left', padx=2)

        ttk.Separator(toolbar, orient='vertical').pack(side='left', fill='y', padx=8)
        ttk.Button(toolbar, text='New', command=self.new_project).pack(side='left', padx=2)
        ttk.Button(toolbar, text='Open', command=self.open_project).pack(side='left', padx=2)
        ttk.Button(toolbar, text='Save', command=self.save_project_and_exports).pack(side='left', padx=2)
        ttk.Button(toolbar, text='Export As...', command=self.export_as).pack(side='left', padx=2)

        ttk.Separator(toolbar, orient='vertical').pack(side='left', fill='y', padx=8)
        ttk.Button(toolbar, text='Undo', command=self.undo).pack(side='left', padx=2)

        ttk.Label(toolbar, text='Grid').pack(side='left', padx=(12, 4))
        self.grid_var = tk.IntVar(value=self.project.grid_size)
        grid_spin = ttk.Spinbox(
            toolbar,
            from_=4,
            to=128,
            increment=2,
            width=6,
            textvariable=self.grid_var,
            command=self._apply_grid_settings
        )
        grid_spin.pack(side='left')
        grid_spin.bind('<Return>', lambda e: self._apply_grid_settings())
        grid_spin.bind('<FocusOut>', lambda e: self._apply_grid_settings())

        ttk.Label(toolbar, text='Snap').pack(side='left', padx=(12, 4))
        self.snap_var = tk.IntVar(value=self.project.snap_distance)
        snap_spin = ttk.Spinbox(
            toolbar,
            from_=0,
            to=64,
            increment=1,
            width=6,
            textvariable=self.snap_var,
            command=self._apply_grid_settings
        )
        snap_spin.pack(side='left')
        snap_spin.bind('<Return>', lambda e: self._apply_grid_settings())
        snap_spin.bind('<FocusOut>', lambda e: self._apply_grid_settings())

        ttk.Label(toolbar, text='Canvas W').pack(side='left', padx=(12, 4))
        self.canvas_w_var = tk.IntVar(value=self.project.canvas_width)
        canvas_w_spin = ttk.Spinbox(
            toolbar,
            from_=500,
            to=20000,
            increment=100,
            width=8,
            textvariable=self.canvas_w_var,
            command=self._apply_canvas_settings
        )
        canvas_w_spin.pack(side='left')
        canvas_w_spin.bind('<Return>', lambda e: self._apply_canvas_settings())
        canvas_w_spin.bind('<FocusOut>', lambda e: self._apply_canvas_settings())

        ttk.Label(toolbar, text='Canvas H').pack(side='left', padx=(12, 4))
        self.canvas_h_var = tk.IntVar(value=self.project.canvas_height)
        canvas_h_spin = ttk.Spinbox(
            toolbar,
            from_=500,
            to=20000,
            increment=100,
            width=8,
            textvariable=self.canvas_h_var,
            command=self._apply_canvas_settings
        )
        canvas_h_spin.pack(side='left')
        canvas_h_spin.bind('<Return>', lambda e: self._apply_canvas_settings())
        canvas_h_spin.bind('<FocusOut>', lambda e: self._apply_canvas_settings())

        left = ttk.Frame(self.root, padding=6)
        left.grid(row=1, column=0, sticky='ns')

        ttk.Label(left, text='Selected Item').grid(row=0, column=0, columnspan=2, sticky='w')

        self.label_var = tk.StringVar()
        self.role_var = tk.StringVar(value='room')
        self.category_var = tk.StringVar(value='')
        self.kind_var = tk.StringVar(value='')

        ttk.Label(left, text='Label').grid(row=1, column=0, sticky='w', pady=4)
        label_entry = ttk.Entry(left, textvariable=self.label_var, width=18)
        label_entry.grid(row=1, column=1, sticky='ew', pady=4)
        label_entry.bind('<Return>', lambda e: self.update_selected_properties())
        label_entry.bind('<FocusOut>', lambda e: self.update_selected_properties())

        ttk.Label(left, text='Category').grid(row=2, column=0, sticky='w', pady=4)
        ttk.Label(left, textvariable=self.category_var).grid(row=2, column=1, sticky='w', pady=4)

        ttk.Label(left, text='Kind').grid(row=3, column=0, sticky='w', pady=4)
        ttk.Label(left, textvariable=self.kind_var).grid(row=3, column=1, sticky='w', pady=4)

        ttk.Label(left, text='Nav Role').grid(row=4, column=0, sticky='w', pady=4)
        self.role_combo = ttk.Combobox(
            left,
            textvariable=self.role_var,
            values=['room', 'corridor', 'blocked'],
            state='readonly',
            width=16
        )
        self.role_combo.grid(row=4, column=1, sticky='ew', pady=4)
        self.role_combo.bind('<<ComboboxSelected>>', lambda e: self.update_selected_properties())

        self.status_var = tk.StringVar(value='Ready')
        ttk.Label(left, textvariable=self.status_var, wraplength=220).grid(row=10, column=0, columnspan=2, sticky='w', pady=(20, 0))
        left.columnconfigure(1, weight=1)

        canvas_frame = ttk.Frame(self.root)
        canvas_frame.grid(row=1, column=1, sticky='nsew')
        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(canvas_frame, bg='#1e2227', highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky='nsew')

        xscroll = ttk.Scrollbar(canvas_frame, orient='horizontal', command=self.canvas.xview)
        yscroll = ttk.Scrollbar(canvas_frame, orient='vertical', command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=xscroll.set, yscrollcommand=yscroll.set)
        xscroll.grid(row=1, column=0, sticky='ew')
        yscroll.grid(row=0, column=1, sticky='ns')

        self.canvas.config(scrollregion=(0, 0, self.project.canvas_width, self.project.canvas_height))
        self.canvas.bind('<Button-1>', self.on_mouse_down)
        self.canvas.bind('<B1-Motion>', self.on_mouse_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_mouse_up)
        self.canvas.bind('<Double-Button-1>', self.on_double_click)

        self.canvas.bind('<ButtonPress-2>', self.on_middle_mouse_down)
        self.canvas.bind('<B2-Motion>', self.on_middle_mouse_drag)

    def _bind_shortcuts(self) -> None:
        self.root.bind('<Control-s>', lambda e: self.save_project_and_exports())
        self.root.bind('<Control-z>', lambda e: self.undo())
        self.root.bind('<Delete>', lambda e: self.delete_selected())

    # ------------ Undo ------------
    def push_undo_state(self) -> None:
        self.undo_stack.append(copy.deepcopy(self.project))
        if len(self.undo_stack) > self.max_undo:
            self.undo_stack.pop(0)

    def undo(self) -> None:
        if not self.undo_stack:
            self.status('Nothing to undo')
            return

        self.project = self.undo_stack.pop()
        self.next_id = max((item.id for item in self.project.items), default=0) + 1
        self.selected_ids.clear()
        self.selected_id = None
        self.group_orig_pos.clear()
        self.marquee_start = None
        self.marquee_end = None
        self.grid_var.set(self.project.grid_size)
        self.snap_var.set(self.project.snap_distance)
        self.canvas_w_var.set(self.project.canvas_width)
        self.canvas_h_var.set(self.project.canvas_height)

        self.canvas.config(scrollregion=(0, 0, self.project.canvas_width, self.project.canvas_height))
        self.redraw()
        self.status('Undo')

    # ------------ Project lifecycle ------------
    def new_project(self) -> None:
        self.project = Project()
        self.project_path = None
        self.next_id = 1
        self.selected_ids.clear()
        self.selected_id = None
        self.group_orig_pos.clear()
        self.marquee_start = None
        self.marquee_end = None
        self.undo_stack.clear()
        self.grid_var.set(self.project.grid_size)
        self.snap_var.set(self.project.snap_distance)
        self.canvas_w_var.set(self.project.canvas_width)
        self.canvas_h_var.set(self.project.canvas_height)
        self.canvas.config(scrollregion=(0, 0, self.project.canvas_width, self.project.canvas_height))
        self.redraw()
        self.status('New project')

    def open_project(self) -> None:
        path = filedialog.askopenfilename(filetypes=[('JSON project', '*.json')])
        if not path:
            return
        try:
            self.project = load_project(path)
            self.project_path = path
            self.next_id = max((item.id for item in self.project.items), default=0) + 1
            self.selected_ids.clear()
            self.selected_id = None
            self.group_orig_pos.clear()
            self.marquee_start = None
            self.marquee_end = None
            self.undo_stack.clear()
            self.grid_var.set(self.project.grid_size)
            self.snap_var.set(self.project.snap_distance)
            self.canvas_w_var.set(self.project.canvas_width)
            self.canvas_h_var.set(self.project.canvas_height)
            self.canvas.config(scrollregion=(0, 0, self.project.canvas_width, self.project.canvas_height))
            self.redraw()
            self.status(f'Opened {os.path.basename(path)}')
        except Exception as exc:
            messagebox.showerror('Open failed', str(exc))

    def save_project_and_exports(self) -> None:
        if not self.project_path:
            self.project_path = filedialog.asksaveasfilename(
                defaultextension='.json',
                filetypes=[('JSON project', '*.json')],
                initialfile='map_project.json',
            )
            if not self.project_path:
                return

        self._apply_grid_settings()
        self._apply_canvas_settings()

        try:
            result = export_all(self.project, self.output_dir, self.project_path)
            self.status(
                f'Saved project + exports:\n'
                f'JSON: {result.nav_json.name}\n'
                f'BIN: {result.nav_bin.name}\n'
                f'OUT: {result.nav_json.parent}'
            )
        except Exception as exc:
            messagebox.showerror('Save failed', str(exc))

    def export_as(self) -> None:
        folder = filedialog.askdirectory(initialdir=self.output_dir)
        if not folder:
            return
        self.output_dir = folder
        self.save_project_and_exports()

    # ------------ Selection helpers ------------
    def status(self, message: str) -> None:
        self.status_var.set(message)

    def get_selected(self) -> Optional[EditorItem]:
        if self.selected_id is None:
            return None
        for item in self.project.items:
            if item.id == self.selected_id:
                return item
        return None

    def get_selected_items(self) -> List[EditorItem]:
        return [item for item in self.project.items if item.id in self.selected_ids]

    def _refresh_property_panel(self) -> None:
        item = self.get_selected()
        if item is None:
            self.label_var.set('')
            self.category_var.set('')
            self.kind_var.set('')
            self.role_var.set('room')
            self.role_combo.state(['disabled'])
            return

        self.label_var.set(item.label)
        self.category_var.set(item.category)
        self.kind_var.set(item.kind)
        self.role_var.set(item.role if item.category == 'nav' else '')
        if item.category == 'nav':
            self.role_combo.state(['!disabled'])
        else:
            self.role_combo.state(['disabled'])

    def select_item(self, item_id: Optional[int], additive: bool = False, toggle: bool = False) -> None:
        if item_id is None:
            if not additive:
                self.selected_ids.clear()
                self.selected_id = None
        else:
            if toggle:
                if item_id in self.selected_ids:
                    self.selected_ids.remove(item_id)
                    if self.selected_id == item_id:
                        self.selected_id = next(iter(self.selected_ids), None)
                else:
                    self.selected_ids.add(item_id)
                    self.selected_id = item_id
            elif additive:
                self.selected_ids.add(item_id)
                self.selected_id = item_id
            else:
                self.selected_ids = {item_id}
                self.selected_id = item_id

        self._refresh_property_panel()
        self.redraw()

    def select_items_in_rect(self, x1: float, y1: float, x2: float, y2: float, additive: bool = False) -> None:
        left = min(x1, x2)
        right = max(x1, x2)
        top = min(y1, y2)
        bottom = max(y1, y2)

        hits: List[int] = []
        for item in self.project.items:
            if item.kind == 'point':
                px, py = item.x, item.y
                intersects = left <= px <= right and top <= py <= bottom
            else:
                ix1, iy1, ix2, iy2 = item.x, item.y, item.x + item.w, item.y + item.h
                intersects = not (ix2 < left or ix1 > right or iy2 < top or iy1 > bottom)

            if intersects:
                hits.append(item.id)

        if additive:
            self.selected_ids.update(hits)
            if hits:
                self.selected_id = hits[-1]
        else:
            self.selected_ids = set(hits)
            self.selected_id = hits[-1] if hits else None

        self._refresh_property_panel()
        self.redraw()

    def update_selected_properties(self) -> None:
        item = self.get_selected()
        if item is None:
            return

        new_label = self.label_var.get().strip() or item.label
        new_role = item.role
        if item.category == 'nav':
            new_role = self.role_var.get().strip() or 'room'

        if new_label == item.label and new_role == item.role:
            return

        self.push_undo_state()
        item.label = new_label
        item.role = new_role
        self._refresh_property_panel()
        self.redraw()

    def delete_selected(self) -> None:
        if not self.selected_ids:
            return
        self.push_undo_state()
        self.project.items = [item for item in self.project.items if item.id not in self.selected_ids]
        self.selected_ids.clear()
        self.selected_id = None
        self._refresh_property_panel()
        self.redraw()
        self.status('Deleted selected items')

    def find_item_at(self, x: float, y: float) -> Optional[EditorItem]:
        for item in reversed(self.project.items):
            if item.kind == 'rect':
                if item.x <= x <= item.x + item.w and item.y <= y <= item.y + item.h:
                    return item
            elif item.kind == 'circle':
                cx = item.x + item.w / 2.0
                cy = item.y + item.h / 2.0
                r = min(item.w, item.h) / 2.0
                if (x - cx) ** 2 + (y - cy) ** 2 <= r ** 2:
                    return item
            else:
                if abs(x - item.x) <= POINT_HIT_RADIUS and abs(y - item.y) <= POINT_HIT_RADIUS:
                    return item
        return None

    def _handle_at(self, item: EditorItem, x: float, y: float) -> Optional[str]:
        if item.kind == 'point':
            return None
        for name, hx, hy in self._item_handles(item):
            if abs(x - hx) <= HANDLE_SIZE and abs(y - hy) <= HANDLE_SIZE:
                return name
        return None

    def _item_handles(self, item: EditorItem) -> List[Tuple[str, float, float]]:
        if item.kind == 'point':
            return []
        x1, y1, x2, y2 = item.x, item.y, item.x + item.w, item.y + item.h
        return [
            ('nw', x1, y1), ('n', (x1 + x2) / 2, y1), ('ne', x2, y1),
            ('e', x2, (y1 + y2) / 2), ('se', x2, y2), ('s', (x1 + x2) / 2, y2),
            ('sw', x1, y2), ('w', x1, (y1 + y2) / 2),
        ]

    def _apply_grid_settings(self) -> None:
        self.project.grid_size = max(1, int(self.grid_var.get()))
        self.project.snap_distance = max(0, int(self.snap_var.get()))
        self.redraw()

    def _apply_canvas_settings(self) -> None:
        self.project.canvas_width = max(500, int(self.canvas_w_var.get()))
        self.project.canvas_height = max(500, int(self.canvas_h_var.get()))
        self.canvas.config(scrollregion=(0, 0, self.project.canvas_width, self.project.canvas_height))
        self.redraw()
        self.status(f'Canvas set to {self.project.canvas_width} x {self.project.canvas_height}')

    def _snap_value(self, value: float) -> float:
        grid = self.project.grid_size
        return round(value / grid) * grid if grid > 0 else value

    def _snap_item(self, item: EditorItem) -> None:
        item.x = self._snap_value(item.x)
        item.y = self._snap_value(item.y)

        if item.kind == 'point':
            return

        item.w = max(MIN_SIZE, self._snap_value(item.w))
        item.h = max(MIN_SIZE, self._snap_value(item.h))

        snap = float(self.project.snap_distance)
        x1, y1, x2, y2 = item.x, item.y, item.x + item.w, item.y + item.h
        for other in self.project.items:
            if other.id == item.id or other.kind == 'point':
                continue
            ox1, oy1, ox2, oy2 = other.x, other.y, other.x + other.w, other.y + other.h
            for a, b, axis in [
                (x1, ox1, 'x1'), (x1, ox2, 'x1'), (x2, ox1, 'x2'), (x2, ox2, 'x2'),
                (y1, oy1, 'y1'), (y1, oy2, 'y1'), (y2, oy1, 'y2'), (y2, oy2, 'y2'),
            ]:
                if abs(a - b) <= snap:
                    delta = b - a
                    if axis == 'x1':
                        item.x += delta
                    elif axis == 'x2':
                        item.w = max(MIN_SIZE, item.w + delta)
                    elif axis == 'y1':
                        item.y += delta
                    elif axis == 'y2':
                        item.h = max(MIN_SIZE, item.h + delta)
                    x1, y1, x2, y2 = item.x, item.y, item.x + item.w, item.y + item.h

    def _snap_selected_group(self) -> None:
        for item in self.get_selected_items():
            self._snap_item(item)

    def _new_item_for_tool(self, tool: str, x: float, y: float) -> Optional[EditorItem]:
        base_x = self._snap_value(x)
        base_y = self._snap_value(y)

        if tool == 'nav_rect':
            return EditorItem(self.next_id, 'nav', 'rect', base_x, base_y, 160, 120, f'room_{self.next_id}', 'room')
        if tool == 'nav_circle':
            return EditorItem(self.next_id, 'nav', 'circle', base_x, base_y, 140, 140, f'room_{self.next_id}', 'room')
        if tool == 'trigger_rect':
            return EditorItem(self.next_id, 'trigger', 'rect', base_x, base_y, 160, 120, f'trigger_{self.next_id}', '')
        if tool == 'trigger_circle':
            return EditorItem(self.next_id, 'trigger', 'circle', base_x, base_y, 120, 120, f'trigger_{self.next_id}', '')
        if tool == 'object_rect':
            return EditorItem(self.next_id, 'object', 'rect', base_x, base_y, 60, 60, f'object_{self.next_id}', '')
        if tool == 'object_circle':
            return EditorItem(self.next_id, 'object', 'circle', base_x, base_y, 60, 60, f'object_{self.next_id}', '')
        if tool == 'point':
            return EditorItem(self.next_id, 'point', 'point', base_x, base_y, 0, 0, f'point_{self.next_id}', '')
        return None

    # ------------ Canvas pan ------------
    def on_middle_mouse_down(self, event: tk.Event) -> None:
        self.canvas.scan_mark(event.x, event.y)

    def on_middle_mouse_drag(self, event: tk.Event) -> None:
        self.canvas.scan_dragto(event.x, event.y, gain=1)

    # ------------ Canvas events ------------
    def on_mouse_down(self, event: tk.Event) -> None:
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        self.drag_start = (x, y)

        shift = bool(event.state & 0x0001)
        tool = self.tool.get()
        hit = self.find_item_at(x, y)

        self.marquee_start = None
        self.marquee_end = None
        self.group_orig_pos.clear()

        new_item = self._new_item_for_tool(tool, x, y)
        if new_item is not None:
            self.push_undo_state()
            self.project.items.append(new_item)
            self.next_id += 1
            self.select_item(new_item.id)
            if new_item.kind == 'point':
                self.drag_mode = 'move_group'
                self.group_orig_pos[new_item.id] = (new_item.x, new_item.y)
            else:
                self.drag_mode = 'resize_se'
                self.orig_geom = (new_item.x, new_item.y, new_item.w, new_item.h)
            return

        if tool == 'delete':
            if hit:
                if shift:
                    self.select_item(hit.id, toggle=True)
                else:
                    self.select_item(hit.id)
                    self.delete_selected()
            return

        if hit:
            if shift:
                self.select_item(hit.id, toggle=True)
                self.drag_mode = None
                return

            if hit.id not in self.selected_ids:
                self.select_item(hit.id)

            self.push_undo_state()

            if len(self.selected_ids) == 1:
                handle = self._handle_at(hit, x, y)
                if handle:
                    self.orig_geom = (hit.x, hit.y, hit.w, hit.h)
                    self.drag_mode = f'resize_{handle}'
                    return

            self.drag_mode = 'move_group'
            for item in self.get_selected_items():
                self.group_orig_pos[item.id] = (item.x, item.y)
        else:
            if not shift:
                self.selected_ids.clear()
                self.selected_id = None
                self._refresh_property_panel()
            self.drag_mode = 'marquee'
            self.marquee_start = (x, y)
            self.marquee_end = (x, y)
            self.redraw()

    def on_mouse_drag(self, event: tk.Event) -> None:
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        dx = x - self.drag_start[0]
        dy = y - self.drag_start[1]

        if self.drag_mode == 'marquee':
            self.marquee_end = (x, y)
            self.redraw()
            return

        item = self.get_selected()
        if not self.drag_mode:
            return

        if self.drag_mode == 'move_group':
            for moving_item in self.get_selected_items():
                ox, oy = self.group_orig_pos.get(moving_item.id, (moving_item.x, moving_item.y))
                moving_item.x = ox + dx
                moving_item.y = oy + dy
            self.redraw()
            return

        if item is None:
            return

        if self.drag_mode.startswith('resize_') and item.kind != 'point':
            ox, oy, ow, oh = self.orig_geom
            handle = self.drag_mode.replace('resize_', '')
            x1, y1, x2, y2 = ox, oy, ox + ow, oy + oh
            if 'w' in handle:
                x1 = min(x2 - MIN_SIZE, ox + dx)
            if 'e' in handle:
                x2 = max(x1 + MIN_SIZE, ox + ow + dx)
            if 'n' in handle:
                y1 = min(y2 - MIN_SIZE, oy + dy)
            if 's' in handle:
                y2 = max(y1 + MIN_SIZE, oy + oh + dy)
            item.x, item.y = x1, y1
            item.w, item.h = x2 - x1, y2 - y1
            if item.kind == 'circle':
                size = max(MIN_SIZE, max(item.w, item.h))
                if 'w' in handle:
                    item.x = x2 - size
                if 'n' in handle:
                    item.y = y2 - size
                item.w = size
                item.h = size

        self.redraw()

    def on_mouse_up(self, event: tk.Event) -> None:
        shift = bool(event.state & 0x0001)

        if self.drag_mode == 'marquee' and self.marquee_start and self.marquee_end:
            self.select_items_in_rect(
                self.marquee_start[0],
                self.marquee_start[1],
                self.marquee_end[0],
                self.marquee_end[1],
                additive=shift
            )
            self.marquee_start = None
            self.marquee_end = None
            self.drag_mode = None
            return

        if self.drag_mode == 'move_group':
            self._snap_selected_group()
            self.redraw()
            self.drag_mode = None
            return

        item = self.get_selected()
        if item is not None and self.drag_mode and self.drag_mode.startswith('resize_'):
            self._snap_item(item)
            self.redraw()

        self.drag_mode = None

    def on_double_click(self, event: tk.Event) -> None:
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)
        hit = self.find_item_at(x, y)
        if hit:
            self.select_item(hit.id)
            self.root.after(10, lambda: self.root.focus_force())

    # ------------ Drawing ------------
    def redraw(self) -> None:
        self.canvas.delete('all')
        self._draw_grid()

        for item in self.project.items:
            selected = item.id in self.selected_ids
            primary = item.id == self.selected_id

            if item.category == 'nav':
                fill = '#476ea5' if item.role != 'blocked' else '#8f4545'
                outline = '#dbe8ff' if selected else '#b5c7e6'
            elif item.category == 'trigger':
                fill = '#8a6a1f'
                outline = '#ffd27a' if selected else '#f0c060'
            elif item.category == 'object':
                fill = '#3f7a3f'
                outline = '#ccffcc' if selected else '#9fdf9f'
            else:
                fill = ''
                outline = '#fff36a' if selected else '#e5d94a'

            width = 3 if selected else 2
            if primary:
                outline = '#ffffff'
                width = 4

            if item.kind == 'rect':
                self.canvas.create_rectangle(
                    item.x, item.y, item.x + item.w, item.y + item.h,
                    fill=fill, outline=outline, width=width
                )
                self.canvas.create_text(
                    item.x + item.w / 2, item.y + item.h / 2,
                    text=item.label, fill='white', font=('Segoe UI', 11, 'bold')
                )
            elif item.kind == 'circle':
                self.canvas.create_oval(
                    item.x, item.y, item.x + item.w, item.y + item.h,
                    fill=fill, outline=outline, width=width
                )
                self.canvas.create_text(
                    item.x + item.w / 2, item.y + item.h / 2,
                    text=item.label, fill='white', font=('Segoe UI', 11, 'bold')
                )
            else:
                px, py = item.x, item.y
                self.canvas.create_line(px - POINT_DRAW_RADIUS, py, px + POINT_DRAW_RADIUS, py, fill=outline, width=3)
                self.canvas.create_line(px, py - POINT_DRAW_RADIUS, px, py + POINT_DRAW_RADIUS, fill=outline, width=3)
                self.canvas.create_text(
                    px + 12, py - 12,
                    text=item.label, fill='white', anchor='sw', font=('Segoe UI', 10, 'bold')
                )

        if len(self.selected_ids) == 1:
            item = self.get_selected()
            if item is not None and item.kind != 'point':
                for _, hx, hy in self._item_handles(item):
                    self.canvas.create_rectangle(
                        hx - HANDLE_SIZE, hy - HANDLE_SIZE,
                        hx + HANDLE_SIZE, hy + HANDLE_SIZE,
                        fill='#f1f5f9', outline='#111827'
                    )

        if self.drag_mode == 'marquee' and self.marquee_start and self.marquee_end:
            x1, y1 = self.marquee_start
            x2, y2 = self.marquee_end
            self.canvas.create_rectangle(
                x1, y1, x2, y2,
                outline='#ffffff',
                width=2,
                dash=MARQUEE_DASH
            )

    def _draw_grid(self) -> None:
        grid = self.project.grid_size
        if grid <= 0:
            return
        w = self.project.canvas_width
        h = self.project.canvas_height
        color = '#2f343b'
        for x in range(0, w + 1, grid):
            self.canvas.create_line(x, 0, x, h, fill=color)
        for y in range(0, h + 1, grid):
            self.canvas.create_line(0, y, w, y, fill=color)


def main() -> None:
    root = tk.Tk()
    style = ttk.Style(root)
    try:
        style.theme_use('clam')
    except Exception:
        pass
    NavEditorApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()