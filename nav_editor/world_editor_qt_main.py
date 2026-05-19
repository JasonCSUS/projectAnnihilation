from __future__ import annotations

import os
import sys
import copy
from pathlib import Path
from typing import Dict, Optional

from PySide6.QtCore import Qt, QRectF, QPointF, Signal
from PySide6.QtGui import QAction, QColor, QBrush, QPen, QPainter
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QFileDialog,
    QMessageBox,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTabWidget,
    QListWidget,
    QTreeWidget,
    QTreeWidgetItem,
    QSplitter,
    QComboBox,
    QGraphicsView,
    QGraphicsScene,
    QGraphicsRectItem,
    QGraphicsEllipseItem,
    QGraphicsSimpleTextItem,
    QGraphicsItem,
    QSpinBox,
)

from world_editor_model_qt import Project, EditorItem, EntityDefinition, load_project
from world_editor_export_qt import export_all

POINT_DRAW_RADIUS = 8.0
MIN_SHAPE_SIZE = 20.0
HANDLE_SIZE = 10.0
LABEL_HIT_RADIUS = 28.0


class ToolState:
    def __init__(self) -> None:
        self.value = 'select'

    def set(self, value: str) -> None:
        self.value = value


class MapRectItem(QGraphicsRectItem):
    def __init__(self, editor_item: EditorItem) -> None:
        super().__init__(0, 0, editor_item.w, editor_item.h)
        self.editor_item = editor_item
        self.label_items: list[QGraphicsSimpleTextItem] = []
        self.setPos(editor_item.x, editor_item.y)
        self.setFlags(
            QGraphicsItem.ItemIsSelectable |
            QGraphicsItem.ItemSendsGeometryChanges
        )

    def sync_back(self) -> None:
        self.editor_item.x = self.pos().x()
        self.editor_item.y = self.pos().y()
        self.editor_item.w = self.rect().width()
        self.editor_item.h = self.rect().height()

    def itemChange(self, change, value):
        result = super().itemChange(change, value)
        if change == QGraphicsItem.ItemPositionHasChanged:
            self.sync_back()
            if self.scene() and hasattr(self.scene(), 'update_labels_for_item'):
                self.scene().update_labels_for_item(self)
        return result


class MapEllipseItem(QGraphicsEllipseItem):
    def __init__(self, editor_item: EditorItem) -> None:
        super().__init__(0, 0, editor_item.w, editor_item.h)
        self.editor_item = editor_item
        self.label_items: list[QGraphicsSimpleTextItem] = []
        self.setPos(editor_item.x, editor_item.y)
        self.setFlags(
            QGraphicsItem.ItemIsSelectable |
            QGraphicsItem.ItemSendsGeometryChanges
        )

    def sync_back(self) -> None:
        self.editor_item.x = self.pos().x()
        self.editor_item.y = self.pos().y()
        self.editor_item.w = self.rect().width()
        self.editor_item.h = self.rect().height()

    def itemChange(self, change, value):
        result = super().itemChange(change, value)
        if change == QGraphicsItem.ItemPositionHasChanged:
            self.sync_back()
            if self.scene() and hasattr(self.scene(), 'update_labels_for_item'):
                self.scene().update_labels_for_item(self)
        return result


class MapPointItem(QGraphicsItem):
    def __init__(self, editor_item: EditorItem) -> None:
        super().__init__()
        self.editor_item = editor_item
        self.label_items: list[QGraphicsSimpleTextItem] = []
        self.setPos(editor_item.x, editor_item.y)
        self.setFlags(
            QGraphicsItem.ItemIsSelectable |
            QGraphicsItem.ItemSendsGeometryChanges
        )

    def boundingRect(self) -> QRectF:
        return QRectF(
            -POINT_DRAW_RADIUS - 2,
            -POINT_DRAW_RADIUS - 2,
            POINT_DRAW_RADIUS * 2 + 4,
            POINT_DRAW_RADIUS * 2 + 4,
        )

    def paint(self, painter: QPainter, option, widget=None) -> None:
        color = QColor(255, 243, 106) if not self.isSelected() else QColor(255, 255, 255)
        painter.setPen(QPen(color, 3))
        painter.drawLine(QPointF(-POINT_DRAW_RADIUS, 0), QPointF(POINT_DRAW_RADIUS, 0))
        painter.drawLine(QPointF(0, -POINT_DRAW_RADIUS), QPointF(0, POINT_DRAW_RADIUS))

    def sync_back(self) -> None:
        self.editor_item.x = self.pos().x()
        self.editor_item.y = self.pos().y()

    def itemChange(self, change, value):
        result = super().itemChange(change, value)
        if change == QGraphicsItem.ItemPositionHasChanged:
            self.sync_back()
            if self.scene() and hasattr(self.scene(), 'update_labels_for_item'):
                self.scene().update_labels_for_item(self)
        return result


class EditorScene(QGraphicsScene):
    selection_changed = Signal(object)

    def __init__(self, project: Project, tool_state: ToolState, parent=None) -> None:
        super().__init__(parent)
        self.project = project
        self.tool_state = tool_state
        self._is_rebuilding = False

        self.item_map: Dict[int, QGraphicsItem] = {}
        self.label_hit_centers: Dict[int, QPointF] = {}
        self.next_id = max((item.id for item in self.project.items), default=0) + 1
        self.setSceneRect(0, 0, project.canvas_width, project.canvas_height)

        self.creating_item: Optional[EditorItem] = None
        self.create_start: Optional[QPointF] = None

        self.is_resizing = False
        self.resize_item: Optional[QGraphicsItem] = None
        self.resize_handle: Optional[str] = None
        self.resize_orig_rect: Optional[QRectF] = None
        self.resize_orig_pos: Optional[QPointF] = None
        self.resize_press_scene_pos: Optional[QPointF] = None

        self.is_dragging = False
        self.drag_item: Optional[QGraphicsItem] = None
        self.drag_orig_pos: Optional[QPointF] = None
        self.drag_press_scene_pos: Optional[QPointF] = None

        self.selectionChanged.connect(self._emit_selected_item)
        self.rebuild()

    def snap_scene_pos(self, value) -> QPointF:
        p = value if isinstance(value, QPointF) else value.toPointF()
        return QPointF(self._snap_value(p.x()), self._snap_value(p.y()))

    def _snap_value(self, value: float) -> float:
        grid = max(1, self.project.grid_size)
        return round(value / grid) * grid

    def _effective_tool(self, event) -> str:
        base = self.tool_state.value
        if base == 'select' and (event.modifiers() & Qt.ShiftModifier):
            return 'transform'
        return base

    def rebuild(self) -> None:
        self._is_rebuilding = True
        try:
            old_map = self.item_map
            self.item_map = {}
            self.label_hit_centers = {}

            old_map.clear()

            self.clear()
            self._draw_grid()

            draw_order = {'nav': 0, 'trigger': 1, 'object': 2, 'point': 3}
            for item in sorted(self.project.items, key=lambda i: draw_order.get(i.category, 99)):
                gitem = self._create_graphics_item(item)
                self.addItem(gitem)
                self.item_map[item.id] = gitem
                self._apply_style(gitem, item)
                self._create_labels(gitem, item)
        finally:
            self._is_rebuilding = False

    def _draw_grid(self) -> None:
        color = QColor('#2f343b')
        pen = QPen(color, 0)
        grid = max(1, self.project.grid_size)
        w = self.project.canvas_width
        h = self.project.canvas_height

        for x in range(0, w + 1, grid):
            self.addLine(x, 0, x, h, pen)
        for y in range(0, h + 1, grid):
            self.addLine(0, y, w, y, pen)

    def _create_graphics_item(self, item: EditorItem) -> QGraphicsItem:
        if item.kind == 'rect':
            return MapRectItem(item)
        if item.kind == 'circle':
            return MapEllipseItem(item)
        return MapPointItem(item)

    def _apply_style(self, gitem: QGraphicsItem, item: EditorItem) -> None:
        if gitem.scene() is None:
            return

        if item.category == 'nav':
            fill = QColor(70, 110, 165, 90) if item.role != 'blocked' else QColor(150, 60, 60, 110)
            outline = QColor(200, 220, 240, 220)
            z = 10
        elif item.category == 'trigger':
            fill = QColor(190, 140, 40, 55)
            outline = QColor(255, 220, 120, 230)
            z = 20
        elif item.category == 'object':
            fill = QColor(90, 150, 90, 110)
            outline = QColor(170, 240, 170, 230)
            z = 30
        else:
            fill = QColor(0, 0, 0, 0)
            outline = QColor(255, 255, 0, 255)
            z = 40

        if gitem.isSelected():
            outline = QColor(255, 255, 255, 255)

        gitem.setZValue(z)
        if isinstance(gitem, (QGraphicsRectItem, QGraphicsEllipseItem)):
            gitem.setBrush(QBrush(fill))
            gitem.setPen(QPen(outline, 2))
        gitem.update()

    def _create_labels(self, gitem: QGraphicsItem, item: EditorItem) -> None:
        for lbl in getattr(gitem, 'label_items', []):
            try:
                lbl.setParentItem(None)
                if lbl.scene() is self:
                    self.removeItem(lbl)
            except RuntimeError:
                pass

        gitem.label_items = []

        label = QGraphicsSimpleTextItem(item.label)
        label.setBrush(QColor('white'))
        label.setZValue(100)
        label.setParentItem(gitem)
        label.setAcceptedMouseButtons(Qt.NoButton)
        gitem.label_items.append(label)

        if item.kind == 'point':
            label.setPos(12, -22)
        else:
            center = QRectF(0, 0, item.w, item.h).center()
            br = label.boundingRect()
            if item.category == 'trigger':
                label.setPos(center.x() - br.width() / 2, center.y() - br.height() - 10)
            else:
                label.setPos(center.x() - br.width() / 2, center.y() + 10)

        label_scene_poly = label.mapToScene(label.boundingRect())
        self.label_hit_centers[item.id] = label_scene_poly.boundingRect().center()

    def update_labels_for_item(self, gitem: QGraphicsItem) -> None:
        self._create_labels(gitem, gitem.editor_item)
        self.update()
        for view in self.views():
            view.viewport().update()

    def _emit_selected_item(self) -> None:
        if self._is_rebuilding:
            return

        selected = [it for it in self.selectedItems() if hasattr(it, 'editor_item')]

        for gitem in list(self.item_map.values()):
            if hasattr(gitem, 'editor_item'):
                self._apply_style(gitem, gitem.editor_item)

        self.selection_changed.emit(selected[0].editor_item if selected else None)
        self.update()
        for view in self.views():
            view.viewport().update()

    def _resize_handles_for_item(self, gitem: QGraphicsItem) -> dict[str, QRectF]:
        if not isinstance(gitem, (QGraphicsRectItem, QGraphicsEllipseItem)):
            return {}

        r = gitem.sceneBoundingRect()
        cx = r.center().x()
        cy = r.center().y()
        hs = HANDLE_SIZE / 2.0

        pts = {
            'nw': QPointF(r.left(), r.top()),
            'n': QPointF(cx, r.top()),
            'ne': QPointF(r.right(), r.top()),
            'e': QPointF(r.right(), cy),
            'se': QPointF(r.right(), r.bottom()),
            's': QPointF(cx, r.bottom()),
            'sw': QPointF(r.left(), r.bottom()),
            'w': QPointF(r.left(), cy),
        }
        return {name: QRectF(p.x() - hs, p.y() - hs, HANDLE_SIZE, HANDLE_SIZE) for name, p in pts.items()}

    def drawForeground(self, painter: QPainter, rect: QRectF) -> None:
        super().drawForeground(painter, rect)

        selected = [it for it in self.selectedItems() if isinstance(it, (QGraphicsRectItem, QGraphicsEllipseItem))]
        if len(selected) != 1:
            return

        gitem = selected[0]
        painter.setPen(QPen(QColor(17, 24, 39), 1))
        painter.setBrush(QBrush(QColor(241, 245, 249)))

        for handle_rect in self._resize_handles_for_item(gitem).values():
            painter.drawRect(handle_rect)

    def _handle_hit_test(self, scene_pos: QPointF) -> tuple[Optional[QGraphicsItem], Optional[str]]:
        selected = [it for it in self.selectedItems() if isinstance(it, (QGraphicsRectItem, QGraphicsEllipseItem))]
        if len(selected) != 1:
            return None, None

        gitem = selected[0]
        for name, rect in self._resize_handles_for_item(gitem).items():
            if rect.contains(scene_pos):
                return gitem, name
        return None, None

    def _pick_item_under_label(self, scene_pos: QPointF) -> Optional[QGraphicsItem]:
        if self._is_rebuilding or self.creating_item is not None or self.is_resizing or self.is_dragging:
            return None

        selected_items = [it for it in self.selectedItems() if hasattr(it, 'editor_item')]
        selected_id = selected_items[0].editor_item.id if selected_items else None

        best_item_id = None
        best_dist_sq = LABEL_HIT_RADIUS * LABEL_HIT_RADIUS

        for item_id, center in self.label_hit_centers.items():
            if selected_id is not None and item_id == selected_id:
                continue

            dx = scene_pos.x() - center.x()
            dy = scene_pos.y() - center.y()
            dist_sq = dx * dx + dy * dy

            if dist_sq <= best_dist_sq:
                best_dist_sq = dist_sq
                best_item_id = item_id

        if best_item_id is None:
            return None

        return self.item_map.get(best_item_id)

    def _top_editor_item_at(self, scene_pos: QPointF) -> Optional[QGraphicsItem]:
        for it in self.items(scene_pos):
            if hasattr(it, 'editor_item'):
                return it
            parent = it.parentItem()
            if parent is not None and hasattr(parent, 'editor_item'):
                return parent
        return None

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.LeftButton:
            scene_pos = self.snap_scene_pos(event.scenePos())
            tool = self._effective_tool(event)

            if tool == 'transform':
                hit_item, hit_handle = self._handle_hit_test(scene_pos)
                if hit_item is not None and hit_handle is not None:
                    self.is_resizing = True
                    self.resize_item = hit_item
                    self.resize_handle = hit_handle
                    self.resize_orig_rect = hit_item.rect()
                    self.resize_orig_pos = hit_item.pos()
                    self.resize_press_scene_pos = scene_pos
                    event.accept()
                    return

                top_item = self._top_editor_item_at(scene_pos)
                if top_item is not None:
                    if not top_item.isSelected():
                        self.clearSelection()
                        top_item.setSelected(True)

                    self.is_dragging = True
                    self.drag_item = top_item
                    self.drag_orig_pos = top_item.pos()
                    self.drag_press_scene_pos = scene_pos
                    event.accept()
                    return

            if tool == 'select':
                under_label_item = self._pick_item_under_label(scene_pos)
                if under_label_item is not None:
                    self.clearSelection()
                    under_label_item.setSelected(True)
                    event.accept()
                    return

            if tool in ('nav_rect', 'nav_circle', 'trigger_rect', 'trigger_circle'):
                self.creating_item = self._new_item_for_tool(scene_pos)
                self.create_start = scene_pos
                if self.creating_item is not None:
                    self.project.items.append(self.creating_item)
                    self.next_id += 1
                    self.rebuild()
                    if self.creating_item.id in self.item_map:
                        self.item_map[self.creating_item.id].setSelected(True)
                    event.accept()
                    return

            if tool == 'point':
                new_item = self._new_item_for_tool(scene_pos)
                if new_item is not None:
                    self.project.items.append(new_item)
                    self.next_id += 1
                    self.rebuild()
                    if new_item.id in self.item_map:
                        self.item_map[new_item.id].setSelected(True)
                    event.accept()
                    return

        super().mousePressEvent(event)
        self._sync_all_items_back()

    def mouseMoveEvent(self, event) -> None:
        if self.creating_item is not None and self.create_start is not None:
            current = self.snap_scene_pos(event.scenePos())

            x1 = min(self.create_start.x(), current.x())
            y1 = min(self.create_start.y(), current.y())
            x2 = max(self.create_start.x(), current.x())
            y2 = max(self.create_start.y(), current.y())

            w = max(MIN_SHAPE_SIZE, x2 - x1)
            h = max(MIN_SHAPE_SIZE, y2 - y1)

            if self.creating_item.kind == 'circle':
                size = max(w, h)
                w = size
                h = size

            self.creating_item.x = x1
            self.creating_item.y = y1
            self.creating_item.w = w
            self.creating_item.h = h

            self.rebuild()
            if self.creating_item.id in self.item_map:
                self.item_map[self.creating_item.id].setSelected(True)
            event.accept()
            return

        if (
            self.is_resizing and
            self.resize_item is not None and
            self.resize_handle is not None and
            self.resize_press_scene_pos is not None
        ):
            current = self.snap_scene_pos(event.scenePos())

            orig_scene_x = self.resize_orig_pos.x()
            orig_scene_y = self.resize_orig_pos.y()
            orig_w = self.resize_orig_rect.width()
            orig_h = self.resize_orig_rect.height()

            left = orig_scene_x
            top = orig_scene_y
            right = orig_scene_x + orig_w
            bottom = orig_scene_y + orig_h

            if 'w' in self.resize_handle:
                left = min(right - MIN_SHAPE_SIZE, current.x())
            if 'e' in self.resize_handle:
                right = max(left + MIN_SHAPE_SIZE, current.x())
            if 'n' in self.resize_handle:
                top = min(bottom - MIN_SHAPE_SIZE, current.y())
            if 's' in self.resize_handle:
                bottom = max(top + MIN_SHAPE_SIZE, current.y())

            new_w = max(MIN_SHAPE_SIZE, right - left)
            new_h = max(MIN_SHAPE_SIZE, bottom - top)

            if isinstance(self.resize_item, QGraphicsEllipseItem):
                size = max(new_w, new_h)
                if 'w' in self.resize_handle:
                    left = right - size
                else:
                    right = left + size

                if 'n' in self.resize_handle:
                    top = bottom - size
                else:
                    bottom = top + size

                new_w = size
                new_h = size

            self.resize_item.setPos(QPointF(left, top))
            self.resize_item.setRect(0, 0, new_w, new_h)

            if hasattr(self.resize_item, 'sync_back'):
                self.resize_item.sync_back()
            if hasattr(self, 'update_labels_for_item'):
                self.update_labels_for_item(self.resize_item)

            self.update()
            for view in self.views():
                view.viewport().update()

            event.accept()
            return

        if (
            self.is_dragging and
            self.drag_item is not None and
            self.drag_orig_pos is not None and
            self.drag_press_scene_pos is not None
        ):
            current = self.snap_scene_pos(event.scenePos())
            dx = current.x() - self.drag_press_scene_pos.x()
            dy = current.y() - self.drag_press_scene_pos.y()

            new_pos = QPointF(
                self._snap_value(self.drag_orig_pos.x() + dx),
                self._snap_value(self.drag_orig_pos.y() + dy),
            )

            self.drag_item.setPos(new_pos)

            if hasattr(self.drag_item, 'sync_back'):
                self.drag_item.sync_back()
            if hasattr(self, 'update_labels_for_item'):
                self.update_labels_for_item(self.drag_item)

            self.update()
            for view in self.views():
                view.viewport().update()

            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if self.creating_item is not None:
            self.creating_item = None
            self.create_start = None
            self._sync_all_items_back()
            event.accept()
            return

        if self.is_resizing:
            self.is_resizing = False
            self.resize_item = None
            self.resize_handle = None
            self.resize_orig_rect = None
            self.resize_orig_pos = None
            self.resize_press_scene_pos = None
            self._sync_all_items_back()
            self.update()
            for view in self.views():
                view.viewport().update()
            event.accept()
            return

        if self.is_dragging:
            self.is_dragging = False
            self.drag_item = None
            self.drag_orig_pos = None
            self.drag_press_scene_pos = None
            self._sync_all_items_back()
            self.update()
            for view in self.views():
                view.viewport().update()
            event.accept()
            return

        super().mouseReleaseEvent(event)
        self._sync_all_items_back()

    def _sync_all_items_back(self) -> None:
        for gitem in self.item_map.values():
            if hasattr(gitem, 'sync_back'):
                gitem.sync_back()

    def _new_item_for_tool(self, pos: QPointF) -> Optional[EditorItem]:
        base_x = self._snap_value(pos.x())
        base_y = self._snap_value(pos.y())
        tool = self.tool_state.value

        if tool == 'nav_rect':
            return EditorItem(self.next_id, 'nav', 'rect', base_x, base_y, 160, 120, f'nav_{self.next_id}', 'room')
        if tool == 'nav_circle':
            return EditorItem(self.next_id, 'nav', 'circle', base_x, base_y, 140, 140, f'nav_{self.next_id}', 'room')
        if tool == 'trigger_rect':
            return EditorItem(self.next_id, 'trigger', 'rect', base_x, base_y, 160, 120, f'trigger_{self.next_id}', '')
        if tool == 'trigger_circle':
            return EditorItem(self.next_id, 'trigger', 'circle', base_x, base_y, 120, 120, f'trigger_{self.next_id}', '')
        if tool == 'point':
            return EditorItem(self.next_id, 'point', 'point', base_x, base_y, 0, 0, f'point_{self.next_id}', '')
        return None


class EditorView(QGraphicsView):
    def __init__(self, scene: EditorScene) -> None:
        super().__init__(scene)
        self.setRenderHint(QPainter.Antialiasing, True)
        self.setBackgroundBrush(QColor('#1e2227'))
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self._panning = False
        self._last_pan = None

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MiddleButton:
            self._panning = True
            self._last_pan = event.position()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:
        if self._panning and self._last_pan is not None:
            delta = event.position() - self._last_pan
            self._last_pan = event.position()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - int(delta.x()))
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - int(delta.y()))
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MiddleButton:
            self._panning = False
            self._last_pan = None
            self.setCursor(Qt.ArrowCursor)
            event.accept()
            return
        super().mouseReleaseEvent(event)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('World Editor (PySide6)')
        self.resize(1680, 980)

        self.project = Project()
        self.project_path: Optional[str] = None
        self.output_dir = os.path.abspath(Path(__file__).resolve().parent.parent / 'assets')
        self.tool_state = ToolState()

        self._build_ui()
        self._refresh_entity_list()
        self._update_scene_from_project()

    def _on_custom_field_selected(self) -> None:
        item = self.custom_tree.currentItem()
        if item is None:
            return
        self.custom_key.setText(item.text(0))
        self.custom_value.setText(item.text(1))

    def _build_ui(self) -> None:
        toolbar = self.addToolBar('Main')
        toolbar.setMovable(False)
        for text, handler in [
            ('New', self.new_project),
            ('Open', self.open_project),
            ('Save', self.save_project_and_exports),
            ('Export As...', self.export_as),
        ]:
            act = QAction(text, self)
            act.triggered.connect(handler)
            toolbar.addAction(act)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)
        self.map_tab = QWidget()
        self.entities_tab = QWidget()
        self.tabs.addTab(self.map_tab, 'Map')
        self.tabs.addTab(self.entities_tab, 'Entities')

        self._build_map_tab()
        self._build_entities_tab()

        self.status_label = QLabel('Ready')
        self.statusBar().addPermanentWidget(self.status_label)

    def _build_map_tab(self) -> None:
        outer = QHBoxLayout(self.map_tab)
        splitter = QSplitter()
        outer.addWidget(splitter)

        left = QWidget()
        left_form = QFormLayout(left)

        self.tool_combo = QComboBox()
        self.tool_combo.addItems(['select', 'transform', 'nav_rect', 'nav_circle', 'trigger_rect', 'trigger_circle', 'point'])
        self.tool_combo.currentTextChanged.connect(self.tool_state.set)
        left_form.addRow('Tool', self.tool_combo)

        self.grid_spin = QSpinBox()
        self.grid_spin.setRange(1, 128)
        self.grid_spin.setValue(self.project.grid_size)
        self.grid_spin.valueChanged.connect(self._apply_grid)

        self.snap_spin = QSpinBox()
        self.snap_spin.setRange(0, 128)
        self.snap_spin.setValue(self.project.snap_distance)
        self.snap_spin.valueChanged.connect(self._apply_snap)

        self.canvas_w_spin = QSpinBox()
        self.canvas_w_spin.setRange(500, 20000)
        self.canvas_w_spin.setValue(self.project.canvas_width)
        self.canvas_w_spin.valueChanged.connect(self._apply_canvas_size)

        self.canvas_h_spin = QSpinBox()
        self.canvas_h_spin.setRange(500, 20000)
        self.canvas_h_spin.setValue(self.project.canvas_height)
        self.canvas_h_spin.valueChanged.connect(self._apply_canvas_size)

        left_form.addRow('Grid', self.grid_spin)
        left_form.addRow('Snap', self.snap_spin)
        left_form.addRow('Canvas W', self.canvas_w_spin)
        left_form.addRow('Canvas H', self.canvas_h_spin)

        self.label_edit = QLineEdit()
        self.label_edit.editingFinished.connect(self.update_selected_properties)

        self.category_label = QLabel('')
        self.kind_label = QLabel('')

        self.role_combo = QComboBox()
        self.role_combo.addItems(['room', 'corridor', 'blocked'])
        self.role_combo.currentTextChanged.connect(self.update_selected_properties)

        left_form.addRow('Label', self.label_edit)
        left_form.addRow('Category', self.category_label)
        left_form.addRow('Kind', self.kind_label)
        left_form.addRow('Nav Role', self.role_combo)

        hint = QLabel('Shift while on Select = temporary Transform')
        left_form.addRow(hint)

        delete_btn = QPushButton('Delete Selected')
        delete_btn.clicked.connect(self.delete_selected)
        left_form.addRow(delete_btn)

        self.scene = EditorScene(self.project, self.tool_state)
        self.scene.selection_changed.connect(self._on_scene_selection_changed)
        self.view = EditorView(self.scene)

        splitter.addWidget(left)
        splitter.addWidget(self.view)
        splitter.setSizes([300, 1200])

    def _build_entities_tab(self) -> None:
        outer = QHBoxLayout(self.entities_tab)
        splitter = QSplitter()
        outer.addWidget(splitter)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        self.entity_list = QListWidget()
        self.entity_list.currentRowChanged.connect(self._on_entity_selected)
        left_layout.addWidget(self.entity_list)

        btn_row = QHBoxLayout()
        for text, handler in [('Add', self.add_entity), ('Duplicate', self.duplicate_entity), ('Delete', self.delete_entity)]:
            btn = QPushButton(text)
            btn.clicked.connect(handler)
            btn_row.addWidget(btn)
        left_layout.addLayout(btn_row)

        right = QWidget()
        form = QFormLayout(right)

        self.entity_name = QLineEdit()
        self.sprite_path = QLineEdit()

        form.addRow('Entity Name', self.entity_name)
        form.addRow('Sprite File Name', self.sprite_path)

        self.custom_tree = QTreeWidget()
        self.custom_tree.setHeaderLabels(['Field', 'Value'])
        self.custom_tree.itemSelectionChanged.connect(self._on_custom_field_selected)
        form.addRow(QLabel('Custom Fields'))
        form.addRow(self.custom_tree)

        custom_row = QHBoxLayout()
        self.custom_key = QLineEdit()
        self.custom_value = QLineEdit()
        add_custom = QPushButton('Add / Update Custom')
        add_custom.clicked.connect(self.upsert_entity_custom)
        remove_custom = QPushButton('Remove Selected')
        remove_custom.clicked.connect(self.remove_entity_custom)
        custom_row.addWidget(self.custom_key)
        custom_row.addWidget(self.custom_value)
        custom_row.addWidget(add_custom)
        custom_row.addWidget(remove_custom)

        holder = QWidget()
        holder.setLayout(custom_row)
        form.addRow(holder)

        save_btn = QPushButton('Save Entity')
        save_btn.clicked.connect(self.save_active_entity)
        form.addRow(save_btn)

        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setSizes([320, 1100])

    def status(self, message: str) -> None:
        self.status_label.setText(message)

    def _on_scene_selection_changed(self, editor_item: Optional[EditorItem]) -> None:
        if editor_item is None:
            self.label_edit.setText('')
            self.category_label.setText('')
            self.kind_label.setText('')
            self.role_combo.setEnabled(False)
            return

        self.label_edit.setText(editor_item.label)
        self.category_label.setText(editor_item.category)
        self.kind_label.setText(editor_item.kind)
        self.role_combo.setEnabled(editor_item.category == 'nav')
        if editor_item.category == 'nav':
            self.role_combo.setCurrentText(editor_item.role)

    def _current_selected_item(self) -> Optional[EditorItem]:
        selected = [it for it in self.scene.selectedItems() if hasattr(it, 'editor_item')]
        return selected[0].editor_item if selected else None

    def update_selected_properties(self) -> None:
        item = self._current_selected_item()
        if item is None:
            return

        item.label = self.label_edit.text().strip() or item.label
        if item.category == 'nav':
            item.role = self.role_combo.currentText()

        self.scene.rebuild()
        if item.id in self.scene.item_map:
            self.scene.item_map[item.id].setSelected(True)

    def delete_selected(self) -> None:
        selected_ids = {it.editor_item.id for it in self.scene.selectedItems() if hasattr(it, 'editor_item')}
        if not selected_ids:
            return

        self.project.items = [item for item in self.project.items if item.id not in selected_ids]
        self.scene.rebuild()

    def _apply_grid(self, value: int) -> None:
        self.project.grid_size = value
        self.scene.rebuild()

    def _apply_snap(self, value: int) -> None:
        self.project.snap_distance = value

    def _apply_canvas_size(self) -> None:
        self.project.canvas_width = self.canvas_w_spin.value()
        self.project.canvas_height = self.canvas_h_spin.value()
        self.scene.setSceneRect(0, 0, self.project.canvas_width, self.project.canvas_height)
        self.scene.rebuild()

    def _refresh_entity_list(self) -> None:
        self.entity_list.clear()
        self.entity_list.addItems([entity.name for entity in self.project.entities])

    def _on_entity_selected(self, row: int) -> None:
        if row < 0 or row >= len(self.project.entities):
            return

        entity = self.project.entities[row]
        self.entity_name.setText(entity.name)
        self.sprite_path.setText(entity.sprite_path)

        self.custom_tree.clear()
        for key, value in entity.custom_fields.items():
            self.custom_tree.addTopLevelItem(QTreeWidgetItem([key, value]))

    def add_entity(self) -> None:
        base = 'entity_1'
        existing = {e.name for e in self.project.entities}
        name = base
        idx = 1
        while name in existing:
            idx += 1
            name = f'entity_{idx}'

        self.project.entities.append(EntityDefinition(name=name))
        self._refresh_entity_list()
        self.entity_list.setCurrentRow(len(self.project.entities) - 1)

    def duplicate_entity(self) -> None:
        row = self.entity_list.currentRow()
        if row < 0 or row >= len(self.project.entities):
            return

        entity = copy.deepcopy(self.project.entities[row])
        entity.name = f'{entity.name}_copy'
        self.project.entities.append(entity)
        self._refresh_entity_list()

    def delete_entity(self) -> None:
        row = self.entity_list.currentRow()
        if row < 0 or row >= len(self.project.entities):
            return

        del self.project.entities[row]
        self._refresh_entity_list()
        self.custom_tree.clear()

    def upsert_entity_custom(self) -> None:
        key = self.custom_key.text().strip()
        value = self.custom_value.text().strip()
        if not key:
            return

        for i in range(self.custom_tree.topLevelItemCount()):
            item = self.custom_tree.topLevelItem(i)
            if item.text(0) == key:
                item.setText(1, value)
                self.custom_tree.setCurrentItem(item)
                return

        item = QTreeWidgetItem([key, value])
        self.custom_tree.addTopLevelItem(item)
        self.custom_tree.setCurrentItem(item)

    def remove_entity_custom(self) -> None:
        item = self.custom_tree.currentItem()
        if item is None:
            return

        idx = self.custom_tree.indexOfTopLevelItem(item)
        self.custom_tree.takeTopLevelItem(idx)
        self.custom_key.clear()
        self.custom_value.clear()

    def save_active_entity(self) -> None:
        row = self.entity_list.currentRow()
        if row < 0 or row >= len(self.project.entities):
            return

        entity = self.project.entities[row]
        entity.name = self.entity_name.text().strip() or entity.name
        entity.sprite_path = self.sprite_path.text().strip()
        entity.custom_fields = {
            self.custom_tree.topLevelItem(i).text(0): self.custom_tree.topLevelItem(i).text(1)
            for i in range(self.custom_tree.topLevelItemCount())
        }

        self._refresh_entity_list()
        self.entity_list.setCurrentRow(row)

    def new_project(self) -> None:
        self.project = Project()
        self.project_path = None
        self.grid_spin.setValue(self.project.grid_size)
        self.snap_spin.setValue(self.project.snap_distance)
        self.canvas_w_spin.setValue(self.project.canvas_width)
        self.canvas_h_spin.setValue(self.project.canvas_height)
        self._refresh_entity_list()
        self._update_scene_from_project()
        self.status('New project')

    def open_project(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, 'Open project', '', 'JSON project (*.json)')
        if not path:
            return

        try:
            self.project = load_project(path)
            self.project_path = path
            self.grid_spin.setValue(self.project.grid_size)
            self.snap_spin.setValue(self.project.snap_distance)
            self.canvas_w_spin.setValue(self.project.canvas_width)
            self.canvas_h_spin.setValue(self.project.canvas_height)
            self._refresh_entity_list()
            self._update_scene_from_project()
            self.status(f'Opened {os.path.basename(path)}')
        except Exception as exc:
            QMessageBox.critical(self, 'Open failed', str(exc))

    def save_project_and_exports(self) -> None:
        self.save_active_entity()
        if not self.project_path:
            path, _ = QFileDialog.getSaveFileName(self, 'Save project', 'world_project.json', 'JSON project (*.json)')
            if not path:
                return
            self.project_path = path

        try:
            result = export_all(self.project, self.output_dir, self.project_path)
            self.status(
                f'Saved project + exports: {result.nav_json.name}, {result.entities_json.name}, {result.nav_bin.name}'
            )
        except Exception as exc:
            QMessageBox.critical(self, 'Save failed', str(exc))

    def export_as(self) -> None:
        folder = QFileDialog.getExistingDirectory(self, 'Select export folder', self.output_dir)
        if not folder:
            return
        self.output_dir = folder
        self.save_project_and_exports()

    def _update_scene_from_project(self) -> None:
        self.scene.project = self.project
        self.scene.next_id = max((item.id for item in self.project.items), default=0) + 1
        self.scene.setSceneRect(0, 0, self.project.canvas_width, self.project.canvas_height)
        self.scene.rebuild()


def main() -> None:
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()