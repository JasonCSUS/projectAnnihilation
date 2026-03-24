from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, Any, List, Tuple, Literal
import json

ItemCategory = Literal['nav', 'trigger', 'object', 'point']
ItemKind = Literal['rect', 'circle', 'point']


@dataclass
class EditorItem:
    id: int
    category: ItemCategory
    kind: ItemKind
    x: float
    y: float
    w: float
    h: float
    label: str
    role: str = 'room'  # only meaningful for category == 'nav'

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> 'EditorItem':
        # Backward compatibility with older "Shape" files.
        category = str(data.get('category', 'nav'))
        kind = str(data.get('kind', 'rect'))

        # Old files may store a point as a shape later; default safely.
        if kind not in ('rect', 'circle', 'point'):
            kind = 'rect'
        if category not in ('nav', 'trigger', 'object', 'point'):
            category = 'nav'

        return EditorItem(
            id=int(data['id']),
            category=category,
            kind=kind,
            x=float(data.get('x', 0.0)),
            y=float(data.get('y', 0.0)),
            w=float(data.get('w', 0.0)),
            h=float(data.get('h', 0.0)),
            label=str(data.get('label', f"item_{data['id']}")),
            role=str(data.get('role', 'room')),
        )


@dataclass
class Project:
    canvas_width: int = 4000
    canvas_height: int = 4000
    grid_size: int = 20
    snap_distance: int = 16
    simplify_tolerance: float = 6.0
    circle_resolution: int = 32
    items: List[EditorItem] | None = None

    def __post_init__(self) -> None:
        if self.items is None:
            self.items = []

    # Backward-compat alias for older app/export code.
    @property
    def shapes(self) -> List[EditorItem]:
        return self.items

    @shapes.setter
    def shapes(self, value: List[EditorItem]) -> None:
        self.items = value

    def to_dict(self) -> Dict[str, Any]:
        return {
            'canvas_width': self.canvas_width,
            'canvas_height': self.canvas_height,
            'grid_size': self.grid_size,
            'snap_distance': self.snap_distance,
            'simplify_tolerance': self.simplify_tolerance,
            'circle_resolution': self.circle_resolution,
            'items': [item.to_dict() for item in self.items],
        }

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> 'Project':
        raw_items = data.get('items')
        if raw_items is None:
            raw_items = data.get('shapes', [])

        return Project(
            canvas_width=int(data.get('canvas_width', 4000)),
            canvas_height=int(data.get('canvas_height', 4000)),
            grid_size=int(data.get('grid_size', 20)),
            snap_distance=int(data.get('snap_distance', 16)),
            simplify_tolerance=float(data.get('simplify_tolerance', 6.0)),
            circle_resolution=int(data.get('circle_resolution', 32)),
            items=[EditorItem.from_dict(item) for item in raw_items],
        )


def save_project(path: str, project: Project) -> None:
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(project.to_dict(), f, indent=2)


def load_project(path: str) -> Project:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    return Project.from_dict(data)


def item_bbox(item: EditorItem) -> Tuple[float, float, float, float]:
    return item.x, item.y, item.x + item.w, item.y + item.h