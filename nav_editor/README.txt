Graybox Nav Editor
==================

Files:
- nav_editor_app.py   : Tkinter editor UI
- nav_editor_model.py : project file model and serialization
- nav_export.py       : export pipeline using shapely + Pillow

What it does:
- Opens a windowed editor
- Lets you add rectangles and circles
- Move, resize, delete, relabel, and change role (room/corridor/blocked)
- Grid snapping and nearby edge snapping
- Ctrl+S / Save exports:
  - map_project.json      (editable source project)
  - navmesh.json          (readable reference export)
  - navmesh_polygons.nav  (binary runtime export for C++)
  - map_preview.png       (debug preview)

Default export folder:
- One folder up from the editor script, then into /assets

Example:
- if the editor lives in project/tools/nav_editor/
- exports go to project/tools/assets/

Dependencies used:
- tkinter
- shapely
- Pillow (PIL)

Notes:
- navmesh_polygons.nav is written in the format expected by your C++ NavMesh::LoadFromFile:
  [int polygon_count]
  repeated:
    [int vertex_count]
    [int x][int y] * vertex_count
    [int neighbor_count]
    [int neighbor_index] * neighbor_count
- navmesh.json is exported alongside it for reference/tooling.
