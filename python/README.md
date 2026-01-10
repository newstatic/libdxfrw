# pydxfrw

Python bindings for libdxfrw - a library to read and write DXF/DWG files.

## Features

- Read DWG files (AutoCAD R13 to R2018)
- Read DXF files (ASCII and binary)
- Access to all common entity types:
  - Lines, Circles, Arcs
  - Polylines (LWPolyline)
  - Text and MText
  - Blocks and Inserts
  - Dimensions
  - Hatches
  - And more...

## Installation

### Prerequisites

- Python 3.7+
- CMake 3.14+
- C++14 compiler
- pybind11

```bash
# Install pybind11
pip install pybind11

# Build and install
cd python
pip install .
```

## Usage

```python
import pydxfrw

# Read a DWG file
data = pydxfrw.read_dwg("drawing.dwg")

# Read a DXF file
data = pydxfrw.read_dxf("drawing.dxf")

# Auto-detect by extension
data = pydxfrw.read("drawing.dwg")

# Check if read was successful
if data.valid:
    print(f"Version: {data.version}")
    print(f"Total entities: {data.total_entities}")
    print(f"Total layers: {data.total_layers}")
    print(f"Total blocks: {data.total_blocks}")

    # Access layers
    for layer in data.layers:
        print(f"Layer: {layer.name}, Color: {layer.color}")

    # Access text entities
    for text in data.texts:
        print(f"Text: {text.content} at ({text.position.x}, {text.position.y})")

    # Access lines
    for line in data.lines:
        print(f"Line from ({line.start.x}, {line.start.y}) to ({line.end.x}, {line.end.y})")

    # Access circles
    for circle in data.circles:
        print(f"Circle at ({circle.center.x}, {circle.center.y}) radius {circle.radius}")

    # Access polylines
    for poly in data.lwpolylines:
        print(f"Polyline with {len(poly.vertices)} vertices, closed={poly.closed}")

    # Access block inserts
    for insert in data.inserts:
        print(f"Insert block '{insert.block_name}' at ({insert.position.x}, {insert.position.y})")
else:
    print(f"Error: {data.error}")
```

## API Reference

### Functions

- `read_dwg(filename: str) -> DxfData` - Read a DWG file
- `read_dxf(filename: str) -> DxfData` - Read a DXF file
- `read(filename: str) -> DxfData` - Auto-detect format by extension

### Classes

#### DxfData
Main container for all drawing data.

Attributes:
- `filename: str` - Source filename
- `version: str` - AutoCAD version (e.g., "R2010", "R2018")
- `valid: bool` - True if file was read successfully
- `error: str` - Error message if valid is False
- `layers: List[Layer]` - All layers
- `texts: List[Text]` - TEXT entities
- `mtexts: List[MText]` - MTEXT entities
- `lines: List[Line]` - LINE entities
- `circles: List[Circle]` - CIRCLE entities
- `arcs: List[Arc]` - ARC entities
- `lwpolylines: List[LWPolyline]` - LWPOLYLINE entities
- `inserts: List[Insert]` - INSERT (block reference) entities
- `blocks: List[Block]` - Block definitions
- `dimensions: List[Dimension]` - Dimension entities
- `hatches: List[Hatch]` - Hatch entities
- `total_entities: int` - Total entity count
- `total_layers: int` - Total layer count
- `total_blocks: int` - Total block count

#### Coord
3D coordinate.

Attributes:
- `x: float`
- `y: float`
- `z: float`

#### Layer
Layer definition.

Attributes:
- `name: str`
- `color: int`
- `is_on: bool`
- `line_type: str`

#### Text
TEXT entity.

Attributes:
- `content: str`
- `position: Coord`
- `height: float`
- `angle: float`
- `layer: str`
- `style: str`

#### Line
LINE entity.

Attributes:
- `start: Coord`
- `end: Coord`
- `layer: str`

#### Circle
CIRCLE entity.

Attributes:
- `center: Coord`
- `radius: float`
- `layer: str`

#### LWPolyline
LWPOLYLINE entity.

Attributes:
- `vertices: List[PolylineVertex]`
- `closed: bool`
- `layer: str`
- `elevation: float`

## License

This project is licensed under the GPL v2 license, same as libdxfrw.
