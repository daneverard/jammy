# Traffic Simulator - MVP

A browser-based traffic simulator where users can draw roads, spawn cars as autonomous agents, and watch them drive with human-like limitations (reaction time, imperfect perception), taking random turns at junctions and turning around at dead ends.

## Features

### Road Network
- **Draw Roads**: Click-drag on canvas to create road segments
- **Edit Roads**: Select and move vertices to adjust geometry
- **Delete Roads**: Remove segments with delete tool or key
- **Junction Creation**: Endpoints within snap radius automatically form junctions
- **One-way/Two-way**: Mark segments as one-way or two-way
- **Speed Limits**: Set per-segment speed limits (optional)

### Simulation
- **Car Spawning**: Place cars on the network with customizable count
- **Human-like Driving**: 
  - Reaction time (0.4-1.8s per car)
  - Perception noise (?5%)
  - Desired headway (1.5s default)
  - Acceleration/braking limits
  - Individual top speed variability (30-80 km/h)
- **Junction Behavior**: Random routing at intersections
- **Dead-end Handling**: Cars turn around smoothly at dead ends
- **Collision Avoidance**: Maintains safe following distances

### UI & Controls
- **Canvas Tools**: Select, Draw, Edit, Delete
- **Pan & Zoom**: Mouse wheel to zoom, Space + drag to pan
- **Undo/Redo**: Z/Y or Ctrl+Z/Ctrl+Y
- **Simulation Controls**: Start, Pause, Reset, Speed (0.25? - 4?)
- **Metrics**: Live stats for cars, speeds, delays
- **Save/Load**: Export and import scenario files

### Performance
- **Target**: 60 FPS for networks up to ~200 segments and ~300 cars
- **Auto Performance Mode**: Simplified rendering if FPS drops below 30

## Quick Start

1. **Open `index.html`** in a modern web browser
2. **Draw a Road Network**:
   - Select "Draw Road" tool (or press `D`)
   - Click and drag on the canvas to create road segments
   - Endpoints within 10px snap together to form junctions
3. **Place Cars**:
   - Enter desired car count (default: 50)
   - Click "Place Cars"
4. **Start Simulation**:
   - Click "Start"
   - Watch cars navigate the network, turn at junctions, and handle dead ends

## Keyboard Shortcuts

- `S` - Select tool
- `D` - Draw Road tool
- `E` - Edit tool
- `Del` / `Backspace` - Delete selected
- `Z` / `Ctrl+Z` - Undo
- `Y` / `Ctrl+Y` - Redo
- `Space` - Pan (hold and drag)
- `+` / `-` - Zoom in/out
- `Mouse Wheel` - Zoom at cursor position

## Example Scenarios

Three example scenarios are included in the `scenarios/` directory:

1. **straight-road-culdesac.json** - A straight road with a cul-de-sac
2. **simple-4way-loop.json** - A simple 4-way junction forming a loop
3. **neighborhood-mixed.json** - A small neighborhood with mixed one-way/two-way roads

To load a scenario:
1. Click "Load Scenario"
2. Select a JSON file from the `scenarios/` directory

## Technical Details

### Car Dynamics Parameters

- **Desired Headway**: 1.5s (range 0.8-2.5s)
- **Min Gap**: 2.0m
- **Acceleration Limit**: 1.6 m/s?
- **Deceleration Limit**: 2.5 m/s?
- **Emergency Braking**: 6.0 m/s?
- **Reaction Time**: Normal(?=0.9s, ?=0.25s), clamped [0.4s, 1.8s]
- **Top Speed**: Normal(?=50 km/h, ?=8), clamped [30 km/h, 80 km/h]
- **Perception Noise**: ?5%

### File Format

Scenarios are saved as JSON with the following structure:

```json
{
  "network": {
    "segments": [...],
    "version": "1.0"
  },
  "settings": {
    "desiredHeadway": 1.5,
    "globalSpeedLimit": 50,
    "snapRadius": 10
  },
  "version": "1.0",
  "timestamp": "..."
}
```

## Browser Compatibility

Works in latest versions of:
- Chrome/Edge
- Firefox
- Safari

Mobile: View-only recommended; tablet editing allowed.

## Limitations (MVP)

- Single-lane modeling (no overtaking)
- No traffic lights or priority rules
- No pedestrians or public transport
- Simplified physics
- Desktop-focused editing

## Future Enhancements

Potential future features:
- Multi-lane roads with overtaking
- Traffic lights and stop signs
- Roundabouts
- Vehicle classes (trucks, buses)
- Data import (GeoJSON)
- Cloud save and collaboration

## License

See LICENSE file for details.