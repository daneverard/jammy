# Quickstart Guide - Traffic Simulator

## Getting Started in 2 Minutes

### Step 1: Draw Your First Road

1. Open `index.html` in your web browser
2. Click the **"Draw Road"** button (or press `D`)
3. Click and drag on the canvas to draw a road segment
4. Release to create the road

**Tip**: Draw multiple roads connecting at endpoints to create junctions.

### Step 2: Place Cars

1. In the right sidebar, set the number of cars (default: 50)
2. Click **"Place Cars"**
3. Cars will spawn randomly on your road network

### Step 3: Run Simulation

1. Click **"Start"** button
2. Watch cars navigate your network!

### Basic Interactions

**Drawing**:
- Select **Draw Road** tool
- Click and drag to create a road
- Endpoints near each other (within 10px) automatically snap into junctions

**Editing**:
- Select **Edit** tool (or press `E`)
- Click on a road to select it
- Drag the vertices (small points) to reshape the road

**Selecting**:
- Use **Select** tool (or press `S`)
- Click on roads or cars to select them
- Delete selected items with `Delete` key

**Navigation**:
- **Zoom**: Scroll mouse wheel or use `+`/`-` keys
- **Pan**: Hold `Space` and drag, or middle-click and drag

### Creating Common Patterns

**Straight Road**:
- Draw a single line from point A to point B

**T-Junction**:
- Draw three roads connecting at one point
- Endpoints will automatically snap to create the junction

**Loop/Network**:
- Draw multiple roads with connecting endpoints
- Cars will randomly choose routes at junctions

**Dead End**:
- Draw a road with one endpoint not connected to anything
- Cars reaching it will turn around automatically

### Tips & Tricks

1. **Use the Grid**: Toggle the grid for straighter roads
2. **Snap Radius**: Adjust in settings if endpoints aren't connecting
3. **Multiple Cars**: Add more cars with the "+ Add Car" button
4. **Speed Control**: Use simulation speed dropdown (0.25? to 4?)
5. **Metrics**: Watch the sidebar for live statistics
6. **Save Your Work**: Click "Save Scenario" to export your network
7. **Load Examples**: Try the example scenarios in the `scenarios/` folder

### Keyboard Shortcuts Reference

| Key | Action |
|-----|--------|
| `S` | Select tool |
| `D` | Draw Road tool |
| `E` | Edit tool |
| `Del` | Delete selected |
| `Z` | Undo |
| `Y` | Redo |
| `Space` | Pan (hold and drag) |
| `+` / `-` | Zoom in/out |
| `Mouse Wheel` | Zoom at cursor |

### Troubleshooting

**Cars not moving?**
- Make sure you clicked "Start"
- Check that cars are actually on roads

**Can't edit roads?**
- Make sure simulation is reset (no cars on roads)
- Roads with active cars cannot be edited

**Endpoints not connecting?**
- Increase "Snap Radius" in settings
- Draw endpoints closer together

**Performance issues?**
- Reduce number of cars
- System will auto-enable performance mode if FPS drops

### Next Steps

- Explore the example scenarios
- Try creating complex networks with loops
- Experiment with different numbers of cars
- Save your favorite configurations

Happy simulating! ??