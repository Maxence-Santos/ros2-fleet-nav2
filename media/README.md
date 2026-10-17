# Demo media

Add a portfolio-ready capture here:

| File | Purpose |
|------|---------|
| `demo.gif` or `demo.mp4` | 20–40 s clip of the three-robot demo |

## Recommended capture (host)

```bash
# Terminal 1
./start_fleet_demo.sh --gui

# Then record Gazebo + RViz with your preferred tool, e.g.:
# - GNOME Screenshot / SimpleScreenRecorder
# - OBS Studio
# - wf-recorder / peek (GIF)
```

**Shot list:**

1. Gazebo: blue, orange, and green robots, plus the three walking people.
2. RViz Publish Point: click one robot (within 0.7 m), then click its goal. The other two do not move.
3. The selected robot follows the planned line and passes another robot or a walker.

Keep the width at or below 1280 px so the clip embeds in the GitHub README.
