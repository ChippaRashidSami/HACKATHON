# Physics Tuning Examples

This directory contains examples of different physics configurations for Gazebo simulation:

## World Files

- `earth-gravity.world` - Standard Earth gravity (9.8 m/s²)
- `moon-gravity.world` - Lunar gravity (1.62 m/s²)
- `zero-g.world` - Zero gravity environment
- `high-friction.world` - Surfaces with high friction coefficients
- `low-bounce.world` - Surfaces with low restitution (low bounce)

## Configuration Parameters

### Gravity Tuning
- Earth: `<gravity>0 0 -9.8</gravity>`
- Moon: `<gravity>0 0 -1.62</gravity>`
- Mars: `<gravity>0 0 -3.71</gravity>`
- Zero-G: `<gravity>0 0 0</gravity>`

### Friction Tuning
- High friction: `<mu>100.0</mu>`
- Low friction: `<mu>0.01</mu>`

### Restitution Tuning
- Bouncy: `<restitution>0.8</restitution>`
- Non-bouncy: `<restitution>0.1</restitution>`

These examples can be used to simulate different environments and physical conditions for your humanoid robot.