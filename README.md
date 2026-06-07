# fleet-helm

> **Steer the fleet. Bearings, formations, collision avoidance.**

[![crates.io](https://img.shields.io/crates/v/fleet-helm.svg)](https://crates.io/crates/fleet-helm)
[![docs.rs](https://docs.rs/fleet-helm/badge.svg)](https://docs.rs/fleet-helm)
[![license](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![tests](https://img.shields.io/badge/tests-14-passing-green.svg)]()

Coordinates multiple agents into coherent fleet movements with collision avoidance, formation flying, and waypoint navigation.

---

## Why This Exists

When agents work together, they need to **move together** — without crashing into each other. Existing multi-agent frameworks either ignore spatial coordination or require heavy simulation environments (ROS, Gazebo). `fleet-helm` provides lightweight primitives for:

- **Bearing**: Direction + magnitude in 2D space
- **Formations**: Line, Grid, Circle, V-shape, Swarm patterns
- **Collision Avoidance**: Detect risks and generate avoidance steering
- **Waypoint Navigation**: Plan routes through named checkpoints

Zero dependencies. Works on embedded. Suitable for real-time coordination of 10-1000 agents.

---

## Architecture

```
    ┌─────────────────────────────────────────────┐
    │              Fleet Navigator                 │
    │  waypoint₁ → waypoint₂ → ... → waypointₙ   │
    └──────────────────┬──────────────────────────┘
                       │ route plan
    ┌──────────────────▼──────────────────────────┐
    │            Formation Engine                  │
    │  Line │ Grid │ Circle │ V-Shape │ Swarm     │
    └──────────────────┬──────────────────────────┘
                       │ target positions
    ┌──────────────────▼──────────────────────────┐
    │         Collision Avoidance                  │
    │  detect_risks → avoid → steer_agents         │
    └──────────────────┬──────────────────────────┘
                       │ HelmDecisions
    ┌──────────────────▼──────────────────────────┐
    │             Agent Fleet                      │
    │  agent₁  agent₂  agent₃  ...  agentₙ        │
    └─────────────────────────────────────────────┘
```

---

## Installation

```toml
[dependencies]
fleet-helm = "0.1.0"
```

---

## Quick Start

```rust
use fleet_helm::{Bearing, AgentState, Formation, FleetNavigator, Waypoint};

// Create a fleet of agents
let agents = vec![
    AgentState::new(0, 0.0, 0.0),
    AgentState::new(1, 5.0, 0.0),
    AgentState::new(2, 10.0, 0.0),
];

// Arrange into a circle formation
let positions = Formation::Circle.positions(3, 5.0, 5.0, 3.0);

// Navigate waypoints
let mut nav = FleetNavigator::new();
nav.add_waypoint(Waypoint::new("alpha", 0.0, 0.0));
nav.add_waypoint(Waypoint::new("bravo", 10.0, 10.0));
let route = nav.plan_route();
println!("Total distance: {:.1}", nav.total_distance());
```

---

## Usage Examples

### Example 1: Collision Detection + Avoidance

```rust
use fleet_helm::{AgentState, detect_collisions, avoid_collisions};

let agents = vec![
    AgentState::new(0, 0.0, 0.0).with_radius(1.0),
    AgentState::new(1, 2.5, 0.0).with_radius(1.0),  // close!
];

// Detect collision risks
let risks = detect_collisions(&agents, 5.0);
assert_eq!(risks.len(), 1);

// Generate avoidance steering
let corrections = avoid_collisions(&agents, &risks);
// Agent 0 pushed left, Agent 1 pushed right
```

### Example 2: Formation Patterns

```rust
use fleet_helm::Formation;

// Line formation (3 agents, 2-unit spacing)
let line = Formation::Line.positions(3, 0.0, 0.0, 2.0);
assert_eq!(line.len(), 3);

// Grid formation (9 agents, 3×3)
let grid = Formation::Grid.positions(9, 0.0, 0.0, 1.0);

// V-shape (fighter formation)
let v = Formation::VShape.positions(6, 0.0, 0.0, 2.0);

// Swarm (organic, non-uniform)
let swarm = Formation::Swarm.positions(20, 0.0, 0.0, 5.0);
```

### Example 3: Waypoint Navigation

```rust
use fleet_helm::{FleetNavigator, Waypoint};

let mut nav = FleetNavigator::new();
nav.add_waypoint(Waypoint::new("start", 0.0, 0.0));
nav.add_waypoint(Waypoint::new("mid", 3.0, 4.0));
nav.add_waypoint(Waypoint::new("end", 3.0, 8.0));

let route = nav.plan_route();
// [("start", 0.0), ("mid", 5.0), ("end", 9.0)]
assert_eq!(route.len(), 3);
```

---

## API Reference

| Type | Description |
|------|-------------|
| `Bearing` | 2D direction + magnitude (normalize, scale, add, sub) |
| `AgentState` | Agent position, velocity, collision radius |
| `HelmDecision` | Steering decision with bearing, speed, priority |
| `Waypoint` | Named checkpoint with position and optional ETA |
| `Formation` | Line, Grid, Circle, VShape, Swarm patterns |
| `FleetNavigator` | Waypoint route planner |
| `detect_collisions` | O(n²) collision risk detection |
| `avoid_collisions` | Generate avoidance bearings |

---

## Performance

```
test result: ok. 14 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

- Collision detection: O(n²) — suitable for fleets up to ~1000 agents
- Formation generation: O(n) for all patterns
- Route planning: O(w) where w = number of waypoints

---

## License

MIT © [SuperInstance](https://github.com/SuperInstance)

---

*Part of the [Exocortex](https://github.com/SuperInstance/exocortex) project.*
