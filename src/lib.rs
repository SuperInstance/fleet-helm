//! # fleet-helm — Fleet Coordination and Steering
//!
//! Coordinates multiple agents into coherent fleet movements with
//! collision avoidance, formation flying, and waypoint navigation.

use std::collections::HashMap;

// ─── Bearing ─────────────────────────────────────────────────────────────────

/// A 2D bearing (direction + magnitude).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bearing {
    pub dx: f64,
    pub dy: f64,
}

impl Bearing {
    pub fn new(dx: f64, dy: f64) -> Self {
        Self { dx, dy }
    }

    pub fn zero() -> Self {
        Self { dx: 0.0, dy: 0.0 }
    }

    pub fn magnitude(&self) -> f64 {
        (self.dx * self.dx + self.dy * self.dy).sqrt()
    }

    pub fn normalized(&self) -> Self {
        let m = self.magnitude();
        if m < 1e-10 { Self::zero() } else { Self { dx: self.dx / m, dy: self.dy / m } }
    }

    pub fn angle(&self) -> f64 {
        self.dy.atan2(self.dx)
    }

    /// Scale the bearing by a factor.
    pub fn scale(&self, factor: f64) -> Self {
        Self { dx: self.dx * factor, dy: self.dy * factor }
    }

    /// Add two bearings.
    pub fn add(&self, other: &Bearing) -> Self {
        Self { dx: self.dx + other.dx, dy: self.dy + other.dy }
    }

    /// Subtract two bearings.
    pub fn sub(&self, other: &Bearing) -> Self {
        Self { dx: self.dx - other.dx, dy: self.dy - other.dy }
    }
}

// ─── HelmDecision ────────────────────────────────────────────────────────────

/// A steering decision from the helm.
#[derive(Debug, Clone)]
pub struct HelmDecision {
    pub bearing: Bearing,
    pub speed: f64,
    pub priority: u32,
}

impl HelmDecision {
    pub fn new(bearing: Bearing, speed: f64, priority: u32) -> Self {
        Self { bearing, speed, priority }
    }

    pub fn stop() -> Self {
        Self { bearing: Bearing::zero(), speed: 0.0, priority: 0 }
    }
}

// ─── Waypoint ────────────────────────────────────────────────────────────────

/// A named checkpoint with position and optional ETA.
#[derive(Debug, Clone)]
pub struct Waypoint {
    pub name: String,
    pub x: f64,
    pub y: f64,
    pub eta: Option<f64>,
}

impl Waypoint {
    pub fn new(name: &str, x: f64, y: f64) -> Self {
        Self { name: name.to_string(), x, y, eta: None }
    }

    pub fn with_eta(mut self, eta: f64) -> Self {
        self.eta = Some(eta);
        self
    }

    pub fn bearing_to(&self, other: &Waypoint) -> Bearing {
        Bearing::new(other.x - self.x, other.y - self.y)
    }

    pub fn distance_to(&self, other: &Waypoint) -> f64 {
        self.bearing_to(other).magnitude()
    }
}

// ─── Agent State ─────────────────────────────────────────────────────────────

/// An agent's current state in the fleet.
#[derive(Debug, Clone)]
pub struct AgentState {
    pub id: usize,
    pub x: f64,
    pub y: f64,
    pub vx: f64,
    pub vy: f64,
    pub radius: f64,
}

impl AgentState {
    pub fn new(id: usize, x: f64, y: f64) -> Self {
        Self { id, x, y, vx: 0.0, vy: 0.0, radius: 1.0 }
    }

    pub fn with_radius(mut self, r: f64) -> Self {
        self.radius = r;
        self
    }

    pub fn bearing_to(&self, other: &AgentState) -> Bearing {
        Bearing::new(other.x - self.x, other.y - self.y)
    }

    pub fn distance_to(&self, other: &AgentState) -> f64 {
        self.bearing_to(other).magnitude()
    }
}

// ─── Collision Avoidance ─────────────────────────────────────────────────────

/// A potential collision detected.
#[derive(Debug, Clone)]
pub struct CollisionRisk {
    pub agent_a: usize,
    pub agent_b: usize,
    pub distance: f64,
    pub time_to_collision: f64,
}

/// Detect collision risks between agents.
pub fn detect_collisions(agents: &[AgentState], min_distance: f64) -> Vec<CollisionRisk> {
    let mut risks = Vec::new();
    for i in 0..agents.len() {
        for j in (i+1)..agents.len() {
            let dist = agents[i].distance_to(&agents[j]);
            let combined_radius = agents[i].radius + agents[j].radius;
            if dist < min_distance + combined_radius {
                // Relative velocity
                let rel_vx = agents[j].vx - agents[i].vx;
                let rel_vy = agents[j].vy - agents[i].vy;
                let rel_speed = (rel_vx * rel_vx + rel_vy * rel_vy).sqrt();

                risks.push(CollisionRisk {
                    agent_a: agents[i].id,
                    agent_b: agents[j].id,
                    distance: dist,
                    time_to_collision: if rel_speed > 0.01 {
                        (dist - combined_radius) / rel_speed
                    } else {
                        f64::INFINITY
                    },
                });
            }
        }
    }
    risks
}

/// Generate avoidance bearings for agents at risk.
pub fn avoid_collisions(agents: &[AgentState], risks: &[CollisionRisk]) -> HashMap<usize, Bearing> {
    let mut corrections: HashMap<usize, Bearing> = HashMap::new();

    for risk in risks {
        let a = agents.iter().find(|a| a.id == risk.agent_a);
        let b = agents.iter().find(|b| b.id == risk.agent_b);

        if let (Some(a), Some(b)) = (a, b) {
            // Push each agent away from the other
            let away_b = a.bearing_to(b).normalized().scale(-1.0);
            let away_a = b.bearing_to(a).normalized().scale(-1.0);

            corrections.entry(a.id)
                .and_modify(|b| { b.dx += away_b.dx; b.dy += away_b.dy; })
                .or_insert(away_b);
            corrections.entry(b.id)
                .and_modify(|b| { b.dx += away_a.dx; b.dy += away_a.dy; })
                .or_insert(away_a);
        }
    }

    corrections
}

// ─── Formation ───────────────────────────────────────────────────────────────

/// Formation patterns for fleet arrangement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Formation {
    Line,
    Grid,
    Circle,
    VShape,
    Swarm,
}

impl Formation {
    /// Generate positions for n agents in this formation centered at (cx, cy).
    pub fn positions(&self, n: usize, cx: f64, cy: f64, spacing: f64) -> Vec<(f64, f64)> {
        match self {
            Formation::Line => {
                let total_len = (n as f64 - 1.0) * spacing;
                let start_x = cx - total_len / 2.0;
                (0..n).map(|i| (start_x + i as f64 * spacing, cy)).collect()
            }
            Formation::Grid => {
                let cols = (n as f64).sqrt().ceil() as usize;
                (0..n).map(|i| {
                    let row = i / cols;
                    let col = i % cols;
                    (cx + col as f64 * spacing, cy + row as f64 * spacing)
                }).collect()
            }
            Formation::Circle => {
                (0..n).map(|i| {
                    let angle = 2.0 * std::f64::consts::PI * i as f64 / n as f64;
                    (cx + spacing * angle.cos(), cy + spacing * angle.sin())
                }).collect()
            }
            Formation::VShape => {
                let half = n / 2;
                (0..n).map(|i| {
                    let side = if i < half { -1.0 } else { 1.0 };
                    let idx = if i < half { i } else { i - half };
                    (cx + side * idx as f64 * spacing * 0.5, cy - idx as f64 * spacing)
                }).collect()
            }
            Formation::Swarm => {
                // Random-ish swarm using simple hash
                (0..n).map(|i| {
                    let angle = (i as f64 * 2.39996).sin() * std::f64::consts::PI;
                    let r = spacing * (0.5 + 0.5 * ((i * 7 + 3) as f64).sin());
                    (cx + r * angle.cos(), cy + r * angle.sin())
                }).collect()
            }
        }
    }
}

// ─── FleetNavigator ──────────────────────────────────────────────────────────

/// Plans routes for multiple agents through waypoints.
pub struct FleetNavigator {
    waypoints: Vec<Waypoint>,
}

impl FleetNavigator {
    pub fn new() -> Self {
        Self { waypoints: Vec::new() }
    }

    pub fn add_waypoint(&mut self, wp: Waypoint) {
        self.waypoints.push(wp);
    }

    /// Plan a route through all waypoints in order.
    pub fn plan_route(&self) -> Vec<(String, f64)> {
        let mut route = Vec::new();
        let mut total = 0.0;

        for i in 0..self.waypoints.len() {
            if i > 0 {
                total += self.waypoints[i-1].distance_to(&self.waypoints[i]);
            }
            route.push((self.waypoints[i].name.clone(), total));
        }
        route
    }

    /// Total route distance.
    pub fn total_distance(&self) -> f64 {
        let mut total = 0.0;
        for i in 1..self.waypoints.len() {
            total += self.waypoints[i-1].distance_to(&self.waypoints[i]);
        }
        total
    }

    pub fn waypoint_count(&self) -> usize {
        self.waypoints.len()
    }
}

impl Default for FleetNavigator {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bearing_magnitude() {
        let b = Bearing::new(3.0, 4.0);
        assert!((b.magnitude() - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_bearing_normalized() {
        let b = Bearing::new(3.0, 4.0).normalized();
        assert!((b.magnitude() - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_bearing_add_sub() {
        let a = Bearing::new(1.0, 2.0);
        let b = Bearing::new(3.0, 4.0);
        let sum = a.add(&b);
        assert_eq!(sum.dx, 4.0);
        assert_eq!(sum.dy, 6.0);
        let diff = b.sub(&a);
        assert_eq!(diff.dx, 2.0);
    }

    #[test]
    fn test_waypoint_distance() {
        let a = Waypoint::new("a", 0.0, 0.0);
        let b = Waypoint::new("b", 3.0, 4.0);
        assert!((a.distance_to(&b) - 5.0).abs() < 0.001);
    }

    #[test]
    fn test_waypoint_bearing() {
        let a = Waypoint::new("a", 0.0, 0.0);
        let b = Waypoint::new("b", 1.0, 0.0);
        let bearing = a.bearing_to(&b);
        assert!((bearing.dx - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_detect_no_collisions() {
        let agents = vec![
            AgentState::new(0, 0.0, 0.0),
            AgentState::new(1, 100.0, 100.0),
        ];
        let risks = detect_collisions(&agents, 10.0);
        assert!(risks.is_empty());
    }

    #[test]
    fn test_detect_collision() {
        let agents = vec![
            AgentState::new(0, 0.0, 0.0).with_radius(1.0),
            AgentState::new(1, 3.0, 0.0).with_radius(1.0),
        ];
        let risks = detect_collisions(&agents, 5.0);
        assert_eq!(risks.len(), 1);
        assert!((risks[0].distance - 3.0).abs() < 0.001);
    }

    #[test]
    fn test_avoid_collisions() {
        let agents = vec![
            AgentState::new(0, 0.0, 0.0).with_radius(1.0),
            AgentState::new(1, 2.0, 0.0).with_radius(1.0),
        ];
        let risks = detect_collisions(&agents, 5.0);
        let corrections = avoid_collisions(&agents, &risks);
        assert!(!corrections.is_empty());
    }

    #[test]
    fn test_formation_line() {
        let pos = Formation::Line.positions(3, 0.0, 0.0, 2.0);
        assert_eq!(pos.len(), 3);
        assert!((pos[0].0 - (-2.0)).abs() < 0.001);
        assert!((pos[2].0 - 2.0).abs() < 0.001);
    }

    #[test]
    fn test_formation_circle() {
        let pos = Formation::Circle.positions(4, 0.0, 0.0, 5.0);
        assert_eq!(pos.len(), 4);
        // All should be ~5 from center
        for (x, y) in &pos {
            let dist = (x*x + y*y).sqrt();
            assert!((dist - 5.0).abs() < 0.1);
        }
    }

    #[test]
    fn test_formation_grid() {
        let pos = Formation::Grid.positions(9, 0.0, 0.0, 1.0);
        assert_eq!(pos.len(), 9);
    }

    #[test]
    fn test_formation_v_shape() {
        let pos = Formation::VShape.positions(6, 0.0, 0.0, 2.0);
        assert_eq!(pos.len(), 6);
    }

    #[test]
    fn test_navigator_route() {
        let mut nav = FleetNavigator::new();
        nav.add_waypoint(Waypoint::new("start", 0.0, 0.0));
        nav.add_waypoint(Waypoint::new("mid", 3.0, 4.0));
        nav.add_waypoint(Waypoint::new("end", 3.0, 8.0));
        let route = nav.plan_route();
        assert_eq!(route.len(), 3);
        assert!((route[1].1 - 5.0).abs() < 0.001); // 3-4-5 triangle
        assert!((nav.total_distance() - 9.0).abs() < 0.001);
    }

    #[test]
    fn test_helm_decision() {
        let d = HelmDecision::new(Bearing::new(1.0, 0.0), 5.0, 1);
        assert_eq!(d.speed, 5.0);
        assert_eq!(d.priority, 1);
    }
}
