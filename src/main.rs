use std::collections::{BTreeSet, BinaryHeap};
use std::cmp::Ordering;
use itertools::Itertools;

fn square_dist(a: (isize, isize), b: (isize, isize)) -> usize {
    (a.0 - b.0).abs().max((a.1 - b.1).abs()) as usize
}
fn diamond_dist(a: (isize, isize), b: (isize, isize)) -> usize {
    ((a.0 - b.0).abs() + (a.1 - b.1).abs()) as usize
}

const BROADCAST_RANGE: usize = 2;
const DEFAULT_SIM_TIME: usize = 1000;

fn step(pos: (isize, isize), dir: u8) -> (isize, isize) {
    match dir {
        0 => (pos.0, pos.1 + 1),
        1 => (pos.0 + 1, pos.1),
        2 => (pos.0, pos.1 - 1),
        3 => (pos.0 - 1, pos.1),
        _ => panic!(),
    }
}
fn will_collide(posdir1: ((isize, isize), u8), posdir2: ((isize, isize), u8)) -> bool {
    match diamond_dist(posdir1.0, posdir2.0) {
        0 => true,
        1 => posdir1.1 != posdir2.1 && (step(posdir1.0, posdir1.1) == posdir2.0 || step(posdir2.0, posdir2.1) == posdir1.0),
        2 => step(posdir1.0, posdir1.1) == step(posdir2.0, posdir2.1),
        _ => false,
    }
}

pub trait Controller {
    type Broadcast: Clone + Copy;
    fn new() -> Self;
    fn broadcast(&self) -> Self::Broadcast;
    fn calculate(&mut self, info: &Info, other: Option<(&Info, &Self::Broadcast)>) -> u8;
}

#[derive(Clone, Copy, Debug)]
pub struct Info {
    pos: (isize, isize),
    dir: u8,
    target: (isize, isize),
}

pub struct SimpleController;
impl Controller for SimpleController {
    type Broadcast = ();
    fn new() -> Self { SimpleController }
    fn broadcast(&self) {}
    fn calculate(&mut self, info: &Info, _: Option<(&Info, &Self::Broadcast)>) -> u8 {
        let dist = diamond_dist(info.pos, info.target);
        if diamond_dist(step(info.pos, info.dir), info.target) < dist { 0 }
        else if diamond_dist(step(info.pos, (info.dir + 1) % 4), info.target) < dist { 1 }
        else { 3 }
    }
}

#[derive(PartialEq, Eq, PartialOrd, Ord, Clone, Copy, Debug)]
struct Course {
    first: Option<u8>,
    length: usize,

    pos: (isize, isize),
    dir: u8,
    target: (isize, isize),
}
#[derive(PartialEq, Eq, Debug)]
struct SortedCoursePair(Course, Course);
impl SortedCoursePair {
    fn metric(&self) -> usize {
        let h1 = self.0.length + diamond_dist(self.0.pos, self.0.target);
        let h2 = self.1.length + diamond_dist(self.1.pos, self.1.target);
        h1.max(h2)
    }
    fn len(&self) -> usize {
        self.0.length.max(self.1.length)
    }
}
impl Ord for SortedCoursePair {
    fn cmp(&self, other: &Self) -> Ordering {
        (other.metric(), other.len()).cmp(&(self.metric(), self.len()))
    }
}
impl PartialOrd for SortedCoursePair {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct CompleteController;
impl CompleteController {
    fn plot_path(a: &Info, b: &Info) -> (u8, u8) {
        debug_assert_ne!(a.pos, b.pos);
        if a.pos < b.pos {
            let r = Self::plot_path(b, a);
            return (r.1, r.0);
        }

        let mut heap: BinaryHeap<SortedCoursePair> = BinaryHeap::with_capacity(1024);
        let mut heap_set: BTreeSet<(Course, Course)> = Default::default();
        let mut visited_set: BTreeSet<(Course, Course)> = Default::default();
        macro_rules! heap_insert {
            ($a:expr, $b:expr $(,)?) => {{
                let v = ($a, $b);
                if !visited_set.contains(&v) && heap_set.insert(v) {
                    heap.push(SortedCoursePair(v.0, v.1))
                }
            }}
        }

        heap_insert!(
            Course { first: None, length: 0, pos: a.pos, dir: a.dir, target: a.target },
            Course { first: None, length: 0, pos: b.pos, dir: b.dir, target: b.target },
        );
        while let Some(SortedCoursePair(a, b)) = heap.pop() {
            assert!(heap_set.remove(&(a, b))); // take it out of the heap set and put it in the visited set
            assert!(visited_set.insert((a, b)));

            if a.pos != a.target && b.pos != b.target {
                for &da in &[0, 1, 3] {
                    for &db in &[0, 1, 3] {
                        let new_a_dir = (a.dir + da) % 4;
                        let new_b_dir = (b.dir + db) % 4;
                        if will_collide((a.pos, new_a_dir), (b.pos, new_b_dir)) { continue }
                        let new_a_pos = step(a.pos, new_a_dir);
                        let new_b_pos = step(b.pos, new_b_dir);
                        if new_a_pos == a.target && new_b_pos == b.target { return (a.first.unwrap_or(da), b.first.unwrap_or(db)); }

                        heap_insert!(
                            Course { first: a.first.or(Some(da)), length: a.length + 1, pos: new_a_pos, dir: new_a_dir, target: a.target },
                            Course { first: b.first.or(Some(db)), length: b.length + 1, pos: new_b_pos, dir: new_b_dir, target: b.target },
                        );
                    }
                }
            }
            else if a.pos != a.target {
                for &d in &[0, 1, 3] {
                    let new_dir = (a.dir + d) % 4;
                    let new_pos = step(a.pos, new_dir);
                    if new_pos == a.target { return (a.first.unwrap(), b.first.unwrap()); }

                    heap_insert!(Course { first: a.first, length: a.length + 1, pos: new_pos, dir: new_dir, target: a.target }, b);
                }
            }
            else if b.pos != b.target {
                for &d in &[0, 1, 3] {
                    let new_dir = (b.dir + d) % 4;
                    let new_pos = step(b.pos, new_dir);
                    if new_pos == b.target { return (a.first.unwrap(), b.first.unwrap()); }

                    heap_insert!(a, Course { first: b.first, length: b.length + 1, pos: new_pos, dir: new_dir, target: b.target });
                }
            }
            else { unreachable!(); }
        }
        panic!("this should be impossible");
    }
}
impl Controller for CompleteController {
    type Broadcast = ();
    fn new() -> Self { Self }
    fn broadcast(&self) { }
    fn calculate(&mut self, info: &Info, other: Option<(&Info, &Self::Broadcast)>) -> u8 {
        match other {
            None => SimpleController.calculate(info, None),
            Some((other, _)) => CompleteController::plot_path(info, other).0,
        }
    }
}

pub enum SimulationResult {
    Completed,
    Collision,
    Running,
}
pub struct Simulator<T> {
    aircraft: Vec<(Info, T, usize)>,
    paths: Vec<Vec<(isize, isize)>>,
}
impl<T> Simulator<T> where T: Controller {
    fn comms_group(&self, index: usize) -> Vec<usize> {
        debug_assert!(index < self.aircraft.len());

        let mut group = Vec::with_capacity(self.aircraft.len());
        group.push(index);
        loop {
            let mut added = false;
            for (i, other) in self.aircraft.iter().enumerate() {
                if group.contains(&i) { continue }
                if group.iter().any(|&x| square_dist(other.0.pos, self.aircraft[x].0.pos) <= BROADCAST_RANGE) {
                    group.push(i);
                    added = true;
                }
            }
            if !added {
                group.swap_remove(0);
                return group;
            }
        }
    }
    pub fn from_scenario(scenario: &[Info]) -> Self {
        let aircraft = scenario.iter().copied().enumerate().map(|(i, info)| (info, T::new(), i)).collect();
        let paths = scenario.iter().map(|x| vec![x.pos]).collect();
        Simulator { aircraft, paths }
    }
    pub fn tick(&mut self, max_ticks: usize) -> SimulationResult {
        for _ in 0..max_ticks {
            if self.aircraft.is_empty() { return SimulationResult::Completed; }

            let snapshots: Vec<_> = self.aircraft.iter().map(|x| (x.0, x.1.broadcast())).collect();
            for i in 0..self.aircraft.len() {
                let group = self.comms_group(i);
                let other = if group.is_empty() { None } else {
                    let snapshot = &snapshots[group[0]];
                    Some((&snapshot.0, &snapshot.1))
                };
                let delta = self.aircraft[i].1.calculate(&snapshots[i].0, other);
                assert!(match delta { 0 | 1 | 3 => true, _ => false });
                self.aircraft[i].0.dir = (self.aircraft[i].0.dir + delta) % 4;
            }

            for (i, a) in self.aircraft.iter().enumerate() {
                for b in self.aircraft[i+1..].iter() {
                    if will_collide((a.0.pos, a.0.dir), (b.0.pos, b.0.dir)) {
                        return SimulationResult::Collision;
                    }
                }
            }

            for i in (0..self.aircraft.len()).rev() {
                let info = &mut self.aircraft[i];
                info.0.pos = step(info.0.pos, info.0.dir);
                self.paths[info.2].push(info.0.pos);
                if info.0.pos == info.0.target { self.aircraft.swap_remove(i); }
            }
        }
        if self.aircraft.is_empty() { SimulationResult::Completed } else { SimulationResult::Running }
    }
    fn get_time(&self) -> usize {
        self.paths.iter().map(|x| x.len()).max().unwrap() - 1
    }
}

fn for_scenarios<F>(n: usize, k: usize, mut f: F)
where F: FnMut(u64, Vec<Info>)
{
    assert!(k > 0 && k <= n * n);
    let space: Vec<_> = (0..n as isize).cartesian_product(0..n as isize).collect();
    let directions = &[0, 1, 2, 3];
    let mut count = 0;
    for starts in space.iter().copied().permutations(k) {
        for targets in (0..k).map(|_| space.iter().copied()).multi_cartesian_product() {
            if (0..k).any(|i| targets[i] == starts[i]) { continue }
            for directions in (0..k).map(|_| directions.iter().copied()).multi_cartesian_product() {
                let scenario = (0..k).map(|i| Info { pos: starts[i], dir: directions[i], target: targets[i] }).collect();
                count += 1;
                f(count, scenario);
            }
        }
    }
}

#[test]
fn test_simple() {
    for_scenarios(8, 1, |i, scenario| {
        let mut sim = Simulator::<SimpleController>::from_scenario(&scenario);
        match sim.tick(DEFAULT_SIM_TIME) {
            SimulationResult::Completed => (),
            SimulationResult::Collision => panic!("sim {} ({:?}) had a collision", i, scenario),
            SimulationResult::Running => panic!("sim {} ({:?}) incomplete", i, scenario),
        }
    });
}

fn independent_time(scenario: &[Info]) -> usize {
    let mut max = 0;
    for single in scenario.windows(1) {
        let mut sim = Simulator::<SimpleController>::from_scenario(single);
        match sim.tick(DEFAULT_SIM_TIME) {
            SimulationResult::Completed => max = max.max(sim.get_time()),
            _ => panic!(),
        }
    }
    max
}

fn main() {
    let mut total = 0u64;
    let mut nonzero_total = 0u64;
    let mut delay_rounds = 0u64;
    let mut max_delay_rounds = 0u64;
    for_scenarios(8, 2, |i, scenario| {
        let mut sim = Simulator::<CompleteController>::from_scenario(&scenario);
        match sim.tick(DEFAULT_SIM_TIME) {
            SimulationResult::Completed => {
                let d = (sim.get_time() - independent_time(&scenario)) as u64;
                total += 1;
                if d != 0 { nonzero_total += 1; }
                delay_rounds += d;
                if d > max_delay_rounds {
                    max_delay_rounds = d;
                    println!("new max delay: {} ({:?})", d, scenario);
                }
                max_delay_rounds = max_delay_rounds.max(d);
            }
            SimulationResult::Collision => panic!("sim {} ({:?}) had a collision", i, scenario),
            SimulationResult::Running => panic!("sim {} ({:?}) incomplete", i, scenario),
        }
    });
    println!("\npassed all {} scenarios", total);
    println!("max delay: {}", max_delay_rounds);
    println!("avg delay: {}", delay_rounds as f64 / total as f64);
    println!("avg nonzero delay: {}", delay_rounds as f64 / nonzero_total as f64);
}
