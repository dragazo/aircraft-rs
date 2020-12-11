use std::collections::{BTreeSet, BinaryHeap, VecDeque};
use std::cmp::Ordering;
use std::str::FromStr;
use itertools::Itertools;

fn square_dist(a: (isize, isize), b: (isize, isize)) -> usize {
    (a.0 - b.0).abs().max((a.1 - b.1).abs()) as usize
}
fn diamond_dist(a: (isize, isize), b: (isize, isize)) -> usize {
    ((a.0 - b.0).abs() + (a.1 - b.1).abs()) as usize
}

const BROADCAST_RANGE: usize = 2;

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

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
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

#[derive(PartialEq, Eq, PartialOrd, Ord, Clone, Debug)]
struct Course {
    path: VecDeque<u8>,
    pos: (isize, isize),
    dir: u8,
    target: (isize, isize),
}
impl Course {
    fn info(&self) -> Info {
        Info { pos: self.pos, dir: self.dir, target: self.target }
    }
}
#[derive(PartialEq, Eq, Debug)]
struct SortedCoursePair(Course, Course);
impl SortedCoursePair {
    fn metrics(&self) -> (usize, usize) {
        let h1 = self.0.path.len() + diamond_dist(self.0.pos, self.0.target);
        let h2 = self.1.path.len() + diamond_dist(self.1.pos, self.1.target);
        (h1, h2)
    }
    fn long_metric(&self) -> usize {
        let (h1, h2) = self.metrics();
        h1.max(h2)
    }
    fn short_metric(&self) -> usize {
        let (h1, h2) = self.metrics();
        h1.min(h2)
    }
    fn len(&self) -> usize {
        self.0.path.len().max(self.1.path.len())
    }
    fn info_pair(&self) -> (Info, Info) {
        (self.0.info(), self.1.info())
    }
}
impl Ord for SortedCoursePair {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap is a max heap - we want lowest metric and highest length, so invert metric and leave len the same
        match other.long_metric().cmp(&self.long_metric()) { // shortest overal time (both aircraft)
            Ordering::Equal => match other.short_metric().cmp(&self.short_metric()) { // shortest individual time (tie breaker)
                Ordering::Equal => self.len().cmp(&other.len()), // whichever path is closest to being completed
                x => x,
            }
            x => x,
        }
    }
}
impl PartialOrd for SortedCoursePair {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub struct CompleteController {
    pre_computed_path: Option<VecDeque<u8>>,
}
impl CompleteController {
    fn plot_path(a: &Info, b: &Info) -> (VecDeque<u8>, VecDeque<u8>) {
        debug_assert_ne!(a.pos, b.pos);
        if a.pos < b.pos {
            let r = Self::plot_path(b, a);
            return (r.1, r.0);
        }

        let mut heap: BinaryHeap<SortedCoursePair> = BinaryHeap::with_capacity(1024);
        let mut heap_set: BTreeSet<(Info, Info)> = Default::default();
        let mut visited_set: BTreeSet<(Info, Info)> = Default::default();
        macro_rules! heap_insert {
            ($a:expr, $b:expr $(,)?) => {{
                let v = SortedCoursePair($a, $b);
                let info = v.info_pair();
                if !visited_set.contains(&info) && heap_set.insert(info) {
                    heap.push(SortedCoursePair(v.0, v.1))
                }
            }}
        }

        heap_insert!(
            Course { path: Default::default(), pos: a.pos, dir: a.dir, target: a.target },
            Course { path: Default::default(), pos: b.pos, dir: b.dir, target: b.target },
        );
        while let Some(front) = heap.pop() {
            let info = front.info_pair();
            assert!(heap_set.remove(&info)); // take it out of the heap set and put it in the visited set
            assert!(visited_set.insert(info));
            let SortedCoursePair(a, b) = front;

            if a.pos != a.target && b.pos != b.target {
                for &da in &[0, 1, 3] {
                    for &db in &[0, 1, 3] {
                        let new_a_dir = (a.dir + da) % 4;
                        let new_b_dir = (b.dir + db) % 4;
                        if will_collide((a.pos, new_a_dir), (b.pos, new_b_dir)) { continue }
                        let mut new_a_path = a.path.clone();
                        let mut new_b_path = b.path.clone();
                        new_a_path.push_back(da);
                        new_b_path.push_back(db);
                        let new_a_pos = step(a.pos, new_a_dir);
                        let new_b_pos = step(b.pos, new_b_dir);
                        if new_a_pos == a.target && new_b_pos == b.target { return (new_a_path, new_b_path); }

                        heap_insert!(
                            Course { path: new_a_path, pos: new_a_pos, dir: new_a_dir, target: a.target },
                            Course { path: new_b_path, pos: new_b_pos, dir: new_b_dir, target: b.target },
                        );
                    }
                }
            }
            else if a.pos != a.target {
                for &d in &[0, 1, 3] {
                    let new_dir = (a.dir + d) % 4;
                    let new_pos = step(a.pos, new_dir);
                    let mut new_path = a.path.clone();
                    new_path.push_back(d);
                    if new_pos == a.target { return (new_path, b.path); }

                    heap_insert!(Course { path: new_path, pos: new_pos, dir: new_dir, target: a.target }, b.clone());
                }
            }
            else if b.pos != b.target {
                for &d in &[0, 1, 3] {
                    let new_dir = (b.dir + d) % 4;
                    let new_pos = step(b.pos, new_dir);
                    let mut new_path = b.path.clone();
                    new_path.push_back(d);
                    if new_pos == b.target { return (a.path, new_path); }

                    heap_insert!(a.clone(), Course { path: new_path, pos: new_pos, dir: new_dir, target: b.target });
                }
            }
            else { unreachable!(); }
        }
        unreachable!("if this happend, the proof was wrong");
    }
}
impl Controller for CompleteController {
    type Broadcast = ();
    fn new() -> Self { Self { pre_computed_path: None } }
    fn broadcast(&self) { }
    fn calculate(&mut self, info: &Info, other: Option<(&Info, &Self::Broadcast)>) -> u8 {
        match &mut self.pre_computed_path {
            Some(pre) => pre.pop_front().unwrap(),
            None => match other {
                None => SimpleController.calculate(info, None),
                Some((other, _)) => {
                    let mut r = Self::plot_path(info, other).0;
                    let v = r.pop_front().unwrap();
                    self.pre_computed_path = Some(r);
                    v
                }
            }
        }
    }
}

#[derive(PartialEq, Eq)]
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
                let info = &mut self.aircraft[i];
                let delta = info.1.calculate(&snapshots[i].0, other);
                assert!(match delta { 0 | 1 | 3 => true, _ => false });
                info.0.dir = (info.0.dir + delta) % 4; // we can update dir now since everything is cached in snapshots - but changing pos would break collision logic
                self.paths[info.2].push(step(info.0.pos, info.0.dir)); // add next pos to path before checking for collisions so we can draw failures
            }

            for (i, a) in self.aircraft.iter().enumerate() {
                for b in self.aircraft[i + 1..].iter() {
                    if will_collide((a.0.pos, a.0.dir), (b.0.pos, b.0.dir)) {
                        return SimulationResult::Collision;
                    }
                }
            }

            for i in (0..self.aircraft.len()).rev() {
                let info = &mut self.aircraft[i];
                info.0.pos = step(info.0.pos, info.0.dir);
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
        match sim.tick(1000) {
            SimulationResult::Completed => (),
            SimulationResult::Collision => panic!("sim {} ({:?}) had a collision", i, scenario),
            SimulationResult::Running => panic!("sim {} ({:?}) incomplete", i, scenario),
        }
    });
}

fn independent_time(scenario: &[Info], max_rounds: usize) -> usize {
    let mut max = 0;
    for single in scenario.windows(1) {
        let mut sim = Simulator::<SimpleController>::from_scenario(single);
        match sim.tick(max_rounds) {
            SimulationResult::Completed => max = max.max(sim.get_time()),
            _ => panic!(),
        }
    }
    max
}

macro_rules! crash {
    () => { std::process::exit(1); };
    ($msg:expr, $($args:expr)*) => {{
        eprintln!($msg, $($args)*);
        crash!();
    }}
}
fn parse_int<T: FromStr>(val: &str) -> T {
    match T::from_str(val) {
        Ok(v) => v,
        Err(_) => crash!("failed to parse '{}' as integer of valid range", val),
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let print_help = || {
        eprintln!("usage: {} [mode]", args[0]);
        eprintln!("    mode: help, full, case");
        
    };

    if args.len() < 2 {
        print_help();
        crash!();
    }
    match args[1].as_str() {
        "-h" | "--help" | "help" => print_help(),
        "case" => {
            if args.len() < 5 { crash!(r"usage: {} case controller max_rounds [pos_x:pos_y:dir:target_x:target_y]+
    controller - simple | complete
    example: case simple 1000 0:0:1:5:20 10:5:3:-10:0
", args[0]); }
            let max_rounds = parse_int(&args[3]);
            let mut scenario = vec![];
            for spec in &args[4..] {
                let v: Vec<isize> = spec.split(':').map(|x| parse_int(x)).collect();
                if v.len() != 5 { crash!("failed to parse spec '{}'", spec); }
                if v[2] < 0 || v[2] > 3 { crash!("invalid direction: {} (must be 0-3)", v[2]); }
                let info = Info { pos: (v[0], v[1]), dir: v[2] as u8, target: (v[3], v[4]) };
                scenario.push(info);
            }
            fn tick_by<T: Controller>(sim: &mut Simulator<T>, count: usize) -> bool {
                match sim.tick(count) {
                    SimulationResult::Collision => { println!("COLLISION!!"); false },
                    SimulationResult::Running => { println!("INCOMPLETE!!"); false },
                    SimulationResult::Completed => true,
                }
            }
            match args[2].as_str() {
                "simple" => {
                    let mut sim = Simulator::<SimpleController>::from_scenario(&scenario);
                    let ok = tick_by(&mut sim, max_rounds);
                    for (i, x) in sim.paths.iter().enumerate() {
                        println!("path {} - {:?}", i + 1, x);
                    }
                    if ok {
                        println!("{} rounds", sim.get_time());
                        println!("{} delay rounds", sim.get_time() - independent_time(&scenario, max_rounds));
                    }
                }
                "complete" => {
                    let mut sim = Simulator::<CompleteController>::from_scenario(&scenario);
                    let ok = tick_by(&mut sim, max_rounds);
                    for (i, x) in sim.paths.iter().enumerate() {
                        println!("path {} - {:?}", i + 1, x);
                    }
                    if ok {
                        println!("{} rounds", sim.get_time());
                        println!("{} delay rounds", sim.get_time() - independent_time(&scenario, max_rounds));
                    }
                }
                x => crash!("unrecognized controller type: '{}'", x),
            }
        }
        "full" => {
            if args.len() != 4 { crash!("usage: {} full n k\n    n - size of grid (n x n)\n    k - number of aircraft", args[0]); }
            let n = parse_int(&args[2]);
            let k = parse_int(&args[3]);
            let max_rounds = 1000;

            let mut count = 0u64;
            let mut non_trivial_count = 0u64;
            let mut delay_rounds = 0u64;
            let mut max_delay_rounds = 0u64;
            for_scenarios(n, k, |i, scenario| {
                let mut sim = Simulator::<CompleteController>::from_scenario(&scenario);
                match sim.tick(max_rounds) {
                    SimulationResult::Completed => {
                        let d = (sim.get_time() - independent_time(&scenario, max_rounds)) as u64;
                        let trivial = Simulator::<SimpleController>::from_scenario(&scenario).tick(max_rounds) == SimulationResult::Completed;
                        if trivial { assert_eq!(d, 0); } else { non_trivial_count += 1; }
                        count += 1;
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
            println!("\npassed all {} scenarios ({} non-trivial)", count, non_trivial_count);
            println!("max delay: {}", max_delay_rounds);
            println!("avg delay: {}", delay_rounds as f64 / count as f64);
            println!("avg non-trivial delay: {}", delay_rounds as f64 / non_trivial_count as f64);
        }
        x => {
            eprintln!("unrecognized mode '{}'", x);
            print_help();
            crash!();
        }
    }
}
