# aircraft-rs

`aircraft-rs` is a rust implementation for an automated aircraft controller with limited information on the locations of other aircraft.
Aircraft travel within an infinite grid space and have the option to turn left or right at each time step; however, they cannot turn around or stop.
The `SimpleController` navigates any aircraft to its target as quickly as possible, but does not attempt to avoid collisions.
The `CompleteController` navigates aircraft to their targets while avoiding collisions; it does this by following the `SimpleController` logic until it detects another nearby aircraft, at which point they both perform an A* search for the mutually shortest path to their targets which also avoids collisions.

## Building

As a standard rust project, you need only clone this repository and run `cargo build --release` to get an optimized executable.

## Command Line Interface

The command line interface has several modes:

* `case [controller] [max_rounds] [pos_x:pos_y:dir:target_x:target_y]+` - this will test the specified controller (`simple` or `complete`) on the given scenario (the 1+ trailing arguments) using at most `max_rounds` rounds of controller logic. Additionally, at the end of execution, the paths taken by the aircraft will be shown.
* `full [n] [k]` - this will test the `CompleteController` on every possible scenario involving `k` aircraft placed within an `n x n` space. The time required to do this increases very quickly. Statistics are printed at the end which measure the controller's performance.
