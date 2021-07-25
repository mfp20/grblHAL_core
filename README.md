# grblHAL offloaded core ##

## 1. Introduction
**This is a development version, not for users. It can literally set your house on fire.**

This is a fork of [grblHAL core](https://github.com/grblHAL/core), modified to split the core from the motion computing so that it can be offloaded on host, extra cores in the mcu, or external co-processor (ex: fpga).
This change is needed in order to import a couple of ideas from [Klipper firmware](https://www.klipper3d.org/) (or on [github](https://github.com/KevinOConnor/klipper)):
* off load the computational heavy-lifting from the MCU in order to maximize performance,
* being able to use more MCUs/boards in sync as one single printer.

## 2. The Plan

The goal is to make the motion computation optional, in order to be able to offload it to another core or on another mcu (ex: main host currently sending g-codes).
The plan is to go from current architecture to a new one, introducing minimum changes to core code and board drivers.

*1. Current architecture (courtesy of http://awesome.tech/)*
[![Current architecture](docs/current_architecture.png?raw=true)](https://awesome.tech/grbl-demystified/)

*2. New architecture*
![New architecture](docs/new_architecture.png?raw=true)

Basically there's the need to push motion data directly in the last buffer available before the execution of steps, so that new use cases can be enabled.

*3. New use cases*
![Use cases](docs/use_cases.png?raw=true)

Use case nr. 4 shows the HAL running on core0, grbl core running on core1 and the motion is computed on the host.
In this way the I/O, the core tasks and the motion computing can run concurrently, maximizing performance
and reducing jitter as pure I/O irqs raise on a different mcu/core/cpu.

Use case nr. 5 instead, shows multiple boards working together. On startup the host computes deltas between its own time and every other mcu "board time"; then maintain this deltas using specialized sync messages to minimize jitter. All the motion is computed on host using its own "build time" and then segments are sent using the appropriate board times (computed using the deltas) in order to trigger events at the right time on every board.

Changes must take into consideration the concurrency issues and introduce proper semaphores, mutexes, 
critical sections and so on.
At the same time there's the need to add some extra sync information to critical structures in order to enable syncing among multiple MCUs and the host.

## 3. Roadmap

The roadmap follow the use cases picture:
1. abstract motion computation from grbl core (to move motion computing to mcu's second core)
2. define the format to transmit steps and control messages ('sys' and 'hal' calls) on the serial console (to move motion computing to the remote host)
3. introduce concurrency control elements to make hal and core being able to run on different mcu cores
4. introduce syncing variables in core structures and whatever syncing method needed

## 4. Vanilla workflow

Current input and motion computation goes as follow:
1. main loop identifies lines incoming on the serial console, clean up and capitalize,
2. if the line is a g-code, it is handled to the g-code parser
3. if the g-code is a motion command, prepare the plan_line_data_t and call one of the motion control methods
4. control methods (ex: mc_arc) approximate curves in multiple lines and call the planner
5. the planner (re)computes velocity profiles and adds a new linear movement (plan_block_t) to its buffer
6. the main program constantly call the stepper to turn planner blocks into segments and push them in the segments buffer
7. the stepper driver interrupt pops pre-computed segments from the step segment buffer and executes them by pulsing the stepper pins

The motion planner recomputes all block velocities each time a new block is added, so every block in buffer can change in time. Then the stepper pops the first block in queue to make segments.
To avoid clashes between the motion planner recalculating all blocks in buffer and the motion stepper popping the blocks, each time the stepper starts a new block it will 'check out' the block, 
making the planner unable to modify it again. When the stepper finishes one block, it sets the block as finished so the planner can re-use the same memory location in its circular buffer. 
Note: blocks aren't really popped/pushed as it is a circular buffer and its elements are reused.

Points 3-6 are most of the computational heavy lifting; a very little more computing happens in the interrupt handler (step 7).

At point 3 the g-code input of the pipeline is very small (just a few bytes of text), optimal for transmition on a communication interface (ex: usb to host) or multithread-safe queue (to another mcu core).
Each g-code can produce multiple lines, each line multiple blocks, each block multiple segments, and each segment is made of many motor steps. 
At point 6 the g-code has been translated in steps and counts many bytes of binary data added to the segments buffer.
If motion offloading is selected, the same steps 3 to 7 run on mcu's second core or on external host. Steps 2, 6 and 7 are slightly changed in order to implement the new architecture.

Currently changes are implemented using some defines in config.h, so the offloading is set at compile time to produce an experimental build. 
But the final version might be able to set offloading at runtime (ex: on boot).

### 4.1. General changes

`grbl_hal_t` gets

```c
status_code_t (*push_motion_data)(char *block, char *message);
bool (*pop_motion_data)(char *block, char *message);
```

We add `motion.{h|c}` to host general motion computing structures and methods. In particular
- `struct grbl_motion_t`: global motion struct; `protocol_execute_motion_data` is the function pointer the protocol calls in order to dispatch a motion input line.
- `motion_computing_loop()`: to be run on dedicated core or external host. It receives new gcodes (if any), then keeps the buffers full until all planner bocks are checked out.

In offloaded mode planner buffer is reduced in size so that some RAM is made available for an increased segment buffer and a new `stepper_t` buffer.

All calls to `sys` and `hal` in stepper.c are moved and/or adapted for deferred execution. In particular whatever realtime parameters were set in planner block buffer and segment buffer prep,
are set in `static stepper_t *st` instead, at the very last moment before going live.

[TODO]
* select which calls to `sys` and `hal` in current code must be set at motion computing time, and which ones must be set at execution time instead.
* be sure calls to `sys` and `hal` use single byte data only (ie: are atomic) to avoid concurrency issues, or introduce the needed syncing methods.

### 4.2. Changes to protocol.c (workflow step 2)

The `protocol_main_loop()` receives motion commands and handles them to `protocol_execute_motion_data()`.

We add `execute_gcode()` (in gcode.c) as the motion pipeline's entry point for gcodes.

In monolithic mode and offloaded-to-core mode, grbllib sets `protocol_execute_motion_data` to `execute_gcode()`.
In turn it handles the new gcode to `gc_execute_block()` for local motion computing or `hal.push_motion_data()` for execution on the other core.

In offloaded-to-host mode grbllib sets `protocol_execute_motion_data` to `execute_steps()` (from stepper.c).

[TODO]

### 4.3. Changes to stepper.c (workflow step 6 and 7)

We add a new method `execute_steps()` able to push steps directly to the (new) `stepper_t` buffer, effectively bypassing all the motion pipeline.
Steps pushed via this new method must be ready for direct execution without further manipulation. This implies the need of deeper changes to the stepper code.

We added the steps buffer

```c
static stepper_t steps_buffer[STEPS_BUFFER_SIZE];
```

As we reduce the planner buffer size, STEPS_BUFFER_SIZE is defined as "half of the memory not used by the planner block buffer".
The other half is used to increase the segment buffer size.

Then we turned the old variable into a pointer to currently running steps data

```c
static stepper_t *st;
```

The interrupt handler manages the pointer in order to execute the right steps_buffer element.

[TODO]
* write `execute_steps()`

#### 4.3.1. st_prep_segment_buffer()

The realtime execution system continously calls `st_prep_segment_buffer(true, false)` to be sure the segment buffer never goes empty until all the planned blocks (ie: gcode issued by the user) are checked-out.

In monolithic mode `st_prep_segment_buffer(true, false)` (re)fills segment buffer only (as it used to do in old code).

When the offloading mode is selected, the motion core runs `st_prep_segment_buffer(false, true)` to (re)fill the steps buffer instead.

[TODO]

#### 4.3.2. Changes to the stepper driver interrupt handler

We add a new method `st_prep_steps_buffer()`: executes the last part of the Bresenham and Adaptive Multi-Axis Step Smoothing (AMASS) algorithms and keeps the steps buffer full.
The code is mostly the same of the old interrupt handler (revisited to skip `sys` and `hal` calls, they will be set by the new interrupt handler).

The HAL raises an interrupt to generate stepper pulses with precise timing. The IRQ is handled by `st_exec_interrupt_handler()`.
It has 2 jobs to do:
1. in offloading mode it has to manage the st pointer (ie: st always points to fresh data)
2. call `sys` and `hal` to check state and set realtime parameters

### 4.4. Other changes

[TODO]

## 5. Notes

For more info, to partecipate, or to have an idea of current status have a look at the [original discussion](https://github.com/grblHAL/core/discussions/34).
