# grblHAL offloaded core ##

## 1. Introduction

This is a fork of [grblHAL core](https://github.com/grblHAL/core), modified to split the core from the motion computing so that it can be offloaded on host, extra cores in the mcu or external co-processor (ex: fpga). It is a development version, not for users.

## 2. The Plan

The idea is to go from current architecture 

![Current architecture](docs/current_architecture.png?raw=true)
(src: [https://awesome.tech/grbl-demystified/](https://awesome.tech/grbl-demystified/))

to a new one

![New architecture](docs/new_architecture.png?raw=true)

with minimum changes to core code and board drivers. The goal is to make the motion computation optional,
in order to be able to offload it to another core or on another mcu (ex: main host currently sending g-codes).
Basically there's the need to push motion data directly in the last buffer available before the execution of steps.
This would allow new use cases

![Use cases](docs/use_cases.png?raw=true)

## 3. Workflow

Current input and motion computation goes as follow:
1. main loop identifies lines incoming on the serial console, clean up and capitalize,
2. if the line is a g-code, it is handled to the g-code parser
3. if the g-code is a motion command, prepare the plan_line_data_t and call one of the motion control methods
4. control methods (ex: mc_arc) approximate curves in multiple lines and call the planner
5. the planner (re)computes velocity profiles and adds a new linear movement (plan_block_t) to its buffer
6. the main program constantly call the stepper to turn planner blocks into segments and push them in the segments buffer
7. the stepper driver interrupt pops pre-computed segments from the step segment buffer and executes them by pulsing the stepper pins

Points 3-6 are most of the computational heavy lifting; a very little more computing happens in the interrupt handler. The motion planner recomputes all block velocities each time a new block is added, so the block buffer changes in time.
To avoid clashes between the motion planner recalculating all blocks in buffer and the motion stepper popping the blocks, each time the stepper starts a new block it will 'check out' the block, making the planner unable to modify it again.
When the stepper finishes one block, it sets the block as finished so the planner can re-use the same memory location in its circular buffer.

At point 3 the g-code input of the pipeline is very small (just a few bytes of text), optimal for transmition on a communication interface (to host, ex: usb) or multithread-safe queue (to core).
Each g-code can produce multiple lines, each line multiple blocks, each block multiple segments, and each segment is made of many motor steps. 
At point 6 the g-code has been translated in steps and counts many bytes of binary data added to the segments buffer.
If motion offloading is selected, the same steps 3 to 7 run on mcu's second core or on external host.
Steps 2, 6 and 7 are slightly changed in order to implement the new architecture.

Currently changes are implemented using 3 defines in config.h, so the offloading is set at compile time to produce an experimental build. 
But the final version might be able to set offloading at runtime (ex: on boot).

### 3.1. Changes to protocol.c (workflow step 2)

The `protocol_main_loop()` receives motion commands and handle them to `execute_gcode()` (gcodes).
So we introduce a new method `st_push_segment()` able to push segments in the segments buffer instead of handling them to the gcode parser.
Segments pushed via this new method are considered to be ready for direct execution without further manipulation.
[TODO]

### 3.2. Changes to stepper.c (workflow step 6 and 7)

#### 3.2.1. st_prep_buffer()

The realtime execution system continously calls `st_prep_buffer()` to be sure the segment buffer never goes empty until all the planned blocks (ie: gcode issued by the user) are executed.
[TODO]

#### 3.2.2. Changes to the stepper driver interrupt handler (workflow step 7)

The HAL raises an interrupt to generate stepper pulses with precise timing. The IRQ is handled by `stepper_driver_interrupt_handler()`; 
it pops pre-computed segments from the step segment buffer and then executes them by pulsing the stepper pins appropriately.
Currently the interrupt handler contains the last part of the Bresenham and Adaptive Multi-Axis Step Smoothing (AMASS) algorithms.
[TODO]

### 3.3. Other changes

[TODO]

## 4. Notes

For more info have a look at the [original discussion](https://github.com/grblHAL/core/discussions/34).
