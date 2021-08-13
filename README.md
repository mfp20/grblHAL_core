# grblHAL offloaded kernel ##

## 1. Introduction
**This is a development version, not for users. It can literally set your house on fire.**

This is a fork of [grblHAL core](https://github.com/grblHAL/core), modified to split the core from the motion computing so that it can be offloaded on host, extra cores in the mcu, or external co-processor (ex: fpga).
This change is needed in order to import a couple of ideas from [Klipper firmware](https://www.klipper3d.org/) (or on [github](https://github.com/KevinOConnor/klipper)):
* off load the computational heavy-lifting from the MCU in order to maximize performance,
* being able to use more MCUs/boards in sync as one single printer.

The name is changed to reflect changes: grblHAL+klipper become grblhk, and "core" is changed to "kernel" to avoid ambiguities with mcu cores. The board driver become the "HAL".

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
and reducing jitter as pure I/O irqs raise on a different mcu core.

Use case nr. 5 instead, shows multiple boards working together. On startup the host computes deltas between its own time and every other mcu "board time"; then maintain this deltas using specialized sync messages to minimize jitter. All the motion is computed on host using its own "build time" and then steps are sent using the appropriate board times (computed using the deltas) in order to trigger events at the right time on every board.

Changes must take into consideration the concurrency issues and introduce proper semaphores, mutexes, critical sections and so on.
At the same time there's the need to add some extra sync information to critical structures in order to enable syncing among multiple MCUs and the host.

Currently changes are implemented using some defines in config.h, so the offloading is set at compile time to produce an experimental build. 
But the final version might be able to set offloading at runtime (ex: on boot).

## 3. Vanilla workflow

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

## 4. Notes

The reference board for testing this grbl core is [RPi Pico and the customized board driver](https://github.com/mfp20/grblhk-hal-RP2040).

For more info, to partecipate, or to have an idea of current status have a look at
- [original discussion](https://github.com/grblHAL/core/discussions/34)
- [details about changes in code](https://github.com/grblHAL/core/discussions/52)
- [details about the protocol](https://github.com/grblHAL/core/discussions/55)
