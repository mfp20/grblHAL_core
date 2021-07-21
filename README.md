## grblHAL offloaded core ##

This is a fork of [grblHAL core](https://github.com/grblHAL/core), modified to split the core from the motion computing so that it can be offloaded on host, extra cores in the mcu or external co-processor (ex: fpga). It is a development version, not for users.

The idea is to go from current architecture 

![Current architecture](docs/current_architecture.png?raw=true)
(src: [https://awesome.tech/grbl-demystified/](https://awesome.tech/grbl-demystified/))

to a new one

![New architecture](docs/new_architecture.png?raw=true)

with minimum changes to core code and board drivers. This would allow new use cases

![Use cases](docs/use_cases.png?raw=true)

For more info have a look at the [original discussion](https://github.com/grblHAL/core/discussions/34).
