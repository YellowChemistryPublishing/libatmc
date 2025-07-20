![Developed by atmc Badge](https://img.shields.io/badge/atmc-We%20made%20this!-%23303030?labelColor=%23c80000)

# libatmc

Our in-house set of embedded support libraries for the generic embedded platform! There's a bit of a gap in embedded systems programming. You've got you're Arduino / `stm32duino`
ecosystem where "slow thing but work", and your very low-level "if you don't do any register twiddling you're fired". I needed a library that was safe, efficient, realtime
focussed, without sacrificing the readability and expressiveness of a proper library, which doesn't really exist right now! So I wrote one.

`libatmc` currently comprises the following set of non-experimental libraries:

|                  |                                                          |
| ---------------- | -------------------------------------------------------- |
| `libatmcCore`    | Provides core interfaces with specific microcontrollers. |
| `libatmcDrivers` | Handful of drivers that I've implemented for you to use! |
| `libsysIO`       | File IO library for FAT formatted SD cards.              |

Current support is limited to only any stm32, but the API architecture is designed to be easily extended and flexible, which should make the longer term goal of supporting esp32s
(and more) relatively straightforward.

`libatmc` is intended to be a drop-in solution where you can just start writing code, so if you run into any issues that are not very conducive to "it just works!", please raise an
issue!
