![Developed by atmc Badge](https://img.shields.io/badge/atmc-We%20made%20this!-%23303030?labelColor=%23c80000)

# libatmc

Embedded support libraries for the generic embedded platform! There's a gap in embedded systems programming: You have the Arduino / `stm32duino` ecosystem where "slow thing but
work", and the very low-level "if you don't do any register twiddling you're fired". I needed a library that was safe, efficient, realtime focussed, without sacrificing the
readability and expressiveness of a proper library, which doesn't really exist right now! So I wrote one.

`libatmc` currently comprises the following set of non-experimental libraries:

|                |                                                          |
| -------------- | -------------------------------------------------------- |
| `atmc.Core`    | Provides core interfaces with specific microcontrollers. |
| `atmc.Drivers` | Handful of drivers that I've implemented for you to use! |
| `atmc.hwIO`    | File IO library for FAT formatted SD cards.              |

Current support is limited to only any stm32, but the library architecture is designed to be easily extended and flexible, which should make the longer term goal of supporting
RP2040, esp32, and more relatively straightforward.

`libatmc` is intended to be a drop-in solution where you can just start writing code, so if you run into any issues that are not very conducive to "it just works!", please raise an
issue!

## See Also

|                          |                                    |
| ------------------------ | ---------------------------------- |
| License                  | [LICENSE](LICENSE)                 |
| Style Guide              | [codestyle.md](codestyle.md)       |
| C/C++ Formatting Rules   | [clang-format](.clang-format)      |
| Global Library Config    | [Config.h](core/Config.h)          |
| stm32 Usage Instructions | [readme/stm32.md](readme/stm32.md) |
