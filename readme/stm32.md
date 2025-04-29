# STM32 Configuration Guide

## Preamble

It turns out STM32CubeMX is a very useful codegen tool!
A development `.ioc` file we use to test is already provided, but please configure your own as you need.
Don't forget to regenerate HAL code whenever you change your board configuration!

Whilst this means that you, the humble embedded systems engineer, has a high level of control over your peripherals, there are some caveats, as `libatmc` expects some specific settings set in many cases.

## General

Under `System Core -> NVIC -> Code Generation`, ensure the following are set.

|                                           | `Generate IRQ handler` |
| :---------------------------------------- | :--------------------- |
| `Hard fault interrupt`                    | :white_large_square:   |
| `System service call via SWI instruction` | :white_large_square:   |
| `Pendable request for system service`     | :white_large_square:   |
| `Time base: System tick timer`            | :white_large_square:   |

The hardfault interrupt is replaced with our own handler, which provides slightly improved debugging utility.
The systick handler is also replaced so that stm32 code invokes `HAL_IncTick` and `xPortSysTickHandler`.
`SVC_Handler` and `PendSV_Handler` are deferred to FreeRTOS.

## ADC

You may configure ADC _almost_ however you'd like, but make sure to do the following.

1. Under `NVIC Settings`:
    - `ADCx global interrupts` is checked, for whichever ADC(s) you intend to use.
2. Under `DMA Settings`:
    - A DMA request is added for the ADC(s) you intend to use.
    - Under `DMA Request Settings`:
       - `Mode` is `Circular`.
       - `Data Width` is `Half Word` for both `Peripheral` and `Memory`.
3. Under `Parameter Settings`:
    - If using more than one rank, `Scan Conversion Mode` is `Enabled`.
    - `Continuous Conversion Mode` and `Discontinuous Conversion Mode` is `Disabled`.
    - `End of Conversion Selection` is `End of sequence of conversion`.
    - `Conversion Data Management Mode` is `DMA Circular Mode`.
    - `External Trigger Conversion Edge` is `None`.

