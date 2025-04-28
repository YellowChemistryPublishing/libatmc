SECTIONS
{
  .dma_data : /* Space before ':' is critical */
  {
    *(.dma_data)
  } >RAM_D2
}