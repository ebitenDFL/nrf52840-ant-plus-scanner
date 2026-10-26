/* ANT Scanner - nRF52840 + S340 6.1.x
 * Based on Nordic's S340 nRF52840 GCC linker memory layout.
 */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x31000, LENGTH = 0xcf000
  RAM (rwx)  : ORIGIN = 0x20002000, LENGTH = 0x3e000
}

/* SoftDevice Handler observer tables. */
SECTIONS
{
  .sdh_soc_observers :
  {
    PROVIDE(__start_sdh_soc_observers = .);
    KEEP(*(SORT(.sdh_soc_observers*)))
    PROVIDE(__stop_sdh_soc_observers = .);
  }

  .sdh_ant_observers :
  {
    PROVIDE(__start_sdh_ant_observers = .);
    KEEP(*(SORT(.sdh_ant_observers*)))
    PROVIDE(__stop_sdh_ant_observers = .);
  }

  .sdh_state_observers :
  {
    PROVIDE(__start_sdh_state_observers = .);
    KEEP(*(SORT(.sdh_state_observers*)))
    PROVIDE(__stop_sdh_state_observers = .);
  }

  .sdh_stack_observers :
  {
    PROVIDE(__start_sdh_stack_observers = .);
    KEEP(*(SORT(.sdh_stack_observers*)))
    PROVIDE(__stop_sdh_stack_observers = .);
  }

  .sdh_req_observers :
  {
    PROVIDE(__start_sdh_req_observers = .);
    KEEP(*(SORT(.sdh_req_observers*)))
    PROVIDE(__stop_sdh_req_observers = .);
  }

  .mem_section_dummy_rom :
  {
  }
} INSERT AFTER .text;

/* Keep Nordic's dummy RAM section expected by SDK tooling. */
SECTIONS
{
  . = ALIGN(4);
  .mem_section_dummy_ram :
  {
  }
} INSERT AFTER .data;

INCLUDE "nrf_common.ld"
