/*----------------------------------------------------------------------------*/
/* xwr_r4f.cmd                                                                */
/*                                                                            */
/* (c) Texas Instruments 2018, All rights reserved.                           */
/*                                                                            */

/* USER CODE BEGIN (0) */
/* USER CODE END */


/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Memory Map                                                                 */
MEMORY{
    VECTORS  (X)  : origin=0x00000000 length=0x00000100
    PROG_RAM (RWX) : origin=0x00000100 length=0x0003FF00
    DATA_RAM (RWX) : origin=0x08000000 length=0x00030000
}

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS{
    .intvecs : {} > VECTORS
    .text    : {} > PROG_RAM ALIGN(8)  run = DATA_RAM, LOAD_START(_libLoadStart), LOAD_END(_libLoadEnd), RUN_START(_libRunAddr)
    .const   : {} > PROG_RAM ALIGN(8)  run = DATA_RAM, LOAD_START(_constLoadStart), LOAD_END(_constLoadEnd), RUN_START(_constRunAddr)
    .cinit   : {} > PROG_RAM ALIGN(8)
    .pinit   : {} > PROG_RAM ALIGN(8)
    .bss     : {} > DATA_RAM ALIGN(8)
    .data    : {} > DATA_RAM
    .stack   : {} > DATA_RAM ALIGN(32)

}
/*----------------------------------------------------------------------------*/

