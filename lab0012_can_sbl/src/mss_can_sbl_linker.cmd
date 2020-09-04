/*----------------------------------------------------------------------------*/
/* sbl_linker.cmd                                                                */
/*                                                                            */
/* (c) Texas Instruments 2018, All rights reserved.                           */
/*                                                                            */

/* USER CODE BEGIN (0) */
/* USER CODE END */


/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/* Section Configuration                                                      */
SECTIONS
{
    tcmalibs: > PROG_RAM ALIGN(8)
    {
		main.obj
        -lboot.aer4ft(.text)
    }
    systemHeap : {}  > DATA_RAM
    _appVecs    : > VECTORS LOAD_START(_appVecs)
}
