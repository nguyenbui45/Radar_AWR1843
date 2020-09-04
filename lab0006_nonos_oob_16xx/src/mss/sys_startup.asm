;-------------------------------------------------------------------------------
; sys_startup.asm
;
;   @brief
;     Startup assembly file for MSS core
;
;  \par
;  NOTE:
;      (C) Copyright 2016 Texas Instruments, Inc.
;
;  Redistribution and use in source and binary forms, with or without
;  modification, are permitted provided that the following conditions
;  are met:
;
;    Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
;
;    Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the
;    documentation and/or other materials provided with the
;    distribution.
;
;    Neither the name of Texas Instruments Incorporated nor the names of
;    its contributors may be used to endorse or promote products derived
;    from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
;  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
;  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
;  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
;  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
;  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
;  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
    .sect ".startup"
    .text
    .arm


    .ref vimIrqDispatcher

M_ARM_MODE_SYS  .equ    (0x1FU)

SYS_UDEFABT_DISP   .word     sys_udefAbtDisp
SYS_PREFABT_DISP   .word     sys_prefAbtDisp
SYS_DATAABT_DISP   .word     sys_dataAbtDisp
SYS_SWI_DISP       .word     sys_swiDisp


SYS_IRQ_CALL       .word     vimIrqDispatcher
;-------------------------------------------------------------------------------
; ARM Cortex R4 Reset handler - startup code
; SourceId :
; DesignId :
; Requirements:

    .def     _sysResetEntry_
    .asmfunc

_sysResetEntry_
        .global _c_int00
        bl    _c_int00

        ; Done, loop forever
LOOP0:
        b      LOOP0
        .endasmfunc

;-------------------------------------------------------------------------------
; Temporary exception infinite loop
; SourceId :
; DesignId :
; Requirements:

    .def     _sysLoop_
    .asmfunc

_sysLoop_
LOOP1:
        b      LOOP1
        .endasmfunc

;-------------------------------------------------------------------------------
; ARM undefined exception handler
; SourceId :
; DesignId :
; Requirements:

    .def     _sysUndefEntry_
    .asmfunc

_sysUndefEntry_
        srsdb    sp!, #M_ARM_MODE_SYS     ; Save LR_irq and SPSR_irq to System mode stack
        cps      #M_ARM_MODE_SYS          ; Switch to System mode
        push     {r0-r3, r12, lr}         ; Store normal scratch registers
        ldr      r0, SYS_UDEFABT_DISP
        bx       r0
        .endasmfunc

;-------------------------------------------------------------------------------
; ARM supervisory call handler
; SourceId :
; DesignId :
; Requirements:

    .def     _sysSvcEntry_
    .asmfunc

_sysSvcEntry_
        srsdb    sp!, #M_ARM_MODE_SYS     ; Save LR_irq and SPSR_irq to System mode stack
        cps      #M_ARM_MODE_SYS          ; Switch to System mode
        push     {r0-r3, r12, lr}         ; Store normal scratch registers
        ldr      r1, SYS_SWI_DISP
        bx       r1
        .endasmfunc

;-------------------------------------------------------------------------------
; ARM Prefetch exception handler
; SourceId :
; DesignId :
; Requirements:

    .def     _sysPrefetchEntry_
    .asmfunc

_sysPrefetchEntry_
        srsdb    sp!, #M_ARM_MODE_SYS     ; Save LR_irq and SPSR_irq to System mode stack
        cps      #M_ARM_MODE_SYS          ; Switch to System mode
        push     {r0-r3, r12, lr}         ; Store normal scratch registers
        ldr      r0, SYS_PREFABT_DISP
        bx       r0

        .endasmfunc

;-------------------------------------------------------------------------------
; ARM Data abort handler
; SourceId :
; DesignId :
; Requirements:

    .def     _sysDAbortEntry_
    .asmfunc

_sysDAbortEntry_
        srsdb    sp!, #M_ARM_MODE_SYS     ; Save LR_irq and SPSR_irq to System mode stack
        cps      #M_ARM_MODE_SYS          ; Switch to System mode
        push     {r0-r3, r12, lr}         ; Store normal scratch registers
        ldr      r0, SYS_DATAABT_DISP
        bx       r0

        .endasmfunc

;-------------------------------------------------------------------------------
; ARM IRQ interrupt handler
; SourceId :
; DesignId :
; Requirements:

       .def _sysIrqEntry_
_sysIrqEntry_:  .asmfunc

   sub      lr, lr, #4
   srsdb    sp!, #M_ARM_MODE_SYS     ; Save LR_irq and SPSR_irq to System mode stack
   cps      #M_ARM_MODE_SYS          ; Switch to System mode
   push     {r0-r3, r12, lr}         ; Store normal scratch registers
   ldr      r3, SYS_IRQ_CALL        ; passing params r0, r1, r2
   blx      r3
   pop      {r0-r3, r12, lr}         ; Restore registers
   rfeia    sp!                      ; Return using RFE from System mode stack

   .endasmfunc
.end


;-------------------------------------------------------------------------------
; ARM Undef instruction exception handler
; SourceId :
; DesignId :
; Requirements:

  .def sys_udefAbtDisp
sys_udefAbtDisp:  .asmfunc

    add      r2, sp, #24              ; get address of LR_abt
    ldmia    r2!, {r0, r1}            ; r0 - LR_abt, r1 - SPSR_abt, r2 - SP
    sub      r0, r0, #4               ; aborted instruction address
    blx      r3
    pop      {r0-r3, r12, lr}         ; Restore registers
    rfeia    sp!                      ; Return using RFE from System mode stack

  .endasmfunc

;-------------------------------------------------------------------------------
; ARM prefetch exception handler
; SourceId :
; DesignId :
; Requirements:

  .def sys_prefAbtDisp
sys_prefAbtDisp:  .asmfunc

    add      r2, sp, #24              ; get address of LR_abt
    ldmia    r2!, {r0, r1}            ; r0 - LR_abt, r1 - SPSR_abt, r2 - SP
    blx      r3
    pop      {r0-r3, r12, lr}         ; Restore registers
    rfeia    sp!                      ; Return using RFE from System mode stack
  .endasmfunc

;-------------------------------------------------------------------------------
; ARM data exception handler
; SourceId :
; DesignId :
; Requirements:

  .def sys_dataAbtDisp
sys_dataAbtDisp:  .asmfunc

    add      r2, sp, #24              ; get address of LR_abt
    ldmia    r2!, {r0, r1}            ; r0 - LR_abt, r1 - SPSR_abt, r2 - SP
    blx      r3
    pop      {r0-r3, r12, lr}         ; Restore registers
    rfeia    sp!                      ; Return using RFE from System mode stack
  .endasmfunc

;-------------------------------------------------------------------------------
; ARM Software Interrupt SVC handler
; SourceId :
; DesignId :
; Requirements:


  .def sys_swiDisp
sys_swiDisp:  .asmfunc

    mov      r1, #0
    cmp      r0, r1                   ; if svc num is 0 then switch arm mode
    beq      SWM
    blx      r3
    b        RTN
SWM:
    add      r2, sp, #28              ; get address of SPSR_svc
    ldr      r0, [r2]
    bic      r0, r0, #0x0000001F      ; clear mode bits
    orr      r0, r0, #M_ARM_MODE_SYS  ; modify spsr mode bits to system mode
    str      r0, [r2]
RTN:
    pop      {r0-r3, r12, lr}         ; Restore registers
    rfeia    sp!                      ; Return using RFE from System mode stack

  .endasmfunc


;/*
; * END OF sys_startup.asm FILE
; */

