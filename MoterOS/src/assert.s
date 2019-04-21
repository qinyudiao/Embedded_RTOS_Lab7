;******************************************************************************
;
; Indicate that the code in this file preserves 8-byte alignment of the stack.
;
;******************************************************************************
  PRESERVE8

;******************************************************************************
;
; Place code into the reset code section.
;
;******************************************************************************
  AREA    RESET, CODE, READONLY
  THUMB

  EXPORT __aeabi_assert

__aeabi_assert
  BKPT #0     ; Trap into debugger

	END
