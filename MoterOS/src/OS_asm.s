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


  
  EXPORT  SVC_Handler
	EXPORT PendSV_Handler
	EXTERN cur_tcb
	EXTERN next_tcb
    EXTERN save_ctx_global
    EXTERN ticks_since_boot
	;EXTERN disableTimeget
	;EXTERN enableTimeget
  EXTERN C_SVC_handler

   
  EXPORT	TEST_OS_Id
  EXPORT TEST_OS_Sleep
  EXPORT	TEST_OS_Kill
  EXPORT	TEST_OS_Time
  EXPORT	TEST_OS_AddThread
  EXPORT setR9
    
TEST_OS_Id
	SVC		#0
	BX		LR

TEST_OS_Kill
	SVC		#1
	BX		LR

TEST_OS_Sleep
	SVC		#2
	BX		LR

TEST_OS_Time
	SVC		#3
	BX		LR

TEST_OS_AddThread
	SVC		#4
	BX		LR





PendSV_Handler
	CPSID IF ; Disable interrupts

;	PUSH {R0-R3,R14}
;	BL disableTimeget
;	POP {R0-R3,R14}

    LDR R0, =cur_tcb ; RO <= &cur_tcb
    LDR R1, [R0]     ; R1 <= cur_tcb
    ; Check if we should save context
    LDR R2, =save_ctx_global
    LDR R2, [R2]
    CBZ R2, Load_Ctx
Save_Ctx
		PUSH {R4-R11}
    ; Update SP to next task
    STR SP, [R1]    ; cur_tcb->sp = SP
Choose_Next_Task
;    LDR R3, =ticks_since_boot
;    LDR R3, [R3]
    LDR R6, =next_tcb ; R6 <= &next_tcb

    LDR R1, [R6]     ; R1 <= next_tcb
    STR R1, [R0]    ; cur_tcb <= next_tcb	
    LDR R5, [R1,#4] ; R5 <= cur_tcb->next
    STR R5, [R6]    ; next_tcb = cur_tcb->next
 
;    LDR R2, [R0,#8] ; R2 <= cur_tcb->wake_time
;    SUBS R3, R2, R3 ; R3 <= wake_time - ticks_since_boot
;    BGT Choose_Next_Task
Load_Ctx
    LDR SP, [R1]    ; SP <= cur_tcb->sp
    POP {R4-R11}

;    PUSH {R0-R3,R14}
;    BL enableTimeget
;    POP {R0-R3,R14}

    CPSIE IF ; Enable interrupts
    BX LR
    
SVC_Handler
	CPSID IF ; Disable interrupts
  LDR R0,[SP,#24] ; Return address
  LDRH R0,[R0,#-2] ; SVC instruction is 2 bytes
  BIC R0,#0xFF00 ; Extract ID in R0
  MOV R1, SP
  PUSH {LR}
  BL C_SVC_handler
  POP {LR}
  STR R0,[SP] ; Store return value
  CPSIE IF ; Enable interrupts
  BX LR ; Return from exception
  
setR9
  MOV R9, R0
  BX LR

  END