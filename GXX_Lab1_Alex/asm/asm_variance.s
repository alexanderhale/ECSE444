	AREA test, CODE, READONLY
	
	export asm_variance             ; label must be exported if it is to be used as a function in C
asm_variance

	PUSH {R4, R5}                   ; saving context according to calling convention
	PUSH {R1}						; save to refresh the loop counter
	
loop_one
	SUBS R1, R1, #1                 ; loop counter (N = N-1)
	BLT out                         ; loop has finished?
	ADD R4, R0, R1, LSL #2          ; creating base address for the next element in the array
	VLDR.f32 S1, [R4]               ; load next element of array into S1
	VADD.f32 S0, S0, S1             ; add array element to the running total (stored in S0)
	VMRS APSR_nzcv, FPSCR           ; need to move the FPU status register to achive floating point conditional execution
	B loop_one                      ; loop
	
out
	POP {R1}						; refresh loop counter
	VMOV.f32 S3, R1					; load length of array into floating point form
	PUSH {R1}
	VDIV.f32 S0, S3					; divide total by number of elements to get the average, stored in S0

loop_two
	SUBS R1, R1, #1					; loop counter (N = N-1)
	BLT out_two                     ; loop has finished?
	ADD R4, R0, R1, LSL #2          ; creating base address for the next element in the array
	VLDR.f32 S1, [R4]               ; load next element of array into S1
	VSUB.f32 S1, S1, S0				; array element - average
	VMLA.f32 S2, S1, S1				; variance += (array element - average)^2  (variance stored in S2)
	B loop_two						; loop

out_two
	POP {R1}
	VMOV.f32 S3, R1					; load length of array into floating point form
	VDIV.f32 S2, S3			     	; divide accumulated value by number of elements to get the variance, stored in S2
	VSTR.f32 S2, [R2]               ; store the dot product in the pointer (float *a_variance) that was provided
	
	POP {R4, R5}                    ; restore context
	BX LR                           ; return
	
	END