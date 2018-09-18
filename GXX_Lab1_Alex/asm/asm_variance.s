	AREA test, CODE, READONLY
	
	export asm_variance             ; label must be exported if it is to be used as a function in C
asm_variance

	PUSH {R1-R5}                    ; saving context according to calling convention
	VMOV.f32 S3, R1					; load length of array into floating point form
	MOV R5, #-1						; initialize loop counter
	
loop_one
	ADD R5, R5, #1                  ; loop counter (N = N+1)
	CMP R5, R1
	BGT out                         ; loop has finished?
	ADD R4, R0, R5, LSL #2          ; creating base address for the next element in the array
	VLDR.f32 S1, [R4]               ; load next element of array into S1
	VADD.f32 S0, S0, S1             ; add array element to the running total (stored in S0)
	B loop_one                      ; loop
	
out
	MOV R6, #-1						; refresh loop counter
	VDIV.f32 S0, S3					; divide total by number of elements to get the average, stored in S0

loop_two
	ADD R5, R5, #1                  ; loop counter (N = N+1)
	CMP R5, R1
	BGT out_two                     ; loop has finished?
	ADD R4, R0, R5, LSL #2          ; creating base address for the next element in the array
	VLDR.f32 S1, [R4]               ; load next element of array into S1
	VSUB.f32 S1, S1, S0				; array element - average
	VMLA.f32 S2, S1, S1				; variance += (array element - average)^2  (variance stored in S2)
	B loop_two						; loop

out_two
	VDIV.f32 S2, S3			     	; divide accumulated value by number of elements to get the variance, stored in S2
	VSTR.f32 S2, [R2]               ; store the dot product in the pointer (float *a_variance) that was provided
	
	POP {R1-R5}                     ; restore context
	BX LR                           ; return
	
	END