	AREA test, CODE, READONLY
	
	export asm_dot                  ; label must be exported if it is to be used as a function in C
asm_dot

	PUSH{R4, R5}                    ; saving context according to calling convention
	
loop
	SUBS R2, R2, #1                 ; loop counter (N = N-1)
	BLT done                        ; loop has finished?
	ADD R4, R0, R2, LSL #2          ; creating base address for the next element in the first array
	ADD R5, R1, R2, LSL #2          ; creating base address for the next element in the second array
	VLDR.f32 S0, [R4]               ; load next element of first array into S0
	VLDR.f32 S1, [R5]               ; load next element of second array into S1
	VMLA.f32 S2, S0, S1             ; dot product += S0*S1
	VMRS APSR_nzcv, FPSCR           ; need to move the FPU status register to achive floating point conditional execution
	B loop                          ; loop
	
done
	VSTR.f32 S2, [R3]               ; store the dot product in the pointer (float *a_dot_product) that was provided
	
	POP{R4, R5}                     ; restore context
	BX LR                           ; return
	
	END