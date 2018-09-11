	AREA text, CODE, READONLY
	export sma_asm
sma_asm

// ALERT ALERT THIS HAS NOT BEEN DEBUGGED AT ALL, WROTE AS AN EXERCISE

// R0 = input pointer
// R1 = output pointer
// R2 = length of array
// R3 = depth for calculation

	MOV R4, R3					// copy of depth for odd/even verification
	PUSH {R0}
	PUSH {R1}
	PUSH {R2}
	PUSH {R3}

LOOP:
	AND R4, R4, #00000001		// clear all digits except the rightmost
	CMP R4, #1
	BEQ ODD 					// if the last digit was a 1, D is odd

EVEN:
	MOV R5, #0			// outer loop counter
	MOV R8, R2 			// R8 is 4x N (used to stop iteration)
	LSL R8, #2

EVEN_OUTER_LOOP:
	CMP R5, R8
	BGT	ENDS			// if we've reached the end of the array, exit
	LDR R6, [R0, R5]	// R6 is working sum. Move x[n] into sum using base register (R0) + current shift (R5)

	MOV R10, R3 		// store D/2 for comparison later
	LSR R10, #1

	SUB R9, R5, R10		// i - D/2
	CMP R9, #0
	BLT SKIP 			// if (i - D/2) < 0, skip a few lines
	LDR R9, [R0, R9]	// load up x[i - D/2]
	ADD R6, R6, R9		// sum += s[1 - D/2]

SKIP:
	MOV R9, #1			// inner loop counter

EVEN_INNER_LOOP:
	CMP R9, R10
	BGE EVEN_END_INNER_LOOP	// if inner counter surpasses D/2, leaves
	MOV R11, R5			// upper_index = i*4
	LSL R11, #2			// upper_index = i
	ADD R11, R11, R9	// upper_index = i + j
	CMP R11, R2			
	BGE EVEN_HIGH		// if upper_index is out of bounds, skip
	LSR R11, #2			// 4x(i+j) = offset
	LDR R11, [R0, R11] 	// load upward-shifted input value into R11
	ADD R6, R6, R11		// add upper value to sum

EVEN_HIGH:
	MOV R11, R5			// upper_index = i*4
	LSL R11, #2			// upper_index = i
	SUB R11, R11, R9	// upper_index = i - j
	CMP R11, #0			
	BLT EVEN_LOW		// if upper_index is out of bounds, skip
	LSR R11, #2			// 4x(i-j) = offset
	LDR R11, [R0, R11] 	// load upward-shifted input value into R11
	ADD R6, R6, R11		// add upper value to sum

EVEN_LOW:
	ADD R9, R9, #1		// increment counter
	B EVEN_INNER_LOOP

EVEN_END_INNER_LOOP:
	// TODO divide R6 by D, forget how to do that except when dividing by powers of 2
	STR R6, [R1, R5]	// store result to output (shifted by appropriate counter)
	ADD R5, R5, #4
	B EVEN_OUTER_LOOP

ODD:
	MOV R5, #0			// outer loop counter
	MOV R8, R2 			// R8 is 4x N (used to stop iteration)
	LSL R8, #2

ODD_OUTER_LOOP:
	CMP R5, R8
	BGT	ENDS			// if we've reached the end of the array, exit
	LDR R6, [R0, R5]	// R6 is working sum. Move x[n] into sum using base register (R0) + current shift (R5)

	MOV R9, #1			// inner loop counter
	MOV R10, R3 		// store D/2 for comparison later
	LSR R10, #1

ODD_INNER_LOOP:
	CMP R9, R10
	BGT ODD_END_INNER_LOOP	// if inner counter surpasses D/2, leaves
	MOV R11, R5			// upper_index = i*4
	LSL R11, #2			// upper_index = i
	ADD R11, R11, R9	// upper_index = i + j
	CMP R11, R2			
	BGE ODD_HIGH		// if upper_index is out of bounds, skip
	LSR R11, #2			// 4x(i+j) = offset
	LDR R11, [R0, R11] 	// load upward-shifted input value into R11
	ADD R6, R6, R11		// add upper value to sum

ODD_HIGH:
	MOV R11, R5			// upper_index = i*4
	LSL R11, #2			// upper_index = i
	SUB R11, R11, R9	// upper_index = i - j
	CMP R11, #0			
	BLT ODD_LOW		// if upper_index is out of bounds, skip
	LSR R11, #2			// 4x(i-j) = offset
	LDR R11, [R0, R11] 	// load upward-shifted input value into R11
	ADD R6, R6, R11		// add upper value to sum

ODD_LOW:
	ADD R9, R9, #1		// increment counter
	B ODD_INNER_LOOP

ODD_END_INNER_LOOP:
	// TODO divide R6 by D, forget how to do that except when dividing by powers of 2
	STR R6, [R1, R5]	// store result to output (shifted by appropriate counter)
	ADD R5, R5, #4
	B ODD_OUTER_LOOP

ENDS:

	POP {R3}
	POP {R2}
	POP {R1}
	POP {R0}
	BX LR
	
END
