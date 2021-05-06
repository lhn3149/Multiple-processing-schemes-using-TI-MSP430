;-------------------------------------------------------------------------------
; MSP430 Assembler Code Template for use with TI Code Composer Studio
;******************************************************************************
; MSP430G2553 Demo - USI-UART, 9600 Echo ISR, ~1 MHz SMCLK
;
; Description: Echo a received character, RX ISR used.
; USI_RX interrupt triggers TX Echo.
; Default SMCLK = DCOCLK ~= 1.05 MHz
; Baud rate divider with SMCLK @9600 = 1MHz/9600 = 104.15
; Original functionality by M. Buccini / G. Morton
; Texas Instruments Inc., May 2005
; Built with Code Composer Essentials Version: 1.0
; Adapted for DB365 by Dorin Patru 05/14/08; updated May 2011
; Upgraded for LaunchPad and CCS 5.4 by Dorin Patru December 2013

; left right scale (logarithmic, Linear)
; up down frequency (100Hz, 200Hz)
; sampling 100Hz, 200Hz, user input
; use UART to display on the terminal

;******************************************************************************

;
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file
			; declare C functions and variables
            .global setup
            .global waitForCenter
            .global waitForUpDown
            .global waitForLeftRight
            .global getSamples
            .global convertSamples
            .global displaySamples
			.global UART_samples


;-------------------------------------------------------------------------------
;Function Definitions
;-------------------------------------------------------------------------------
setup:
SWdelay		.equ	0x0006	; delay value used by the SW timer

			.bss	meas_base, 10			;; 2 bytes for each of 5 sensors
			.bss	meas_latest, 10			;; 2 bytes for each of 5 sensors
			.bss	processed,40	;; 2 bytes for 20 samples
			.bss	sensor_status, 1		; keep track of which LED is on, 1 bit -> 1 LED
;------sensor status-----------;
; Center:	0010 0000	0x20
; Up :		0001 0000	0x10
; Right:	0000 1000	0x08
; Down :	0000 0100	0x04
; Left :	0000 0010	0x02
;-----------------;
;--------LED turn on code---------;
; Center:	0x01
; LED 1 :	0x10
; LED 2	:	0x20
; LED 3	:	0x40
; LED 4	:	0x80
; LED 5	:	0xe8
; LED 6	:	0xd8
; LED 7	:	0xb8
; LED 8	:	0x78
;-------------------------------;
			.bss	conversion, 1				; log/linear conversion
			.bss 	period_check,1 			; 	interrupt flag
			.bss	hwdelay, 2				; determine how much delay (0.2s/0.5s)

			.text
            .retain                         ; Override ELF conditional linking
                                            ; and retain current section.
            .retainrefs                     ; And retain any sections that have
                                            ; references to current section.

;-------------------------------------------------------------------------------

				bis.b	#0xfe, &P1DIR	; set up P1 as outputs except for 1.0
				bic.b	#0xff, &P1OUT	; P1 outputs off
				call 	#meas_base_val 	; record base value (threshold) for non-touched sensor

				clr &TA0CTL			; Turn off TimerA0
				clr &TA1CTL			; turn off TimerA1
				clr	&P1OUT		; clear leds
				clr r5
				clr r7
				clr r8
				clr r15
				clr sensor_status
				clr period_check

clruart			clr UART_samples(r5); clear UART_samples array
				add.w #2, r5
				cmp.w #0x28, r5 ; 0x28 = 40
				jl clruart

;//ret
clrmeas			clr meas_latest(r5); clear meas_latest array
				add.w #2, r5
				cmp.w #0xa, r5
				jl clrmeas
				clr r5
				ret

;------------------------------------------------------------------------------------
waitForCenter:	mov.b	#0x01, &P1OUT
				call 	#SWtimer2
				cmp.b	#0x20, sensor_status	; center sensor
				jne		NotCenter				;
				ret								; if pressed, return
NotCenter		call #meas_latest_val			; if not pressed, check again
				call #det_sensor
				jmp waitForCenter
;----------------------------------------------------------------------------------------
waitForUpDown:	mov.b	#0x90, &P1OUT			; turn on LEDs 1 & 4
				call 	#SWtimer2
				mov.b	#0x68, &P1OUT			; turn on LEDs 5 & 8
				call 	#SWtimer2

				cmp.b	#0x10, sensor_status	; up ?
				jne		NotUp
				mov.w	#0x61A8, &hwdelay		; 0.2s delay for up
				ret								; divide SMCLK 1MHz by 8 (125k) -> TA0CCR0 25k (61A8) for 0.2s,  62.5k (F424)  for 0.5

NotUp			cmp.b	#0x04, sensor_status	; down ?
				jne		NotDown
				mov.w	#0xF424, &hwdelay		; 0.5s delay for DOWN ?
				ret
NotDown			call 	#meas_latest_val		; check again
				call 	#det_sensor
				jmp		waitForUpDown
;-------------------------------------------------------------
waitForLeftRight:
				mov.b	#0x60, &P1OUT			; turn on LEDs 2 & 3
				call 	#SWtimer2
				mov.b	#0x98, &P1OUT			; turn on LEDs 6 & 7
				call	#SWtimer2
				cmp.b	#0x02, sensor_status	; left?
				jne		NotLeft
				mov.b	#0, &conversion			; log
				ret

NotLeft			cmp.b	#0x08, sensor_status	; right?
				jne		NotRight
				mov.b	#0x1, &conversion		; linear
				ret

NotRight		call 	#meas_latest_val		; check again
				call 	#det_sensor
				jmp		waitForLeftRight
;---------------------------------------------------------------------------------------------
convertSamples:	clr 	r5
				clr 	r6
				cmp.b 	#0x1, &conversion 	; linear ?
				jne		Log 				; otherwise, log
Linear			mov.w	UART_samples(r5), processed(r5)
				rra.w	processed(r5) 		; rotate right 7 times: 3 first bit now 3 last bit
				rra.w	processed(r5)
				rra.w	processed(r5)
				rra.w	processed(r5)
				rra.w	processed(r5)
				rra.w	processed(r5)
				rra.w	processed(r5)
				mov.w	processed(r5), r6
				bic.w 	#0xfff8, r6
				inc		r6

				mov.w 	r6, processed(r5) ; 1 start at 000
				add.w	#2, r5
				cmp.w	#0x28, r5 		; check if 20 datas in UART_samples have been checked
				jl		Linear
				ret

Log				clr 	r13				; data placeholder
				clr		r14				; value to check if we get the first "1"
				clr 	r15				; processed value 0-8
				mov.b  	#0x08, r15		; 0000 1000
				mov.w	#0x7fff, r14	; 0111 1111
				mov.w	UART_samples(r5), processed(r5)
				rlc.w	processed(r5) 	; rotate right 6 times: bring mattered bits to front
				rlc.w	processed(r5)
				rlc.w	processed(r5)
				rlc.w	processed(r5)
				rlc.w	processed(r5)


CompCarry		rlc.w	processed(r5)
				mov.w 	processed(r5),r13
				bic.w 	r14, r13
				cmp.w	#0x8000, r13
				jeq		Processed
				dec.b	r15
				clr 	r13
				jmp 	CompCarry

Processed		clr		processed(r5)
				mov.b	r15, processed(r5)
				add.w	#2, r5
				cmp.w	#0x28, r5		; Check if 20 datas in UART_samples have been checked
				jl		Log
				nop
				ret
;----------------------------------------------------------------------------
displaySamples:	mov.w &hwdelay, &TA1CCR0	; 10ms delay -> TA0CCR0: 10000d = (1MHz*10 ms) ~ 2710h
				mov.w #(MC_1 + ID_3 + TASSEL_2), &TA1CTL ; divide SMCLK 1MHz by 8 (125k) -> TA0CCR0 25k (61A8) for 0.5s,  62.5k (F424)  for 0.5
				mov.w #(CCIE), &TA1CCTL0 ; enable CCIE interrupt
				mov.b #0x20,&UCA0TXBUF
				clr	sensor_status
				clr &P1OUT
				clr r7
				clr r8

dispLoop		eint
				clr &period_check
TX				mov.b processed(r7),r8
				add.b #0x30,r8					; make number in ASCII format to print
				mov.b r8,&UCA0TXBUF				; print the number in ascii ; when HW delay, it doesnt print out

;------- sensor status-----------;
; Center:	0010 0000	0x20
; Up :		0001 0000	0x10
; Right:	0000 1000	0x08
; Down :	0000 0100	0x04
; Left :	0000 0010	0x02
;-----------------;
;--------LED turn on code---------;
; Center:	0x01
; LED 1 :	0x10
; LED 2	:	0x20
; LED 3	:	0x40
; LED 4	:	0x80
; LED 5	:	0xe8
; LED 6	:	0xd8
; LED 7	:	0xb8
; LED 8	:	0x78
;-------------------------------;


centercheck		call #meas_latest_val			; center pressed?
				call #det_sensor
				cmp.b	#0x20, sensor_status
				jeq	dispfini					; end display

processed0		cmp.w #0x00, processed(r7)
				jeq noled
processed4		mov.b #0x80, &P1OUT
				cmp.w #0x01, processed(r7)
				jeq	dispcheck
processed3		mov.b #0xc0, &P1OUT
				cmp.b #0x02, processed(r7)
				jeq	dispcheck
processed2		mov.b #0xe0, &P1OUT
				cmp.b #0x03, processed(r7)
				jeq	dispcheck
processed1		mov.b #0xf0, &P1OUT
				cmp.b #0x04, processed(r7)
				jeq	dispcheck

processed5		call #SWtimer2
				mov.b #0xe8, &P1OUT
				cmp.b #0x05, processed(r7)
				jeq dispcheck

processed6		mov.b #0xc8, &P1OUT
				cmp.b #0x06, processed(r7)
				jeq dispcheck

processed7		mov.b #0x88, &P1OUT
				cmp.b #0x07, processed(r7)
				jeq dispcheck

processed8		mov.b #0x08, &P1OUT
				cmp.b #0x08, processed(r7)
				jeq dispcheck

noled			bic.b #0xff, &P1OUT				; clear all LEDs
				jmp dispcheck

dispcheck		tst.b &period_check 		; standby between hwdelay
				jz centercheck				; check if done 0.2s 0.5s

				add.w #2, r7					; if 0.2s/0.5 has passed, go to next converted_samples
				cmp.w #0x28, r7					; check if 20 values have been displayed
				jne dispLoop

dispfini		bic.b #0xff, &P1OUT		; clear all LEDs
				ret								;

;########################### Delay and Checking values of touched sensors #################################;
;########################### Grab from Lab 8 for checking touch position ################################## ;
;-------------------------------------------------------------------------------
; Measure base line values routine
;-------------------------------------------------------------------------------
meas_base_val:	mov.b	#0x02, R5	; initialize R5 to point to P2.x
				mov.b	#0x00, R6	; initialize R6 to the base of meas_base
meas_base_again:call #meas_setup	;
;-------------------------------------------------------------------------------
; Clear TAR and start TA0 in continuous mode; use BIS and not MOV
; so that you don't cancel previous settings
;-------------------------------------------------------------------------------
			bis #MC_2 + TACLR, &TA0CTL 	;
;-------------------------------------------------------------------------------
; Call the SW delay routine, which here it is used to provide the accumulation
; period; could use instead ACLK fed from VLO
;-------------------------------------------------------------------------------
			call #SWtimer			;
;-------------------------------------------------------------------------------
; Now, after the accumulation period has passed, generate a SW based
; capture trigger by toggeling CCIS0
;-------------------------------------------------------------------------------
			xor	#CCIS0, &TA0CCTL1	;
;-------------------------------------------------------------------------------
; Save the baseline captured value in meas_base array
;-------------------------------------------------------------------------------
			mov	TA0CCR1, meas_base(R6)	; note the use of the SYMBOLIC AM
			bic #MC1+MC0, &TA0CTL 	; Stop TA
			sub #5, meas_base(R6)	; Adjust this baseline
			bic.b 	R5,&P2SEL2		; Stop the oscillation on the latest. pin
			rla.b	R5				; Prepare next x
			add.b	#0x02, R6		; Prepare the next index into the array
			cmp.b	#0x40, R5		; Check if done with all five sensors
			jne		meas_base_again	;
			ret						;
;-------------------------------------------------------------------------------
; Measure latest values routine
;-------------------------------------------------------------------------------
meas_latest_val:
			mov.b	#0x02, R5	; initialize R5 to point to P2.1
			mov.b	#0x00, R6		; initialize R6 to the base of meas_base
meas_latest_again:
			call #meas_setup	;
			bis #MC_2 + TACLR, &TA0CTL 	; Continuous, Clear TAR
			call #SWtimer			;
			xor #CCIS0, &TA0CCTL1	; Trigger SW capture
			mov TA0CCR1, meas_latest(R6)	; Save captured value in array
			bic #MC1+MC0, &TA0CTL 	; Stop timer
			bic.b 	R5,&P2SEL2		; Stop the oscillation on the latest. pin
			rla.b	R5				; Prepare next x
			add.b	#0x02, R6		; Prepare the next index into the array
			cmp.b	#0x40, R5		; Check if done with all five sensors
			jne		meas_latest_again	;
			ret						;

meas_setup:	bic.b R5,&P2DIR 		; P2.x input
			bic.b R5,&P2SEL 	;
			bis.b R5,&P2SEL2	;
		 	mov #TASSEL_3, &TA0CTL 	;; The oscillation from P2.x is driving INCLK input of TA0
									; No division of this clock source
			mov #CM_3 + CCIS_2 + CAP, &TA0CCTL1 	;; Setting up to capture the value of TAR on either rising or falling edges
													; using SW based trigger
			ret						;

;-------------------------------------------------------------------------------
; Determine which sensor was pressed routine
;-------------------------------------------------------------------------------
det_sensor:	mov.b #11, sensor_status
			clr.b	sensor_status	;
			mov.b	#0x02, R5		; initialize R5 to point to P2.1    0000 0010
			mov.b	#0x00, R6		; initialize R6 to the base of meas_base
CheckNextSensor:
			cmp	meas_latest(R6), meas_base(R6)	;
			jl	NotThisSensor		;
			bis.b	R5, sensor_status	; Update sensor_status
NotThisSensor:
			rla.b	R5				; Prepare next x, check rotation: Center-Down-Left-Up-Right
			add.b	#0x02, R6		; Prepare the next index into the array
			cmp.b	#0x40, R5		; Check if done with all 5 sensors 0100 0000
			jne		CheckNextSensor	;
			ret						;

;------------------------------------------------
; ---------- 2 Timers ------------
;------------------------------------------------

SWtimer:	mov	#SWdelay, r9
Reloadr5	mov	#SWdelay, r11
ISr50		dec	r11
			jnz	ISr50
			dec	r9
			jnz	Reloadr5
			ret

SWtimer2:	mov	#0xf, r9
delay1		mov	#0xf, r11
delay2		dec	r11
			jnz	delay2
			dec	r9
			jnz	delay1
			ret

TIMER_ISR1:
			nop
			mov.b #0x01, &period_check
			reti
;-------------------------------------------------------------------------------
; Stack Pointer definition
;-------------------------------------------------------------------------------
            .global __STACK_END
            .sect   .stack

;-------------------------------------------------------------------------------
; Interrupt Vectors

			.sect	".int13"			; TimerA1
isr_timer1:	.short	TIMER_ISR1

.end

