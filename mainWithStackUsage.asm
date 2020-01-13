	;; for AVR: atmega16 (no external occilator used for the clock.)
	;; We are running at 1MHz...

	
	.nolist			; Don't include in the .lst file if we ask the
				; assembler to generate one.
	.include "./include/m16Adef.inc"


	;; =====================================================================
	;; Interrupt vector
	jmp   RESET	; Reset Handler
	jmp   EXT_INT0	; IRQ0 Handler
	jmp   EXT_INT1	; IRQ1 Handler
	jmp   TIM2_COMP	; Timer2 Compare Handler
	jmp   TIM2_OVF	; Timer2 Overflow Handler
	jmp   TIM1_CAPT	; Timer1 Capture Handler
	jmp   TIM1_COMPA	; Timer1 CompareA Handler
	jmp   TIM1_COMPB	; Timer1 CompareB Handler
	jmp   TIM1_OVF	; Timer1 Overflow Handler
	jmp   TIM0_OVF	; Timer0 Overflow Handler
	jmp   SPI_STC	; SPI Transfer Complete Handler
	jmp   USART_RXC	; USART RX Complete Handler
	jmp   USART_UDRE	; UDR Empty Handler
	jmp   USART_TXC	; USART TX Complete Handler
	jmp   ADC	; ADC Conversion Complete Handler
	jmp   EE_RDY	; EEPROM Ready Handler
	jmp   ANA_COMP	; Analog Comparator Handler
	jmp   TWSI	; Two-wire Serial Interface Handler
	jmp   EXT_INT2	; IRQ2 Handler
	jmp   TIM0_COMP	; Timer0 Compare Handler
	jmp   SPM_RDY	; Store Program Memory Ready Handler;


	;; =====================================================================
RESET:
	ldi	r16, high(RAMEND)	; Main program start
	out	SPH, r16	; Set Stack Pointer to top of RAM
	ldi	r16, low(RAMEND)
	out	SPL, r16
	; Stack set up now perform the remaining initialisation tasks
	call	INIT
CONTINUE:
	call	ADJUST
	call	KEEP_TIME
	call	DISPLAY_TIME
	rjmp	CONTINUE


	;; =====================================================================
INIT:
	call	DISABLE_JTAG
	clr	secondCount	; Clear the regs we store time in
	clr	minuteCount
	clr	hourCount

	ldi	r16, low(ddrSecondPins)
	out	DDRD, r16	; Set the Data Direction Register for Port D (seconds)
	out	PortD, secondCount ; Set second pins low

	ldi	r16, low(ddrMinutePins)
	out	DDRC, r16	; Set the Data Direction Register for port C (minutes)
	out	PortC, minuteCount ; Set minute pins low

	; Port A is shared between hours and the time adjustment indicator LEDs
	ldi	r16, low(ddrHourAndAdjustLEDIndicatorPins)
	out	DDRA, r16	; Set the Data Direction Register for port A (hours)
	out	PortA, hourCount	; Set hour pins low

	ldi	r16, low(ddrAdjustPins)
	;; Set the Data Direction Register for Port B (time adjustment controls
	;; and time signal)
	out	DDRB, r16	; Also note that PB2 is used for our time signal
	ldi	r16, low(adjustPinsPullups)
	; Set pull up's high (except for PB2 (time signal) which is active high)
	out	PortB, r16
	
	ldi	r16, low(int2En)
	out	GICR, r16	; Set external interrupt 2 to be enabled
	sei			; Enable interrupts
	ret
