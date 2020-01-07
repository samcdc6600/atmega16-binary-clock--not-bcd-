	;; for AVR: atmega16 (no external occilator used for the clock.)

	.nolist			; Don't include in the .lst file if we ask the
				; assembler to generate one.
	.include "./include/m16Adef.inc"


	;; =====================================================================
	;; Definitions
	.def	secondCount = r17
	.def	minuteCount = r18
	.def	hourCount = r19

	.equ	minAndHour = 0b00111100
	.equ	day = 0b00010100
	.equ	ddrSecondPins = 0b00111111	; Pins PD0 - PD7 (PD0 - PD5 used)
	.equ	ddrMinutePins = 0b00111111	; Pins PC7 - PC0 (PC5 - PC0 used)
	.equ	ddrHourPins = 0b00011111	; Pins PA0 - PA7 (PA0 - PA5 used)
	.equ	inputPins = 0b00000000	; Pins PB0 - PB7 (PB0 ())

	
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

	
RESET:				  ;=============================================
	ldi	r16, high(RAMEND)	; Main program start
	out	SPH, r16	; Set Stack Pointer to top of RAM
	ldi	r16, low(RAMEND)
	out	SPL, r16
	; Stack set up now perform the reming initialisation tasks
	call	INIT
CONTINUE:
	call	KEEP_TIME
	call	DISPLAY_TIME
	call	ADJUST_TIME
	rjmp	CONTINUE


INIT:				;===============================================
	call	DISABLE_JTAG
	clr	secondCount	; Clear the regs we store time in
	clr	minuteCount
	clr	hourCount

	ldi	r16, low(ddrSecondPins)
	out	DDRD, r16	; Set the Data Direction Register for Port D (seconds)
	out	PortD, secondCount ; Set second pins low

	ldi	r16, low(ddrMinutePins)
	ldi	r16, 0b11111111
	out	DDRC, r16	; Set the Data Direction Register for port C (minutes)
	out	PortC, minuteCount ; Set minute pins low
	
	ldi	r16, low(ddrHourPins)
	out	DDRA, r16	; Set the Data Direction Register for port A (hours)
	out	PortA, hourCount	; Set hour pins low
	
	ldi	r16, 0b00100000
	out	GICR, r16	; Set external interrupt 2 to be enabled
	sei			; Enable interrupts
	ret


	;; DISABLE_JTAG dissables the JTAG interface (PC7 - PC2) in software
	;; within two cycles. This allows the use of the PC7 - PC2 pins as
	;; general IO while still allowing the chip to be programmed as we have
	;; not set any fuses. The manual says to set the JTD bit (bit 7
	;; according to the manual (pg 236) but that does not seem to work.)
	;; Post #3 on this site: https://www.avrfreaks.net/forum/how-disable-jtag-c
	;; said to use 0x80 and that does work.
	;; From the manual "In order to avoid unintentional disabling or
	;; enabling of the JTAG interface, a timed sequence must be followed
	;; when changing this bit: The application software must write this bit
	;; to the desired value twice within four cycles to change its value."
DISABLE_JTAG:			;===============================================
	ldi	r16, 0x80
	out	MCUCSR, r16
	out	MCUCSR, r16
	ret


KEEP_TIME:		      ;=================================================
	cpi	secondCount, low(minAndHour) ; Have we counted 60 seconds?
	brne	SUB_UNIT
	clr	secondCount	; Start counting the next minute
	inc	minuteCount
	
	cpi	minuteCount, low(minAndHour) ; Have we counted an hour?
	brne	SUB_UNIT
	clr	minuteCount	; start counting the next hour
	inc	hourCount

	cpi	hourCount, low(day) ; Have we counted a day?
	brne	SUB_UNIT
	clr	hourCount
SUB_UNIT:
	ret

	
DISPLAY_TIME:			;===============================================
	out	PortD, secondCount
	out	PortC, minuteCount
	out	PortA, hourCount
	ret


ADJUST_TIME:			;===============================================
	ret



EXT_INT0:	; IRQ0 Handler
EXT_INT1:	; IRQ1 Handler
TIM2_COMP:	; Timer2 Compare Handler
TIM2_OVF:	; Timer2 Overflow Handler
TIM1_CAPT:	; Timer1 Capture Handler
TIM1_COMPA:	; Timer1 CompareA Handler
TIM1_COMPB:	; Timer1 CompareB Handler
TIM1_OVF:	; Timer1 Overflow Handler
TIM0_OVF:	; Timer0 Overflow Handler
SPI_STC:	; SPI Transfer Complete Handler
USART_RXC:	; USART RX Complete Handler
USART_UDRE:	; UDR Empty Handler
USART_TXC:	; USART TX Complete Handler
ADC:		; ADC Conversion Complete Handler
EE_RDY:		; EEPROM Ready Handler
ANA_COMP:	; Analog Comparator Handler
TWSI:		; Two-wire Serial Interface Handler
EXT_INT2:	; IRQ2 Handler
				; Note that another second has passed
	inc	secondCount	; Time flies :O :'(
	reti
TIM0_COMP:	; Timer0 Compare Handler
SPM_RDY:	; Store Program Memory Ready Handler;
