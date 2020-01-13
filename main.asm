	;; for AVR: atmega16 (no external occilator used for the clock.)
	;; We are running at 1MHz...

	
	.nolist			; Don't include in the .lst file if we ask the
				; assembler to generate one.
	.include "./include/m16Adef.inc"


	;; =====================================================================
	;; Definitions
	.def	secondCount = r17
	.def	minuteCount = r18
	.def	hourCount = r19
	.def	currentlyAdjusting = r20
	;; Constants
 	.equ	minAndHour = 0b00111100
 	.equ	day = 0b00011000
	.equ	maxUnsignedByte = 0b11111111
	;; We set all pins for ports PA, PC and PD to outputs so we dont have to
	;; worry about pull ups to save power on the pins we are not actually
	;; using, that is assuming it works like that :)
	.equ	ddrSecondPins = 0b11111111	; Pins PD0 - PD7 (PD0 - PD5 used)
	.equ	ddrMinutePins = 0b11111111	; Pins PC7 - PC0 (PC5 - PC0 used)
	;; Pins PA0 - PA7 (PA0 - PA4 used for hours and PA5 - PA7 used for
	;; adjust LED Indicator Pins)
	.equ	ddrHourAndAdjustLEDIndicatorPins = 0b11111111
	;; For changing the time. Pins PB0 - PB7 (PB0, PB1, PB3 and PB4 used as
	.equ	ddrAdjustPins = 0b00100000 ; input and PB5 used as output
	;; Enable the pullups for PA (except for PA2 (as it is active high) and
	;; PA5 (we set this low as it is used to toggle master reset 1 (1mr) on
	;; the decade counter the ocsilator is connected to (it is active high))
	.equ	adjustPinsPullups = 0b11011011 ; ocsilator)
	.equ	adjustPinsPullupsMR1High = 0b11111011
	;; Pin outs for adjust LEDs
	.equ	adjustSecondsLEDIndicatorPin = 0b10000000
	.equ	adjustMinutesLEDIndicatorPin = 0b01000000
	.equ	adjustHoursLEDIndicatorPin = 0b00100000
	;; To dissable (mask out) adjust LEDS
	.equ	noAdjustLEDs = 0b00011111
	;; To mask out PB2 (INT2) and PA5 (1mr)
	.equ	noInt2Or1Mr = 0b11011011
	;; To enable int 2 (pin PB2)
	.equ	int2En = 0b00100000
	;; Adjust input masks
	.equ	adjustUpPin = 0b11011010
	.equ	adjustTimeUnitSelection = 0b11011001
	.equ	adjustDown = 0b11010011
	.equ	dissableAdjust = 0b11001011
	;; These variables should be > 0
	.equ	debounceFactor0 = 0b11111111
	.equ	debounceFactor1 = 0b11111000
	.equ	debounceFactor2 = 0b00000001

	
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
	; Stack set up now perform the remaining initialisation tasks
	call	INIT
CONTINUE:
	call	ADJUST
	call	KEEP_TIME
	call	DISPLAY_TIME
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


	;; DISABLE_JTAG dissables the JTAG interface (PC7 - PC2) in software.
	;; This allows the use of the PC7 - PC2 pins as general IO while still
	;; allowing the chip to be programmed as we have not set any fuses.
	;; The manual says to set the JTD bit (bit 7 according to the manual (pg
	;; 236) but that does not seem to work.) Post #3 on this site:
	;; https://www.avrfreaks.net/forum/how-disable-jtag-c
	;; said to use 0x80 and that does work. From the manual "In order to
	;; avoid unintentional disabling or enabling of the JTAG interface, a
	;; timed sequence must be followed when changing this bit: The
	;; application software must write this bit to the desired value twice
	;; within four cycles to change its value."
DISABLE_JTAG:			;===============================================
	ldi	r16, 0x80
	out	MCUCSR, r16
	out	MCUCSR, r16
	ret


	;; Handle time adjustment
ADJUST:				;===============================================
	;; Note that we only check for 1 pin being high at a time. Pressing
	;; more then one button simultaniously will not activate time
	;; adjustment, that is if you can do it fast enough ;)
	call	READ_ADJUST_PINS
	cpi	r16, low(adjustUpPin)
	breq	PIN_HIGH
	cpi	r16, low(adjustTimeUnitSelection)
	breq	PIN_HIGH
	cpi	r16, low(adjustDown)
	breq	PIN_HIGH
	rjmp	PIN_LOW
PIN_HIGH:
	call	ADJUSTING_DELAY
PIN_LOW:
	ret


READ_ADJUST_PINS:		;===============================================
	in	r16, PinB
	andi	r16, noInt2Or1Mr
	ret


	;; An adjustment input has gone high. Alow time for adjustment and
	;; Call adjust proper
ADJUSTING_DELAY:		;===============================================
	cli			; Dissable interrupts
	;; Always start on minutes
	ldi	currentlyAdjusting, adjustMinutesLEDIndicatorPin
	call	SET_ADJUSTING_LEDS
	call	DEBOUNCE
WAIT_FOR_USER_INPUT:
	call	ADJUSTING_PROPER
	cpi	r16, low(dissableAdjust)
	brne	WAIT_FOR_USER_INPUT
	sei			; Enable interrupts
	;; Enable 1MR on decade counter
	ldi	r16, low(adjustPinsPullupsMR1High)
	out	PortB, r16
	;; From a quick look at the data sheet we think that our decade counter
	;; (CD74HCT390E) can handle frequencies upto 6MHz at 2V and 25 deg C and
	;; upto 30MHz at 4 - 5V at 25 deg C (we are using 3.3V for the chips,
	nop			; with the exception of the input to 1MR (as
	nop			; our micro runs on 5V.) We are not sure if this
	nop			; is bad, it seems to work however.) Each nop
	nop			; takes 1 cycle so after 8 there should be more
	nop	      		; then (ldi) (1 / 125000)s between the two
	nop			; out's, this also gives time for the pin on our
	nop			; micro to go high. We don't think we need the
	nop			; nop's but we add them just to make sure.
	ldi	r16, low(adjustPinsPullups) ; Dissable 1MR on decade counter
	out	PortB, r16
	clr	currentlyAdjusting
	call	DEBOUNCE
	ret


	;; Check adjustment inputs and adjust time accordingly
ADJUSTING_PROPER:		;===============================================
	call	READ_ADJUST_PINS
	cpi	r16, low(dissableAdjust)
	;; Return to ADJUSTING_DELAY if dissableAdjust is pressed
	;; ADJUSTING_DELAY will re-enable interrupts, strobe the crystal enable
	breq	RET_FROM_ADJUSTING_PROPER ; off and then on again and then exit
	cpi	r16, low(adjustTimeUnitSelection)
	breq	CHANGE_TIME_SELECTION
	cpi	r16, low(adjustUpPin)
	breq	ADJUST_TIME_UP
	cpi	r16, low(adjustDown)
	breq	ADJUST_TIME_DOWN
	rjmp	RET_FROM_ADJUSTING_PROPER
	
CHANGE_TIME_SELECTION:
	call	CHANGE_TIME_SELECTION_PROPER
	call	DEBOUNCE
	rjmp	SET_ADJUSTING_LEDS_ANDRET_FROM_ADJUSTING_PROPER
ADJUST_TIME_UP:
	call	ADJUST_TIME_UP_PROPER
	call	DEBOUNCE
	rjmp	SET_ADJUSTING_LEDS_ANDRET_FROM_ADJUSTING_PROPER
ADJUST_TIME_DOWN:
	call	ADJUST_TIME_DOWN_PROPER
	call	DEBOUNCE

SET_ADJUSTING_LEDS_ANDRET_FROM_ADJUSTING_PROPER:
	call	DISPLAY_TIME
	call	SET_ADJUSTING_LEDS
	
RET_FROM_ADJUSTING_PROPER:
	ret


CHANGE_TIME_SELECTION_PROPER:	;===============================================
	cpi	currentlyAdjusting, low(adjustHoursLEDIndicatorPin)
	breq	CYCLE_BACK_AROUND
	ror	currentlyAdjusting
	rjmp	RET_FROM_CHANGE_TIME_SELECTION_PROPER	
CYCLE_BACK_AROUND:
	ldi	currentlyAdjusting, low(adjustSecondsLEDIndicatorPin)
RET_FROM_CHANGE_TIME_SELECTION_PROPER:
	ret


ADJUST_TIME_UP_PROPER:		;===============================================
	cpi	currentlyAdjusting, low(adjustSecondsLEDIndicatorPin)
	breq	UP_SECONDS
	cpi	currentlyAdjusting, low(adjustMinutesLEDIndicatorPin)
	breq	UP_MINUTES
	cpi	currentlyAdjusting, low(adjustHoursLEDIndicatorPin)
	breq	UP_HOURS
	rjmp	RET_ADJUST_TIME_UP_PROPER
	
UP_SECONDS:
	inc	secondCount
	cpi	secondCount, low(minAndHour)
	brlo	RET_ADJUST_TIME_UP_PROPER
	clr	secondCount
	rjmp	RET_ADJUST_TIME_UP_PROPER
	
UP_MINUTES:
	inc	minuteCount
	cpi	minuteCount, low(minAndHour)
	brlo	RET_ADJUST_TIME_UP_PROPER
	clr	minuteCount
	rjmp	RET_ADJUST_TIME_UP_PROPER
	
UP_HOURS:
	inc	hourCount
	cpi	hourCount, low(day)
	brlo	RET_ADJUST_TIME_UP_PROPER
	clr	hourCount
	
RET_ADJUST_TIME_UP_PROPER:
	ret


ADJUST_TIME_DOWN_PROPER:	;===============================================
	cpi	currentlyAdjusting, low(adjustSecondsLEDIndicatorPin)
	breq	DOWN_SECONDS
	cpi	currentlyAdjusting, low(adjustMinutesLEDIndicatorPin)
	breq	DOWN_MINUTES
	cpi	currentlyAdjusting, low(adjustHoursLEDIndicatorPin)
	breq	DOWN_HOURS
	rjmp	RET_ADJUST_TIME_DOWN_PROPER

DOWN_SECONDS:
	dec	secondCount
	cpi	secondCount, low(maxUnsignedByte)
	brne	RET_ADJUST_TIME_DOWN_PROPER
	ldi	secondCount, low(minAndHour)
	dec	secondCount
	rjmp	RET_ADJUST_TIME_DOWN_PROPER
	
DOWN_MINUTES:
	dec	minuteCount
	cpi	minuteCount, low(maxUnsignedByte)
	brne	RET_ADJUST_TIME_DOWN_PROPER
	ldi	minuteCount, low(minAndHour)
	dec	minuteCount
	rjmp	RET_ADJUST_TIME_DOWN_PROPER
	
DOWN_HOURS:
	dec	hourCount
	cpi	hourCount, low(maxUnsignedByte)
	brne	RET_ADJUST_TIME_DOWN_PROPER
	ldi	hourCount, low(day)
	dec	hourCount
	rjmp	RET_ADJUST_TIME_DOWN_PROPER


RET_ADJUST_TIME_DOWN_PROPER:
	ret


SET_ADJUSTING_LEDS:		;===============================================
	push	r16
	clr	r16
	mov	r16, hourCount
	or	r16, currentlyAdjusting
	out	PortA,	r16
	pop	r16
	ret


DEBOUNCE:			;===============================================
	clr	r31
	clr	r30
	clr	r29
DEBOUNCE_LOOP_0_START:
	inc	r31
	
DEBOUNCE_LOOP_1_START:
	inc	r30
	
DEBOUNCE_LOOP_2_START:
	inc	r29
	
	cpi	r29, low(debounceFactor0)
	brne	DEBOUNCE_LOOP_2_START

	cpi	r30, low(debounceFactor1)
	brne	DEBOUNCE_LOOP_1_START
	
	cpi	r31, low(debounceFactor2)
	brne	DEBOUNCE_LOOP_0_START
	ret


KEEP_TIME:		      ;=================================================
	cpi	secondCount, low(minAndHour) ; Have we counted 60 seconds?
	brlo	SUB_UNIT
	clr	secondCount	; Start counting the next minute
	inc	minuteCount
	
	cpi	minuteCount, low(minAndHour) ; Have we counted an hour?
	brlo	SUB_UNIT
	clr	minuteCount	; start counting the next hour
	inc	hourCount

	cpi	hourCount, low(day) ; Have we counted a day?
	brlo	SUB_UNIT
	clr	hourCount
SUB_UNIT:
	ret

	
DISPLAY_TIME:			;===============================================
	out	PortD, secondCount
	out	PortC, minuteCount
	out	PortA, hourCount
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
