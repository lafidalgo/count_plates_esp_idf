#include "soc/soc_ulp.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_cntl_reg.h"

.data

.global HX711HiWord
HX711HiWord:
.long 0x00

.global HX711LoWord
HX711LoWord:
.long 0x00

.global trshHoldOverADMSB 
trshHoldOverADMSB:
.long 0x00

.global trshHoldOverADLSB
trshHoldOverADLSB:
.long 0x00

.global trshHoldUnderADMSB 
trshHoldUnderADMSB:
.long 0x00

.global trshHoldUnderADLSB
trshHoldUnderADLSB:
.long 0x00

.global dataReady
dataReady:
.long 0x00

.global repeatMeasureCount
repeatMeasureCount:
.long 0x00

.global repeatMeasureQuantity
repeatMeasureQuantity:
.long 0x00

.global setRepeatMeasure
setRepeatMeasure:
.long 0x00

.global wakeUp
wakeUp:
.long 0x00

.global HX711HiWordAcc
HX711HiWordAcc:
.long 0x00

.global HX711LoWordAcc
HX711LoWordAcc:
.long 0x00

.bss//Variaveis sao declaradas dentro da secao .bss

.text//O codigo é feito dentro da secao .text
 
	// Some macros
	.macro DOUT_read
		READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 6, 1)
	.endm
	
	/* These two marco for set bus high and set low when GPIO_L is called, enable W1TS. */
	.macro SCLK_high
		WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 7, 1, 1)
	.endm
	
	.macro SCLK_low
		WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 7, 1, 0)
	.endm

	.global main
	main://O codigo e iniciado aqui, equivale ao void setup()
		move 	r3, dataReady
		move 	r0, 0x00
		st 		r0, r3, 0
		WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S+6, 1, 1)//sobe add0

 	ReadHX711:
		move    r1, 0           // Initialzation HX711 read storage to 0
		move    r3, HX711HiWord
		st      r1, r3, 0       // Offset points to HX711HiWord
		st      r1, r3, 4       // Offset points to HX711LoWord
		move    r2, 2           // count of passes through bit clocking code
		stage_rst
		stage_inc   8           // Setup to read hi 8 bits HX711 output
	ReadCycleBegin:
		SCLK_low
	CheckReady:
		DOUT_read
		jumpr   CheckReady, 1,  ge  // Ready when DOUT goes low
	RdBit:
		SCLK_high
		SCLK_low
		DOUT_read
		lsh     r1, r1, 1
		jumpr   GotAZeroBit, 1, lt
		add     r1, r1, 1       // If last read was a 1, OR it with accumulator
	GotAZeroBit:
		stage_dec   1
		jumps   RdBit,  0,  gt  // if bit count down not 0, go read another bit
		st      r1, r3, 0       //  store accumulated read, 8 or 16 bits
		sub     r2, r2, 1       //  Have we read two words?
		jump    ReadDone,   eq
	
		stage_inc   16          // else setup to read 16 bits of low HX711 output
		move    r3, HX711LoWord // point r3 for the low word read
		move    r1, 0           // init r1 for low word read (Is this needed? Be safe)
		jump    RdBit
	
	ReadDone:                   				// must cycle SCLK one more time to set gain to
		SCLK_high               				// 128 on next HX711 read, then leave SCLK hi for
		SCLK_low                				// for > 60 uS to power down HX711, ~10 clocks
		move 	r0, setRepeatMeasure			// Jump p/ check_set_repeat_measure se setRepeatMeasure == 1
		ld 		r0, r0, 0						//
		jumpr 	check_set_repeat_measure, 1, EQ // 
		move 	r0, repeatMeasureCount			// Jump p/ check_measure_high se repeatMeasureCount > 1
		ld 		r0, r0, 0						//
		jumpr 	check_measure_high, 1, GT 		// 
		jump 	compare_trshHoldOver			// Jump p/ compare_trshHoldOver

	compare_trshHoldOver:
		move 	r2, trshHoldOverADMSB
		ld 		r2, r2, 0
		move 	r3, HX711HiWord
		ld 		r3, r3, 0
		sub 	r0, r2, r3
		jumpr 	msb_igual_over, 0, eq
		jump 	set_repeat_measure, ov // overflow significa que r3 > r2 portanto vloadAD > threshHold
		// msb ad menor q threshhold
		jump 	compare_trshHoldUnder

	msb_igual_over:
		move 	r2, trshHoldOverADLSB
		ld 		r2, r2, 0
		move 	r3, HX711LoWord
		ld 		r3, r3, 0
		sub 	r0, r2, r3
		jump 	set_repeat_measure, ov // overflow significa que r3 > r2 portanto vloadAD > threshHold
		jump 	compare_trshHoldUnder

	compare_trshHoldUnder:
		move 	r2, trshHoldUnderADMSB
		ld 		r2, r2, 0
		move 	r3, HX711HiWord
		ld 		r3, r3, 0
		sub 	r0, r3, r2
		jumpr 	msb_igual_under, 0, eq
		jump 	set_repeat_measure, ov // overflow significa que r3 > r2 portanto threshHold > vloadAD
		// msb ad menor q threshhold
		SCLK_high
		halt

	msb_igual_under:
		move 	r2, trshHoldUnderADLSB
		ld 		r2, r2, 0
		move 	r3, HX711LoWord
		ld 		r3, r3, 0
		sub 	r0, r3, r2
		jump 	set_repeat_measure, ov // overflow significa que r3 > r2 portanto threshHold > vloadAD
		SCLK_high
		halt
	
	check_set_repeat_measure:
		move 	r3, setRepeatMeasure	// setRepeatMeasure = 0
		move 	r0, 0x00				//
		st 		r0, r3, 0				//
		move 	r3, wakeUp				// wakeUp = 0
		move 	r0, 0x00				//
		st 		r0, r3, 0				//
		jump set_repeat_measure			// Jump p/ set_repeat_measure

	set_repeat_measure:
		move 	r3, repeatMeasureCount	// repeatMeasureCount = repeatMeasureQuantity
		move 	r0, repeatMeasureQuantity//
		ld 		r0, r0, 0				//
		st 		r0, r3, 0				//
		move 	r3, HX711LoWordAcc		// HX711LoWordAcc = 0
		move 	r0, 0x00				//
		st		r0, r3, 0				//
		move 	r3, HX711HiWordAcc		// HX711HiWordAcc = 0
		move 	r0, 0x00				//
		st 		r0, r3, 0				//
		jump 	check_measure_high		// Jump p/ check_measure_high

	check_measure_high:
		move 	r0, HX711HiWord 		// Jump p/ check_measure_low se HX711HiWord == 0
		ld 		r0, r0, 0				//
		jumpr 	check_measure_low, 0, eq//
		jump 	accumulate_measure		// Jump p/ accumulate_measure se HX711HiWord != 0
	
	check_measure_low:
		move 	r0, HX711LoWord			// Jump p/ ReadHX711 se HX711LoWord == 0
		ld 		r0, r0, 0				//
		jumpr 	ReadHX711, 0, eq		//
		jump	accumulate_measure		// Jump p/ accumulate_measure se HX711LoWord != 0

	accumulate_measure:
		move 	r3, repeatMeasureCount 	// repeatMeasureCount -= 1
		ld 		r3, r3, 0				//
		sub 	r0, r3, 1				//
		move 	r3, repeatMeasureCount	//
		st 		r0, r3, 0				//
		move 	r3, HX711LoWordAcc      // HX711LoWordAcc + HX711LoWord
		ld 		r3, r3, 0				//
		move 	r2, HX711LoWord			//
		ld 		r2, r2, 0				//
		add 	r0, r3, r2				//
		jump 	accumulate_overflow, ov // Jump p/ accumulate_overflow se HX711LoWordAcc + HX711LoWord ultrapassa 16 bits
		move 	r3, HX711LoWordAcc     	// HX711LoWordAcc = HX711LoWordAcc + HX711LoWord
		st 		r0, r3, 0				//
		move 	r3, HX711HiWordAcc		// HX711HiWordAcc += HX711HiWord
		ld 		r3, r3, 0				//
		move 	r2, HX711HiWord			//
		ld 		r2, r2, 0				//
		add 	r0, r3, r2				//
		move 	r3, HX711HiWordAcc		//
		st 		r0, r3, 0				//
		move 	r0, repeatMeasureCount	// Jump p/ finished_acc se repeatMeasureCount == 1
		ld 		r0, r0, 0				//
		jumpr 	finished_acc, 1, EQ 	// 
		jump 	ReadHX711				// Jump p/ ReadHX711 se repeatMeasureCount != 1
	
	accumulate_overflow:
		move 	r3, HX711LoWordAcc		// HX711LoWordAcc = HX711LoWordAcc + HX711LoWord
		st 		r0, r3, 0				//
		move 	r3, HX711HiWordAcc		// HX711HiWordAcc += (HX711HiWord + 1)
		ld 		r3, r3, 0				//
		move 	r2, HX711HiWord			//
		ld 		r2, r2, 0				//
		add 	r0, r3, r2				//
		add 	r0, r0, 1				//
		move 	r3, HX711HiWordAcc		//
		st 		r0, r3, 0				//
		move 	r0, repeatMeasureCount	// Jump p/ finished_acc se repeatMeasureCount == 1
		ld 		r0, r0, 0				//
		jumpr 	finished_acc, 1, EQ 	//
		jump 	ReadHX711				// Jump p/ ReadHX711 se repeatMeasureCount != 1

	finished_acc:
		move 	r3, dataReady			// dataReady = 1
		move 	r0, 0x01				//
		st 		r0, r3, 0				//
		SCLK_high
		move 	r3, repeatMeasureCount	// repeatMeasureCount = 0
		move 	r0, 0x00				//
		st 		r0, r3, 0				//
		move 	r0, wakeUp				// Jump p/ no_wake_up se wakeUp == 0
		ld 		r0, r0, 0				//
		jumpr 	no_wake_up, 0, EQ 		//
		jump 	wake_up					// Jump p/ wake_up se wakeUp != 0
	
	no_wake_up:
		move 	r3, wakeUp				// wakeUp = 1
		move 	r0, 0x01				//
		st 		r0, r3, 0				//
		halt							// Finaliza programa