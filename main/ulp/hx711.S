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

.global thresholdType
thresholdType:
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
	
	ReadDone:                   // must cycle SCLK one more time to set gain to
		SCLK_high               // 128 on next HX711 read, then leave SCLK hi for
		SCLK_low                // for > 60 uS to power down HX711, ~10 clocks
		SCLK_high

	compare_trshHoldOver:
		move r3, thresholdType
		move r0, 0x01
		st r0, r3, 0
		move r2, trshHoldOverADMSB
		ld r2, r2, 0
		move r3, HX711HiWord
		ld r3, r3, 0
		sub r0, r2, r3
		jumpr msb_igual_over, 0, eq
		jump wake_up, ov // overflow significa que r3 > r2 portanto vloadAD > threshHold
		// msb ad menor q threshhold
		jump compare_trshHoldUnder

	msb_igual_over:
		move r2, trshHoldOverADLSB
		ld r2, r2, 0
		move r3, HX711LoWord
		ld r3, r3, 0
		sub r0, r2, r3
		jump wake_up, ov // overflow significa que r3 > r2 portanto vloadAD > threshHold
		jump compare_trshHoldUnder

	compare_trshHoldUnder:
		move r3, thresholdType
		move r0, 0x02
		st r0, r3, 0
		move r2, trshHoldUnderADMSB
		ld r2, r2, 0
		move r3, HX711HiWord
		ld r3, r3, 0
		sub r0, r3, r2
		jumpr msb_igual_under, 0, eq
		jump wake_up, ov // overflow significa que r3 > r2 portanto threshHold > vloadAD
		// msb ad menor q threshhold
		halt

	msb_igual_under:
		move r2, trshHoldUnderADLSB
		ld r2, r2, 0
		move r3, HX711LoWord
		ld r3, r3, 0
		sub r0, r3, r2
		jump wake_up, ov // overflow significa que r3 > r2 portanto threshHold > vloadAD
		halt