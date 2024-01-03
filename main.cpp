#include "Pokitto.h"
#include <File>
#ifndef POK_SIM
#include <SoftwareI2C.h>
#endif

#define LISTENINGSAMPLERATE 11025
#define audioBufferSize 512
unsigned char *audioBuffer = (unsigned char *) 0x20000000;
int bufferOffset[4]={ audioBufferSize*3,  0, audioBufferSize, audioBufferSize*2 };

File musicFile;

uint8_t currentBuffer = 0;
uint8_t completeBuffer = 0;         // Which section is full
long int audioOffset=0;
long int bufferCount=0;

char musicName[] = "11025_8bit_mono.pcm"; 
#define my_TIMER_16_0_IRQn 16          // for Timer setup
#define my_TIMER_32_0_IRQn 18          // for Timer setup

#include "timer_11u6x.h"
#include "clock_11u6x.h"

using PC = Pokitto::Core;
using PD = Pokitto::Display;

pwmout_t audiopwm;
pwmout_t* obj = &audiopwm;


void setVol(uint32_t v){

#ifndef POK_SIM
    v = 255 - (192 - v) * (192 - v) * 255 / 36864;
    uint32_t hwVolume = v ? (v>>1) | 0xF : 0;
    uint32_t swVolume = v ? (v | 0xF) + 1 : 0;
    SoftwareI2C(P0_4, P0_5).write(0x5e, hwVolume);
    SoftwareI2C(P0_5, P0_4).write(0x5e, hwVolume); // fix for newer boards with i2C right way around
#endif
}

inline void audioTimer32(void){
	if (Chip_TIMER_MatchPending(LPC_TIMER32_0, 1)) {
        pwmout_write(&audiopwm,(float)audioBuffer[audioOffset] / (float)255);
        //pwmout_write(&audiopwm,(float)random(255) / 255);
        //writeDAC( audioBuffer[audioOffset] );
        
        if(++audioOffset == audioBufferSize*4){
            audioOffset = 0;
        }
        currentBuffer = audioOffset/audioBufferSize;

 	    Chip_TIMER_ClearMatch(LPC_TIMER32_0, 1);
    }
}

void initTimer32(uint32_t sampleRate){
     /* Initialize 32-bit timer 0 clock */
	Chip_TIMER_Init(LPC_TIMER32_0);
    /* Timer rate is system clock rate */
	uint32_t timerFreq = Chip_Clock_GetSystemClockRate();
	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(LPC_TIMER32_0);
	/* Enable both timers to generate interrupts when time matches */
	Chip_TIMER_MatchEnableInt(LPC_TIMER32_0, 1);
    /* Setup 32-bit timer's duration (32-bit match time) */
	Chip_TIMER_SetMatch(LPC_TIMER32_0, 1, (timerFreq / sampleRate));
	/* Setup both timers to restart when match occurs */
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER32_0, 1);
	/* Start both timers */
	Chip_TIMER_Enable(LPC_TIMER32_0);
	/* Clear both timers of any pending interrupts */
	NVIC_ClearPendingIRQ((IRQn_Type)my_TIMER_32_0_IRQn);
    /* Redirect IRQ vector - Jonne*/
    NVIC_SetVector((IRQn_Type)my_TIMER_32_0_IRQn, (uint32_t)&audioTimer32);
	/* Enable both timer interrupts */
	NVIC_EnableIRQ((IRQn_Type)my_TIMER_32_0_IRQn);
}

inline void updateStream(){
    // Update music playing
    if( currentBuffer != completeBuffer){
        completeBuffer = currentBuffer;
        if(!musicFile.read(&audioBuffer[bufferOffset[completeBuffer]], audioBufferSize)){
            // reset the file or something
        }
    }
}


int main() {
    PD::persistence = true;
    PD::invisiblecolor = 0;

    PC::begin();
    PC::update(); // must update once before initing the 32bit timer
    
    // enable amp
    LPC_GPIO_PORT->SET[1] = (1 << 17);

    setVol(60);

    //#define POK_AUD_PIN P2_19
    //pwmout_init(&audiopwm, P1_19); // external PEX pin
    pwmout_init(&audiopwm,POK_AUD_PIN);
    pwmout_period_us(&audiopwm,POK_AUD_PWM_US); //was 31us
    pwmout_write(&audiopwm,0.1f);    

    //enableDAC();
    
    if(musicFile.openRO(musicName)){
        PD::print(0,0,"Music found!");
        // Something stops timer32 from working
        initTimer32(LISTENINGSAMPLERATE);
    }else{
        PD::print(0,0,"No Music!");
    };
    
    while(PC::isRunning()){
        if( !PC::update() ) 
            continue;
            
        char tempText[16];
        sprintf(tempText,"A:%d\n",audioBuffer[audioOffset]);
        PD::print(0,24,tempText);

        // Update music playing
        if( currentBuffer != completeBuffer){
            completeBuffer = currentBuffer;
            if(!musicFile.read(&audioBuffer[bufferOffset[completeBuffer]], audioBufferSize)){
                PD::print(0,8,"End File");
            }else {
            //    char tempText[16];
            //    sprintf(tempText,"C:%d",bufferCount++);
            //    PD::print(0,16,tempText);
            }
        }        
    }

    return 1;    
}

