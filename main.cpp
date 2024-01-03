#include "Pokitto.h"
#include <File>

#define LISTENINGSAMPLERATE 22050
#define audioBufferSize 512
unsigned char *audioBuffer = (unsigned char *) 0x20000000;
unsigned char audioBuffer2[audioBufferSize*4];// = (unsigned char *) 0x20000800;
int bufferOffset[4]={ audioBufferSize*3,  0, audioBufferSize, audioBufferSize*2 };

File musicFile;

uint8_t currentBuffer = 0;
uint8_t completeBuffer = 0;         // Which section is full
long int audioOffset=0;
long int bufferCount=0;

char musicName[] = "22050_8bit_mono.pcm"; 
#define my_TIMER_16_0_IRQn 16          // for Timer setup
#include "timer_11u6x.h"
#include "clock_11u6x.h"

using PC = Pokitto::Core;
using PD = Pokitto::Display;

pwmout_t audiopwm;


inline void audioTimer(void){
	if (Chip_TIMER_MatchPending(LPC_TIMER16_0, 1)) {
        //writeDAC( myMixSound() );
        pwmout_write(&audiopwm,(float)audioBuffer[audioOffset] / 255);
        
        if(++audioOffset == audioBufferSize*4){
            audioOffset = 0;
        }
        currentBuffer = audioOffset/audioBufferSize;

 	    Chip_TIMER_ClearMatch(LPC_TIMER16_0, 1);
    }
}

// timer init stolen directly from Pokittolib
void initTimer(uint32_t sampleRate){
    /* Initialize 32-bit timer 0 clock */
	Chip_TIMER_Init(LPC_TIMER16_0);
    /* Timer rate is system clock rate */
	uint32_t timerFreq = Chip_Clock_GetSystemClockRate();
	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(LPC_TIMER16_0);
	/* Enable both timers to generate interrupts when time matches */
	Chip_TIMER_MatchEnableInt(LPC_TIMER16_0, 1);
    /* Setup 32-bit timer's duration (32-bit match time) */
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 1, (timerFreq / sampleRate));
	/* Setup both timers to restart when match occurs */
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER16_0, 1);
	/* Start both timers */
	Chip_TIMER_Enable(LPC_TIMER16_0);
	/* Clear both timers of any pending interrupts */
	NVIC_ClearPendingIRQ((IRQn_Type)my_TIMER_16_0_IRQn);
    /* Redirect IRQ vector - Jonne*/
    NVIC_SetVector((IRQn_Type)my_TIMER_16_0_IRQn, (uint32_t)&audioTimer);
	/* Enable both timer interrupts */
	NVIC_EnableIRQ((IRQn_Type)my_TIMER_16_0_IRQn);
}

//pwmout_write(&audiopwm,(float)0/(float)255);

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

    pwmout_init(&audiopwm,POK_AUD_PIN);
    pwmout_period_us(&audiopwm,POK_AUD_PWM_US); //was 31us
    pwmout_write(&audiopwm,0.1f);    
    
    if(musicFile.openRO(musicName)){
        PD::print(0,0,"Music found!");
        initTimer(LISTENINGSAMPLERATE);
    }else{
        PD::print(0,0,"No Music!");
    };
    
    while(PC::isRunning()){
        if( !PC::update() ) 
            continue;
            
        //PD::drawBitmap(rand() % (PD::width - 32), rand() % (PD::height - 64)+32, Smile);
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

