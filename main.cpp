#include "Pokitto.h"
#include <File>

#define LISTENINGSAMPLERATE 11025
#define audioBufferSize 512
unsigned char *audioBuffer = (unsigned char *) 0x20000000;
int bufferOffset[4]={ audioBufferSize*3,  0, audioBufferSize, audioBufferSize*2 };

File musicFile;

uint8_t currentBuffer = 0;
uint8_t completeBuffer = 0;         // Which section is full
long int audioOffset=0;
long int bufferCount=0;
float myVolume = 0.025;

char musicName[] = "11025_16bit_mono.pcm"; 
#define my_TIMER_32_0_IRQn 18          // for Timer setup
#include "timer_11u6x.h"
#include "clock_11u6x.h"

using PC = Pokitto::Core;
using PD = Pokitto::Display;

pwmout_t audiopwm;
pwmout_t* obj = &audiopwm;

inline void audioTimer32(void){
	if (Chip_TIMER_MatchPending(LPC_TIMER32_0, 1)) {

        int16_t c = static_cast<int16_t>((audioBuffer[audioOffset + 1] << 8) | audioBuffer[audioOffset]);
        c = (c & 0x8000) ? static_cast<int16_t>(c | 0xFFFF0000) : c;
        float t = static_cast<float>(c) / 32767.0 + 0.5;
    
        //float t = (float)audioBuffer[audioOffset] / (float)255; // 8 bit pwm from stream
        
        pwmout_write(&audiopwm, t * myVolume);

        audioOffset +=2;
        if(audioOffset >= audioBufferSize*4){
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

    // Awful hack to get pwm audio working, there is some conflict with the backlight.
    pwmout_init(&audiopwm, POK_BACKLIGHT_PIN);
    pwmout_init(&audiopwm, POK_AUD_PIN);
    pwmout_period_us(&audiopwm, 31);
    DigitalOut screenLED(POK_BACKLIGHT_PIN);
    screenLED = 1;
    
    if(musicFile.openRO(musicName)){
        PD::print(0,0,"Music found!");
        initTimer32(LISTENINGSAMPLERATE);
    }else{
        PD::print(0,0,"No Music!");
    };
    
    
    // empty the audio buffer before starting
    for(int t=0; t<audioBufferSize*4; t++){
        audioBuffer[t] = 0;
    }
    
    while(PC::isRunning()){

        if( !PC::update() ) 
            continue;

        if(Pokitto::Buttons::downBtn()){
            myVolume -= 0.025;
        }

        if(Pokitto::Buttons::upBtn()){
            myVolume += 0.025;
        }

        if(myVolume < 0.0) myVolume = 0.0;
        if(myVolume > 1.0) myVolume = 1.0;

        char tempText[16];
        sprintf(tempText,"Vol:%d   ", int(myVolume * 100));
        PD::print(0,64,tempText);

        // Update music playing
        if( currentBuffer != completeBuffer){
            completeBuffer = currentBuffer;
            if(!musicFile.read(&audioBuffer[bufferOffset[completeBuffer]], audioBufferSize)){
                PD::print(0,8,"End File");
            }else {
                char tempText[16];
                sprintf(tempText,"C:%d",bufferCount++);
                PD::print(0,16,tempText);
            }
        }        
    }

    return 1;    
}

