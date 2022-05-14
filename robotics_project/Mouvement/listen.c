/*
EPFL MICRO-301 GR59
Gilles Regamey (296642) - Mathieu Schertenleib (313318)
may 2022
*/

#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <string.h>
#include "main.h"
#include "move.h"
#include "listen.h"
#include "odometrie.h"
#include "process_image.h"
#include "communications.h"

// Main function, sync communication and picks up commands.
void listen( BaseSequentialStream* in, BaseSequentialStream* out){
    set_body_led(1); //ready to listen
    volatile uint8_t k = chSequentialStreamGet(in);
    volatile char c1,c2,c3,c4;
    if(k != '!'){   //ready to take in instruction
        chprintf(out,"ASCII %c, Hex %x, Dec %d.\r\n",k,k,k);
    }else{
    	instr:
        /*
        CLR = Sets position and movement sequence.
        MOVE = Moves according to speed instruction.
        POS = Sends current position
        STOP = Stops
        PIC = Takes a picture.
        SCAN = Does a "lidar" turn.
        BEEP = beeps at a frequency for 100ms
        */

       c1 = (char)chSequentialStreamGet(in);
       c2 = (char)chSequentialStreamGet(in);
       c3 = (char)chSequentialStreamGet(in);
       if(c1 == '!' || c2 == '!' || c3 == '!') goto instr; //you didnt see it.
       char scom[4] = {c1,c2,c3,'\0'};

       if(strcmp(scom,"CLR")==0){
           set_body_led(0);
           float newx = ReceiveFloatFromComputer(in);
           float newy = ReceiveFloatFromComputer(in);
           float newp = ReceiveFloatFromComputer(in);
           set_pos(newx,newy,newp);
       }else if(strcmp(scom,"PIC")==0){
           set_body_led(0);
           get_picture();
       }else if(strcmp(scom,"POS")==0){
           set_body_led(0);
           SendFloatToComputer(out,get_posx());
           SendFloatToComputer(out,get_posy());
           SendFloatToComputer(out,get_angle());
       }else{
           c4 = (char)chSequentialStreamGet(in);
           if(c4 == '!' ) goto instr; //what ? there's no line 61.
           char lcom[5] = {c1,c2,c3,c4,'\0'};

           if(strcmp(lcom,"MOVE")==0){
               set_body_led(0);
               ReceiveSpeedInstMove(in,out);
           }else if(strcmp(lcom,"STOP")==0){
               set_body_led(0);
               stop(out);
           }else if(strcmp(lcom,"SCAN")==0){
               set_body_led(0);
               scan(out);
           }else if(strcmp(lcom,"BEEP")==0){
        	   uint16_t freq = ReceiveUint16FromComputer(in);
               set_body_led(0);
               dac_start();
               dac_play(freq);
               chThdSleepMilliseconds(100);
               dac_stop();
           }else{
               chprintf(out,"INVALID",lcom); //in case of bad instruction.
           }
       }
    }
}
