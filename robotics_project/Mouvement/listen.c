#include "ch.h"
#include "hal.h"
#include "leds.h"
#include <usbcfg.h>
#include <chprintf.h>
#include <string.h>
#include <main.h>
#include <move.h>
#include <listen.h>
#include <odometrie.h>

void listen( BaseSequentialStream* in, BaseSequentialStream* out){
    set_body_led(1); //ready to listen
    volatile uint8_t k = chSequentialStreamGet(in);
    if(k != '!'){   //ready to take in instruction
        chprintf(out,"ASCII %c, Hex %x, Dec %d.\r\n",k,k,k);
    }else{
        /*
        CLS = Clear position and movement sequence.
        MOVE = Moves according to speed instruction.
        POS = sends position
        STOP = stop and send position
        PIC = takes a picture.
        SCAN = does a lidar turn.
        BEEP = beeps at 440Hz for 100ms
        */

       volatile char c1 = (char)chSequentialStreamGet(in);
       volatile char c2 = (char)chSequentialStreamGet(in);
       volatile char c3 = (char)chSequentialStreamGet(in);
       char scom[4] = {c1,c2,c3,'\0'};

       if(strcmp(scom,"CLS")==0){
           chprintf(out,"Reset pos to 0,0,0.\r\n");
           set_pos(0,0,0);
       }else if(strcmp(scom,"PIC")==0){
           chprintf(out,"%s\r\n",scom);
       }else if(strcmp(scom,"POS")==0){
           chprintf(out,"%s\r\n",scom);
           chprintf(out,"Robot at : %0.2f X %0.2f Y %0.2f Phi\r\n",get_posx(),get_posy(),get_angle());
       }else{
           volatile char c4 = (char)chSequentialStreamGet(in);
           char lcom[5] = {c1,c2,c3,c4,'\0'};

           if(strcmp(lcom,"MOVE")==0){
               chprintf(out,"%s\r\n",lcom);
               ReceiveSpeedInstMove(in,out);
           }else if(strcmp(lcom,"STOP")==0){
               chprintf(out,"%s\r\n",lcom);
               stop(out);
           }else if(strcmp(lcom,"SCAN")==0){
               chprintf(out,"%s\r\n",lcom);
               scan(out);
           }else if(strcmp(lcom,"BEEP")==0){
               chprintf(out,"%s!\r\n",lcom);
               dac_start();
               dac_play(440);
               chThdSleepMilliseconds(100);
               dac_stop();
           }else{
               chprintf(out,"command does not exist : %s\r\n",lcom);
           }
       }
    }
}