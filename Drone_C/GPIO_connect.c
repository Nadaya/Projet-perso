// this file is use to be used to GPIO connection with C and using a Raspnerry pi 4 

#include <stdio.h>
#include <stdlib.h>
#include<wiringPi.h> 


// pour compiler : gcc -o blinker blinker.c -l wiringPi  works only for x86  NOT the rasp
// pour executer : sudo ./blinker

const int servo_pin = 17;
const int  LED = 21;
int pwm_frequency = 50; 
int pwm;
int compress, decompress, stop;

void GPIO_connection(){

   if( wiringPiSetupGpio() == -1){
    printf("Error when initialing WiringPi\n");
    exit (1);
   }

    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);

    pwmSetMode(PWM_MODE_MS); // mode use to controle PWM signal's wide impulse
    pinMode(servo_pin, PWM_OUTPUT); // needs to be a PWM_OUTPUT bc it is a motor that we want to control
    pwmToneWrite(servo_pin, pwm_frequency);

    compress = 10; 
    decompress = 5; 
    stop = 0; 
}

void close(){
    wiringPiSetupGpio();
    pwmWrite(servo_pin, compress);
}

void open(){
    wiringPiSetupGpio();
    pwmWrite(servo_pin, decompress);
}

void test_pince(){
    for (int i =0; i<5; i++){
        open();
        delay(1000); // 1 sec
        close();
        delay(1000);
    }
}

int main(){

    test_pince();
    return 0;
}

