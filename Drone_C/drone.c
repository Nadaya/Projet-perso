#include "drone.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <mavlink/include/common/mavlink.h>

mavlink_ais_vessel_t *vessel; 

// function that initialize any MAV_TYPE of vessel. In our case it will be a drone name focon
// utiliser directement la fonction mavlink_msg_ais_vessel_pack()
void initialize_vessel(uint16_t type, int32_t lon, int32_t lat, char name[20]){
    vessel->type =  type; 
    vessel->lon = lon; 
    vessel->lat = lat;
    strcpy(vessel->name,name);
}

void connection(){
    
}