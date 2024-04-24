#include "drone.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h> 
#include <mavlink/include/common/mavlink.h>
#include <asm-generic/termbits.h>
#include <sys/types.h>

mavlink_ais_vessel_t *vessel; 
struct termios tty;
int32_t home_lat, home_lon, home_alt;
int32_t lat, lon, alt;


#define SERIAL_PORT "/dev/ttyAMA0" // Port UART sur la Raspberry Pi
#define BAUD_RATE B57600 // Vitesse de transmission en bauds
#define SYSTEM_ID 1 // ID du système (de votre drone)
#define COMPONENT_ID 1 // ID du composant (de votre drone)


// function that initialize any MAV_TYPE of vessel. In our case it will be a drone name focon
// utiliser directement la fonction mavlink_msg_ais_vessel_pack()
void initialize_vessel(uint16_t type, int32_t lon, int32_t lat, char name[20]){
    vessel->type =  type; 
    vessel->lon = lon; 
    vessel->lat = lat;
    strcpy(vessel->name,name);
}

int open_serial_port(const char *port){

    int fd; 
    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);    // open the file associated to opening serial port 
    if (fd == -1){
        perro("error when opening serial port\n"); 
        return -1;
    }

    tcgetattr(fd, &tty);    // get the actual serial port attribut associated to the file fd
    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;          // material flux control so the rasp allow flux comming from the drone
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 5;
    tcsetattr(fd, TCSANOW, &tty);

    return fd;

}

void close_serial_port(int fd){
    close(fd);
}

void send_mavlink_message(int fd, mavlink_message_t *msg){
    uint8_t buf[MAVLINK_MAX_PACKET_LEN]; 
    int len; 

    len = mavlink_msg_to_send_buffer(buf,msg);  // convert the mavlink msg in a buffer of data
    write(fd,buf,len);                          // write the data on the serial port
}

void receive_mavlink_message(int fd){
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_message_t msg; 
    mavlink_status_t status; 
    ssize_t n; 

    n = read(fd,buf, sizeof(buf));
    if(n>0){
        for (int i=0; i<n; i++){
            if(mavlink_parse_char(MAVLINK_COMM_0, buf[i],&msg, &status)){
                printf("Message MAVLink recu. ID du message : %d\n", msg.msgid);
            }
        }
    }
}

void connection(){
    int serial_fd; 
    serial_fd = open_serial_port(SERIAL_PORT);
    if(serial_fd == -1) exit(EXIT_FAILURE);
}

void set_home(mavlink_message_t *msg){
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        mavlink_global_position_int_cov_t pos;
        mavlink_msg_global_position_int_decode(msg, &pos);
        home_lat = pos.lat;
        home_alt = pos.alt;
        home_lon = pos.lon;
        break;
    default:
        break;
    }
}

void coodinate(mavlink_message_t *msg){
    switch (msg->msgid)
    {
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        mavlink_global_position_int_cov_t pos;
        mavlink_msg_global_position_int_decode(msg, &pos);
        lat = pos.lat;
        alt = pos.alt;
        lon = pos.lon;
        break;
    default:
        break;
    }
}

