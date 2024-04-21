
// arm the motors
void arm();

// save the departure coordinates to get back if needed
void set_hom();

// establish the connection between Raspberry Pi an the flight controller
void connection();

// load the mission in the drone, an other function start its execution 
void upload_mission();

// erase the created mission (all the WAYPOINT)
void clear_mission();

//
void takeoff(int angle, int x, int y, int z);

// force the drone to execute the next command of it mission, even if did not finish the actual task
void execute();

// put the drone in a mode mission, without it the drone will never execute a loaded mission
void mission_mode();

/* once the connection established and the calibration done, it allows the drone to take off at a z altitude 
angle= 0 -> drone heading to North
x representes latitude in degres and y the longitude */
void arm_takeoff(int angle, int x, int y, int z);

// add a WAYPOINT to the mission 
void new_command(int angle, int x, int y, int z);

// return the actual coordinates of the drone (longitude, latutude and altitude regarding the ground not the sea)
void coodinate();

// return the different inclinations of the drone 
// ATTENTION changer valeur de retour de la fonction 
void inclinaison();

// convert cm in longitude and latitude position to correct the position of the drone
int convert_cm_GPS(int x, int altitude);

// place the drone in a certain position above the blue cube after the image treatement
// x and y correspond to the distance (in cm) between the drone and the cube center
void search_position(int x, int y);

// place the drone in a certain orientation, depending the angle
void search_orientation(int angle);

// drone goes down
void down(int z);

// drone goes up
void up(int z);

// place the drone in a certain altitude
void altitude(int z);

// move the drone in the wanted coodinates 
void target(int angle, int x, int y, int z);

// drone moves in a square movement
void square(int lg);  

// drone moves in a square movement and treat an image to find the blue cube
int quadrillage(int lg, int dist, int n); 