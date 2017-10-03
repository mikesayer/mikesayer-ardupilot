/*
 * Aerobatics.h
 *
 *  Created on: Aug 26, 2017
 *      Author: mike
 */

#ifndef AEROBATICS_H_
#define AEROBATICS_H_


// this is the command from mission and RC switch
enum AeroEnum {NONE=0, LOOP=4,ROLL,TWO_ROLLS,CUBAN_EIGHT,EIGHT,STALL_TURN};

class Plane;



class Aerobatics {
public:
    Aerobatics();
  //  bool add_maneuver(maneuver_t  maneuver)
    void update(); /* call often to see if a maneuver is required */
    void aero_command(int command);
private :


    AeroEnum read_aero_switch(int ctrl_chan);
    bool check_for_abort(AeroEnum );
    int limit_value(int value,int min,int max);

    void set_rc_chan_val(int chan,int val);
    void release_all_rc_chans();
    void start_aero(int roll,int pitch,int thr,int yaw);
    AeroEnum convert_mission_command(int command);
    void send_message(const char *message);
};

#endif /* AEROBATICS_H_ */
