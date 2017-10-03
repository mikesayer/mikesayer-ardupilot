/*
 * Aerobatics.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: Mike Sayer
 */
#include "Plane.h"
#include <string>

enum FlightMode start_mode = FLY_BY_WIRE_A;
enum aero_state_t {
	WAIT_FOR_SWITCH_LOW,
	ACC_FOR_MANUEVER,
	MANUEVER_IDLE,
	LOOP_START,
	TOP_OF_LOOP,
	BOTTOM_OF_LOOP,
	START_STABILISE,
	STABILISE_TIME,
	ROLL_START,
	ROLL_TOP,
	ROLL_BOTTOM,
	CUBAN_START,
	CUBAN_TOP_1,
	CUBAN_START_ROLL,
	CUBAN_ROLL_END,
	CUBAN_LOOP_START,
	CUBAN_LOOP_BOTTOM,
	CUBAN_TOP_2,
	CUBAN_START_ROLL_2,
	CUBAN_ROLL_END_2,
	CUBAN_LOOP_START2,
	CUBAN_LOOP_BOTTOM_2,
	EIGHT_START,
	EIGHT_TOP,
	EIGHT_DOWN,
	EIGHT_BOTTOM,
	EIGHT_TOP_BUNT,
	EIGHT_END
};
enum aero_state_t aero_state = WAIT_FOR_SWITCH_LOW;
enum aero_state_t next_aero_state = WAIT_FOR_SWITCH_LOW;

enum aero_state_t old_aero_state = MANUEVER_IDLE;
int aero_timer = 0;
int old_aero_ctrl_chan = 5;
int mission_command = 0;
bool allow_mission_command = 0;
int number_of_rolls = 1;
FlightMode set_mode = MANUAL; // used to look for external mode change
#define AERO_DEBUG

class Plane;

Aerobatics::Aerobatics() {
	aero_state = WAIT_FOR_SWITCH_LOW;
}

// called by scheduler
void Plane::aerobatics_update() {

	aerobatics.update();
}

// gets called from do_parachute in commands_logic.cpp
// command is param 1
void Aerobatics::aero_command(int command) {
	mission_command = command;

}

// checks to see if the next maneuver should be started or continued
// Operates in mission control mode or switch control mode
// In switch control mode the switch must first go low, then a maneuver is triggered by going higher
// value chooses the actual maneuver
// A maneuver can be aborted by taking switch low, an abort also occurs if the altitude is less than min_alt

void Aerobatics::update() {
	//  Note:  plane is a global
	// this should be called at 50 Hz

	AeroEnum do_this_aero = NONE;
	// due to lack of parameter space, the elevator and aileron movements share a location
	int loop_elev = plane.g.aero_elev_ail / 10000;
	int roll_ail = plane.g.aero_elev_ail - loop_elev * 10000;
	int ctrl_channel = plane.g.aero_ctrl_chan & 15;
	int thr_chan = plane.rcmap.throttle() - 1;
	int roll_chan = plane.rcmap.roll() - 1;
	int pitch_chan = plane.rcmap.pitch() - 1;
	int loop_thr = 1800;	//plane.g.aero_lthr_rthr /10000;
	int roll_thr = 1800; //plane.g.aero_lthr_rthr- loop_thr*10000;
	int acc_thr = 1800; // used at start to accelerate to maneuver speed
	// make sure that the programmed parameters are within range.
	loop_elev = limit_value(loop_elev, plane.channel_pitch->get_radio_min(),
			plane.channel_pitch->get_radio_max());
	roll_ail = limit_value(roll_ail, plane.channel_roll->get_radio_min(),
			plane.channel_roll->get_radio_max());
	loop_thr = limit_value(loop_thr, plane.channel_throttle->get_radio_min(),
			plane.channel_throttle->get_radio_max());
	roll_thr = limit_value(roll_thr, plane.channel_throttle->get_radio_min(),
			plane.channel_throttle->get_radio_max());

#ifdef AERO_DEBUG
	if (aero_state != old_aero_state) {
		// send gcs message on change of state
		switch (aero_state) {
			case WAIT_FOR_SWITCH_LOW:
			send_message("WAIT_FOR_SWITCH_LOW");
			break;
			case MANUEVER_IDLE:
			send_message("MANUEVER_IDLE");
			break;
			case LOOP_START:
			send_message("LOOP_START");
			break;
			case TOP_OF_LOOP:
			send_message("TOP_OF_LOOP");
			break;
			case BOTTOM_OF_LOOP:
			send_message("BOTTOM_OF_LOOP");
			break;
			case START_STABILISE:
			send_message("START_STABILISE");
			break;
			case STABILISE_TIME:
			send_message("STABILISE_TIME");
			break;
			case ROLL_START:
			send_message("ROLL_START");
			break;
			case ROLL_TOP:
			send_message("ROLL_TOP");
			break;
			case ROLL_BOTTOM:
			send_message("ROLL_BOTTOM");
			break;
			case CUBAN_START:
			send_message("CUBAN_START");
			break;
			case CUBAN_TOP_1:
			send_message("CUBAN_TOP_1");
			break;
			case CUBAN_START_ROLL:
			send_message("CUBAN_START_ROLL");
			break;
			case CUBAN_ROLL_END:
			send_message("CUBAN_ROLL_END");
			break;
			case CUBAN_LOOP_START:
			send_message("CUBAN_LOOP_START");
			break;
			case CUBAN_LOOP_BOTTOM:
			send_message("CUBAN_LOOP_BOTTOM");
			break;
			case CUBAN_TOP_2:
			send_message("CUBAN_TOP_2");
			break;
			case CUBAN_START_ROLL_2:
			send_message("CUBAN_START_ROLL_2");
			break;
			case CUBAN_ROLL_END_2:
			send_message("CUBAN_ROLL_END_2");
			break;
			case CUBAN_LOOP_START2:
			send_message("CUBAN_LOOP_START2");
			break;
			case CUBAN_LOOP_BOTTOM_2:
			send_message("CUBAN_LOOP_BOTTOM_2");
			break;
			case EIGHT_START:
			send_message("EIGHT_START");
			break;
			case EIGHT_TOP:
			send_message("EIGHT_TOP");
			break;
			case EIGHT_DOWN:
			send_message("EIGHT_DOWN");
			break;
			case EIGHT_BOTTOM:
			send_message("EIGHT_BOTTOM");
			break;
			case EIGHT_TOP_BUNT:
			send_message("EIGHT_TOP_BUNT");
			break;
			case EIGHT_END:
			send_message("EIGHT_END");
			break;

			default:
			break;
		}

	}
	old_aero_state = aero_state;
#endif
	// disable if control channel is zero
	// if it has changed also release all channels
	if (ctrl_channel == 0) {
		if (old_aero_ctrl_chan != 0)
			release_all_rc_chans();
		old_aero_ctrl_chan = 0;
		return;
	}
	old_aero_ctrl_chan = ctrl_channel;
	if (aero_timer > 0)
		aero_timer--;	// counts 20 msec periods, limit to stop overflow
	if (plane.control_mode >= INITIALISING) {
		aero_state = WAIT_FOR_SWITCH_LOW;
		return;
	}
	// test the 5th bit
	if ((plane.g.aero_ctrl_chan & 16) != 0) {
		allow_mission_command = true;
	} else {
		allow_mission_command = false;
	}

	AeroEnum switch_state = read_aero_switch(ctrl_channel);

	if (allow_mission_command) {
		if (switch_state != NONE) {
			// mission_command set by do_parachute()
			do_this_aero = convert_mission_command(mission_command);
		}
	} else {
		do_this_aero = switch_state;
	}
	mission_command = 0; // clear the mission request once logged
	// pitch only goes +-90
	float pitch = degrees(plane.ahrs.pitch) - plane.g.pitch_trim_cd * 0.01f;
	float roll = degrees(plane.ahrs.roll);
	// make roll go 0-360 instead of +-180
	if (roll < 0)
		roll = 360 + roll;

	// main aerobatic state machine
	switch (aero_state) {
	case ACC_FOR_MANUEVER:
		plane.control_mode = FLY_BY_WIRE_A;
		set_mode = FLY_BY_WIRE_A;
		set_rc_chan_val(plane.rcmap.throttle() - 1, acc_thr);

		if (aero_timer <= 0) {
			plane.control_mode = ACRO;
			set_mode = ACRO;
			aero_state = next_aero_state;
		}
		break;
	case WAIT_FOR_SWITCH_LOW:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1000);
#endif
		if (switch_state == NONE) {
			aero_state = MANUEVER_IDLE;
		}
		break;
	case MANUEVER_IDLE:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1100);
#endif
		// record start mode so we can return after maneuver
		start_mode = plane.control_mode;
		aero_timer=100;
		switch (do_this_aero) {
		case LOOP:
			send_message("Loop"); // send gcs message
			acc_thr = loop_thr;
			aero_state = ACC_FOR_MANUEVER;
			next_aero_state = LOOP_START;
			break;
		case ROLL:
			send_message("Roll");
			acc_thr = roll_thr;
			number_of_rolls = 1;
			aero_state = ACC_FOR_MANUEVER;
			next_aero_state = ROLL_START;
			break;
		case TWO_ROLLS:
			send_message("Two Rolls");
			number_of_rolls = 2;
			acc_thr = roll_thr;
			aero_state = ACC_FOR_MANUEVER;
			next_aero_state = ROLL_START;
			break;
		case CUBAN_EIGHT:
			send_message("CUBAN EIGHT");
			acc_thr = loop_thr;
			aero_state = ACC_FOR_MANUEVER;
			next_aero_state = CUBAN_START;
			break;
		case EIGHT:
			send_message("EIGHT");
			acc_thr = loop_thr;
			aero_state = ACC_FOR_MANUEVER;
			next_aero_state = EIGHT_START;
			break;
		case NONE:
			release_all_rc_chans();
			break;
		default:
			// this shouldn't be required, but just in case, make sure we have control in IDLE
			//	send_message("NONE");
			release_all_rc_chans();
			break;
		} // end switch (do_this_aero)
		break;
		//#############################################
	case LOOP_START:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1200);
#endif
		start_aero(0, loop_elev, loop_thr, 0);
		if (pitch > 45) {
			aero_state = TOP_OF_LOOP;
		}

		break;
	case TOP_OF_LOOP:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1300);
#endif

		if (pitch < -25) {
			aero_state = BOTTOM_OF_LOOP;
			set_rc_chan_val(thr_chan, 0);
		}

		break;
	case BOTTOM_OF_LOOP:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1400);
#endif

		if (pitch > -10) {
			aero_state = START_STABILISE;
		}

		break;
		//#############################################
	case START_STABILISE:
		// come here at the end of every maneuver and after an abort
		aero_state = STABILISE_TIME;
		plane.control_mode = FLY_BY_WIRE_A;
		set_mode = FLY_BY_WIRE_A;
		mission_command = 0;
		aero_timer = 100;
		release_all_rc_chans();
		break;
	case STABILISE_TIME:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1500);
#endif
		if (aero_timer <= 0) {
			plane.control_mode = start_mode;
			set_mode = start_mode;
			if (allow_mission_command) {
				aero_state = MANUEVER_IDLE;
			} else {
				aero_state = WAIT_FOR_SWITCH_LOW;
			}
		}
		break;
		//#############################################
	case ROLL_START:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1600);
#endif
		start_aero(roll_ail, 0, roll_thr, 0);
		if (roll > 45 && roll < 315) {
			aero_state = ROLL_TOP;
		}

		break;
	case ROLL_TOP:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1700);
#endif

		if (roll > 225) {
			aero_state = ROLL_BOTTOM;
		}

		break;
	case ROLL_BOTTOM:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1800);
#endif

		if (roll > 350 || roll < 45) {
			number_of_rolls--;
			if (number_of_rolls > 0)
				aero_state = ROLL_START;
			else
				aero_state = START_STABILISE;
		}

		break;

		//#############################################
	case CUBAN_START:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1510);
#endif
		start_aero(0, loop_elev, loop_thr, 0);
		if (pitch > 45) {
			aero_state = CUBAN_TOP_1;
		}

		break;
	case CUBAN_TOP_1:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1511);
#endif

		if (pitch < -35) {
			aero_state = CUBAN_START_ROLL;
			//	set_rc_chan_val(thr_chan, 0);
			set_rc_chan_val(pitch_chan, 0);
			aero_timer = 10;

		}

		break;
	case CUBAN_START_ROLL:

		if (aero_timer <= 0) {
			set_rc_chan_val(roll_chan, roll_ail);
			aero_state = CUBAN_ROLL_END;
		}

		break;
	case CUBAN_ROLL_END:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1512);
#endif

		if (roll < 10 || roll > 350) {
			aero_state = CUBAN_LOOP_START;
			set_rc_chan_val(thr_chan, loop_thr);
			set_rc_chan_val(roll_chan, 0);
			aero_timer = 10;
		}

		break;
	case CUBAN_LOOP_START:
		if (aero_timer <= 0) {
			set_rc_chan_val(pitch_chan, loop_elev);
			aero_state = CUBAN_LOOP_BOTTOM;
		}
		break;
	case CUBAN_LOOP_BOTTOM:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1513);
#endif
		if (pitch > 45) {
			aero_state = CUBAN_TOP_2;
		}
		break;
	case CUBAN_TOP_2:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1514);
#endif

		if (pitch < -35) {
			aero_state = CUBAN_START_ROLL_2;
			set_rc_chan_val(thr_chan, 0);
			set_rc_chan_val(pitch_chan, 0);
			aero_timer = 10;
		}

		break;
	case CUBAN_START_ROLL_2:
		// use a timer to separate the elevator and roll controls, to avoid a spin
		if (aero_timer <= 0) {
			set_rc_chan_val(roll_chan, roll_ail);
			aero_state = CUBAN_ROLL_END_2;
		}
		break;
	case CUBAN_ROLL_END_2:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1515);
#endif

		if (roll < 10 || roll > 350) {
			aero_state = CUBAN_LOOP_START2;
			aero_timer = 10;
			set_rc_chan_val(thr_chan, loop_thr);
			set_rc_chan_val(roll_chan, 0);
		}

		break;
	case CUBAN_LOOP_START2:
		if (aero_timer <= 0) {
			set_rc_chan_val(pitch_chan, loop_elev);
			aero_state = CUBAN_LOOP_BOTTOM_2;
		}
		break;
	case CUBAN_LOOP_BOTTOM_2:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1516);
#endif

		if (pitch > -10) {
			aero_state = START_STABILISE;
		}

		break;

//######################################################
	case EIGHT_START:

#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1200);
#endif
		start_aero(0, loop_elev, loop_thr, 0);
		if (pitch > 45) {
			aero_state = EIGHT_TOP;
		}
		break;
	case EIGHT_TOP:

		if (pitch < -45) {
			aero_state = EIGHT_DOWN;
		}

		break;

	case EIGHT_DOWN:
		// detect near vertical down
		// at 90 degrees roll flips from 180 back to zero.
		if ((pitch < -70) || (roll < 135)) {
			aero_state = EIGHT_BOTTOM;
			int trim = plane.channel_pitch->get_radio_trim();
			// apply same down elevator as up
			set_rc_chan_val(pitch_chan, (trim - loop_elev) + trim);
		}
		break;
	case EIGHT_BOTTOM:
		// approaching top of bunt
		if (pitch > 70) {
			aero_state = EIGHT_TOP_BUNT;
		}
		break;
	case EIGHT_TOP_BUNT:

		if (pitch < -70) {
			aero_state = EIGHT_END;
			set_rc_chan_val(pitch_chan, loop_elev);
		}

		break;
	case EIGHT_END:

		if (pitch > -10) {
			aero_state = START_STABILISE;
		}
		break;
//######################################################
	default:
#ifdef AERO_DEBUG
		set_rc_chan_val(6, 1950);
#endif
		aero_state = WAIT_FOR_SWITCH_LOW;
		break;
	} // end of switch (aero_state)
//----------------------------------------------------
	// check for an abort condition where required
	if ((aero_state != MANUEVER_IDLE) && (aero_state != WAIT_FOR_SWITCH_LOW)
			&& (aero_state != START_STABILISE)
			&& (aero_state != STABILISE_TIME)) {
		check_for_abort(switch_state);
	}

} // end update_state()

void Aerobatics::send_message(const char *message) {
	plane.gcs().send_text(MAV_SEVERITY_WARNING, message);

} // end send_message()

int Aerobatics::limit_value(int value, int min, int max) {
	int _value = value;

	if (_value > max)
		_value = max;
	if (_value < min)
		_value = min;
	return _value;
} // end limit_value()

AeroEnum Aerobatics::convert_mission_command(int command) {
	// convert mission command integer to AeroEnum
	// make sure there are no illegal commands
	switch (command) {

	case NONE:
		return NONE;
	case LOOP:
		return LOOP;
	case ROLL:
		return ROLL;
	case TWO_ROLLS:
		return TWO_ROLLS;
	case CUBAN_EIGHT:
		return CUBAN_EIGHT;
	case EIGHT:
		return EIGHT;
	case STALL_TURN:
		return STALL_TURN;
	default:
		return NONE;

	} // end switch command

} // end convert_mission_command()

void Aerobatics::start_aero(int roll, int pitch, int thr, int yaw) {
	// sets the specified channels, use 0 to leave as is
	aero_timer = 0;
	set_rc_chan_val(plane.rcmap.roll() - 1, roll);
	set_rc_chan_val(plane.rcmap.pitch() - 1, pitch);
	set_rc_chan_val(plane.rcmap.throttle() - 1, thr);
	set_rc_chan_val(plane.rcmap.yaw() - 1, yaw);
}

bool Aerobatics::check_for_abort(AeroEnum switch_state) {

	if (switch_state == NONE || plane.relative_altitude < plane.g.aero_min_alt
			|| set_mode != plane.control_mode) {
		// abort
		//	send_message("ABORT");
		aero_state = START_STABILISE;
		return true;
	}
	return false;
}

AeroEnum Aerobatics::read_aero_switch(int ctrl_chan) {
	uint16_t pulsewidth = hal.rcin->read(ctrl_chan - 1);

	if (pulsewidth > 2000 || pulsewidth < 1200)
		return NONE;
	if (pulsewidth > 1200 && pulsewidth < 1400)
		return CUBAN_EIGHT;
	if (pulsewidth > 1400 && pulsewidth < 1600)
		return EIGHT;
	if (pulsewidth > 1600 && pulsewidth < 1800)
		return ROLL;
	return LOOP;

}

void Aerobatics::set_rc_chan_val(int chan, int val) {
	// force the RC input to a value, channels are 0 to n
	hal.rcin->set_override(chan, val);
} // end set_rc_chan_val

void Aerobatics::release_all_rc_chans() {
#ifdef AERO_DEBUG
	//send_message("Release all channels");
#endif
	hal.rcin->set_override(0, 0);
	hal.rcin->set_override(1, 0);
	hal.rcin->set_override(2, 0);
	hal.rcin->set_override(3, 0);
}
