#ifndef IK_CALC_C
#define IK_CALC_C

#define _SUPPRESS_PLIB_WARNING  // Removes spam
#define _DISABLE_OPENADC10_CONFIGPORT_WARNING
#include <xc.h>
#include <math.h>
#include <plib.h>
#include "main_declarations.h"
#include "ik_calc.h"
#include "toolpaths.h"

UINT8 ik_calc(float x, float y, float* a1, float* a2) {
  float l = sqrtf(x*x + y*y);
  unsigned char return_value = 0;

  // If it can't reach the point, default to best position
  if (l > (L1+L2)) {
    *a1 = atan2f(y,x);
    *a2 = 0;
    return_value = 1;
  } else if ( (L1 > L2) && (l < (L1-L2)) ) {
    *a1 = atan2f(y,x);
    *a2 = M_PI;
    return_value = 2;
  }
  else {
    *a1 = atan2f(y,x) - acosf((L2*L2-l*l-L1*L1)/(-2*l*L1));
    *a2 = M_PI - acosf((l*l-L1*L1-L2*L2)/(-2*L1*L2));
  }
  return return_value;
}

// Does a defined toolpath
#define DX (curr_element.e1-x)
#define DY (curr_element.e2-y)
#define MOVE_LEN (sqrtf(powf(DX,2) + powf(DY,2)))
UINT8 ik_path(float time, UINT8* command_type, float* a1, float* a2) {
    static float x, y, v, vx, vy, start_wait = 0, old_move_len;
    static INT16 command_pos = 0;
    static UINT8 curr_pos_read = 0;
    UINT8 perform_ik = 1;

    // reset if time resets from zero (new session)
    static float prev_time = 1e10;
    if (prev_time > time) {
        command_pos = 0;
        curr_pos_read = 0;
        start_wait = 0;
        x = L1 + L2;
        y = 0;
        v = DEFAULT_SPEED;
    }

    if ( (command_pos >= toolpath.size) || command_pos < 0 ){
        *command_type = ELEMENT_TYPE_EXIT;
        return 255;
    }
    element curr_element = toolpath.path[command_pos];

    if (curr_pos_read) { // Continuation of old command
        switch(curr_element.type) {
            case ELEMENT_TYPE_DO_NOTHING:
                if (time > (start_wait + curr_element.e1)){
                    command_pos += 1;
                    curr_pos_read = 0;
                }
                break;
            case ELEMENT_TYPE_INTERPOLATE_POS:
                //sprintf(buf,"----------- x = %f, y = %f\n\r",x,y);
                //putsUART1(buf);
                if (start_wait > 0) {
                    /* Waits a bit at the end of an interpolate pos command to
                     * make sure the end effector has time to move to the
                     * position. This is not optimal and needs redesign,
                     * preferrably through the use of some trajectory planning
                     * algorithm
                     */
                    if (time > (start_wait + INTERPOLATE_POS_END_DELAY)) {
                        command_pos += 1;
                        curr_pos_read = 0;
                    }
                } else if (old_move_len > MOVE_LEN) {
                    old_move_len = MOVE_LEN;
                    x += vx*(time-prev_time);
                    y += vy*(time-prev_time);
                } else {
                    x = curr_element.e1;
                    y = curr_element.e2;
                    start_wait = time;
                    //command_pos += 1;
                    //curr_pos_read = 0;
                }
                break;
            default:
                command_pos += 1;
                curr_pos_read = 0;
                break;
        }
    } else { // New command
        switch(curr_element.type) {
            case ELEMENT_TYPE_DO_NOTHING:
                start_wait = time;
                curr_pos_read = 1;
                break;
            case ELEMENT_TYPE_MOVE_TO_POS:
                x = curr_element.e1;
                y = curr_element.e2;
                command_pos += 1;
                break;
            case ELEMENT_TYPE_SET_SPEED:
                v = curr_element.e1;
                command_pos += 1;
                break;
            case ELEMENT_TYPE_INTERPOLATE_POS:
                vx = v*DX/MOVE_LEN;
                vy = v*DY/MOVE_LEN;
                old_move_len = 1.2*MOVE_LEN;
                curr_pos_read = 1;
                start_wait = 0;
                break;
            case ELEMENT_TYPE_SET_ANGLES:
                *a1 = curr_element.e1;
                *a2 = curr_element.e1;
                perform_ik = 0;
                command_pos += 1;
                break;
            case ELEMENT_TYPE_DEPLOY_END_EFFECTOR:
            case ELEMENT_TYPE_RETRACT_END_EFFECTOR:
            case ELEMENT_TYPE_EXIT:
                command_pos += 1;
                break;
            case ELEMENT_TYPE_GO_TO_ABS_COMMAND:
                command_pos = (int)(curr_element.e1 + 0.1);
                break;
            case ELEMENT_TYPE_GO_TO_REL_OFFSET:
                command_pos += (int)(curr_element.e1 + 0.1);
                break;
        }

    }

    prev_time = time;
    *command_type = curr_element.type;
    if (perform_ik)
        return ik_calc(x,y,a1,a2);
    else
        return 0;
}

// Automatically generated circle toolpath
UINT8 ik_circle(float time, UINT8* command_type, float* a1, float* a2) {
    static UINT8 state = 0;

    static float prev_time = 1e10;
    // Reset it if time resets from zero
    if(prev_time > time) {
        state = 0;
    }
    prev_time = time;

    float circ_angle,x,y;
    // Default position
    x = CIRCLE_MID_X + CIRCLE_R;
    y = CIRCLE_MID_Y;

    switch(state) {
        case 0: // Begin by moving end effector to default position
            *command_type = ELEMENT_TYPE_MOVE_TO_POS;
            state = 1;
            break;
        case 1: // Wait until deploy end effector
            if (time > DELAY_START_TO_DEPLOY) {
                *command_type = ELEMENT_TYPE_DEPLOY_END_EFFECTOR;
                state = 2;
            } else {
                *command_type = ELEMENT_TYPE_DO_NOTHING;
            }
            break;
        case 2: // Wait until start moving
            if (time > DELAY_START_TO_CIRCLE) {
                state = 3;
            }
            *command_type = ELEMENT_TYPE_DO_NOTHING;
            break;
        case 3: // Perform the circle movements
            circ_angle = CIRCLE_ROTATIONAL_SPEED*(time-DELAY_START_TO_CIRCLE);
            x = CIRCLE_MID_X + CIRCLE_R*cosf(circ_angle);
            y = CIRCLE_MID_Y + CIRCLE_R*sinf(circ_angle);
            *command_type = ELEMENT_TYPE_INTERPOLATE_POS;
            break;
    }
    return ik_calc(x,y,a1,a2); // Could be optimized
}

#endif