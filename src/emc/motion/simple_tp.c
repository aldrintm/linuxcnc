/* based on limit3.comp by John Morris (zultron)

      limit3 halpin --> struct_tp_t
      --------          -----------
      in            --> tp->pos_cmd
      out           --> tp->curr_pos
      in_limit      --> notused, removed
      load          --> removed, use tp->enable for control
      min_          --> tp->min_pos
      min_          --> tp->min_pos
      max_          --> tp->max_pos
      maxv          --> tp->max_vel
      maxa          --> tp->max_acc

      limit3 var    --> struct_tp_t
      -------------     -----------
      in_pos_old    --> tp->in_pos_old  (new struct item)
      out_vel_old   --> tp->out_vel_old (new struct item)

      limit3 local  --> struct_tp_t
      ----------        -----------
      in_pos_lim    --> notused, removed

Notes:
  1) tp->min_pos,max_pos are not used by joint->free_tp, axis->teleop_tp
     since limits are managed elsewhere in the motion module
  2) tp->disallow_backoff
         0: (DEFAULT) allow backoff to mitigate overshoot
         1: disallow backoff for special cases
*/
#include "simple_tp.h"
#include "rtapi_math.h"

#define SET_NEXT_STATE(out_pos, out_vel, in_pos)          \
    do {                                                  \
        tp->curr_pos    = out_pos;                        \
        tp->out_vel_old = out_vel;                        \
        tp->curr_vel    = out_vel;                        \
        tp->in_pos_old  = in_pos;                         \
        tiny_dp = fabs(TINY_DP(tp->max_acc,fperiod));     \
        if (fabs(tp->curr_pos - tp->pos_cmd) < tiny_dp) { \
          tp->active = 0;                                 \
        }                                                 \
        return;                                           \
    } while (0)

#define VALID_NEXT(pos, vel) ((pos) <= max_pos && (pos) >= min_pos \
                              && (vel) <= max_vel && (vel) >= min_vel)

void simple_tp_update(simple_tp_t *tp, double fperiod)
{
    double in_vel, min_vel, max_vel, min_pos, max_pos;
    double vel_0_time, vel_0_pos;
    double vel_match_time, vel_match_in_pos, vel_match_out_pos;
    int    out_dir, out_dir_rel;
    double tiny_dp;

    tp->active = 1; // disprove

    if (!tp->enable) {
        tp->pos_cmd = tp->curr_pos;
        SET_NEXT_STATE(tp->curr_pos, 0, tp->curr_pos);
    }

    // Input velocity
    in_vel = (tp->pos_cmd - tp->in_pos_old) / fperiod;
    // Most negative/positive velocity reachable in one period
    min_vel = fmax(tp->out_vel_old - tp->max_acc * fperiod, -tp->max_vel);
    max_vel = fmin(tp->out_vel_old + tp->max_acc * fperiod, tp->max_vel);
    // Most negative/positive position reachable in one period
    min_pos = tp->curr_pos + min_vel * fperiod;
    max_pos = tp->curr_pos + max_vel * fperiod;

    // Direction (sign) of output movement
    out_dir = (tp->out_vel_old < 0) ? -1 : 1;
    // Direction of output movement relative to input movement
    out_dir_rel = (tp->out_vel_old - in_vel < 0) ? -1 : 1;

    // Respect max/min position limits:  stop at limit line
    vel_0_time = fabs(tp->out_vel_old/tp->max_acc); // min time to decel to stop
    vel_0_pos = tp->curr_pos                        // position after stop
        + tp->out_vel_old * (vel_0_time+fperiod)
        + 0.5 * (-out_dir * tp->max_acc) * pow(vel_0_time,2);

    // Follow input signal:  match position and velocity
    // - min time for velocity match
    vel_match_time = fabs(tp->out_vel_old-in_vel) / tp->max_acc;
    // - input position after velocity match
    vel_match_in_pos = tp->pos_cmd + in_vel * vel_match_time;
    // - output position after velocity match
    vel_match_out_pos = tp->curr_pos
        + tp->out_vel_old * (vel_match_time+fperiod)
        + 0.5 * (-out_dir_rel * tp->max_acc) * pow(vel_match_time,2);

    // Respect max/min position limits
    //
    // - If not at the limit line but in danger of overshooting it,
    //   slow down
    if (vel_0_pos >= tp->max_pos && !VALID_NEXT(tp->max_pos,0)) // can't follow max limit
        SET_NEXT_STATE(min_pos, min_vel, tp->pos_cmd);
    if (vel_0_pos <= tp->min_pos && !VALID_NEXT(tp->min_pos,0)) // can't follow min limit
        SET_NEXT_STATE(max_pos, max_vel, tp->pos_cmd);
    // - If input signal is headed out of bounds, or headed in bounds
    //   but no danger of overshooting, the limit is the goal
    if ((vel_match_in_pos < tp->min_pos) // Input below min limit
        || (tp->pos_cmd <= tp->min_pos && vel_match_in_pos < vel_match_out_pos)) {
        if (VALID_NEXT(tp->min_pos,0))
            SET_NEXT_STATE(tp->min_pos, 0, tp->pos_cmd);   // - Park at min limit
        else
            SET_NEXT_STATE(min_pos, min_vel, tp->pos_cmd); // - Head toward min limit
    }
    if ((vel_match_in_pos > tp->max_pos)                   // Input above max limit
        || (tp->pos_cmd >= tp->max_pos && vel_match_in_pos > vel_match_out_pos)) {
        if (VALID_NEXT(tp->max_pos,0))
            SET_NEXT_STATE(tp->max_pos, 0, tp->pos_cmd);   // - Park at max limit
        else
            SET_NEXT_STATE(max_pos, max_vel, tp->pos_cmd); // - Head toward min limit
    }

    // Follow input signal
    //
    // - Try to track input
    if (VALID_NEXT(tp->pos_cmd, in_vel))
        SET_NEXT_STATE(tp->pos_cmd, in_vel, tp->pos_cmd);
    // - Try to match position and velocity without overshooting
    if (tp->curr_pos > tp->pos_cmd) {                       // Output > input:
        if (vel_match_in_pos < vel_match_out_pos)           // - Not overshooting
            SET_NEXT_STATE(min_pos, min_vel, tp->pos_cmd);  //   - Move closer
        else                                                // - Overshooting
            if (tp->disallow_backoff) {
                SET_NEXT_STATE(min_pos, min_vel, tp->pos_cmd);  // for eoffset_pid
            } else {
                SET_NEXT_STATE(max_pos, max_vel, tp->pos_cmd);  //   - Back off
            }
    }
    if (tp->curr_pos < tp->pos_cmd) {                       // Output < input
        if (vel_match_in_pos > vel_match_out_pos)           // - Not overshooting
            SET_NEXT_STATE(max_pos, max_vel, tp->pos_cmd);  //   - Move closer
        else                                                // - Overshooting
            if (tp->disallow_backoff) {
                SET_NEXT_STATE(max_pos, max_vel, tp->pos_cmd);  // for eoffset_pid
            } else {
                SET_NEXT_STATE(min_pos, min_vel, tp->pos_cmd);  //   - Back off
            }
    }
}
