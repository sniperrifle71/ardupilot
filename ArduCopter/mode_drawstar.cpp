#include "Copter.h"


// init - initialise guided controller
bool ModeDrawStar::init(bool ignore_checks)
{
    if(ignore_checks){
        pos_control_start();
        return true;
    }
    else{return false;}

};

// run - runs the guided controller
// should be called at 100hz or more
void ModeDrawStar::run()
{
    pos_control_run();
 }



bool ModeDrawStar::allows_arming(AP_Arming::Method method) const
{
    // always allow arming from the ground station or scripting
    if (AP_Arming::method_is_GCS(method) || method == AP_Arming::Method::SCRIPTING) {
        return true;
    }

    // optionally allow arming from the transmitter
    return option_is_enabled(Option::AllowArmingFromTX);
};


// initialise guided mode's position controller
void ModeDrawStar::pos_control_start()
{
    // initialise position controller
    pva_control_start();
}

void ModeGuided::pva_control_start()
{
    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);


}


// pos_control_run - runs the guided position controller
// called from guided_run
void ModeDrawStar::pos_control_run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        // do not spool down tradheli when on the ground with motor interlock enabled
        make_safe_ground_handling(copter.is_tradheli() && motors->get_interlock());
        return;
    }

    // calculate terrain adjustments
    float terr_offset = 0.0f;
    if (guided_pos_terrain_alt && !wp_nav->get_terrain_offset(terr_offset)) {
        // failure to set destination can only be because of missing terrain data
        copter.failsafe_terrain_on_event();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // send position and velocity targets to position controller
    guided_accel_target_cmss.zero();
    guided_vel_target_cms.zero();

    // stop rotating if no updates received within timeout_ms
    if (millis() - update_time_ms > get_timeout_ms()) {
        if ((auto_yaw.mode() == AutoYaw::Mode::RATE) || (auto_yaw.mode() == AutoYaw::Mode::ANGLE_RATE)) {
            auto_yaw.set_mode(AutoYaw::Mode::HOLD);
        }
    }

    float pos_offset_z_buffer = 0.0; // Vertical buffer size in m
    if (guided_pos_terrain_alt) {
        pos_offset_z_buffer = MIN(copter.wp_nav->get_terrain_margin() * 100.0, 0.5 * fabsF(guided_pos_target_cm.z));
    }
    pos_control->input_pos_xyz(guided_pos_target_cm, terr_offset, pos_offset_z_buffer);

    // run position controllers
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller with auto yaw
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), auto_yaw.get_heading());
}

// helper function to set yaw state and targets
void ModeDrawStar::set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    if (use_yaw && relative_angle) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw && use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, yaw_rate_cds * 0.01f);
    } else if (use_yaw && !use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, 0.0f);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    } else {
        auto_yaw.set_mode_to_default(false);
    }
}


