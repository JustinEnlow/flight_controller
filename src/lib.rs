//! # Flight Control System(FCS)

use std::ops::Add;
use game_utils::{
    control_axis::ControlAxis,
    dimension3::Dimension3
};



pub mod feedback_controller;
pub mod feedforward_controller;
pub mod g_force_safety;
pub mod propulsion_control;




// typical behavior

// get feedforward_desired_acceleration = if autonomous_mode.enabled{
    // flight_controller::feedforward_controller::calculate_autonomous_mode_acceleration()
// }
// else{
    // flight_controller::feedforward_controller::calculate_pilot_control_mode_acceleration()
// }
// get feedback_desired_accel = flight_controller::feedback_controller::calculate
// sum both for desired_accel

// if gsafety enabled, limit desired_accel to gsafe limits

// send output to propulsion control

    

    //should we be requesting propulsion control to calculate available acceleration here, instead of feeding that value in?
    //no. we will calculate available acceleration only whenever the thruster suite is changed in some intentional way
    //unintentional changes, such as thruster failure, can be dealt with elsewhere

    //let mut desired_acceleration = sum_acceleration(
    //    if autonomous_mode.enabled(){
    //        flight_controller::feedforward_controller::calculate_autonomous_mode_acceleration()
    //    }else{
    //        flight_controller::feedforward_controller::calculate_pilot_control_mode_acceleration()
    //    },
    //    feedback_controller::calculate(
    //        //goal_position will be input if autonomous mode, or expected position/attitude as calculcated 
    //        //by results of thruster output if in pilot controlled mode
    //    )
    //);

    //if gsafety.enabled(){
    //    g_force_safety::process(
    //        &mut desired_acceleration,
    //        gsafety_max_acceleration
    //    );
    //}

    /////////////////////////////////////////////////////////////////////////////
    // propulsion_control_system
    /////////////////////////////////////////////////////////////////////////////
    //Once the desired linear and rotational accelerations are established from the combined feedforward
    //and feedback control signals, the PCS must calculate the output of individual thrusters, as well as other
    //devices tasked with generating motion, so that these accelerations will be achieved to within a
    //reasonable degree of accuracy.
    //propulsion_control::calculate_thruster_output(
    //    &desired_acceleration,
    //    thruster_suite,
    //    mass,
    //    zero,
    //    output_thrust
    //);

    /////////////////////////////////////////////////////////////////////////////
    // calculate expected position from resultant thrusts/accelerations/velocities
    /////////////////////////////////////////////////////////////////////////////





pub fn sum_acceleration<T: Copy + Add<Output = T>>(
    feedforward: ControlAxis<Dimension3<T>>, feedback: ControlAxis<Dimension3<T>>
) -> ControlAxis<Dimension3<T>>{
    ControlAxis::new(
        Dimension3::new(
            feedforward.linear().x() + feedback.linear().x(), 
            feedforward.linear().y() + feedback.linear().y(), 
            feedforward.linear().z() + feedback.linear().z(),
        ), 
        Dimension3::new(
            feedforward.rotational().x() + feedback.rotational().x(), 
            feedforward.rotational().y() + feedback.rotational().y(), 
            feedforward.rotational().z() + feedback.rotational().z(),
        ),
    )
}