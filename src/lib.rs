//! # Flight Control System(FCS)

use std::ops::{Mul, Div, Add, Sub, Neg};
use std::cmp::PartialOrd;
use pid_controller::PID;
use game_utils::{
    control_axis::{ControlAxis, AxisContribution}, 
    toggle::Toggle, 
    dimension3::{Dimension3, ClampedDimension3}
};
//use propulsion_control::{ThrusterMountPoint, Thruster};



pub mod feedback_controller;
pub mod feedforward_controller;
pub mod g_force_safety;
pub mod propulsion_control;



pub fn calculate<T>(
    autonomous_mode: &Toggle,
    //position: &ControlAxis<Dimension3<T>>,  //from positioning system
    velocity: &ControlAxis<Dimension3<T>>,  //from IMU
    max_velocity: &ControlAxis<Dimension3<T>>,
    linear_assist: &Toggle,
    rotational_assist: &Toggle,
    gsafety: &Toggle,
    gsafety_max_acceleration: &AxisContribution<ControlAxis<ClampedDimension3<T>>>,
    input: &ControlAxis<Dimension3<T>>,
    pid6dof: &mut ControlAxis<Dimension3<PID<T>>>,
    delta_time: T,
    zero: T,
    // max accel determined by (physical max or user defined virtual max)thrust * mass, or by gsafety settings
    available_acceleration: &AxisContribution<ControlAxis<Dimension3<T>>>,
) -> ControlAxis<Dimension3<T>>
    where
        T: Mul<Output = T>
        + Div<Output = T>
        + Add<Output = T>
        + Sub<Output = T>
        + Neg<Output = T>
        + PartialOrd 
        + Copy
{
    //should we be requesting propulsion control to calculate available acceleration here, instead of feeding that value in?

    let mut desired_acceleration = sum_desired_acceleration(
        feedforward_controller::calculate(
            input, 
            linear_assist, 
            rotational_assist, 
            max_velocity, 
            velocity, 
            delta_time,
            zero, 
            autonomous_mode,
            available_acceleration
        ),
        feedback_controller::calculate(
            //goal_position will be input if autonomous mode, or expected position/attitude as calculcated 
            //by results of thruster output if in pilot controlled mode
            &ControlAxis::new(Dimension3::default(zero), Dimension3::default(zero)),
            &ControlAxis::new(Dimension3::default(zero), Dimension3::default(zero)),
            pid6dof,
            delta_time,
            zero,
            available_acceleration
        )
    );

    if gsafety.enabled(){
        g_force_safety::process(
            &mut desired_acceleration,
            gsafety_max_acceleration
        );
    }

    desired_acceleration

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
}





fn sum_desired_acceleration<T: Copy + Add<Output = T>>(
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