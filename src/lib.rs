//! # Flight Control System(FCS)

use std::ops::{Add, Sub, Mul, Div, Neg};
use game_utils::{
    control_axis::{ControlAxis, AxisContribution},
    dimension3::Dimension3,
    toggle::Toggle,
};
use pid_controller::PID;
use inertial_measurement::IMU;



pub mod feedback_controller;
pub mod feedforward_controller;
pub mod g_force_safety;
pub mod propulsion_control;





pub struct FlightControlSystem<T>{
    //power: Toggle, 
    linear_assist: Toggle, 
    rotational_assist: Toggle, 
    autonomous_mode: Toggle, 
    max_velocity: ControlAxis<Dimension3<T>>,
    imu: IMU<ControlAxis<Dimension3<T>>>,
    gsafety: Toggle, 
    gsafety_max_acceleration: ControlAxis<Dimension3<AxisContribution<T>>>,
    available_acceleration: ControlAxis<Dimension3<AxisContribution<T>>>,
    pid6dof: ControlAxis<Dimension3<PID<T>>>, 
}
impl<T> FlightControlSystem<T>
    where T: Mul<Output = T>
        + Div<Output = T>
        + Add<Output = T>
        + Sub<Output = T>
        + Neg<Output = T>
        + PartialOrd 
        + Copy
{
    pub fn process(self: &mut Self, input: ControlAxis<Dimension3<T>>, delta_time: T, zero: T){
        //if self.power.enabled() == false{return;}

        let mut desired_acceleration = sum_acceleration(
            if self.autonomous_mode.enabled(){
                feedforward_controller::calculate_autonomous_mode_acceleration(
                    &input,
                    &self.imu.position(),
                    &self.imu.velocity(),
                    &self.max_velocity,
                    &self.available_acceleration,
                    delta_time,
                )
            }else{
                feedforward_controller::calculate_pilot_control_mode_acceleration(
                    &input,
                    &self.linear_assist,
                    &self.rotational_assist,
                    &self.max_velocity,
                    &self.imu.velocity(),
                    &self.available_acceleration,
                    delta_time,
                    zero,
                )
            },
            feedback_controller::calculate(
                &ControlAxis::new(
                    Dimension3::default(zero), 
                    Dimension3::default(zero)
                ), 
                &ControlAxis::new(
                    Dimension3::default(zero),
                    Dimension3::default(zero)
                ), 
                &mut self.pid6dof, 
                &self.available_acceleration,
                delta_time, 
                zero, 
            )
        );

        if self.gsafety.enabled(){
            desired_acceleration = g_force_safety::process(
                &desired_acceleration, 
                &self.gsafety_max_acceleration
            )
        }

        let _ = propulsion_control::calculate_thruster_output(
            &desired_acceleration, 
            zero//mass
        );

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
}





pub fn sum_acceleration<T: Copy + Add<Output = T>>(
    feedforward: ControlAxis<Dimension3<T>>, 
    feedback: ControlAxis<Dimension3<T>>,
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