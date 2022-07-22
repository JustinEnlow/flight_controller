//! # Flight Control System(FCS)
use std::ops::{Add, Sub, Mul, Div, Neg};
use game_utils::{
    control_axis::{ControlAxis, AxisContribution},
    dimension3::Dimension3,
    toggle::Toggle,
};
use pid_controller::PID;


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
    velocity: ControlAxis<Dimension3<T>>,
    position: ControlAxis<Dimension3<T>>,
    gsafety: Toggle, 
    gsafety_max_acceleration: ControlAxis<Dimension3<AxisContribution<T>>>,
    available_acceleration: ControlAxis<Dimension3<AxisContribution<T>>>,
    pid6dof: ControlAxis<Dimension3<PID<T>>>, 
    //thruster_suite: &'a[ThrusterMountPoint<T>],
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
    pub fn process(&mut self, input: ControlAxis<Dimension3<T>>, delta_time: T, zero: T){
        //if self.power.enabled() == false{return;}

        let mut desired_acceleration = game_utils::sum_d3_control_axes(
            if self.autonomous_mode.enabled(){
                feedforward_controller::calculate_autonomous_mode_acceleration(
                    &input,
                    &self.position,
                    &self.velocity,
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
                    &self.velocity,
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





// when is it appropriate to place tests here instead of in the code's module
#[cfg(test)]
mod tests{
    // use

    #[test]
    fn some_test(){}
}