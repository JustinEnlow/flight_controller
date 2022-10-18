//! # Flight Control System(FCS)
use game_utils::{
    control_axis::{ControlAxis, AxisContribution},
    dimension3::Dimension3,
    toggle::Toggle,
};
use pid_controller::PID;
use num::Float;
use crate::propulsion_control::ThrusterMountPoint;


pub mod feedback_controller;
pub mod feedforward_controller;
pub mod g_force_safety;
pub mod propulsion_control;
mod utils;





// can use fcs struct or call functions independently
pub struct FlightControlSystem<'a, T>{
    linear_assist: Toggle,
    rotational_assist: Toggle,
    autonomous_mode: Toggle,
    max_velocity: ControlAxis<Dimension3<T>>,
    gsafety: Toggle, 
    gsafety_max_acceleration: ControlAxis<Dimension3<AxisContribution<T>>>,
    available_acceleration: ControlAxis<Dimension3<AxisContribution<T>>>,
    pid6dof: ControlAxis<Dimension3<PID<T>>>, 
    thruster_mount_points: &'a[ThrusterMountPoint<'a, T>],
    //calculated expected position
}
impl<'a, T> FlightControlSystem<'a, T>
    where T: Float
{
    pub fn new(
        linear_assist: Toggle,
        rotational_assist: Toggle,
        autonomous_mode: Toggle,
        max_velocity: ControlAxis<Dimension3<T>>,
        gsafety: Toggle, 
        gsafety_max_acceleration: ControlAxis<Dimension3<AxisContribution<T>>>,
        available_acceleration: ControlAxis<Dimension3<AxisContribution<T>>>,
        pid6dof: ControlAxis<Dimension3<PID<T>>>, 
        thruster_mount_points: &'a[ThrusterMountPoint<T>],
    ) -> Self{
        Self{
            linear_assist,
            rotational_assist,
            autonomous_mode,
            max_velocity,
            gsafety, 
            gsafety_max_acceleration,
            available_acceleration, //this will be determined by passing the thruster suite to the propulsion control system for calculation
            pid6dof, 
            thruster_mount_points,
        }
    }

    pub fn process(&mut self, 
        input: &ControlAxis<Dimension3<T>>, 
        velocity: &ControlAxis<Dimension3<T>>,
        position: &ControlAxis<Dimension3<T>>,
        delta_time: T,
    ){
        let mut desired_acceleration = game_utils::sum_d3_control_axes(
            if self.autonomous_mode.enabled(){
                feedforward_controller::calculate_autonomous_mode_acceleration(
                    &input,
                    position,
                    velocity,
                    &self.max_velocity,
                    &self.available_acceleration,
                    delta_time,
                )
            }else{
                feedforward_controller::calculate_pilot_control_mode_acceleration(
                    input,
                    &self.linear_assist,
                    &self.rotational_assist,
                    &self.max_velocity,
                    velocity,
                    &self.available_acceleration,
                    delta_time,
                )
            },
            feedback_controller::calculate(
                // calculated expected position
                &ControlAxis::new(
                    Dimension3::default(num::zero()), 
                    Dimension3::default(num::zero())
                ), 
                position,
                &mut self.pid6dof, 
                &self.available_acceleration,
                delta_time, 
            )
        );
    
        if self.gsafety.enabled(){
            desired_acceleration = g_force_safety::process(
                &desired_acceleration, 
                &self.gsafety_max_acceleration
            )
        }
    
        // if advanced propulsion simulation is desired, feed desired accel values to propulsion control
        // otherwise feed desired accel values directly to physics sim

        //Once the desired linear and rotational accelerations are established from the combined feedforward
        //and feedback control signals, the PCS must calculate the output of individual thrusters, as well as other
        //devices tasked with generating motion, so that these accelerations will be achieved to within a
        //reasonable degree of accuracy.
        let _ = propulsion_control::calculate_thruster_output(
            &desired_acceleration, 
            self.thruster_mount_points,
            num::zero()//mass
        );
    
        /////////////////////////////////////////////////////////////////////////////
        // calculate expected position from resultant thrusts/accelerations/velocities
        /////////////////////////////////////////////////////////////////////////////
    }
}





// when is it appropriate to place tests here instead of inline tests
#[cfg(test)]
mod tests{
    use crate::{
        FlightControlSystem, 
        propulsion_control::{
            ThrusterMountPoint, 
            Thruster, 
            ThrusterSize, 
            ThrustDirection
        }
    };
    use game_utils::{
        toggle::Toggle, 
        control_axis::{ControlAxis, AxisContribution},
        dimension3::Dimension3
    };
    use pid_controller::PID;


    
    #[test]
    fn some_test(){
        let thruster_suite = [
            ThrusterMountPoint::new(
                Some(Thruster::new(50_000.0, ThrusterSize::Small)), 
                &[ThrustDirection::LinXPos], 
                ThrusterSize::Medium, 
                Dimension3::new(5.0, 0.0, 0.0)
            )
        ];

        let _fcs = FlightControlSystem::new(
            Toggle::new(true), 
            Toggle::new(true), 
            Toggle::new(false), 
            ControlAxis::new(
                Dimension3::default(50.0), 
                Dimension3::default(10.0)
            ), 
            Toggle::new(false), 
            ControlAxis::new(
                Dimension3::default(AxisContribution::new(10.0, 10.0)),
                Dimension3::default(AxisContribution::new(10.0, 10.0)),
            ), 
            ControlAxis::new(
                Dimension3::default(AxisContribution::new(2.5, 2.5)),
                Dimension3::default(AxisContribution::new(2.5, 2.5)),
            ), 
            ControlAxis::new(
                Dimension3::default(PID::new(100.0, 0.0, 0.0, None)),
                Dimension3::default(PID::new(100.0, 0.0, 0.0, None)),
            ), 
            &thruster_suite
        );

        //fcs.process(input, velocity, position, delta_time)
    }
}