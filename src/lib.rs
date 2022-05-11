//! # Flight Control System(FCS)
//! keeps a vessel within specified flight parameters.
//! 
//! Several toggle-able modes may change the behaviour of the FCS.
//! 
//! # Autonomous mode 
//! induces accelerations to bring vessel from current position/orientation to desired position/orientation
//! 
//! # Linear flight assist 
//! restrains linear velocity, making the vessel easier to pilot. Without it, a pilot would need to manually 
//! counter every application of thrust
//! 
//! when linear flight assist is disabled, pilot input directly controls linear acceleration. any motion induced will only
//! be stopped by an equal and opposite application of thrust
//! 
//! # Rotational flight assist 
//! restrains angular velocity, making the vessel easier to pilot. Without it, a pilot would need to manually 
//! counter every application of thrust
//! 
//! when rotational flight assist is disabled, pilot input directly controls angular acceleration. any motion induced will only
//! be stopped by an equal and opposite application of thrust
//! 
//! # g-force safety mode 
//! when enabled, restricts fcs output accelerations to pilot specified limits, to reduce risk to biological life and equipment
//! 
//! # gravity compensation 
//! applies an acceleration to counteract the effect of gravity
//! 
//! # drag compensation 
//! applies an acceleration to counteract the effect of atmospheric drag

use std::ops::{Mul, Div, Add, Sub, Neg};
use std::cmp::PartialOrd;
use pid_controller::PID;
use flight_assist;
use inertial_measurement::IMU;
use game_utils::{control_axis::ControlAxis, toggle::Toggle, dimension3::{Dimension3, ClampedDimension3}};
use gforce_safety::{self, GForceSafety};



// this crate probably isnt necessary for bevy ecs. we can just chain the individual systems together.
// flight_assist -> fcs_output -> gforce_safety     //gravity/drag/skid compensation will be in that chain somewhere...



pub fn execute<T>(
    power: &Toggle,
    imu: &IMU<ControlAxis<Dimension3<T>>>,
    max_velocity: &ControlAxis<Dimension3<T>>,
    output_proportion: &ControlAxis<ClampedDimension3<T>>,
    linear_assist: &Toggle,
    rotational_assist: &Toggle,
    gsafety: &GForceSafety<ClampedDimension3<T>>,
    fcs_output: &mut ControlAxis<Dimension3<T>>,
    input: &ControlAxis<Dimension3<T>>,
    pid6dof: &ControlAxis<Dimension3<PID<T>>>,
    assist_output: &mut ControlAxis<Dimension3<T>>,
    delta_time: T,
    clamp_value: T, //used in flight assist to clamp pid output between -1.0 and 1.0
    zero: T, //to initialize structs with 0.0
)
    where
        T: Mul<Output = T>
        + Div<Output = T>
        + Add<Output = T>
        + Sub<Output = T>
        + Neg<Output = T>
        + PartialOrd 
        + Copy
{
    if power.enabled() == false{return;}

    flight_assist::execute(
        linear_assist, 
        rotational_assist,
        pid6dof,
        max_velocity,
        imu.velocity(),
        input,
        assist_output,
        delta_time,
        clamp_value,
        zero
    );

    //fcs output
    //propulsion control system can determine how to use this output and what to scale it by(max accel/max thrust/etc.)
    process_fcs_output(
        fcs_output, 
        assist_output, 
        output_proportion
    );
    
    // gravity and drag compensation might be combinable if our compensation logic is purely current position/orientation
    // vs expected position/orientation

    // get difference in expected velocity/acceleration by comparing physics engine derived actual velocity/acceleration with 
    // the sum of all forces from the propulsion control system. this output can either be stored in the PCU or in the IMU

    // if gravity compensation enabled{}
    
    // if drag compensation enabled{}
    
    // if anti-skid enabled{}

        
    gforce_safety::execute(
        gsafety, 
        imu.acceleration(), 
        fcs_output, 
        zero
    );
}



pub fn process_fcs_output<T>(
    fcs_output: &mut ControlAxis<Dimension3<T>>, 
    assist_output: &ControlAxis<Dimension3<T>>, 
    output_proportion: &ControlAxis<ClampedDimension3<T>>
)
    where T: Mul<Output = T>
    + Div<Output = T>
    + Add<Output = T>
    + Sub<Output = T>
    + Neg<Output = T>
    + PartialOrd 
    + Copy
{
    fcs_output.linear_mut().set_x(
        assist_output.linear().x() * output_proportion.linear().x()
    );
    fcs_output.linear_mut().set_y(
        assist_output.linear().y() * output_proportion.linear().y()
    );
    fcs_output.linear_mut().set_z(
        assist_output.linear().z() * output_proportion.linear().z()
    );

    fcs_output.rotational_mut().set_x(
        assist_output.rotational().x() * output_proportion.rotational().x()
    );
    fcs_output.rotational_mut().set_y(
        assist_output.rotational().y() * output_proportion.rotational().y()
    );
    fcs_output.rotational_mut().set_z(
        assist_output.rotational().z() * output_proportion.rotational().z()
    );
}




///////////////////////////////////////// WARNING: really outdated /////////////////////////////////////////////////
//
// interpereted as a percentage of max available acceleration, allowing the pilot to limit max acceleration as desired.
// low proportion here, and high max velocity, allows craft to slowly achieve a high velocity.
// high proportion here, and low max velocity, allows craft to quickly achieve a low velocity.
// both are valid flight profiles for different contexts.
//pub struct OutputProportion<T>{
//    linear: ClampedDimension3<T>,
//    rotational: ClampedDimension3<T>,
//}
//impl<T> OutputProportion<T>
//    where
//        T: PartialOrd
//        + Neg<Output = T>
//        + Copy
//{
//    pub fn new(one: T, zero: T) -> Self{
//        Self{
//            linear: ClampedDimension3::new(zero, zero, zero, one, zero),
//            rotational: ClampedDimension3::new(zero, zero, zero, one, zero),
//        }
//    }
//    
//    //pub fn linear(self: &Self) -> ClampedDimension3<T>{self.linear}
//    pub fn linear/*_ref*/(self: &Self) -> &ClampedDimension3<T>{&self.linear}
//    pub fn linear_mut(self: &mut Self) -> &mut ClampedDimension3<T>{&mut self.linear}
//    
//    //pub fn rotational(self: &Self) -> ClampedDimension3<T>{self.rotational}
//    pub fn rotational/*_ref*/(self: &Self) -> &ClampedDimension3<T>{&self.rotational}
//    pub fn rotational_mut(self: &mut Self) -> &mut ClampedDimension3<T>{&mut self.rotational}
//}