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
use components::*;
use pid_controller::PID;
use clamp::*;
use accelerometer::Acceleration;

pub mod components;



pub fn flight_control_system<T>(
    velocity: &Velocity<T>,
    max_velocity: &MaxVelocity<T>,
    acceleration: &Acceleration<T>,
    output_proportion: &OutputProportion<T>,
    linear_assist: &LinearAssist,
    rotational_assist: &RotationalAssist,
    gsafety: &GForceSafety<T>,
    fcs: &mut FlightControlSystem<T>,
    pid3d: &mut PID3d<T>,
    assist_output: &mut AssistOutput<T>,
    delta_time: T,
    clamp_value: T,
    zero: T,
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
    // if autonomous positioning mode{use 3dpid to go from current position/orientation to desired position/orientation}
    // might need to add some object avoidance logic
    if false/*autonomous_mode.enabled()*/{

    }
    else{
        if linear_assist.enabled(){
            assist_output.linear_mut().set_x(   // linear() returns a copy. I think we need linear_mut(), but should verify
                assist_on(&mut pid3d.linear().x(), max_velocity.linear().x(), velocity.linear().x(), 
                fcs.input().linear().x(), delta_time, clamp_value)
            );
            assist_output.linear_mut().set_y(
                assist_on(&mut pid3d.linear().y(), max_velocity.linear().y(), velocity.linear().y(), 
                fcs.input().linear().y(), delta_time, clamp_value)
            );
            assist_output.linear_mut().set_z(
                assist_on(&mut pid3d.linear().z(), max_velocity.linear().z(), velocity.linear().z(), 
                fcs.input().linear().z(), delta_time, clamp_value)
            );
        }
        else{
            assist_output.linear_mut().set_x(
                assist_off(velocity.linear().x(), max_velocity.linear().x(), fcs.input().linear().x(), zero)
            );
            assist_output.linear_mut().set_y(
                assist_off(velocity.linear().y(), max_velocity.linear().y(), fcs.input().linear().y(), zero)
            );
            assist_output.linear_mut().set_z(
                assist_off(velocity.linear().z(), max_velocity.linear().z(), fcs.input().linear().z(), zero)
            );
        }
        if rotational_assist.enabled(){
            assist_output.rotational_mut().set_x(
                assist_on(&mut pid3d.rotational().x(), max_velocity.rotational().x(), velocity.rotational().x(), 
                fcs.input().rotational().x(), delta_time, clamp_value)
            );
            assist_output.rotational_mut().set_y(
                assist_on(&mut pid3d.rotational().y(), max_velocity.rotational().y(), velocity.rotational().y(), 
                fcs.input().rotational().y(), delta_time, clamp_value)
            );
            assist_output.rotational_mut().set_z(
                assist_on(&mut pid3d.rotational().z(), max_velocity.rotational().z(), velocity.rotational().z(), 
                fcs.input().rotational().z(), delta_time, clamp_value)
            );
        }
        else{
            assist_output.rotational_mut().set_x(
                assist_off(velocity.rotational().x(), max_velocity.rotational().x(), fcs.input().rotational().x(), zero)
            );
            assist_output.rotational_mut().set_y(
                assist_off(velocity.rotational().y(), max_velocity.rotational().y(), fcs.input().rotational().y(), zero)
            );
            assist_output.rotational_mut().set_z(
                assist_off(velocity.rotational().z(), max_velocity.rotational().z(), fcs.input().rotational().z(), zero)
            );
        }
    }

    //fcs output
    //propulsion control system can determine how to use this output and what to scale it by(max accel/max thrust/etc.)
    fcs.output_mut().linear_mut().set_x(
        assist_output.linear().x() * output_proportion.linear().x()
    );
    fcs.output_mut().linear_mut().set_y(
        assist_output.linear().y() * output_proportion.linear().y()
    );
    fcs.output_mut().linear_mut().set_z(
        assist_output.linear().z() * output_proportion.linear().z()
    );

    fcs.output_mut().rotational_mut().set_x(
        assist_output.rotational().x() * output_proportion.rotational().x()
    );
    fcs.output_mut().rotational_mut().set_y(
        assist_output.rotational().y() * output_proportion.rotational().y()
    );
    fcs.output_mut().rotational_mut().set_z(
        assist_output.rotational().z() * output_proportion.rotational().z()
    );
    
    // gravity and drag compensation might be combinable if our compensation logic is purely current position/orientation
    // vs expected position/orientation

    // if gravity compensation enabled{}
    
    // if drag compensation enabled{}
    
    // if anti-skid enabled{}

    if gsafety.enabled(){   //do accelerations in the negative need to be limited?
        if acceleration.linear().x() > gsafety.linear().x() || acceleration.linear().x() < gsafety.neg_linear().x(){
            fcs.output_mut().linear_mut().set_x(zero);
        }
        if acceleration.linear().y() > gsafety.linear().y() || acceleration.linear().y() < gsafety.neg_linear().y(){
            fcs.output_mut().linear_mut().set_y(zero);
        }
        if acceleration.linear().z() > gsafety.linear().z() || acceleration.linear().z() < gsafety.neg_linear().z(){
            fcs.output_mut().linear_mut().set_z(zero);
        }

        if acceleration.rotational().x() > gsafety.rotational().x() || acceleration.rotational().x() < gsafety.neg_rotational().x(){
            fcs.output_mut().rotational_mut().set_x(zero);
        }
        if acceleration.rotational().y() > gsafety.rotational().y() || acceleration.rotational().y() < gsafety.neg_rotational().y(){
            fcs.output_mut().rotational_mut().set_y(zero);
        }
        if acceleration.rotational().z() > gsafety.rotational().z() || acceleration.rotational().z() < gsafety.neg_rotational().z(){
            fcs.output_mut().rotational_mut().set_z(zero);
        }
    }
}

fn assist_on<T>(pid: &mut PID<T>, max_velocity: T, velocity: T, input: T, delta_time: T, clamp_value: T) -> T
    where T: Mul<Output = T> + Div<Output = T> + Add<Output = T> + Sub<Output = T> + Neg<Output = T> + PartialOrd + Copy
{
    pid.calculate(max_velocity * input, velocity, delta_time);
    clamp(pid.output().unwrap() / max_velocity, clamp_value)
}

fn assist_off<T>(velocity: T, max_velocity: T, input: T, zero_value: T) -> T
    where T: Mul<Output = T> + Div<Output = T> + Add<Output = T> + Sub<Output = T> + Neg<Output = T> + PartialOrd + Copy
{
    if velocity >= max_velocity && input > zero_value{zero_value}
    else if velocity <= -max_velocity && input < zero_value{zero_value}
    else{input}
}