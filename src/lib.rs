use std::ops::{Mul, Div, Add, Sub, Neg};
use std::cmp::PartialOrd;
use components::*;
use pid_controller::PID;
use clamp::clamp_symmetrical;

pub mod components;



pub fn flight_control_system<T>(
    velocity: &Velocity<T>,
    max_velocity: &MaxVelocity<T>,
    accel_proportion: &DesiredAccelerationProportion<T>,
    max_accel: &MaxAvailableAcceleration<T>,
    linear_assist: &LinearAssist,
    rotational_assist: &RotationalAssist,
    gsafety: &GForceSafety<T>,
    fcs: &mut FlightControlSystem<T>,
    pid3d: &mut PID3d<T>,
    assist_output: &mut AssistOutput<T>,
    delta_time: T,
    clamp_value: T,
    zero_value: T,
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
    //flight assist
    if linear_assist.enabled{
        assist_output.linear.x = assist_on(&mut pid3d.linear.x, max_velocity.linear.x, velocity.linear.x, 
            fcs.input.linear.x, delta_time, clamp_value);
        assist_output.linear.y = assist_on(&mut pid3d.linear.y, max_velocity.linear.y, velocity.linear.y, 
            fcs.input.linear.y, delta_time, clamp_value);
        assist_output.linear.z = assist_on(&mut pid3d.linear.z, max_velocity.linear.z, velocity.linear.z, 
            fcs.input.linear.z, delta_time, clamp_value);
    }
    else{
        assist_output.linear.x = assist_off(velocity.linear.x, max_velocity.linear.x, fcs.input.linear.x, zero_value);
        assist_output.linear.y = assist_off(velocity.linear.y, max_velocity.linear.y, fcs.input.linear.y, zero_value);
        assist_output.linear.z = assist_off(velocity.linear.z, max_velocity.linear.z, fcs.input.linear.z, zero_value);
    }
    if rotational_assist.enabled{
        assist_output.rotational.x = assist_on(
            &mut pid3d.rotational.x, max_velocity.rotational.x, velocity.rotational.x, 
            fcs.input.rotational.x, delta_time, clamp_value);
        assist_output.rotational.y = assist_on(
            &mut pid3d.rotational.y, max_velocity.rotational.y, velocity.rotational.y, 
            fcs.input.rotational.y, delta_time, clamp_value);
        assist_output.rotational.z = assist_on(
            &mut pid3d.rotational.z, max_velocity.rotational.z, velocity.rotational.z, 
            fcs.input.rotational.z, delta_time, clamp_value);
    }
    else{
        assist_output.rotational.x = assist_off(
            velocity.rotational.x, max_velocity.rotational.x, fcs.input.rotational.x, zero_value);
        assist_output.rotational.y = assist_off(
            velocity.rotational.y, max_velocity.rotational.y, fcs.input.rotational.y, zero_value);
        assist_output.rotational.z = assist_off(
            velocity.rotational.z, max_velocity.rotational.z, fcs.input.rotational.z, zero_value);
    }

    //fcs output as accelerations
    fcs.output.linear.x = assist_output.linear.x * (accel_proportion.linear.x * max_accel.linear.x);
    fcs.output.linear.y = assist_output.linear.y * (accel_proportion.linear.y * max_accel.linear.y);
    fcs.output.linear.z = assist_output.linear.z * (accel_proportion.linear.z * max_accel.linear.z);

    fcs.output.rotational.x = assist_output.rotational.x * (accel_proportion.rotational.x * max_accel.rotational.x);
    fcs.output.rotational.y = assist_output.rotational.y * (accel_proportion.rotational.y * max_accel.rotational.y);
    fcs.output.rotational.z = assist_output.rotational.z * (accel_proportion.rotational.z * max_accel.rotational.z);    

    //gsafe
    if gsafety.enabled{
        if fcs.output.linear.x > gsafety.linear.x{fcs.output.linear.x = gsafety.linear.x}
        if fcs.output.linear.y > gsafety.linear.y{fcs.output.linear.y = gsafety.linear.y}
        if fcs.output.linear.z > gsafety.linear.z{fcs.output.linear.z = gsafety.linear.z}

        if fcs.output.rotational.x > gsafety.rotational.x{fcs.output.rotational.x = gsafety.rotational.x}
        if fcs.output.rotational.y > gsafety.rotational.y{fcs.output.rotational.y = gsafety.rotational.y}
        if fcs.output.rotational.z > gsafety.rotational.z{fcs.output.rotational.z = gsafety.rotational.z}
    }
}

pub fn assist_on<T>(pid: &mut PID<T>, max_velocity: T, velocity: T, input: T, delta_time: T, clamp_value: T) -> T
    where T: Mul<Output = T> + Div<Output = T> + Add<Output = T> + Sub<Output = T> + Neg<Output = T> + PartialOrd + Copy
{
    pid.calculate(max_velocity * input, velocity, delta_time);
    clamp_symmetrical(pid.output().unwrap() / max_velocity, clamp_value)
}

pub fn assist_off<T>(velocity: T, max_velocity: T, input: T, zero_value: T) -> T
    where T: Mul<Output = T> + Div<Output = T> + Add<Output = T> + Sub<Output = T> + Neg<Output = T> + PartialOrd + Copy
{
    if velocity >= max_velocity && input > zero_value{zero_value}
    else if velocity <= -max_velocity && input < zero_value{zero_value}
    else{input}
}