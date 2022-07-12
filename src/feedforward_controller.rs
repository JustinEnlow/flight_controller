//! Flight Assist has two independent assists:
//! 
//! # Linear flight assist 
//! restrains linear velocity, making the vessel easier to pilot. Without it, a pilot would need to manually 
//! counter every application of thrust
//! 
//! when linear flight assist is disabled, pilot input directly controls linear thrust. any motion induced will only
//! be stopped by an equal and opposite application of thrust
//! 
//! # Rotational flight assist 
//! restrains angular velocity, making the vessel easier to pilot. Without it, a pilot would need to manually 
//! counter every application of thrust
//! 
//! when rotational flight assist is disabled, pilot input directly controls angular thrust. any motion induced will only
//! be stopped by an equal and opposite application of thrust
//! 
//! returns a desired acceleration

use std::ops::{Mul, Div, Add, Sub, Neg};
use std::cmp::PartialOrd;
use game_utils::{control_axis::{ControlAxis, AxisContribution}, toggle::Toggle, dimension3::Dimension3, clamp};



// https://www.motioncontroltips.com/what-is-a-motion-profile/



pub fn calculate<T>(
    input: &ControlAxis<Dimension3<T>>,
    linear_assist: &Toggle, 
    rotational_assist: &Toggle,
    max_velocity: &ControlAxis<Dimension3<T>>,
    velocity: &ControlAxis<Dimension3<T>>,
    delta_time: T,
    zero: T,
    autonomous_mode: &Toggle,
    available_acceleration: &AxisContribution<ControlAxis<Dimension3<T>>>,
) -> ControlAxis<Dimension3<T>>
    where T: Mul<Output = T>
    + Div<Output = T>
    + Add<Output = T>
    + Sub<Output = T>
    + Neg<Output = T>
    + PartialOrd 
    + Copy
{
    match autonomous_mode.enabled(){
        true => {
            // input interpreted as goal position. 
            // determine control signal by comparing goal position with current position
            ControlAxis::new(
                Dimension3::default(zero),
                Dimension3::default(zero)
            )
        },
        false => {
            ControlAxis::new(
                Dimension3::new(
                    pilot_control_mode(
                        linear_assist.enabled(), 
                        input.linear().x(), 
                        max_velocity.linear().x(), 
                        velocity.linear().x(), 
                        delta_time, 
                        zero,
                        available_acceleration.positive().linear().x(),
                        available_acceleration.negative().linear().x(),
                    ),
                    pilot_control_mode(
                        linear_assist.enabled(), 
                        input.linear().y(), 
                        max_velocity.linear().y(), 
                        velocity.linear().y(), 
                        delta_time, 
                        zero,
                        available_acceleration.positive().linear().y(),
                        available_acceleration.negative().linear().y(),
                    ),
                    pilot_control_mode(
                        linear_assist.enabled(), 
                        input.linear().z(), 
                        max_velocity.linear().z(), 
                        velocity.linear().z(), 
                        delta_time, 
                        zero,
                        available_acceleration.positive().linear().z(),
                        available_acceleration.negative().linear().z(),
                    )
                ), 
                Dimension3::new(
                    pilot_control_mode(
                        rotational_assist.enabled(), 
                        input.rotational().x(), 
                        max_velocity.rotational().x(), 
                        velocity.rotational().x(), 
                        delta_time, 
                        zero,
                        available_acceleration.positive().rotational().x(),
                        available_acceleration.negative().rotational().x(),
                    ),
                    pilot_control_mode(
                        rotational_assist.enabled(), 
                        input.rotational().y(), 
                        max_velocity.rotational().y(), 
                        velocity.rotational().y(), 
                        delta_time, 
                        zero,
                        available_acceleration.positive().rotational().y(),
                        available_acceleration.negative().rotational().y(),
                    ),
                    pilot_control_mode(
                        rotational_assist.enabled(), 
                        input.rotational().z(), 
                        max_velocity.rotational().z(), 
                        velocity.rotational().z(), 
                        delta_time, 
                        zero,
                        available_acceleration.positive().rotational().z(),
                        available_acceleration.negative().rotational().z(),
                    )
                )
            )        
        }
    }
}



//fn autonomous_control_mode<T: Div<Output = T> + Sub<Output = T> + Copy + PartialOrd + Neg<Output = T>>(
//    goal_position: T,
//    position: T,
//    velocity: T,
//    max_velocity: T,
//    delta_time: T,
//    pos_available_accel: T,
//    neg_available_accel: T,
//) -> T{
//    let delta_position = goal_position - position;
//    let desired_velocity = clamp::clamp(
//        delta_position / delta_time, 
//        max_velocity
//    );
//    clamp::clamp_assym(
//        velocity_control(desired_velocity, velocity, delta_time), 
//        pos_available_accel, 
//        neg_available_accel
//    )
//}

fn pilot_control_mode<T: Mul<Output = T> + Div<Output = T> + Add<Output = T> + Sub<Output = T> + Neg<Output = T> + PartialOrd + Copy>(
    assist_enabled: bool, 
    input: T, 
    max_velocity: T, 
    velocity: T, 
    delta_time: T, 
    zero: T,
    pos_available_accel: T,
    neg_available_accel: T,
) -> T{
    if assist_enabled{
        clamp::clamp_assym(
            velocity_control(
                input * max_velocity, 
                velocity, 
                delta_time
            ), 
            pos_available_accel, 
            neg_available_accel
        )
    }
    else{
        clamp::cmp_mul_assym(
            acceleration_control(
                velocity, 
                max_velocity, 
                input, 
                zero
            ), 
            zero, 
            pos_available_accel, 
            neg_available_accel
        )
    }
}

/// velocity = change in position / change in time
/// acceleration = change in velocity / change in time
/// jerk = change in acceleration / change in time
fn velocity_control<T: Div<Output = T> + Sub<Output = T> + Copy + PartialOrd + Neg<Output = T>>(
    desired_velocity: T, 
    velocity: T, 
    delta_time: T,
) -> T{
    let delta_velocity = desired_velocity - velocity;
    delta_velocity / delta_time
}

fn acceleration_control<T>(
    velocity: T, 
    max_velocity: T, 
    input: T, 
    zero: T
) -> T
    where T: Neg<Output = T> + PartialOrd + Copy
{
    if velocity >= max_velocity && input > zero{zero}
    else if velocity <= -max_velocity && input < zero{zero}
    else{input}
}







////////////////////////////////////////////////////////////////////////////////
//                                 Tests 
////////////////////////////////////////////////////////////////////////////////

#[test]
pub fn test_velocity_control(){

    let expected: f32 = 1.0;

    let max_velocity = 50.0;
    let current_velocity = 0.0;
    let input = 1.0;

    let output = clamp::clamp_assym(
        velocity_control(
            input * max_velocity, 
            current_velocity, 
            0.02
        ), 
        1.0, 
        -1.0
    );

    assert!((output - expected).abs() < 0.001);
}

#[test]
pub fn test_acceleration_control(){

    let expected: f32 = 0.0;

    let current_velocity = 55.0;
    let max_velocity = 50.0;
    let input = 1.0;

    let output = acceleration_control(current_velocity, max_velocity, input, 0.0);

    assert!((output - expected).abs() < 0.001);
}