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

use std::ops::{Mul, Div, Add, Sub, Neg};
use std::cmp::PartialOrd;
use game_utils::{control_axis::ControlAxis, toggle::Toggle, dimension3::Dimension3, clamp};



pub enum ControlModel{
    // https://www.motioncontroltips.com/what-is-a-motion-profile/
    Triangle,
    Trapezoidal,
    SCurve,
}



pub fn calculate<T>(
    input: &ControlAxis<Dimension3<T>>,
    linear_assist: &Toggle, 
    rotational_assist: &Toggle,
    max_velocity: &ControlAxis<Dimension3<T>>,
    velocity: &ControlAxis<Dimension3<T>>,
    delta_time: T,
    clamp_value: T, //used to clamp pid output between -1.0 and 1.0
    zero_value: T,
    autonomous_mode: &Toggle,
) -> ControlAxis<Dimension3<T>>
    where T: Mul<Output = T>
    + Div<Output = T>
    + Add<Output = T>
    + Sub<Output = T>
    + Neg<Output = T>
    + PartialOrd 
    + Copy
{
    // should feedforward controller handle autonomous mode as well as manual control?
        // manual mode generates a control signal by processing user input
        // autonomous mode would generate a control signal by comparing a goal position vs our actual position

    match autonomous_mode.enabled(){
        true => {
            ControlAxis::new(
                Dimension3::default(zero_value),
                Dimension3::default(zero_value)
            )
        },
        false => {
            ControlAxis::new(
                Dimension3::new(
                    calculate_control_signal(
                        linear_assist.enabled(), 
                        input.linear().x(), 
                        max_velocity.linear().x(), 
                        velocity.linear().x(), 
                        delta_time, 
                        zero_value, 
                        clamp_value
                    ),
                    calculate_control_signal(
                        linear_assist.enabled(), 
                        input.linear().y(), 
                        max_velocity.linear().y(), 
                        velocity.linear().y(), 
                        delta_time, 
                        zero_value, 
                        clamp_value
                    ),
                    calculate_control_signal(
                        linear_assist.enabled(), 
                        input.linear().z(), 
                        max_velocity.linear().z(), 
                        velocity.linear().z(), 
                        delta_time, 
                        zero_value, 
                        clamp_value
                    )
                ), 
                Dimension3::new(
                    calculate_control_signal(
                        rotational_assist.enabled(), 
                        input.rotational().x(), 
                        max_velocity.rotational().x(), 
                        velocity.rotational().x(), 
                        delta_time, 
                        zero_value, 
                        clamp_value
                    ),
                    calculate_control_signal(
                        rotational_assist.enabled(), 
                        input.rotational().y(), 
                        max_velocity.rotational().y(), 
                        velocity.rotational().y(), 
                        delta_time, 
                        zero_value, 
                        clamp_value
                    ),
                    calculate_control_signal(
                        rotational_assist.enabled(), 
                        input.rotational().z(), 
                        max_velocity.rotational().z(), 
                        velocity.rotational().z(), 
                        delta_time, 
                        zero_value, 
                        clamp_value
                    )
                )
            )        
        }
    }

    //ControlAxis::new(
    //    Dimension3::new(
    //        calculate_control_signal(
    //            linear_assist.enabled(), 
    //            input.linear().x(), 
    //            max_velocity.linear().x(), 
    //            velocity.linear().x(), 
    //            delta_time, 
    //            zero_value, 
    //            clamp_value
    //        ),
    //        calculate_control_signal(
    //            linear_assist.enabled(), 
    //            input.linear().y(), 
    //            max_velocity.linear().y(), 
    //            velocity.linear().y(), 
    //            delta_time, 
    //            zero_value, 
    //            clamp_value
    //        ),
    //        calculate_control_signal(
    //            linear_assist.enabled(), 
    //            input.linear().z(), 
    //            max_velocity.linear().z(), 
    //            velocity.linear().z(), 
    //            delta_time, 
    //            zero_value, 
    //            clamp_value
    //        )
    //    ), 
    //    Dimension3::new(
    //        calculate_control_signal(
    //            rotational_assist.enabled(), 
    //            input.rotational().x(), 
    //            max_velocity.rotational().x(), 
    //            velocity.rotational().x(), 
    //            delta_time, 
    //            zero_value, 
    //            clamp_value
    //        ),
    //        calculate_control_signal(
    //            rotational_assist.enabled(), 
    //            input.rotational().y(), 
    //            max_velocity.rotational().y(), 
    //            velocity.rotational().y(), 
    //            delta_time, 
    //            zero_value, 
    //            clamp_value
    //        ),
    //        calculate_control_signal(
    //            rotational_assist.enabled(), 
    //            input.rotational().z(), 
    //            max_velocity.rotational().z(), 
    //            velocity.rotational().z(), 
    //            delta_time, 
    //            zero_value, 
    //            clamp_value
    //        )
    //    )
    //)
}



fn calculate_control_signal<T>(
    assist_enabled: bool, 
    input: T, 
    max_velocity: T, 
    velocity: T, 
    delta_time: T, 
    zero_value: T, 
    clamp_value: T
) -> T
    where
        T: Mul<Output = T>
        + Div<Output = T>
        + Add<Output = T>
        + Sub<Output = T>
        + Neg<Output = T>
        + PartialOrd 
        + Copy
{
    if assist_enabled{
        velocity_control(input * max_velocity, velocity, delta_time, clamp_value)
    }
    else{
        acceleration_control(velocity, max_velocity, input, zero_value)
    }
}

/// velocity = change in position / change in time
/// acceleration = change in velocity / change in time
/// jerk = change in acceleration / change in time
fn velocity_control<T: Div<Output = T> + Sub<Output = T> + Copy + PartialOrd + Neg<Output = T>>(
    desired_velocity: T, 
    velocity: T, 
    delta_time: T, 
    clamp_value: T,
    //control_model: ControlModel
) -> T{
    //match control_model{
        //Trapezoidal => {
            let delta_velocity = desired_velocity - velocity;
            clamp::clamp(delta_velocity / delta_time, clamp_value)
        //},
        //SCurve => {}
    //}
}

/// acceleration control
fn acceleration_control<T>(
    current_velocity: T, 
    max_velocity: T, 
    input: T, 
    zero_value: T
) -> T
    where T: Neg<Output = T> + PartialOrd + Copy
{
    if current_velocity >= max_velocity && input > zero_value{zero_value}
    else if current_velocity <= -max_velocity && input < zero_value{zero_value}
    else{input}
}







////////////////////////////////////////////////////////////////////////////////
//                                 Tests 
////////////////////////////////////////////////////////////////////////////////

#[test]
pub fn test_velocity_control(){

    let expected: f32 = 1.0;

    //let mut pid = PID::new(0.0, 100.0, 0.0, 0.0, None);
    let max_velocity = 50.0;
    let current_velocity = 0.0;
    let input = 1.0;

    let output = velocity_control(input * max_velocity, current_velocity, 0.02, 1.0);

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