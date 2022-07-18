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



pub fn calculate_autonomous_mode_acceleration<T>(
    goal_position: &ControlAxis<Dimension3<T>>,
    position: &ControlAxis<Dimension3<T>>,
    velocity: &ControlAxis<Dimension3<T>>,
    max_velocity: &ControlAxis<Dimension3<T>>,
    delta_time: T,
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
    // input interpreted as goal position. 
    // determine control signal by comparing goal position with current position
    //ControlAxis::new(
    //    Dimension3::default(zero),
    //    Dimension3::default(zero)
    //)
    ControlAxis::new(
        Dimension3::new(
            calculate_axis_autonomous_mode(
                goal_position.linear().x(), 
                position.linear().x(), 
                velocity.linear().x(), 
                max_velocity.linear().x(), 
                delta_time, 
                available_acceleration.positive().linear().x(), 
                available_acceleration.negative().linear().x()
            ), 
            calculate_axis_autonomous_mode(
                goal_position.linear().y(), 
                position.linear().y(), 
                velocity.linear().y(), 
                max_velocity.linear().y(), 
                delta_time, 
                available_acceleration.positive().linear().y(), 
                available_acceleration.negative().linear().y()
            ), 
            calculate_axis_autonomous_mode(
                goal_position.linear().z(), 
                position.linear().z(), 
                velocity.linear().z(), 
                max_velocity.linear().z(), 
                delta_time, 
                available_acceleration.positive().linear().z(), 
                available_acceleration.negative().linear().z()
            )
        ),
        Dimension3::new(
            calculate_axis_autonomous_mode(
                goal_position.rotational().x(), 
                position.rotational().x(), 
                velocity.rotational().x(), 
                max_velocity.rotational().x(), 
                delta_time, 
                available_acceleration.positive().rotational().x(), 
                available_acceleration.negative().rotational().x()
            ),
            calculate_axis_autonomous_mode(
                goal_position.rotational().y(), 
                position.rotational().y(), 
                velocity.rotational().y(), 
                max_velocity.rotational().y(), 
                delta_time, 
                available_acceleration.positive().rotational().y(), 
                available_acceleration.negative().rotational().y()
            ),
            calculate_axis_autonomous_mode(
                goal_position.rotational().z(), 
                position.rotational().z(), 
                velocity.rotational().z(), 
                max_velocity.rotational().z(), 
                delta_time, 
                available_acceleration.positive().rotational().z(), 
                available_acceleration.negative().rotational().z()
            )
        )
    )
}



fn calculate_axis_autonomous_mode<T>(
    goal_position: T,
    position: T,
    velocity: T,
    max_velocity: T,
    delta_time: T,
    pos_available_accel: T,
    neg_available_accel: T,
) -> T
    where T: Div<Output = T> 
    + Sub<Output = T> 
    + Neg<Output = T>
    + PartialOrd 
    + Copy 
{
    let delta_position = goal_position - position;
    let desired_velocity = clamp::clamp(
        delta_position / delta_time, 
        max_velocity
    );
    clamp::clamp_assym(
        velocity_control(desired_velocity, velocity, delta_time), 
        pos_available_accel, 
        neg_available_accel
    )
}



pub fn calculate_pilot_control_mode_acceleration<T>(
    input: &ControlAxis<Dimension3<T>>,
    linear_assist: &Toggle, 
    rotational_assist: &Toggle,
    max_velocity: &ControlAxis<Dimension3<T>>,
    velocity: &ControlAxis<Dimension3<T>>,
    delta_time: T,
    zero: T,
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
    ControlAxis::new(
        Dimension3::new(
            calculate_axis_pilot_control_mode(
                linear_assist.enabled(), 
                input.linear().x(), 
                max_velocity.linear().x(), 
                velocity.linear().x(), 
                delta_time, 
                zero,
                available_acceleration.positive().linear().x(),
                available_acceleration.negative().linear().x(),
            ),
            calculate_axis_pilot_control_mode(
                linear_assist.enabled(), 
                input.linear().y(), 
                max_velocity.linear().y(), 
                velocity.linear().y(), 
                delta_time, 
                zero,
                available_acceleration.positive().linear().y(),
                available_acceleration.negative().linear().y(),
            ),
            calculate_axis_pilot_control_mode(
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
            calculate_axis_pilot_control_mode(
                rotational_assist.enabled(), 
                input.rotational().x(), 
                max_velocity.rotational().x(), 
                velocity.rotational().x(), 
                delta_time, 
                zero,
                available_acceleration.positive().rotational().x(),
                available_acceleration.negative().rotational().x(),
            ),
            calculate_axis_pilot_control_mode(
                rotational_assist.enabled(), 
                input.rotational().y(), 
                max_velocity.rotational().y(), 
                velocity.rotational().y(), 
                delta_time, 
                zero,
                available_acceleration.positive().rotational().y(),
                available_acceleration.negative().rotational().y(),
            ),
            calculate_axis_pilot_control_mode(
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



fn calculate_axis_pilot_control_mode<T>(
    assist_enabled: bool, 
    input: T, 
    max_velocity: T, 
    velocity: T, 
    delta_time: T, 
    zero: T,
    pos_available_accel: T,
    neg_available_accel: T,
) -> T
    where T: Mul<Output = T> 
    + Div<Output = T> 
    + Add<Output = T> 
    + Sub<Output = T> 
    + Neg<Output = T> 
    + PartialOrd 
    + Copy
{
    if assist_enabled{
        clamp::clamp_assym(
            velocity_control(
                input * max_velocity, velocity, delta_time
            ), 
            pos_available_accel, 
            //negated due to how clamp_assym works
            neg_available_accel.neg()
        )
    }
    else{
        clamp::cmp_mul_assym(
            acceleration_control(
                velocity, max_velocity, input, zero
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
fn velocity_control<T>(desired_velocity: T, velocity: T, delta_time: T) -> T
    where T: Div<Output = T> 
    + Sub<Output = T> 
    + Neg<Output = T>
    + PartialOrd 
    + Copy 
{
    let delta_velocity = desired_velocity - velocity;
    delta_velocity / delta_time
}



fn acceleration_control<T>(velocity: T, max_velocity: T, input: T, zero: T) -> T
    where T: Neg<Output = T> 
    + PartialOrd 
    + Copy
{
    if (velocity >= max_velocity && input > zero) 
    || (velocity <= -max_velocity && input < zero){
        zero
    }
    else{input}
}







////////////////////////////////////////////////////////////////////////////////
//                                 Tests 
////////////////////////////////////////////////////////////////////////////////

#[test]
pub fn test_velocity_control(){
    // for positive input
    let vel_output: f64 = velocity_control(50.0, 0.0, 1.0);
    assert!(vel_output.is_sign_positive());

    // for negative input
    let vel_output: f64 = velocity_control(-50.0, 0.0, 1.0);
    assert!(vel_output.is_sign_negative());

    // for zero input
    let vel_output: f64 = velocity_control(0.0, 0.0, 1.0);
    assert!((vel_output - 0.0).abs() < 0.001);
}

#[test]
pub fn test_acceleration_control(){
    // ensure output clamps at max velocity
    let output: f64 = acceleration_control(55.0, 50.0, 1.0, 0.0);
    assert!((output - 0.0).abs() < 0.001);

    // ensure output clamps at negative max velocity
    let output: f64 = acceleration_control(-55.0, 50.0, -1.0, 0.0);
    assert!((output - 0.0).abs() < 0.001);

    // for positive input
    let acc_output: f64 = acceleration_control(0.0, 50.0, 1.0, 0.0);
    assert!(acc_output.is_sign_positive());

    // for negative input
    let acc_output: f64 = acceleration_control(0.0, 50.0, -1.0, 0.0);
    assert!(acc_output.is_sign_negative());

    // for zero input
    let acc_output: f64 = acceleration_control(0.0, 50.0, 0.0, 0.0);
    assert!((acc_output - 0.0).abs() < 0.001);
}

#[test]
pub fn test_calculate_axis_pilot_control_mode(){
    // for positive input
    let pilot_control_vel_output: f64 = calculate_axis_pilot_control_mode(
        true, 1.0, 50.0, 0.0, 1.0, 0.0, 1.0, 1.0
    );
    assert!(pilot_control_vel_output.is_sign_positive());

    let pilot_control_acc_output: f64 = calculate_axis_pilot_control_mode(
        false, 1.0, 50.0, 0.0, 1.0, 0.0, 1.0, 1.0
    );
    assert!(pilot_control_acc_output.is_sign_positive());

    // for negative input
    let pilot_control_vel_output: f64 = calculate_axis_pilot_control_mode(
        true, -1.0, 50.0, 0.0, 1.0, 0.0, 1.0, 1.0
    );
    assert!(pilot_control_vel_output.is_sign_negative());

    let pilot_control_acc_output: f64 = calculate_axis_pilot_control_mode(
        false, -1.0, 50.0, 0.0, 1.0, 0.0, 1.0, 1.0
    );
    assert!(pilot_control_acc_output.is_sign_negative());

    // for zero input
    let pilot_control_vel_output: f64 = calculate_axis_pilot_control_mode(
        true, 0.0, 50.0, 0.0, 1.0, 0.0, 1.0, 1.0
    );
    assert!((pilot_control_vel_output - 0.0).abs() < 0.001);

    let pilot_control_acc_output: f64 = calculate_axis_pilot_control_mode(
        false, 0.0, 50.0, 0.0, 1.0, 0.0, 1.0, 1.0
    );
    assert!((pilot_control_acc_output - 0.0).abs() < 0.001);
}

#[test]
pub fn idk(){
    let max_velocity = ControlAxis::new(
        Dimension3::default(50.0),
        Dimension3::default(50.0)
    );
    let velocity = ControlAxis::new(
        Dimension3::default(0.0), 
        Dimension3::default(0.0)
    );
    let available_acceleration = AxisContribution::new(
        ControlAxis::new(
            Dimension3::default(1.0),
            Dimension3::default(1.0)
        ), 
        ControlAxis::new(
            Dimension3::default(1.0),
            Dimension3::default(1.0)
        )
    );

    // test veloc mode
    let output_pos: ControlAxis<Dimension3<f64>> = calculate_pilot_control_mode_acceleration(
        &ControlAxis::new(Dimension3::default(1.0), Dimension3::default(1.0)), 
        &Toggle::new(true), 
        &Toggle::new(true), 
        &max_velocity, 
        &velocity, 
        1.0, 
        0.0, 
        &available_acceleration
    );
    assert!((output_pos.linear().x() - 1.0).abs() < 0.001);
    assert!((output_pos.linear().y() - 1.0).abs() < 0.001);
    assert!((output_pos.linear().z() - 1.0).abs() < 0.001);
    assert!((output_pos.rotational().x() - 1.0).abs() < 0.001);
    assert!((output_pos.rotational().y() - 1.0).abs() < 0.001);
    assert!((output_pos.rotational().z() - 1.0).abs() < 0.001);

    let output_neg: ControlAxis<Dimension3<f64>> = calculate_pilot_control_mode_acceleration(
        &ControlAxis::new(Dimension3::default(-1.0), Dimension3::default(-1.0)), 
        &Toggle::new(true), 
        &Toggle::new(true), 
        &max_velocity, 
        &velocity, 
        1.0, 
        0.0, 
        &available_acceleration
    );
    assert!((output_neg.linear().x() - (-1.0)).abs() < 0.001);
    assert!((output_neg.linear().y() - (-1.0)).abs() < 0.001);
    assert!((output_neg.linear().z() - (-1.0)).abs() < 0.001);
    assert!((output_neg.rotational().x() - (-1.0)).abs() < 0.001);
    assert!((output_neg.rotational().y() - (-1.0)).abs() < 0.001);
    assert!((output_neg.rotational().z() - (-1.0)).abs() < 0.001);

    // test accel mode
    let output_pos: ControlAxis<Dimension3<f64>> = calculate_pilot_control_mode_acceleration(
        &ControlAxis::new(Dimension3::default(1.0), Dimension3::default(1.0)), 
        &Toggle::new(false), 
        &Toggle::new(false), 
        &max_velocity, 
        &velocity, 
        1.0, 
        0.0, 
        &available_acceleration
    );
    assert!((output_pos.linear().x() - 1.0).abs() < 0.001);
    assert!((output_pos.linear().y() - 1.0).abs() < 0.001);
    assert!((output_pos.linear().z() - 1.0).abs() < 0.001);
    assert!((output_pos.rotational().x() - 1.0).abs() < 0.001);
    assert!((output_pos.rotational().y() - 1.0).abs() < 0.001);
    assert!((output_pos.rotational().z() - 1.0).abs() < 0.001);

    let output_neg: ControlAxis<Dimension3<f64>> = calculate_pilot_control_mode_acceleration(
        &ControlAxis::new(Dimension3::default(-1.0), Dimension3::default(-1.0)), 
        &Toggle::new(false), 
        &Toggle::new(false), 
        &max_velocity, 
        &velocity, 
        1.0, 
        0.0, 
        &available_acceleration
    );
    assert!((output_neg.linear().x() - (-1.0)).abs() < 0.001);
    assert!((output_neg.linear().y() - (-1.0)).abs() < 0.001);
    assert!((output_neg.linear().z() - (-1.0)).abs() < 0.001);
    assert!((output_neg.rotational().x() - (-1.0)).abs() < 0.001);
    assert!((output_neg.rotational().y() - (-1.0)).abs() < 0.001);
    assert!((output_neg.rotational().z() - (-1.0)).abs() < 0.001);
}