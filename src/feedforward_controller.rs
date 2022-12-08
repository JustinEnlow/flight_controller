//! some documentation ...
use game_utils::{
    control_axis::{ControlAxis, AxisContribution}, 
    toggle::Toggle,
    dimension3::Dimension3,
};
use num::Float;





pub fn calculate_pilot_control_mode_acceleration<T>(
    input: &ControlAxis<Dimension3<T>>,
    linear_assist: &Toggle, 
    rotational_assist: &Toggle,
    max_velocity: &ControlAxis<Dimension3<T>>,
    velocity: &ControlAxis<Dimension3<T>>,
    available_acceleration: &ControlAxis<Dimension3<AxisContribution<T>>>,
    delta_time: T,
) -> ControlAxis<Dimension3<T>>
    where T: Float
{
    ControlAxis::new(
        Dimension3::new(
            calculate_axis_pilot_control_mode(
                linear_assist.enabled(), 
                input.linear().x(), 
                max_velocity.linear().x(), 
                velocity.linear().x(), 
                available_acceleration.linear().x(),
                delta_time, 
            ),
            calculate_axis_pilot_control_mode(
                linear_assist.enabled(), 
                input.linear().y(), 
                max_velocity.linear().y(), 
                velocity.linear().y(), 
                available_acceleration.linear().y(),
                delta_time, 
            ),
            calculate_axis_pilot_control_mode(
                linear_assist.enabled(), 
                input.linear().z(), 
                max_velocity.linear().z(), 
                velocity.linear().z(), 
                available_acceleration.linear().z(),
                delta_time, 
            )
        ), 
        Dimension3::new(
            calculate_axis_pilot_control_mode(
                rotational_assist.enabled(), 
                input.rotational().x(), 
                max_velocity.rotational().x(), 
                velocity.rotational().x(), 
                available_acceleration.rotational().x(),
                delta_time, 
            ),
            calculate_axis_pilot_control_mode(
                rotational_assist.enabled(), 
                input.rotational().y(), 
                max_velocity.rotational().y(), 
                velocity.rotational().y(), 
                available_acceleration.rotational().y(),
                delta_time, 
            ),
            calculate_axis_pilot_control_mode(
                rotational_assist.enabled(), 
                input.rotational().z(), 
                max_velocity.rotational().z(), 
                velocity.rotational().z(), 
                available_acceleration.rotational().z(),
                delta_time, 
            )
        )
    )
}



fn calculate_axis_pilot_control_mode<T>(
    assist_enabled: bool, 
    input: T, 
    max_velocity: T, 
    velocity: T, 
    available_acceleration: AxisContribution<T>,
    delta_time: T, 
) -> T
    where T: Float
{
    if assist_enabled{
        num::clamp(
            velocity_control(
                input * max_velocity, 
                velocity, 
                delta_time
            ),
            -(available_acceleration.negative()),
            available_acceleration.positive()
        )
    }
    else{
        crate::multiply_compare_zero(
            acceleration_control(
                velocity, 
                max_velocity, 
                input
            ), 
            available_acceleration.positive(),
            available_acceleration.negative()
        )
    }
}



fn velocity_control<T>(desired_velocity: T, velocity: T, delta_time: T) -> T
    where T: Float
{
    let delta_velocity = desired_velocity - velocity;
    delta_velocity / delta_time
}

fn acceleration_control<T>(velocity: T, max_velocity: T, input: T) -> T
    where T: Float
{
    if (velocity >= max_velocity && input > num::zero()) 
    || (velocity <= -max_velocity && input < num::zero()){
        num::zero()
    }
    else{input}
}





pub fn calculate_autonomous_mode_acceleration<T>(
    goal_position: &ControlAxis<Dimension3<T>>,
    position: &ControlAxis<Dimension3<T>>,
    velocity: &ControlAxis<Dimension3<T>>,
    max_velocity: &ControlAxis<Dimension3<T>>,
    available_acceleration: &ControlAxis<Dimension3<AxisContribution<T>>>,
    delta_time: T,
) -> ControlAxis<Dimension3<T>>
    where T: Float
{
    ControlAxis::new(
        Dimension3::new(
            calculate_axis_autonomous_mode(
                goal_position.linear().x(), 
                position.linear().x(), 
                velocity.linear().x(), 
                max_velocity.linear().x(), 
                available_acceleration.linear().x(),
                delta_time, 
            ), 
            calculate_axis_autonomous_mode(
                goal_position.linear().y(), 
                position.linear().y(), 
                velocity.linear().y(), 
                max_velocity.linear().y(), 
                available_acceleration.linear().y(),
                delta_time, 
            ), 
            calculate_axis_autonomous_mode(
                goal_position.linear().z(), 
                position.linear().z(), 
                velocity.linear().z(), 
                max_velocity.linear().z(), 
                available_acceleration.linear().z(),
                delta_time, 
            )
        ),
        Dimension3::new(
            calculate_axis_autonomous_mode(
                goal_position.rotational().x(), 
                position.rotational().x(), 
                velocity.rotational().x(), 
                max_velocity.rotational().x(), 
                available_acceleration.rotational().x(),
                delta_time, 
            ),
            calculate_axis_autonomous_mode(
                goal_position.rotational().y(), 
                position.rotational().y(), 
                velocity.rotational().y(), 
                max_velocity.rotational().y(), 
                available_acceleration.rotational().y(),
                delta_time, 
            ),
            calculate_axis_autonomous_mode(
                goal_position.rotational().z(), 
                position.rotational().z(), 
                velocity.rotational().z(), 
                max_velocity.rotational().z(), 
                available_acceleration.rotational().z(),
                delta_time, 
            )
        )
    )
}



fn calculate_axis_autonomous_mode<T>(
    goal_position: T,
    position: T,
    velocity: T,
    max_velocity: T,
    available_acceleration: AxisContribution<T>,
    delta_time: T,
) -> T
    where T: Float
{
    let delta_position = goal_position - position;
    let desired_velocity = num::clamp(
        delta_position / delta_time,
        -(max_velocity), 
        max_velocity
    );
    num::clamp(
        velocity_control(
            desired_velocity, velocity, delta_time
        ), 
        -(available_acceleration.negative()), 
        available_acceleration.positive()
    )
}







////////////////////////////////////////////////////////////////////////////////
//                                 Tests 
////////////////////////////////////////////////////////////////////////////////

// private function tests

// velocity control
#[test]
fn velocity_control_output_is_valid_with_positive_input(){
    let output: f64 = velocity_control(50.0, 0.0, 1.0);
    assert!(output.is_sign_positive());
    assert!(output > 0.0);
}
#[test]
fn velocity_control_output_is_valid_with_negative_input(){
    let output: f64 = velocity_control(-50.0, 0.0, 1.0);
    assert!(output.is_sign_negative());
    assert!(output < 0.0);
}
#[test]
fn velocity_control_output_is_valid_with_zero_input(){
    let output: f64 = velocity_control(0.0, 0.0, 1.0);
    assert!((output - 0.0).abs() < 0.001);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// acceleration control
#[test]
fn acceleration_control_clamps_at_max_velocity(){
    let output: f64 = acceleration_control(55.0, 50.0, 1.0);
    assert!((output - 0.0).abs() < 0.001);
}
#[test]
fn acceleration_control_clamps_at_negative_max_velocity(){
    let output: f64 = acceleration_control(-55.0, 50.0, -1.0);
    assert!((output - 0.0).abs() < 0.001);
}
#[test]
fn acceleration_control_output_is_valid_with_positive_input(){
    let output: f64 = acceleration_control(0.0, 50.0, 1.0);
    assert!(output.is_sign_positive());
    assert!(output > 0.0);
}
#[test]
fn acceleration_control_output_is_valid_with_negative_input(){
    let output: f64 = acceleration_control(0.0, 50.0, -1.0);
    assert!(output.is_sign_negative());
    assert!(output < 0.0);
}
#[test]
fn acceleration_control_output_is_valid_with_zero_input(){
    let output: f64 = acceleration_control(0.0, 50.0, 0.0);
    assert!((output - 0.0).abs() < 0.001);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// axis pilot control mode
#[test]
fn axis_pilot_control_output_valid_when_input_positive_and_assist_enabled(){
    let output: f64 = calculate_axis_pilot_control_mode(
        true, 
        1.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0, 
    );
    assert!(output.is_sign_positive());
    assert!(output > 0.0);
}
#[test]
fn axis_pilot_control_output_valid_when_input_positive_and_assist_disabled(){
    let output: f64 = calculate_axis_pilot_control_mode(
        false, 
        1.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0, 
    );
    assert!(output.is_sign_positive());
    assert!(output > 0.0);
}
#[test]
fn axis_pilot_control_output_valid_when_input_negative_and_assist_enabled(){
    let output: f64 = calculate_axis_pilot_control_mode(
        true, 
        -1.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0, 
    );
    assert!(output.is_sign_negative());
    assert!(output < 0.0);
}
#[test]
fn axis_pilot_control_output_valid_when_input_negative_and_assist_disabled(){
    let output: f64 = calculate_axis_pilot_control_mode(
        false, 
        -1.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0, 
    );
    assert!(output.is_sign_negative());
    assert!(output < 0.0);
}
#[test]
fn axis_pilot_control_output_valid_when_input_zero_and_assist_enabled(){
    let pilot_control_vel_output: f64 = calculate_axis_pilot_control_mode(
        true, 
        0.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0, 
    );
    assert!((pilot_control_vel_output - 0.0).abs() < 0.001);
}
#[test]
fn axis_pilot_control_output_valid_when_input_zero_and_assist_disabled(){
    let pilot_control_acc_output: f64 = calculate_axis_pilot_control_mode(
        false, 
        0.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0, 
    );
    assert!((pilot_control_acc_output - 0.0).abs() < 0.001);
}


///////////////////////////////////////////////////////////////////////////////

// axis autonomous control mode
// test
// test
// test


///////////////////////////////////////////////////////////////////////////////

// public interface tests
#[cfg(test)]
mod tests{
    use game_utils::{
        control_axis::{ControlAxis, AxisContribution}, 
        dimension3::Dimension3,
        toggle::Toggle,
    };
    use crate::feedforward_controller;



    //pilot control mode
    #[test]
    fn test_calculate_pilot_control_mode_acceleration_with_pos_input_and_assists_on(){
        let max_velocity = ControlAxis::new(
            Dimension3::default(50.0),
            Dimension3::default(50.0)
        );
        let velocity = ControlAxis::new(
            Dimension3::default(0.0), 
            Dimension3::default(0.0)
        );
        let available_acceleration = ControlAxis::new(
            Dimension3::default(AxisContribution::new(1.0, 1.0)),
            Dimension3::default(AxisContribution::new(1.0, 1.0)),
        );

        let output: ControlAxis<Dimension3<f64>> = feedforward_controller::calculate_pilot_control_mode_acceleration(
            &ControlAxis::new(Dimension3::default(1.0), Dimension3::default(1.0)), 
            &Toggle::new(true),
            &Toggle::new(true),
            &max_velocity, 
            &velocity, 
            &available_acceleration,
            1.0, 
        );
        assert!((output.linear().x() - 1.0).abs() < 0.001);
        assert!((output.linear().y() - 1.0).abs() < 0.001);
        assert!((output.linear().z() - 1.0).abs() < 0.001);
        assert!((output.rotational().x() - 1.0).abs() < 0.001);
        assert!((output.rotational().y() - 1.0).abs() < 0.001);
        assert!((output.rotational().z() - 1.0).abs() < 0.001);
    }
    #[test]
    fn test_calculate_pilot_control_mode_acceleration_with_neg_input_and_assists_on(){
        let max_velocity = ControlAxis::new(
            Dimension3::default(50.0),
            Dimension3::default(50.0)
        );
        let velocity = ControlAxis::new(
            Dimension3::default(0.0), 
            Dimension3::default(0.0)
        );
        let available_acceleration = ControlAxis::new(
            Dimension3::default(AxisContribution::new(1.0, 1.0)),
            Dimension3::default(AxisContribution::new(1.0, 1.0)),
        );

        let output: ControlAxis<Dimension3<f64>> = feedforward_controller::calculate_pilot_control_mode_acceleration(
            &ControlAxis::new(Dimension3::default(-1.0), Dimension3::default(-1.0)), 
            &Toggle::new(true),
            &Toggle::new(true),
            &max_velocity, 
            &velocity, 
            &available_acceleration,
            1.0, 
        );
        assert!((output.linear().x() - (-1.0)).abs() < 0.001);
        assert!((output.linear().y() - (-1.0)).abs() < 0.001);
        assert!((output.linear().z() - (-1.0)).abs() < 0.001);
        assert!((output.rotational().x() - (-1.0)).abs() < 0.001);
        assert!((output.rotational().y() - (-1.0)).abs() < 0.001);
        assert!((output.rotational().z() - (-1.0)).abs() < 0.001);
    }
    #[test]
    fn test_calculate_pilot_control_mode_acceleration_with_pos_input_and_assists_off(){
        let max_velocity = ControlAxis::new(
            Dimension3::default(50.0),
            Dimension3::default(50.0)
        );
        let velocity = ControlAxis::new(
            Dimension3::default(0.0), 
            Dimension3::default(0.0)
        );
        let available_acceleration = ControlAxis::new(
            Dimension3::default(AxisContribution::new(1.0, 1.0)),
            Dimension3::default(AxisContribution::new(1.0, 1.0)),
        );

        let output: ControlAxis<Dimension3<f64>> = feedforward_controller::calculate_pilot_control_mode_acceleration(
            &ControlAxis::new(Dimension3::default(1.0), Dimension3::default(1.0)), 
            &Toggle::new(false),
            &Toggle::new(false),
            &max_velocity, 
            &velocity, 
            &available_acceleration,
            1.0, 
        );
        assert!((output.linear().x() - 1.0).abs() < 0.001);
        assert!((output.linear().y() - 1.0).abs() < 0.001);
        assert!((output.linear().z() - 1.0).abs() < 0.001);
        assert!((output.rotational().x() - 1.0).abs() < 0.001);
        assert!((output.rotational().y() - 1.0).abs() < 0.001);
        assert!((output.rotational().z() - 1.0).abs() < 0.001);
    }
    #[test]
    fn test_calculate_pilot_control_mode_acceleration_with_neg_input_and_assists_off(){
        let max_velocity = ControlAxis::new(
            Dimension3::default(50.0),
            Dimension3::default(50.0)
        );
        let velocity = ControlAxis::new(
            Dimension3::default(0.0), 
            Dimension3::default(0.0)
        );
        let available_acceleration = ControlAxis::new(
            Dimension3::default(AxisContribution::new(1.0, 1.0)),
            Dimension3::default(AxisContribution::new(1.0, 1.0)),
        );

        let output: ControlAxis<Dimension3<f64>> = feedforward_controller::calculate_pilot_control_mode_acceleration(
            &ControlAxis::new(Dimension3::default(-1.0), Dimension3::default(-1.0)), 
            &Toggle::new(false),
            &Toggle::new(false),
            &max_velocity, 
            &velocity, 
            &available_acceleration,
            1.0, 
        );
        assert!((output.linear().x() - (-1.0)).abs() < 0.001);
        assert!((output.linear().y() - (-1.0)).abs() < 0.001);
        assert!((output.linear().z() - (-1.0)).abs() < 0.001);
        assert!((output.rotational().x() - (-1.0)).abs() < 0.001);
        assert!((output.rotational().y() - (-1.0)).abs() < 0.001);
        assert!((output.rotational().z() - (-1.0)).abs() < 0.001);
    }


    // autonomous control mode
}