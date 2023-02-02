//! takes user input, or a goal position, and translates that into desired accelerations
use game_utils::{
    control_axis::{ControlAxis, AxisContribution}, 
    toggle::Toggle,
    dimension3::Dimension3,
};
use num::Float;





pub fn process_pilot_input<T>(
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
            pilot_input_to_acceleration(
                linear_assist.enabled(),
                input.linear().x(),
                max_velocity.linear().x(),
                velocity.linear().x(),
                available_acceleration.linear().x(),
                delta_time
            ), 
            pilot_input_to_acceleration(
                linear_assist.enabled(),
                input.linear().y(),
                max_velocity.linear().y(),
                velocity.linear().y(),
                available_acceleration.linear().y(),
                delta_time
            ), 
            pilot_input_to_acceleration(
                linear_assist.enabled(),
                input.linear().z(),
                max_velocity.linear().z(),
                velocity.linear().z(),
                available_acceleration.linear().z(),
                delta_time
            )
        ), 
        Dimension3::new(
            pilot_input_to_acceleration(
                rotational_assist.enabled(),
                input.rotational().x(),
                max_velocity.rotational().x(),
                velocity.rotational().x(),
                available_acceleration.rotational().x(),
                delta_time
            ), 
            pilot_input_to_acceleration(
                rotational_assist.enabled(),
                input.rotational().y(),
                max_velocity.rotational().y(),
                velocity.rotational().y(),
                available_acceleration.rotational().y(),
                delta_time
            ), 
            pilot_input_to_acceleration(
                rotational_assist.enabled(),
                input.rotational().z(),
                max_velocity.rotational().z(),
                velocity.rotational().z(),
                available_acceleration.rotational().z(),
                delta_time
            )
        )
    )
}



fn pilot_input_to_acceleration<T>(
    assist_enabled: bool,
    input: T,
    max_velocity: T,
    velocity: T,
    available_acceleration: AxisContribution<T>,
    delta_time: T
) -> T
    where T: Float
{
    if assist_enabled{
        //derivative of velocity is acceleration
        let desired_acceleration = ((input * max_velocity) - velocity) / delta_time;
        
        if desired_acceleration > available_acceleration.positive(){
            available_acceleration.positive()
        }
        else if desired_acceleration < -(available_acceleration.negative()){
            -(available_acceleration.negative())
        }
        else{
            desired_acceleration
        }
    }
    else{
        if(velocity >= max_velocity && input > num::zero()) || (velocity <= -max_velocity && input < num::zero()){
            num::zero()
        }
        else{
            if input > num::zero(){
                input * available_acceleration.positive()
            }
            else if input < num::zero(){
                input * available_acceleration.negative()
            }
            else{
                num::zero()
            }
        }
    }
}





///////////////////////////////// Autonomous Mode /////////////////////////////
//pub fn process_autonomous_mode_input<T>(
//    goal_position: &ControlAxis<Dimension3<T>>,
//    position: &ControlAxis<Dimension3<T>>,
//    max_velocity: &ControlAxis<Dimension3<T>>,
//    velocity: &ControlAxis<Dimension3<T>>,
//    available_acceleration: &ControlAxis<Dimension3<AxisContribution<T>>>,
//    delta_time: T,
//) -> ControlAxis<Dimension3<T>>
//    where T: Float
//{
//    ControlAxis::new(
//        Dimension3::new(
//            autonomous_input_to_acceleration(
//                goal_position.linear().x(), 
//                position.linear().x(), 
//                max_velocity.linear().x(), 
//                velocity.linear().x(), 
//                available_acceleration.linear().x(),
//                delta_time, 
//            ), 
//            autonomous_input_to_acceleration(
//                goal_position.linear().y(), 
//                position.linear().y(), 
//                max_velocity.linear().y(), 
//                velocity.linear().y(), 
//                available_acceleration.linear().y(),
//                delta_time, 
//            ), 
//            autonomous_input_to_acceleration(
//                goal_position.linear().z(), 
//                position.linear().z(), 
//                max_velocity.linear().z(), 
//                velocity.linear().z(), 
//                available_acceleration.linear().z(),
//                delta_time, 
//            )
//        ),
//        Dimension3::new(
//            autonomous_input_to_acceleration(
//                goal_position.rotational().x(), 
//                position.rotational().x(), 
//                max_velocity.rotational().x(), 
//                velocity.rotational().x(), 
//                available_acceleration.rotational().x(),
//                delta_time, 
//            ),
//            autonomous_input_to_acceleration(
//                goal_position.rotational().y(), 
//                position.rotational().y(), 
//                max_velocity.rotational().y(), 
//                velocity.rotational().y(), 
//                available_acceleration.rotational().y(),
//                delta_time, 
//            ),
//            autonomous_input_to_acceleration(
//                goal_position.rotational().z(), 
//                position.rotational().z(), 
//                max_velocity.rotational().z(), 
//                velocity.rotational().z(), 
//                available_acceleration.rotational().z(),
//                delta_time, 
//            )
//        )
//    )
//}



//fn autonomous_input_to_acceleration<T>(
//    goal_position: T,
//    position: T,
//    max_velocity: T,
//    velocity: T,
//    available_acceleration: AxisContribution<T>,
//    delta_time: T
//) -> T
//    where T: Float
//{
//    //derivative of position is velocity
//    let desired_velocity = (goal_position - position) / delta_time;
//    
//    let clamped_desired_velocity = if desired_velocity > max_velocity{
//        max_velocity
//    }
//    else if desired_velocity < -(max_velocity){
//        -(max_velocity)
//    }
//    else{
//        desired_velocity
//    };
//
//
//    //derivative of velocity is acceleration
//    let desired_acceleration = (clamped_desired_velocity - velocity) / delta_time;
//
//    if desired_acceleration > available_acceleration.positive(){
//        available_acceleration.positive()
//    }
//    else if desired_acceleration < -(available_acceleration.negative()){
//        -(available_acceleration.negative())
//    }
//    else{
//        desired_acceleration
//    }
//}










////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// axis pilot control mode
#[test]
fn axis_pilot_control_output_valid_when_input_positive_and_assist_enabled(){
    let output: f64 = pilot_input_to_acceleration(
        true, 
        1.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0
    );
    assert!(output.is_sign_positive());
    assert!(output > 0.0);
}
#[test]
fn axis_pilot_control_output_valid_when_input_positive_and_assist_disabled(){
    let output = pilot_input_to_acceleration(
        false, 
        1.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0
    );
    assert!(output.is_sign_positive());
    assert!(output > 0.0);
}
#[test]
fn axis_pilot_control_output_valid_when_input_negative_and_assist_enabled(){
    let output: f64 = pilot_input_to_acceleration(
        true, -1.0, 
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
    let output: f64 = pilot_input_to_acceleration(
        false, 
        -1.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0
    );
    assert!(output.is_sign_negative());
    assert!(output < 0.0);
}
#[test]
fn axis_pilot_control_output_valid_when_input_zero_and_assist_enabled(){
    let pilot_control_vel_output: f64 = pilot_input_to_acceleration(
        true, 
        0.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0
    );
    assert!((pilot_control_vel_output - 0.0).abs() < 0.001);
}
#[test]
fn axis_pilot_control_output_valid_when_input_zero_and_assist_disabled(){
    let pilot_control_acc_output: f64 = pilot_input_to_acceleration(
        false, 
        0.0, 
        50.0, 
        0.0, 
        AxisContribution::new(1.0, 1.0), 
        1.0
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
    use crate::input_processing;



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

        let output: ControlAxis<Dimension3<f64>> = input_processing::process_pilot_input(
            &ControlAxis::new(Dimension3::default(1.0), Dimension3::default(1.0)), 
            &Toggle::new(true), 
            &Toggle::new(true), 
            &max_velocity, 
            &velocity, 
            &available_acceleration, 
            1.0
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

        let output: ControlAxis<Dimension3<f64>> = input_processing::process_pilot_input(
            &ControlAxis::new(Dimension3::default(-1.0), Dimension3::default(-1.0)), 
            &Toggle::new(true), 
            &Toggle::new(true), 
            &max_velocity, 
            &velocity, 
            &available_acceleration, 
            1.0
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

        let output: ControlAxis<Dimension3<f64>> = input_processing::process_pilot_input(
            &ControlAxis::new(Dimension3::default(1.0), Dimension3::default(1.0)), 
            &Toggle::new(false), 
            &Toggle::new(false), 
            &max_velocity, 
            &velocity, 
            &available_acceleration, 
            1.0
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

        let output: ControlAxis<Dimension3<f64>> = input_processing::process_pilot_input(
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
    #[test]
    fn test_autonomous_mode_does_something(){}
}