use game_utils::{control_axis::{ControlAxis, AxisContribution}, dimension3::Dimension3, clamp};
use pid_controller::PID;
use std::ops::{Mul, Div, Add, Sub, Neg};
use std::cmp::PartialOrd;

pub fn calculate<T>(
    goal_position: &ControlAxis<Dimension3<T>>,
    position: &ControlAxis<Dimension3<T>>,
    pid6dof: &mut ControlAxis<Dimension3<PID<T>>>,
    delta_time: T,
    zero: T,
    available_acceleration: &AxisContribution<ControlAxis<Dimension3<T>>>,
) -> ControlAxis<Dimension3<T>>
    where T: PartialOrd 
    + Neg<Output = T>
    + Mul<Output = T>
    + Div<Output = T>
    + Add<Output = T>
    + Sub<Output = T>
    + Copy
{
    ControlAxis::new(
        Dimension3::new(
            clamp::cmp_mul_assym(
                PID::calculate(
                    &mut pid6dof.linear_mut().x_mut(), 
                    goal_position.linear().x(), 
                    position.linear().x(), 
                    delta_time
                ), 
                zero, 
                available_acceleration.positive().linear().x(), 
                available_acceleration.negative().linear().x()
            ), 
            clamp::cmp_mul_assym(
                PID::calculate(
                    &mut pid6dof.linear_mut().y_mut(), 
                    goal_position.linear().y(), 
                    position.linear().y(), 
                    delta_time
                ), 
                zero, 
                available_acceleration.positive().linear().y(), 
                available_acceleration.negative().linear().y()
            ), 
            clamp::cmp_mul_assym(
                PID::calculate(
                    &mut pid6dof.linear_mut().z_mut(), 
                    goal_position.linear().z(), 
                    position.linear().z(), 
                    delta_time
                ), 
                zero, 
                available_acceleration.positive().linear().z(), 
                available_acceleration.negative().linear().z()
            )
        ),
        Dimension3::new(
            clamp::cmp_mul_assym(
                PID::calculate(
                    &mut pid6dof.rotational_mut().x_mut(), 
                    goal_position.rotational().x(), 
                    position.rotational().x(), 
                    delta_time
                ), 
                zero, 
                available_acceleration.positive().rotational().x(), 
                available_acceleration.negative().rotational().x()
            ), 
            clamp::cmp_mul_assym(
                PID::calculate(
                    &mut pid6dof.rotational_mut().y_mut(), 
                    goal_position.rotational().y(), 
                    position.rotational().y(), 
                    delta_time
                ), 
                zero, 
                available_acceleration.positive().rotational().y(), 
                available_acceleration.negative().rotational().y()
            ), 
            clamp::cmp_mul_assym(
                PID::calculate(
                    &mut pid6dof.rotational_mut().z_mut(), 
                    goal_position.rotational().z(), 
                    position.rotational().z(), 
                    delta_time
                ), 
                zero, 
                available_acceleration.positive().rotational().z(), 
                available_acceleration.negative().rotational().z()
            )
        )
    )
}