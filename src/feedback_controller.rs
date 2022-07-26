use game_utils::{control_axis::{ControlAxis, AxisContribution}, dimension3::Dimension3,};
use pid_controller::PID;
use std::ops::{Mul, Div, Add, Sub, Neg};
use std::cmp::PartialOrd;
use num::Float;
use crate::utils;

pub fn calculate<T>(
    goal_position: &ControlAxis<Dimension3<T>>,
    position: &ControlAxis<Dimension3<T>>,
    pid6dof: &mut ControlAxis<Dimension3<PID<T>>>,
    available_acceleration: &ControlAxis<Dimension3<AxisContribution<T>>>,
    delta_time: T,
) -> ControlAxis<Dimension3<T>>
    where T: Float
        + PartialOrd 
        + Neg<Output = T>
        + Mul<Output = T>
        + Div<Output = T>
        + Add<Output = T>
        + Sub<Output = T>
        + Copy
{
    ControlAxis::new(
        Dimension3::new(
            utils::multiply_compare_zero(
                pid6dof.linear_mut().x_mut().calculate(
                    goal_position.linear().x(), 
                    position.linear().x(), 
                    delta_time
                ), 
                available_acceleration.linear().x().positive(),
                available_acceleration.linear().x().negative()
            ), 
            utils::multiply_compare_zero(
                pid6dof.linear_mut().y_mut().calculate(
                    goal_position.linear().y(), 
                    position.linear().y(), 
                    delta_time
                ), 
                available_acceleration.linear().y().positive(),
                available_acceleration.linear().y().negative()
            ), 
            utils::multiply_compare_zero(
                pid6dof.linear_mut().z_mut().calculate(
                    goal_position.linear().z(), 
                    position.linear().z(), 
                    delta_time
                ), 
                available_acceleration.linear().z().positive(),
                available_acceleration.linear().z().negative()
            )
        ),
        Dimension3::new(
            utils::multiply_compare_zero(
                pid6dof.rotational_mut().x_mut().calculate(
                    goal_position.rotational().x(), 
                    position.rotational().x(), 
                    delta_time
                ), 
                available_acceleration.rotational().x().positive(),
                available_acceleration.rotational().x().negative()
            ), 
            utils::multiply_compare_zero(
                pid6dof.rotational_mut().y_mut().calculate(
                    goal_position.rotational().y(), 
                    position.rotational().y(), 
                    delta_time
                ), 
                available_acceleration.rotational().y().positive(),
                available_acceleration.rotational().y().negative()
            ), 
            utils::multiply_compare_zero(
                pid6dof.rotational_mut().z_mut().calculate(
                    goal_position.rotational().z(), 
                    position.rotational().z(), 
                    delta_time
                ), 
                available_acceleration.rotational().z().positive(),
                available_acceleration.rotational().z().negative()
            )
        )
    )
}