use game_utils::{control_axis::ControlAxis, dimension3::Dimension3};
use pid_controller::PID;
use std::ops::{Mul, Div, Add, Sub, Neg};
use std::cmp::PartialOrd;

pub fn calculate<T>(
    goal_position: &ControlAxis<Dimension3<T>>,
    position: &ControlAxis<Dimension3<T>>,
    pid6dof: &mut ControlAxis<Dimension3<PID<T>>>,
    delta_time: T,
    zero_value: T
) -> ControlAxis<Dimension3<T>>
    where T: PartialOrd 
    + Copy 
    + Neg<Output = T>
    + Mul<Output = T>
    + Div<Output = T>
    + Add<Output = T>
    + Sub<Output = T>
{
    //for each linear and rotational axis, calculate corrective values
    //consider clamping pid output values
    PID::calculate(
        pid6dof.linear_mut().x_mut(),
        goal_position.linear().x(),
        position.linear().x(),
        delta_time
    );
    PID::calculate(
        pid6dof.linear_mut().y_mut(),
        goal_position.linear().y(),
        position.linear().y(),
        delta_time
    );
    PID::calculate(
        pid6dof.linear_mut().z_mut(),
        goal_position.linear().z(),
        position.linear().z(),
        delta_time
    );

    PID::calculate(
        pid6dof.rotational_mut().x_mut(),
        goal_position.rotational().x(),
        position.rotational().x(),
        delta_time
    );
    PID::calculate(
        pid6dof.rotational_mut().y_mut(),
        goal_position.rotational().y(),
        position.rotational().y(),
        delta_time
    );
    PID::calculate(
        pid6dof.rotational_mut().z_mut(),
        goal_position.rotational().z(),
        position.rotational().z(),
        delta_time
    );

    ControlAxis::new(
        Dimension3::new(
            match pid6dof.linear().x().output(){
                Some(val) => val,
                None => zero_value
            },
            match pid6dof.linear().y().output(){
                Some(val) => val,
                None => zero_value
            },
            match pid6dof.linear().z().output(){
                Some(val) => val,
                None => zero_value
            }
        ),
        Dimension3::new(
            match pid6dof.rotational().x().output(){
                Some(val) => val,
                None => zero_value
            },
            match pid6dof.rotational().y().output(){
                Some(val) => val,
                None => zero_value
            },
            match pid6dof.rotational().z().output(){
                Some(val) => val,
                None => zero_value
            }
        )
    )
}