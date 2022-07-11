//! # g-force safety mode 
//! when enabled, restricts fcs output accelerations to pilot specified limits, to reduce risk to biological life and equipment
//! 
//! G-force safety mode is an assistance mode intended to help pilots avoid injury or loss of
//! consciousness from g-force during extreme maneuvers. It accomplishes this by limiting the
//! amount of acceleration from thrusters to below a given threshold.

use game_utils::dimension3::{Dimension3, ClampedDimension3};
use game_utils::control_axis::{ControlAxis, AxisContribution};
use std::ops::Neg;//{Mul, Div, Add, Sub, Neg};




pub fn process<T>(
    desired_acceleration: &mut ControlAxis<Dimension3<T>>,
    gsafety_max_acceleration: &AxisContribution<ControlAxis<ClampedDimension3<T>>>
)
    where T: Copy + PartialOrd + Neg<Output = T>
{
    if desired_acceleration.linear().x() > gsafety_max_acceleration.positive().linear().x(){
        desired_acceleration.linear_mut().set_x(
            gsafety_max_acceleration.positive().linear().x()
        )
    }
    else if desired_acceleration.linear().x() < gsafety_max_acceleration.negative().linear().x(){
        desired_acceleration.linear_mut().set_x(
            gsafety_max_acceleration.negative().linear().x()
        )
    }

    if desired_acceleration.linear().y() > gsafety_max_acceleration.positive().linear().y(){
        desired_acceleration.linear_mut().set_y(
            gsafety_max_acceleration.positive().linear().y()
        )
    }
    else if desired_acceleration.linear().y() < gsafety_max_acceleration.negative().linear().y(){
        desired_acceleration.linear_mut().set_y(
            gsafety_max_acceleration.negative().linear().y()
        )
    }

    if desired_acceleration.linear().z() > gsafety_max_acceleration.positive().linear().z(){
        desired_acceleration.linear_mut().set_z(
            gsafety_max_acceleration.positive().linear().z()
        )
    }
    else if desired_acceleration.linear().z() < gsafety_max_acceleration.negative().linear().z(){
        desired_acceleration.linear_mut().set_z(
            gsafety_max_acceleration.negative().linear().z()
        )
    }


    if desired_acceleration.rotational().x() > gsafety_max_acceleration.positive().rotational().x(){
        desired_acceleration.rotational_mut().set_x(
            gsafety_max_acceleration.positive().rotational().x()
        )
    }
    else if desired_acceleration.rotational().x() < gsafety_max_acceleration.negative().rotational().x(){
        desired_acceleration.rotational_mut().set_x(
            gsafety_max_acceleration.negative().rotational().x()
        )
    }

    if desired_acceleration.rotational().y() > gsafety_max_acceleration.positive().rotational().y(){
        desired_acceleration.rotational_mut().set_y(
            gsafety_max_acceleration.positive().rotational().y()
        )
    }
    else if desired_acceleration.rotational().y() < gsafety_max_acceleration.negative().rotational().y(){
        desired_acceleration.rotational_mut().set_y(
            gsafety_max_acceleration.negative().rotational().y()
        )
    }

    if desired_acceleration.rotational().z() > gsafety_max_acceleration.positive().rotational().z(){
        desired_acceleration.rotational_mut().set_z(
            gsafety_max_acceleration.positive().rotational().z()
        )
    }
    else if desired_acceleration.rotational().z() < gsafety_max_acceleration.negative().rotational().z(){
        desired_acceleration.rotational_mut().set_z(
            gsafety_max_acceleration.negative().rotational().z()
        )
    }
}