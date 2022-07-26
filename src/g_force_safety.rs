//! # g-force safety mode 
//! when enabled, restricts fcs output accelerations to pilot specified limits, to reduce risk to biological life and equipment
//! 
//! G-force safety mode is an assistance mode intended to help pilots avoid injury or loss of
//! consciousness from g-force during extreme maneuvers. It accomplishes this by limiting the
//! amount of acceleration from thrusters to below a given threshold.

use game_utils::dimension3::Dimension3;
use game_utils::control_axis::{ControlAxis, AxisContribution};
use std::ops::Neg;
use num;





pub fn process<T>(
    desired_acceleration: &ControlAxis<Dimension3<T>>,
    gsafety_max_acceleration: &ControlAxis<Dimension3<AxisContribution<T>>>,
) -> ControlAxis<Dimension3<T>>
    where T: Copy + PartialOrd + Neg<Output = T>
{
    ControlAxis::new(
        Dimension3::new(
            num::clamp(
                desired_acceleration.linear().x(), 
                -(gsafety_max_acceleration.linear().x().negative()), 
                gsafety_max_acceleration.linear().x().positive()
            ),
            num::clamp(
                desired_acceleration.linear().y(), 
                -(gsafety_max_acceleration.linear().y().negative()), 
                gsafety_max_acceleration.linear().y().positive()
            ), 
            num::clamp(
                desired_acceleration.linear().z(), 
                -(gsafety_max_acceleration.linear().z().negative()), 
                gsafety_max_acceleration.linear().z().positive()
            ),
        ),
        Dimension3::new(
            num::clamp(
                desired_acceleration.rotational().x(), 
                -(gsafety_max_acceleration.rotational().x().negative()), 
                gsafety_max_acceleration.rotational().x().positive()
            ),
            num::clamp(
                desired_acceleration.rotational().y(), 
                -(gsafety_max_acceleration.rotational().y().negative()), 
                gsafety_max_acceleration.rotational().y().positive()
            ),
            num::clamp(
                desired_acceleration.rotational().z(), 
                -(gsafety_max_acceleration.rotational().z().negative()), 
                gsafety_max_acceleration.rotational().z().positive()
            ), 
        )
    )
}

// clamping already tested in clamp module
// no need to retest here.