//! # g-force safety mode 
//! when enabled, restricts fcs output accelerations to pilot specified limits, to reduce risk to biological life and equipment
//! 
//! G-force safety mode is an assistance mode intended to help pilots avoid injury or loss of
//! consciousness from g-force during extreme maneuvers. It accomplishes this by limiting the
//! amount of acceleration from thrusters to below a given threshold.

use game_utils::dimension3::Dimension3;
use game_utils::control_axis::{ControlAxis, AxisContribution};
use std::ops::Neg;
use game_utils::clamp;





pub fn process<T>(
    //desired_acceleration: &mut ControlAxis<Dimension3<T>>,
    desired_acceleration: &ControlAxis<Dimension3<T>>,
    gsafety_max_acceleration: &AxisContribution<ControlAxis</*Clamped*/Dimension3<T>>>  //ClampedDimension3 doesn't seem to be needed...verify later
) -> ControlAxis<Dimension3<T>>
    where T: Copy + PartialOrd + Neg<Output = T>
{
    ControlAxis::new(
        Dimension3::new(
            clamp::clamp_assym(
                desired_acceleration.linear().x(), 
                gsafety_max_acceleration.positive().linear().x(), 
                gsafety_max_acceleration.negative().linear().x().neg()
            ), 
            clamp::clamp_assym(
                desired_acceleration.linear().y(), 
                gsafety_max_acceleration.positive().linear().y(), 
                gsafety_max_acceleration.negative().linear().y().neg()
            ), 
            clamp::clamp_assym(
                desired_acceleration.linear().z(), 
                gsafety_max_acceleration.positive().linear().z(), 
                gsafety_max_acceleration.negative().linear().z().neg()
            )
        ),
        Dimension3::new(
            clamp::clamp_assym(
                desired_acceleration.rotational().x(), 
                gsafety_max_acceleration.positive().rotational().x(), 
                gsafety_max_acceleration.negative().rotational().x().neg()
            ),
            clamp::clamp_assym(
                desired_acceleration.rotational().y(), 
                gsafety_max_acceleration.positive().rotational().y(), 
                gsafety_max_acceleration.negative().rotational().y().neg()
            ),
            clamp::clamp_assym(
                desired_acceleration.rotational().z(), 
                gsafety_max_acceleration.positive().rotational().z(), 
                gsafety_max_acceleration.negative().rotational().z().neg()
            )
        )
    )
}





#[test]
pub fn test_g_force_safety(){
    let mut desired_acceleration: ControlAxis<Dimension3<f64>> = ControlAxis::new(
        Dimension3::new(55.0, -55.0, 0.0),
        Dimension3::new(0.0, 0.0, 0.0)
    );
    let gsafety_max_acceleration = AxisContribution::new(
        ControlAxis::new(
            Dimension3::default(50.0),
            Dimension3::default(50.0)
        ),
        ControlAxis::new(
            Dimension3::default(50.0),
            Dimension3::default(50.0)
        )
    );

    desired_acceleration = process(&desired_acceleration, &gsafety_max_acceleration);

    assert!((desired_acceleration.linear().x() - gsafety_max_acceleration.positive().linear().x()).abs() < 0.001);
    assert!((desired_acceleration.linear().y() - gsafety_max_acceleration.negative().linear().y().neg()).abs() < 0.001);
}