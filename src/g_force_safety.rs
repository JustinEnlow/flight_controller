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
    desired_acceleration: &ControlAxis<Dimension3<T>>,
    gsafety_max_acceleration: &ControlAxis<Dimension3<AxisContribution<T>>>,    //ClampedDimension3 doesn't seem to be needed...verify later
) -> ControlAxis<Dimension3<T>>
    where T: Copy + PartialOrd + Neg<Output = T>
{
    ControlAxis::new(
        Dimension3::new(
            clamp::clamp_assym(
                desired_acceleration.linear().x(), 
                gsafety_max_acceleration.linear().x().positive(),
                -(gsafety_max_acceleration.linear().x().negative())
            ), 
            clamp::clamp_assym(
                desired_acceleration.linear().y(), 
                gsafety_max_acceleration.linear().y().positive(),
                -(gsafety_max_acceleration.linear().y().negative())
            ), 
            clamp::clamp_assym(
                desired_acceleration.linear().z(), 
                gsafety_max_acceleration.linear().z().positive(),
                -(gsafety_max_acceleration.linear().z().negative())
            )
        ),
        Dimension3::new(
            clamp::clamp_assym(
                desired_acceleration.rotational().x(), 
                gsafety_max_acceleration.rotational().x().positive(),
                -(gsafety_max_acceleration.rotational().x().negative())
            ),
            clamp::clamp_assym(
                desired_acceleration.rotational().y(), 
                gsafety_max_acceleration.rotational().y().positive(),
                -(gsafety_max_acceleration.rotational().y().negative())
            ),
            clamp::clamp_assym(
                desired_acceleration.rotational().z(), 
                gsafety_max_acceleration.rotational().z().positive(),
                -(gsafety_max_acceleration.rotational().z().negative())
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
    let gsafety_max_acceleration = ControlAxis::new(
        Dimension3::new(
            AxisContribution::new(50.0, 50.0), 
            AxisContribution::new(50.0, 50.0), 
            AxisContribution::new(50.0, 50.0)
        ), 
        Dimension3::new(
            AxisContribution::new(50.0, 50.0), 
            AxisContribution::new(50.0, 50.0), 
            AxisContribution::new(50.0, 50.0)
        )
    );

    desired_acceleration = process(&desired_acceleration, &gsafety_max_acceleration);

    assert!((desired_acceleration.linear().x() - gsafety_max_acceleration.linear().x().positive()).abs() < 0.001);
    assert!((desired_acceleration.linear().y() - (-gsafety_max_acceleration.linear().y().negative())).abs() < 0.001);
}