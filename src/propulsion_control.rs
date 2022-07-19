//! #Propulsion Control System

use game_utils::{control_axis::{ControlAxis, AxisContribution}, dimension3::Dimension3};
use std::ops::{Mul, Div, Add, Neg};


// we need to ensure that the negative component of available acceleration does not use values with a "-" sign. 
// by holding the value in the negative component, we are already acknowledging its negativity.
// components that use these values should account for this in their implementations.

// each component of available acceleration will be a positive number, because it is calculated from the max thrust
// each thruster in a suite of thrusters can output, which will always be a positive number, as to disambiguate it
// from its thrust direction based on mount point location

// these positive values are then assigned to either a positive or negative subset of available acceleration, to
// represent their contribution to each side of an axis





#[derive(PartialEq)]
pub enum ThrustDirection{
    Forward,
    Backward,
    Up,
    Down,
    Left,
    Right,
    YawRight,
    YawLeft,
    RollRight,
    RollLeft,
    PitchUp,
    PitchDown
}


#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub enum ThrusterSize{  // need to impl PartialOrd and PartialEq?
    Small,
    Medium,
    Large
}


#[derive(Clone, Copy)]
pub struct Thruster<T: Copy>{
    // this number should always be a positive value so it is (dis?)ambiguated from the axis it is contributing to
    max_thrust: T,
    size: ThrusterSize,
}
impl<T> Thruster<T>
    where T: Copy
{
    pub fn new(max_thrust: T, size: ThrusterSize) -> Self{Self{max_thrust, size}}
    pub fn max_thrust(self: &Self) -> T{self.max_thrust}
    pub fn size(self: &Self) -> ThrusterSize{self.size}
}

pub struct ThrusterMountPoint<'a, T: Copy>{
    // is Option so that having no thruster attached can be represented
    attached_thruster: Option<Thruster<T>>,
    // a list of directions a thruster mounted at this mount point can contribute thrust to
    thrust_direction: &'a[ThrustDirection],//Vec<ThrustDirection>,
    max_thruster_size: ThrusterSize,
}
impl<'a, T> ThrusterMountPoint<'a, T>
    where T: Copy
{
    pub fn new(
        attached_thruster: Option<Thruster<T>>,
        thrust_direction: &'a[ThrustDirection],//Vec<ThrustDirection>, 
        max_thruster_size: ThrusterSize
    ) -> Self{
        Self{
            attached_thruster,
            thrust_direction,
            max_thruster_size
        }
    }
    pub fn attached_thruster(self: &Self) -> &Option<Thruster<T>>{&self.attached_thruster}
    pub fn change_thruster(self: &mut Self, thruster: Thruster<T>) -> Result<(), ()>{
        if thruster.size() > self.max_thruster_size{
            return Err(())      // returning empty tuple because there is only one way to fail, and no needed associated info
        }
        
        self.attached_thruster = Some(thruster);

        Ok(())
    }
    pub fn thrust_direction(self: &Self) -> &'a[ThrustDirection]{self.thrust_direction}
    pub fn max_thruster_size(self: &Self) -> ThrusterSize{self.max_thruster_size}
}





/// Calculates a sum available thrust for each of 6 different axial directions, from a given set of thrusters.
/// intended to be called on instantiation, and/or when thrusters are added/replaced, not on every frame update.
pub fn calculate_available_thrust<T>(
    // does available_thrust need to be passed in as &mut
    // could this be calculated and returned from the function instead?
    thruster_mount_points: &[ThrusterMountPoint<T>],
    zero_value: T
) -> AxisContribution<ControlAxis<Dimension3<T>>>
    where T: Copy + Add<Output = T> + Neg<Output = T> + PartialOrd
{
    let mut available_thrust = AxisContribution::new(
        ControlAxis::new(
            Dimension3::default(zero_value),
            Dimension3::default(zero_value)
        ),
        ControlAxis::new(
            Dimension3::default(zero_value),
            Dimension3::default(zero_value)
        )
    );
    
    // determine max available thrust for each movable direction

    // add up max_thrust from each thruster to a sum thrust for each axis it can contribute to
    for thruster_mount_point in /*&*/thruster_mount_points{
        // if mount point has an attached thruster...
        match thruster_mount_point.attached_thruster(){
            Some(thruster) => {
                // and if it can contribute thrust in desired direction, add its max thrust to the max available thrust for that direction
                for thrust_direction in thruster_mount_point.thrust_direction(){
                    match thrust_direction{
                        ThrustDirection::Right => {
                            let sum = available_thrust.positive().linear().x() + thruster.max_thrust();
                            available_thrust.positive_mut().linear_mut().set_x(sum);
                        },
                        ThrustDirection::Left => {
                            let sum = available_thrust.negative().linear().x() + thruster.max_thrust();
                            available_thrust.negative_mut().linear_mut().set_x(sum);
                        },
                        ThrustDirection::Up => {
                            let sum = available_thrust.positive().linear().y() + thruster.max_thrust();
                            available_thrust.positive_mut().linear_mut().set_y(sum);
                        },
                        ThrustDirection::Down => {
                            let sum = available_thrust.negative().linear().y() + thruster.max_thrust();
                            available_thrust.negative_mut().linear_mut().set_y(sum);
                        },
                        ThrustDirection::Forward => {
                            let sum = available_thrust.positive().linear().z() + thruster.max_thrust();
                            available_thrust.positive_mut().linear_mut().set_z(sum);
                        },
                        ThrustDirection::Backward => {
                            let sum = available_thrust.negative().linear().z() + thruster.max_thrust();
                            available_thrust.negative_mut().linear_mut().set_z(sum);
                        },
                        ThrustDirection::PitchUp => {
                            let sum = available_thrust.positive().rotational().x() + thruster.max_thrust();
                            available_thrust.positive_mut().rotational_mut().set_x(sum)
                        },
                        ThrustDirection::PitchDown => {
                            let sum = available_thrust.negative().rotational().x() + thruster.max_thrust();
                            available_thrust.negative_mut().rotational_mut().set_x(sum)
                        },
                        ThrustDirection::YawRight => {
                            let sum = available_thrust.positive().rotational().y() + thruster.max_thrust();
                            available_thrust.positive_mut().rotational_mut().set_y(sum)
                        },
                        ThrustDirection::YawLeft => {
                            let sum = available_thrust.negative().rotational().y() + thruster.max_thrust();
                            available_thrust.negative_mut().rotational_mut().set_y(sum)
                        },
                        ThrustDirection::RollRight => {
                            let sum = available_thrust.positive().rotational().z() + thruster.max_thrust();
                            available_thrust.positive_mut().rotational_mut().set_z(sum)
                        },
                        ThrustDirection::RollLeft => {
                            let sum = available_thrust.negative().rotational().z() + thruster.max_thrust();
                            available_thrust.negative_mut().rotational_mut().set_z(sum)
                        },
                    }
                }
            },
            None => {
                //do nothing
            }
        }
    }

    available_thrust
}



/// use available thrust per axis and mass/moment of inertia to calculate available acceleration per axis.
pub fn calculate_available_acceleration<T>(
    available_thrust: &AxisContribution<ControlAxis<Dimension3<T>>>,
    mass: T
) -> AxisContribution<ControlAxis<Dimension3<T>>>
    where T: Mul<Output = T> + Div<Output = T> + Copy
{
    AxisContribution::new(
        ControlAxis::new(
            Dimension3::new(
                available_thrust.positive().linear().x() / mass, 
                available_thrust.positive().linear().y() / mass, 
                available_thrust.positive().linear().z() / mass
            ), 
            Dimension3::new(
                // would use moment of inertia or something. idk...
                available_thrust.positive().rotational().x() / mass, 
                available_thrust.positive().rotational().y() / mass, 
                available_thrust.positive().rotational().z() / mass
            )
        ), 
        ControlAxis::new(
            Dimension3::new(
                available_thrust.negative().linear().x() / mass, 
                available_thrust.negative().linear().y() / mass, 
                available_thrust.negative().linear().z() / mass
            ), 
            Dimension3::new(
                // would use moment of inertia or something. idk...
                available_thrust.negative().rotational().x() / mass,
                available_thrust.negative().rotational().y() / mass,
                available_thrust.negative().rotational().z() / mass
            )
        )
    )
}



pub fn calculate_thruster_output<T>(
    desired_acceleration: &ControlAxis<Dimension3<T>>,
    //thruster_mount_points: &[ThrusterMountPoint<Thruster<T>>],
    mass: T,
    //zero: T,
) -> ControlAxis<Dimension3<T>>
    where T: Mul<Output = T>
    + Copy
{
    //f = m * a

    ControlAxis::new(
        Dimension3::new(
            desired_acceleration.linear().x() * mass, 
            desired_acceleration.linear().y() * mass, 
            desired_acceleration.linear().z() * mass
        ),
        Dimension3::new(
            desired_acceleration.rotational().x() * mass, 
            desired_acceleration.rotational().y() * mass, 
            desired_acceleration.rotational().z() * mass
        )
    )

    //create collection containing any thruster that can contribute to desired thrust for that axis

    //request desired thrust from those thrusters divided by number of thrusters that can contribute
}








#[test]
pub fn test_thruster_size_enum_comparison(){
    assert!(ThrusterSize::Small < ThrusterSize::Medium);
    assert!(ThrusterSize::Small < ThrusterSize::Large);

    assert!(ThrusterSize::Medium > ThrusterSize::Small);
    assert!(ThrusterSize::Medium < ThrusterSize::Large);

    assert!(ThrusterSize::Large > ThrusterSize::Small);
    assert!(ThrusterSize::Large > ThrusterSize::Medium);

    assert!(ThrusterSize::Small == ThrusterSize::Small);
    assert!(ThrusterSize::Medium == ThrusterSize::Medium);
    assert!(ThrusterSize::Large == ThrusterSize::Large);
}

#[test]
pub fn test_calculate_available_thrust(){
    let thruster_suite: [ThrusterMountPoint<f32>; 6] = [
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    ThrusterSize::Small
                )
            ),
            &[ThrustDirection::Right],
            //&Vec::from(ThrustDirection::Right),
            ThrusterSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    ThrusterSize::Small
                )
            ),
            &[ThrustDirection::Left],
            //&Vec::from(ThrustDirection::Left),
            ThrusterSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    ThrusterSize::Small
                )
            ),
            &[ThrustDirection::Up],
            //&Vec::from(ThrustDirection::Up),
            ThrusterSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    ThrusterSize::Small
                )
            ),
            &[ThrustDirection::Down],
            //&Vec::from(ThrustDirection::Down),
            ThrusterSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    ThrusterSize::Small
                )
            ),
            &[ThrustDirection::Forward],
            //&Vec::from(ThrustDirection::Forward),
            ThrusterSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    ThrusterSize::Small
                )
            ),
            &[ThrustDirection::Backward],
            //&Vec::from(ThrustDirection::Backward),
            ThrusterSize::Small
        )
    ];
    
    let available_thrust = calculate_available_thrust(
        &thruster_suite,
        0.0
    );

    let expected = AxisContribution::new(
        ControlAxis::new(
            Dimension3::new(
                20_000.0, 20_000.0, 20_000.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        ),
        ControlAxis::new(
            Dimension3::new(
                20_000.0, 20_000.0, 20_000.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        )
    );

    assert!((expected.positive().linear().x() - available_thrust.positive().linear().x()).abs() < 0.001);
    assert!((expected.positive().linear().y() - available_thrust.positive().linear().y()).abs() < 0.001);
    assert!((expected.positive().linear().z() - available_thrust.positive().linear().z()).abs() < 0.001);
    
    assert!((expected.positive().rotational().x() - available_thrust.positive().rotational().x()).abs() < 0.001);
    assert!((expected.positive().rotational().y() - available_thrust.positive().rotational().y()).abs() < 0.001);
    assert!((expected.positive().rotational().z() - available_thrust.positive().rotational().z()).abs() < 0.001);
    
    assert!((expected.negative().linear().x() - available_thrust.negative().linear().x()).abs() < 0.001);
    assert!((expected.negative().linear().y() - available_thrust.negative().linear().y()).abs() < 0.001);
    assert!((expected.negative().linear().z() - available_thrust.negative().linear().z()).abs() < 0.001);
    
    assert!((expected.negative().rotational().x() - available_thrust.negative().rotational().x()).abs() < 0.001);
    assert!((expected.negative().rotational().y() - available_thrust.negative().rotational().y()).abs() < 0.001);
    assert!((expected.negative().rotational().z() - available_thrust.negative().rotational().z()).abs() < 0.001);
}

#[test]
pub fn test_calculate_available_acceleration(){
    let available_thrust = AxisContribution::new(
        ControlAxis::new(
            Dimension3::new(
                20_000.0, 20_000.0, 20_000.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        ),
        ControlAxis::new(
            Dimension3::new(
                20_000.0, 20_000.0, 20_000.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        )
    );

    let mass: f32 = 2_000.0;

    let available_acceleration = calculate_available_acceleration(
        &available_thrust,
        mass
    );

    let expected = AxisContribution::new(
        ControlAxis::new(
            Dimension3::new(
                10.0, 10.0, 10.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        ),
        ControlAxis::new(
            Dimension3::new(
                10.0, 10.0, 10.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        )
    );

    assert!((expected.positive().linear().x() - available_acceleration.positive().linear().x()).abs() < 0.001);
    assert!((expected.positive().linear().y() - available_acceleration.positive().linear().y()).abs() < 0.001);
    assert!((expected.positive().linear().z() - available_acceleration.positive().linear().z()).abs() < 0.001);

    assert!((expected.positive().rotational().x() - available_acceleration.positive().rotational().x()).abs() < 0.001);
    assert!((expected.positive().rotational().y() - available_acceleration.positive().rotational().y()).abs() < 0.001);
    assert!((expected.positive().rotational().z() - available_acceleration.positive().rotational().z()).abs() < 0.001);

    assert!((expected.negative().linear().x() - available_acceleration.negative().linear().x()).abs() < 0.001);
    assert!((expected.negative().linear().y() - available_acceleration.negative().linear().y()).abs() < 0.001);
    assert!((expected.negative().linear().z() - available_acceleration.negative().linear().z()).abs() < 0.001);

    assert!((expected.negative().rotational().x() - available_acceleration.negative().rotational().x()).abs() < 0.001);
    assert!((expected.negative().rotational().y() - available_acceleration.negative().rotational().y()).abs() < 0.001);
    assert!((expected.negative().rotational().z() - available_acceleration.negative().rotational().z()).abs() < 0.001);
}