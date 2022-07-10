//! #Propulsion Control System

use game_utils::{control_axis::{ControlAxis, AxisContribution}, dimension3::Dimension3};
use std::ops::{Mul, Div, Add, Neg};



/// Calculates a sum available thrust for each of 6 different axial directions, from a given set of thrusters.
/// intended to be called on instantiation, and/or when thrusters are added/replaced, not on every frame update.
pub fn calculate_available_thrust<T>(
    available_thrust: &mut AxisContribution<ControlAxis<Dimension3<T>>>,
    thruster_mount_points: &[ThrusterMountPoint<Thruster<T>>],
    zero_value: T
)
    where T: Copy + Add<Output = T> + Neg<Output = T> + PartialOrd
{
    // clear any previous values in axis_max_thrust
    available_thrust.positive_mut().linear_mut().set_x(zero_value);
    available_thrust.positive_mut().linear_mut().set_y(zero_value);
    available_thrust.positive_mut().linear_mut().set_z(zero_value);

    available_thrust.negative_mut().linear_mut().set_x(zero_value);
    available_thrust.negative_mut().linear_mut().set_y(zero_value);
    available_thrust.negative_mut().linear_mut().set_z(zero_value);

    available_thrust.positive_mut().rotational_mut().set_x(zero_value);
    available_thrust.positive_mut().rotational_mut().set_y(zero_value);
    available_thrust.positive_mut().rotational_mut().set_z(zero_value);

    available_thrust.negative_mut().rotational_mut().set_x(zero_value);
    available_thrust.negative_mut().rotational_mut().set_y(zero_value);
    available_thrust.negative_mut().rotational_mut().set_z(zero_value);
    
    // determine max available thrust for each movable direction

    // add up max_thrust from each thruster to a sum thrust for each axis it can contribute to
    for thruster_mount_point in /*&*/thruster_mount_points{
        // if mount point has an attached thruster...
        match thruster_mount_point.attached_thruster(){
            Some(val) => {
                // and if it can contribute thrust in desired direction, add its max thrust to the max available thrust for that direction
                for thrust_direction in thruster_mount_point.thrust_direction(){
                    match thrust_direction{
                        ThrustDirection::Right => {
                            let sum = available_thrust.positive().linear().x() + *val.max_thrust();
                            available_thrust.positive_mut().linear_mut().set_x(sum);
                        },
                        ThrustDirection::Left => {
                            let sum = available_thrust.negative().linear().x() + *val.max_thrust();
                            available_thrust.negative_mut().linear_mut().set_x(sum);
                        },
                        ThrustDirection::Up => {
                            let sum = available_thrust.positive().linear().y() + *val.max_thrust();
                            available_thrust.positive_mut().linear_mut().set_y(sum);
                        },
                        ThrustDirection::Down => {
                            let sum = available_thrust.negative().linear().y() + *val.max_thrust();
                            available_thrust.negative_mut().linear_mut().set_y(sum);
                        },
                        ThrustDirection::Forward => {
                            let sum = available_thrust.positive().linear().z() + *val.max_thrust();
                            available_thrust.positive_mut().linear_mut().set_z(sum);
                        },
                        ThrustDirection::Backward => {
                            let sum = available_thrust.negative().linear().z() + *val.max_thrust();
                            available_thrust.negative_mut().linear_mut().set_z(sum);
                        },
                        ThrustDirection::PitchUp => {
                            let sum = available_thrust.positive().rotational().x() + *val.max_thrust();
                            available_thrust.positive_mut().rotational_mut().set_x(sum)
                        },
                        ThrustDirection::PitchDown => {
                            let sum = available_thrust.negative().rotational().x() + *val.max_thrust();
                            available_thrust.negative_mut().rotational_mut().set_x(sum)
                        },
                        ThrustDirection::YawRight => {
                            let sum = available_thrust.positive().rotational().y() + *val.max_thrust();
                            available_thrust.positive_mut().rotational_mut().set_y(sum)
                        },
                        ThrustDirection::YawLeft => {
                            let sum = available_thrust.negative().rotational().y() + *val.max_thrust();
                            available_thrust.negative_mut().rotational_mut().set_y(sum)
                        },
                        ThrustDirection::RollRight => {
                            let sum = available_thrust.positive().rotational().z() + *val.max_thrust();
                            available_thrust.positive_mut().rotational_mut().set_z(sum)
                        },
                        ThrustDirection::RollLeft => {
                            let sum = available_thrust.negative().rotational().z() + *val.max_thrust();
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
}



/// use available thrust per axis and mass/moment of inertia to calculate available acceleration per axis.
pub fn calculate_available_acceleration<T>(
    available_acceleration: &mut AxisContribution<ControlAxis<Dimension3<T>>>,
    available_thrust: &AxisContribution<ControlAxis<Dimension3<T>>>,
    mass: T
)
    where T: Mul<Output = T> + Div<Output = T> + Copy
{
    available_acceleration.positive_mut().linear_mut().set_x(
        available_thrust.positive().linear().x() / mass
    );
    available_acceleration.negative_mut().linear_mut().set_x(
        available_thrust.negative().linear().x() / mass
    );

    available_acceleration.positive_mut().linear_mut().set_y(
        available_thrust.positive().linear().y() / mass
    );
    available_acceleration.negative_mut().linear_mut().set_y(
        available_thrust.negative().linear().y() / mass
    );

    available_acceleration.positive_mut().linear_mut().set_z(
        available_thrust.positive().linear().z() / mass
    );
    available_acceleration.negative_mut().linear_mut().set_z(
        available_thrust.negative().linear().z() / mass
    );
}



pub fn calculate_thruster_output<T: Copy + Mul<Output = T>>(
    desired_acceleration: &ControlAxis<Dimension3<T>>,
    //thruster_mount_points: &[ThrusterMountPoint<Thruster<T>>],
    mass: T,
    //zero: T,
    output_thrust: &mut ControlAxis<Dimension3<T>>
){
    //f = m * a

    output_thrust.linear_mut().set_x(desired_acceleration.linear().x() * mass);
    output_thrust.linear_mut().set_y(desired_acceleration.linear().y() * mass);
    output_thrust.linear_mut().set_z(desired_acceleration.linear().z() * mass);

    output_thrust.rotational_mut().set_x(desired_acceleration.rotational().x() * mass);
    output_thrust.rotational_mut().set_y(desired_acceleration.rotational().y() * mass);
    output_thrust.rotational_mut().set_z(desired_acceleration.rotational().z() * mass);

    //create collection containing any thruster that can contribute to desired thrust for that axis

    //request desired thrust from those thrusters divided by number of thrusters that can contribute
}







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


#[derive(Clone, Copy)]
pub enum MountPointSize{
    Small,
    Medium,
    Large
}


#[derive(Clone, Copy)]
pub struct Thruster<T: Copy>{
    max_thrust: T,
    mount_point_size: MountPointSize,
}
impl<T> Thruster<T>
    where T: Copy
{
    pub fn new(max_thrust: T, mount_point_size: MountPointSize) -> Self{
        Thruster{
            max_thrust,
            mount_point_size
        }
    }
    pub fn max_thrust(self: &Self) -> &T{&self.max_thrust}
    pub fn max_mount_point_size(self: &Self) -> &MountPointSize{&self.mount_point_size}
}


pub struct ThrusterMountPoint<'a, T>{
    attached_thruster: Option<T>,
    thrust_direction: &'a[ThrustDirection],//Vec<ThrustDirection>,
    size: MountPointSize,
}
impl<'a, T> ThrusterMountPoint<'a, T>{
    pub fn new(
        attached_thruster: Option<T>,
        thrust_direction: &'a[ThrustDirection],//Vec<ThrustDirection>, 
        size: MountPointSize
    ) -> Self{
        ThrusterMountPoint{
            attached_thruster,
            thrust_direction,
            size
        }
    }
    pub fn attached_thruster(self: &Self) -> &Option<T>{&self.attached_thruster}
    pub fn change_thruster(self: &mut Self, thruster: T){
        self.attached_thruster = Some(thruster);
    }
    pub fn thrust_direction(self: &Self) -> &'a[ThrustDirection]{&self.thrust_direction}//&Vec<ThrustDirection>{&self.thrust_direction}
    pub fn size(self: &Self) -> &MountPointSize{&self.size}
}








#[test]
pub fn test_calculate_available_thrust(){
    let thruster_suite: [ThrusterMountPoint<Thruster<f32>>; 6] = [
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    MountPointSize::Small
                )
            ),
            &[ThrustDirection::Right],
            //Vec::from(
            //    [ThrustDirection::Right]
            //),
            MountPointSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    MountPointSize::Small
                )
            ),
            &[ThrustDirection::Left],
            //Vec::from(
            //    [ThrustDirection::Left]
            //),
            MountPointSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    MountPointSize::Small
                )
            ),
            &[ThrustDirection::Up],
            //Vec::from(
            //    [ThrustDirection::Up]
            //),
            MountPointSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    MountPointSize::Small
                )
            ),
            &[ThrustDirection::Down],
            //Vec::from(
            //    [ThrustDirection::Down]
            //),
            MountPointSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    MountPointSize::Small
                )
            ),
            &[ThrustDirection::Forward],
            //Vec::from(
            //    [ThrustDirection::Forward]
            //),
            MountPointSize::Small
        ),
        ThrusterMountPoint::new(
            Some(
                Thruster::new(
                    20_000.0,
                    MountPointSize::Small
                )
            ),
            &[ThrustDirection::Backward],
            //Vec::from(
            //    [ThrustDirection::Backward]
            //),
            MountPointSize::Small
        )
    ];

    let mut available_thrust = AxisContribution::new(
        ControlAxis::new(
            Dimension3::new(
                0.0, 0.0, 0.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        ),
        ControlAxis::new(
            Dimension3::new(
                0.0, 0.0, 0.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        )
    );
    
    calculate_available_thrust(
        &mut available_thrust,
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

    let mut available_acceleration = AxisContribution::new(
        ControlAxis::new(
            Dimension3::new(
                0.0, 0.0, 0.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        ),
        ControlAxis::new(
            Dimension3::new(
                0.0, 0.0, 0.0
            ),
            Dimension3::new(
                0.0, 0.0, 0.0
            )
        )
    );

    let mass: f32 = 2_000.0;

    calculate_available_acceleration(
        &mut available_acceleration,
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