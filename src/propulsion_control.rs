//! #Propulsion Control System

use game_utils::{control_axis::{ControlAxis, AxisContribution}, dimension3::Dimension3};
use std::ops::{Mul, Div, Add, Sub};
use crate::utils::FcsError;
use num::Float;


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
    LinXPos, 
    LinXNeg,
    LinYPos, 
    LinYNeg,
    LinZPos, 
    LinZNeg,
    RotXPos, 
    RotXNeg,
    RotYPos, 
    RotYNeg,
    RotZPos, 
    RotZNeg,
}


#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub enum ThrusterSize{
    Small,
    Medium,
    Large
}


#[derive(Clone, Copy)]
pub struct Thruster<T>{
    // this number should always be a positive value so it is (dis?)ambiguated from the axis it is contributing to
    max_thrust: T,
    size: ThrusterSize,
}
impl<T> Thruster<T>
    where T: Copy
{
    pub fn new(max_thrust: T, size: ThrusterSize) -> Self{
        Self{
            max_thrust, 
            size
        }
    }
    pub fn max_thrust(&self) -> T{self.max_thrust}
    pub fn size(&self) -> ThrusterSize{self.size}
}

pub struct ThrusterMountPoint<'a, T>{
    // is Option so that having no thruster attached can be represented
    attached_thruster: Option<Thruster<T>>,
    // a list of directions a thruster mounted at this mount point can contribute thrust to
    thrust_direction: &'a[ThrustDirection], // should this be a singular thrust vector instead? seems axis contribution could still be derived from that
    // thrust_vector: Dimension3<T> // to replace thrust_direction?
    max_thruster_size: ThrusterSize,
    mount_location: Dimension3<T>,   // relative to ship center of mass? // used to determine torque?
}
impl<'a, T> ThrusterMountPoint<'a, T>
    where T: Copy 
        + Sub<Output = T> 
        + Float
{
    pub fn new(
        attached_thruster: Option<Thruster<T>>,
        thrust_direction: &'a[ThrustDirection],
        max_thruster_size: ThrusterSize,
        mount_location: Dimension3<T>,
    ) -> Self{
        Self{
            attached_thruster, 
            thrust_direction, 
            max_thruster_size, 
            mount_location,
        }
    }
    pub fn attached_thruster(&self) -> &Option<Thruster<T>>{&self.attached_thruster}
    pub fn change_thruster(&mut self, thruster: Thruster<T>) -> Result<(), FcsError>{
        if thruster.size() > self.max_thruster_size{
            return Err(FcsError::new("Tried to attach a thruster that is too large for mount point."))
        }
        self.attached_thruster = Some(thruster);

        Ok(())
    }
    pub fn thrust_direction(&self) -> &'a[ThrustDirection]{self.thrust_direction}
    pub fn max_thruster_size(&self) -> ThrusterSize{self.max_thruster_size}
    pub fn calculate_distance_from_center(&self, center: Dimension3<T>) -> T{       // ideal center of mass. may not necessarily represent real center of mass
        (
            (center.x() - self.mount_location.x()).powi(2) +
            (center.y() - self.mount_location.y()).powi(2) +
            (center.z() - self.mount_location.z()).powi(2)
        ).sqrt()
    }
}

fn _torque<T: Mul<Output = T> + Copy>(force: T, distance: T) -> T{
    //force(Newtons) * distance to center of mass or turning point(meters) * sin(angle_of_force_generator) 
    force * distance // * 1(if force generator perpendicular)
}





/// Calculates a sum available thrust for each of 6 different axial directions, from a given set of thrusters.
/// intended to be called on instantiation, and/or when thrusters are added/replaced, not on every frame update.
pub fn calculate_available_thrust<T>(
    thruster_mount_points: &[ThrusterMountPoint<T>],
    //zero_value: T
) -> ControlAxis<Dimension3<AxisContribution<T>>>
    where T: Copy + Add<Output = T> + Float
{
    let mut available_thrust = ControlAxis::new(
        Dimension3::default(AxisContribution::new(/*zero_value, zero_value*/num::zero(), num::zero())),
        Dimension3::default(AxisContribution::new(/*zero_value, zero_value*/num::zero(), num::zero()))
    );
    //
    //let mut available_thrust = Dimension3::default(zero);
    //let mut available_torque = Dimension3::default(zero);
    //

    for thruster_mount_point in thruster_mount_points{
        match thruster_mount_point.attached_thruster(){
            Some(thruster) => {
                add_available_thrust_for_thrust_direction(
                    thruster, 
                    thruster_mount_point, 
                    &mut available_thrust
                )
            },
            None => {/*do nothing*/}
        }
    }

    available_thrust
}

fn add_available_thrust_for_thrust_direction<T>(    //rename? sum available thrust per direction?
    thruster: &Thruster<T>, 
    thruster_mount_point: &ThrusterMountPoint<T>,
    available_thrust: &mut ControlAxis<Dimension3<AxisContribution<T>>>,
)
    where T: Add<Output = T> + Copy + Sub<Output = T> + Float
{
    // if thruster can contribute thrust in desired direction, add its max thrust to the max available thrust for that direction
    for thrust_direction in thruster_mount_point.thrust_direction(){
        match thrust_direction{
            ThrustDirection::LinXPos => {
                let sum = available_thrust.linear().x().positive() + thruster.max_thrust();
                available_thrust.linear_mut().x_mut().set_positive(sum);
            },
            ThrustDirection::LinXNeg => {
                let sum = available_thrust.linear().x().negative() + thruster.max_thrust();
                available_thrust.linear_mut().x_mut().set_negative(sum);
            },
            ThrustDirection::LinYPos => {
                let sum = available_thrust.linear().y().positive() + thruster.max_thrust();
                available_thrust.linear_mut().y_mut().set_positive(sum);
            },
            ThrustDirection::LinYNeg => {
                let sum = available_thrust.linear().y().negative() + thruster.max_thrust();
                available_thrust.linear_mut().y_mut().set_negative(sum);
            },
            ThrustDirection::LinZPos => {
                let sum = available_thrust.linear().z().positive() + thruster.max_thrust();
                available_thrust.linear_mut().z_mut().set_positive(sum);
            },
            ThrustDirection::LinZNeg => {
                let sum = available_thrust.linear().z().negative() + thruster.max_thrust();
                available_thrust.linear_mut().z_mut().set_negative(sum);
            },
            ThrustDirection::RotXPos => {
                let sum = available_thrust.rotational().x().positive() 
                + thruster.max_thrust(); // + torque(thruster.max_thrust(), thruster_mount_point.distance_from_center());
                available_thrust.rotational_mut().x_mut().set_positive(sum);
            },
            ThrustDirection::RotXNeg => {
                let sum = available_thrust.rotational().x().negative() 
                + thruster.max_thrust();
                available_thrust.rotational_mut().x_mut().set_negative(sum);
            },
            ThrustDirection::RotYPos => {
                let sum = available_thrust.rotational().y().positive() + thruster.max_thrust();
                available_thrust.rotational_mut().y_mut().set_positive(sum);
            },
            ThrustDirection::RotYNeg => {
                let sum = available_thrust.rotational().y().negative() + thruster.max_thrust();
                available_thrust.rotational_mut().y_mut().set_negative(sum);
            },
            ThrustDirection::RotZPos => {
                let sum = available_thrust.rotational().z().positive() + thruster.max_thrust();
                available_thrust.rotational_mut().z_mut().set_positive(sum);
            },
            ThrustDirection::RotZNeg => {
                let sum = available_thrust.rotational().z().negative() + thruster.max_thrust();
                available_thrust.rotational_mut().z_mut().set_negative(sum);
            },
        }
    }
}



/// use available thrust per axis and mass/moment of inertia to calculate available acceleration per axis.
// torque(or moment) measured in newton-meters = Force(Newtons) * distance((meters)to center of mass/turning point)
pub fn calculate_available_acceleration<T>(
    available_thrust: &ControlAxis<Dimension3<AxisContribution<T>>>,
    mass: T
) -> ControlAxis<Dimension3<AxisContribution<T>>>
    where T: Mul<Output = T> + Div<Output = T> + Copy
{
    ControlAxis::new(
        Dimension3::new(
            AxisContribution::new(
                available_thrust.linear().x().positive() / mass,
                available_thrust.linear().x().negative() / mass
            ),
            AxisContribution::new(
                available_thrust.linear().y().positive() / mass,
                available_thrust.linear().y().negative() / mass
            ),
            AxisContribution::new(
                available_thrust.linear().z().positive() / mass,
                available_thrust.linear().z().negative() / mass
            )
        ),
        Dimension3::new(
            AxisContribution::new(
                calculate_angular_acceleration(available_thrust.rotational().x().positive()),//available_thrust.rotational().x().positive() / mass
                calculate_angular_acceleration(available_thrust.rotational().x().negative())//available_thrust.rotational().x().negative() / mass
            ),
            AxisContribution::new(
                available_thrust.rotational().y().positive() / mass,
                available_thrust.rotational().y().negative() / mass
            ),
            AxisContribution::new(
                available_thrust.rotational().z().positive() / mass,
                available_thrust.rotational().z().negative() / mass
            )
        )
    )
}

// need an actual formula for angular acceleration
fn calculate_angular_acceleration<T>(some_input: T) -> T{some_input}



pub fn calculate_thruster_output<T>(
    desired_acceleration: &ControlAxis<Dimension3<T>>,
    _thruster_mount_points: &[ThrusterMountPoint<T>],
    mass: T,
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
fn test_thruster_size_enum_comparison(){
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
//#[should_panic]
fn thruster_too_large(){
    let thruster = Thruster::new(20_000.0, ThrusterSize::Large);
    let mut mount_point = ThrusterMountPoint::new(
        None, 
        &[ThrustDirection::LinXPos], 
        ThrusterSize::Small,
        Dimension3::default(num::zero()),
    );

    // uncomment to display error message when tests are run
    //match mount_point.change_thruster(thruster){
    //    Ok(val) => val,
    //    Err(e) => panic!("{}", e)
    //}

    assert!(mount_point.change_thruster(thruster).is_err())
}

#[test]
fn test_calculate_available_thrust(){
    let thruster_suite: [ThrusterMountPoint<f32>; 6] = [
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            &[ThrustDirection::LinXPos],
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            &[ThrustDirection::LinXNeg],
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            &[ThrustDirection::LinYPos],
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            &[ThrustDirection::LinYNeg],
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            &[ThrustDirection::LinZPos],
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            &[ThrustDirection::LinZNeg],
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        )
    ];
    
    let available_thrust = calculate_available_thrust(
        &thruster_suite,
        //0.0
    );

    let expected = ControlAxis::new(
        Dimension3::new(
            AxisContribution::new(20_000.0, 20_000.0),
            AxisContribution::new(20_000.0, 20_000.0),
            AxisContribution::new(20_000.0, 20_000.0)
        ),
        Dimension3::new(
            AxisContribution::new(0.0, 0.0),
            AxisContribution::new(0.0, 0.0),
            AxisContribution::new(0.0, 0.0)
        )
    );

    assert!((available_thrust.linear().x().positive() - expected.linear().x().positive()).abs() < 0.001);
    assert!((available_thrust.linear().y().positive() - expected.linear().y().positive()).abs() < 0.001);
    assert!((available_thrust.linear().z().positive() - expected.linear().z().positive()).abs() < 0.001);
    
    assert!((available_thrust.rotational().x().positive() - expected.rotational().x().positive()).abs() < 0.001);
    assert!((available_thrust.rotational().y().positive() - expected.rotational().y().positive()).abs() < 0.001);
    assert!((available_thrust.rotational().z().positive() - expected.rotational().z().positive()).abs() < 0.001);
    
    assert!((available_thrust.linear().x().negative() - expected.linear().x().negative()).abs() < 0.001);
    assert!((available_thrust.linear().y().negative() - expected.linear().y().negative()).abs() < 0.001);
    assert!((available_thrust.linear().z().negative() - expected.linear().z().negative()).abs() < 0.001);
    
    assert!((available_thrust.rotational().x().negative() - expected.rotational().x().negative()).abs() < 0.001);
    assert!((available_thrust.rotational().y().negative() - expected.rotational().y().negative()).abs() < 0.001);
    assert!((available_thrust.rotational().z().negative() - expected.rotational().z().negative()).abs() < 0.001);
}

#[test]
fn test_calculate_available_acceleration(){
    let available_thrust = ControlAxis::new(
        Dimension3::new(
            AxisContribution::new(20_000.0, 20_000.0),
            AxisContribution::new(20_000.0, 20_000.0),
            AxisContribution::new(20_000.0, 20_000.0)
        ),
        Dimension3::new(
            AxisContribution::new(0.0, 0.0),
            AxisContribution::new(0.0, 0.0),
            AxisContribution::new(0.0, 0.0)
        )
    );

    let mass: f32 = 2_000.0;

    let available_acceleration = calculate_available_acceleration(
        &available_thrust,
        mass
    );

    let expected = ControlAxis::new(
        Dimension3::new(
            AxisContribution::new(10.0, 10.0),
            AxisContribution::new(10.0, 10.0),
            AxisContribution::new(10.0, 10.0),
        ),
        Dimension3::new(
            AxisContribution::new(0.0, 0.0),
            AxisContribution::new(0.0, 0.0),
            AxisContribution::new(0.0, 0.0),
        )
    );

    assert!((available_acceleration.linear().x().positive() - expected.linear().x().positive()).abs() < 0.001);
    assert!((available_acceleration.linear().y().positive() - expected.linear().y().positive()).abs() < 0.001);
    assert!((available_acceleration.linear().z().positive() - expected.linear().z().positive()).abs() < 0.001);

    assert!((available_acceleration.rotational().x().positive() - expected.rotational().x().positive()).abs() < 0.001);
    assert!((available_acceleration.rotational().y().positive() - expected.rotational().y().positive()).abs() < 0.001);
    assert!((available_acceleration.rotational().z().positive() - expected.rotational().z().positive()).abs() < 0.001);

    assert!((available_acceleration.linear().x().negative() - expected.linear().x().negative()).abs() < 0.001);
    assert!((available_acceleration.linear().y().negative() - expected.linear().y().negative()).abs() < 0.001);
    assert!((available_acceleration.linear().z().negative() - expected.linear().z().negative()).abs() < 0.001);

    assert!((available_acceleration.rotational().x().negative() - expected.rotational().x().negative()).abs() < 0.001);
    assert!((available_acceleration.rotational().y().negative() - expected.rotational().y().negative()).abs() < 0.001);
    assert!((available_acceleration.rotational().z().negative() - expected.rotational().z().negative()).abs() < 0.001);
}