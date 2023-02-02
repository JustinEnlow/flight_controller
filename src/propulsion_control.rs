//! #Propulsion Control System

use game_utils::{
    control_axis::{ControlAxis, AxisContribution}, 
    dimension3::{Dimension3, Vector3}
};
use crate::FcsError;
use num::Float;


// we need to ensure that the negative component of available acceleration does not use values with a "-" sign. 
// by holding the value in the negative component, we are already acknowledging its negativity.
// components that use these values should account for this in their implementations.

// each component of available acceleration will be a positive number, because it is calculated from the max thrust
// each thruster in a suite of thrusters can output, which will always be a positive number, as to disambiguate it
// from its thrust direction based on mount point location

// these positive values are then assigned to either a positive or negative subset of available acceleration, to
// represent their contribution to each side of an axis



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



pub struct ThrusterMountPoint<T>{
    // is Option so that having no thruster attached can be represented
    attached_thruster: Option<Thruster<T>>,
    thrust_direction: Vector3<T>,
    max_thruster_size: ThrusterSize,
    mount_location: Dimension3<T>,   // relative to ship center of mass? // used to determine torque?
}
impl<T> ThrusterMountPoint<T>
    where T: Float
{
    pub fn new(
        attached_thruster: Option<Thruster<T>>,
        thrust_direction: Vector3<T>,
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
    
    pub fn thrust_direction(&self) -> /*&*/Vector3<T>{/*&*/self.thrust_direction}
    
    //pub fn max_thruster_size(&self) -> ThrusterSize{self.max_thruster_size}
    
    //distance from dry center of mass to specified mount point
    pub fn distance_from_center(&self, center: Dimension3<T>) -> T{
        (
            (center.x() - self.mount_location.x()).powi(2) +
            (center.y() - self.mount_location.y()).powi(2) +
            (center.z() - self.mount_location.z()).powi(2)
        ).sqrt()
    }
}







/// Calculates a sum available thrust for each of 6 different axial directions, from a given set of thrusters.
/// intended to be called on instantiation, and/or when thrusters are added/replaced, not on every frame update.
pub fn calculate_available_thrust<T>(
    thruster_mount_points: &[ThrusterMountPoint<T>],
) -> ControlAxis<Dimension3<AxisContribution<T>>>
    where T: Float
{
    let mut available_thrust = ControlAxis::new(
        Dimension3::default(AxisContribution::new(num::zero(), num::zero())),
        Dimension3::default(AxisContribution::new(num::zero(), num::zero()))
    );

    for thruster_mount_point in thruster_mount_points{
        match thruster_mount_point.attached_thruster(){
            Some(thruster) => {
                sum_available_thrust_per_axis(
                    &thruster,
                    thruster_mount_point.thrust_direction(),
                    //thruster_mount_point.distance_from_center(ship_center_of_mass),
                    //angle between vector from center of mass to mount location, and vector of thrust_direction
                    &mut available_thrust
                )
            },
            None => {/*do nothing*/}
        }
    }

    available_thrust
}



/// thrust direction is a unit vector made up of 3 axes and a magnitude of 1.
/// each axis will be <= 1, all 3 summing to 1
/// so product of axis contribution and thruster max thrust produces axis 
/// specific max thrust possible
fn sum_available_thrust_per_axis<T>(
    thruster: &Thruster<T>,
    thrust_direction: Vector3<T>,
    //distance_from_center: T,
    //angle_of_force_generator: T,
    available_thrust: &mut ControlAxis<Dimension3<AxisContribution<T>>>,
)
    where T: Float
{
    match thrust_direction.x().is_sign_positive(){
        true => {
            let linear_sum = available_thrust.linear().x().positive() + (thrust_direction.x().abs() * thruster.max_thrust());
            available_thrust.linear_mut().x_mut().set_positive(linear_sum);

            //let rotational_sum = force(newtons) * distance_from_center * sin(angle_of_force_generator);
            //available_thrust.rotational_mut().x_mut().set_positive(rotational_sum);
        }
        false => {
            let linear_sum = available_thrust.linear().x().negative() + (thrust_direction.x().abs() * thruster.max_thrust());
            available_thrust.linear_mut().x_mut().set_negative(linear_sum);
        }
    }
    match thrust_direction.y().is_sign_positive(){
        true => {
            let linear_sum = available_thrust.linear().y().positive() + (thrust_direction.y().abs() * thruster.max_thrust());
            available_thrust.linear_mut().y_mut().set_positive(linear_sum);
        }
        false => {
            let linear_sum = available_thrust.linear().y().negative() + (thrust_direction.y().abs() * thruster.max_thrust());
            available_thrust.linear_mut().y_mut().set_negative(linear_sum);
        }
    }
    match thrust_direction.z().is_sign_positive(){
        true => {
            let linear_sum = available_thrust.linear().z().positive() + (thrust_direction.z().abs() * thruster.max_thrust());
            available_thrust.linear_mut().z_mut().set_positive(linear_sum);
        }
        false => {
            let linear_sum = available_thrust.linear().z().negative() + (thrust_direction.z().abs() * thruster.max_thrust());
            available_thrust.linear_mut().z_mut().set_negative(linear_sum);
        }
    }
}



//fn _torque<T: Float>(force: T, distance: T) -> T{
//    //force(Newtons) * distance to center of mass or turning point(meters) * sin(angle_of_force_generator) 
//    force * distance // * 1(if force generator perpendicular)
//}



/// use available thrust per axis and mass/moment of inertia to calculate available acceleration per axis.
// torque(or moment) measured in newton-meters = Force(Newtons) * distance((meters)to center of mass/turning point)
pub fn calculate_available_acceleration<T>(
    available_thrust: &ControlAxis<Dimension3<AxisContribution<T>>>,
    mass: T
) -> ControlAxis<Dimension3<AxisContribution<T>>>
    where T: Float//Mul<Output = T> + Div<Output = T> + Copy
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
    where T: Float//Mul<Output = T> + Copy
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

// simplified version of pcs that doesn't represent the physical model of the thruster setup.
// it converts desired accelerations to generalized desired thrust
pub fn calculate_simplified_thruster_output<T>(
    desired_acceleration: &ControlAxis<Dimension3<T>>,
    mass: T
) -> ControlAxis<Dimension3<T>>
    where T: Float
{
    ControlAxis::new(
        Dimension3::new(
            mass * desired_acceleration.linear().x(),
            mass * desired_acceleration.linear().y(),
            mass * desired_acceleration.linear().z()
        ), 
        Dimension3::default(num::zero())
    )
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
        Vector3::new(0.0, 0.0, 0.0, 1.0), //&[ThrustDirection::LinXPos], 
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
            //&[ThrustDirection::LinXPos],
            Vector3::new(1.0, 0.0, 0.0, 1.0),
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            //&[ThrustDirection::LinXNeg],
            Vector3::new(-1.0, 0.0, 0.0, 1.0),
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            //&[ThrustDirection::LinYPos],
            Vector3::new(0.0, 1.0, 0.0, 1.0),
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            //&[ThrustDirection::LinYNeg],
           Vector3::new(0.0, -1.0, 0.0, 1.0),
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            //&[ThrustDirection::LinZPos],
            Vector3::new(0.0, 0.0, 1.0, 1.0),
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        ),
        ThrusterMountPoint::new(
            Some(Thruster::new(20_000.0, ThrusterSize::Small)),
            //&[ThrustDirection::LinZNeg],
            Vector3::new(0.0, 0.0, -1.0, 1.0),
            ThrusterSize::Small,
            Dimension3::default(num::zero()),
        )
    ];
    
    let available_thrust = calculate_available_thrust(
        &thruster_suite);

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

    // print values for debugging
        println!(
            "available thrust linear x positive: {:?}, expected linear x positive: {:?}",
            available_thrust.linear().x().positive(), expected.linear().x().positive()
        );
        println!(
            "available thrust linear x negative: {:?}, expected linear x negative: {:?}",
            available_thrust.linear().x().negative(), expected.linear().x().negative()
        );
    //

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