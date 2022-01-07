use pid_controller::PID;
use clamp;
use std::ops::Neg;



#[derive(Clone, Copy)]
pub struct ControlAxis<T>{
    linear: T,
    rotational: T,
}
impl<T> ControlAxis<T>
    where T: Copy
{
    //pub fn linear(self: &Self) -> T{self.linear}
    pub fn linear/*_ref*/(self: &Self) -> &T{&self.linear}
    pub fn linear_mut(self: &mut Self) -> &mut T{&mut self.linear}
    
    //pub fn rotational(self: &Self) -> T{self.rotational}
    pub fn rotational/*_ref*/(self: &Self) -> &T{&self.rotational}
    pub fn rotational_mut(self: &mut Self) -> &mut T{&mut self.rotational}
}



/// struct for 3 dimensions of any type
#[derive(Clone, Copy)]
pub struct Dimension3<T>{
    x: T,
    y: T,
    z: T,
}
impl<T> Dimension3<T>
    where T: Copy
{
    pub fn new(x: T, y: T, z: T) -> Self{
        Self{x, y, z}
    }
    
    pub fn x(self: &Self) -> T{self.x}
    pub fn set_x(self: &mut Self, x: T){self.x = x}
    
    pub fn y(self: &Self) -> T{self.y}
    pub fn set_y(self: &mut Self, y: T){self.y = y}
    
    pub fn z(self: &Self) -> T{self.z}
    pub fn set_z(self: &mut Self, z: T){self.z = z}
}



#[derive(Clone, Copy)]
pub struct LinearAssist{
    enabled: bool,  // does this need to be private? do we lose anything by having this as a public field?
}
impl LinearAssist{
    pub fn new() -> Self{Self{enabled: true}}
    pub fn enabled(self: &Self) -> bool{self.enabled}
    pub fn toggle(self: &mut Self){self.enabled = !self.enabled}
}

#[derive(Clone, Copy)]
pub struct RotationalAssist{
    enabled: bool,
}
impl RotationalAssist{
    pub fn new() -> Self{Self{enabled: true}}
    pub fn enabled(self: &Self) -> bool{self.enabled}
    pub fn toggle(self: & mut Self){self.enabled = !self.enabled}
}

#[derive(Clone, Copy)]
pub struct AssistOutput<T>{
    linear: Dimension3<T>,
    rotational: Dimension3<T>,
}
impl<T> AssistOutput<T>
    where T: Copy
{
    pub fn new(zero: T) -> Self{
        Self{
            linear: Dimension3::new(zero, zero, zero),
            rotational: Dimension3::new(zero, zero, zero)
        }
    }
    
    //pub fn linear(self: &Self) -> Dimension3<T>{self.linear}
    pub fn linear/*_ref*/(self: &Self) -> &Dimension3<T>{&self.linear}
    pub fn linear_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.linear}
    
    //pub fn rotational(self: &Self) -> Dimension3<T>{self.rotational}
    pub fn rotational/*_ref*/(self: &Self) -> &Dimension3<T>{&self.rotational}
    pub fn rotational_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.rotational}
}



/// Holds the state of gforce safety system
/// the system limits g forces experienced by the pilot, if enabled
/// represents max allowable g forces, as assigned by the pilot, for each control axis
#[derive(Clone, Copy)]
pub struct GForceSafety<T>{
    enabled: bool,
    linear: Dimension3<T>,
    rotational: Dimension3<T>,
}
impl<T> GForceSafety<T>
    where T: Copy
{
    pub fn new(zero: T) -> Self{
        Self{
            enabled: true,
            linear: Dimension3::new(zero, zero, zero),
            rotational: Dimension3::new(zero, zero, zero)
        }
    }
    
    pub fn enabled(self: &Self) -> bool{self.enabled}
    pub fn toggle(self: &mut Self){self.enabled = !self.enabled}
    
    //pub fn linear(self: &Self) -> Dimension3<T>{self.linear}
    pub fn linear/*_ref*/(self: &Self) -> &Dimension3<T>{&self.linear}
    pub fn linear_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.linear}
    
    //pub fn rotational(self: &Self) -> Dimension3<T>{self.rotational}
    pub fn rotational/*_ref*/(self: &Self) -> &Dimension3<T>{&self.rotational}
    pub fn rotational_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.rotational}
}



/// Holds input and output for the Flight Control System
pub struct FlightControlSystem<T>{
    input: ControlAxis<Dimension3<T>>,
    output: ControlAxis<Dimension3<T>>,
}
impl<T> FlightControlSystem<T>
    where T: Copy
{
    pub fn new(zero: T) -> Self{
        Self{
            input: ControlAxis{
                linear: Dimension3::new(zero, zero, zero),
                rotational: Dimension3::new(zero, zero, zero)
            },
            output: ControlAxis{
                linear: Dimension3::new(zero, zero, zero),
                rotational: Dimension3::new(zero, zero, zero)
            }
        }
    }
    
    //pub fn input(self: &Self) -> ControlAxis<Dimension3<T>>{self.input}
    pub fn input/*_ref*/(self: &Self) ->  &ControlAxis<Dimension3<T>>{&self.input}
    pub fn input_mut(self: &mut Self) -> &mut ControlAxis<Dimension3<T>>{&mut self.input}

    //pub fn output(self: &Self) -> ControlAxis<Dimension3<T>>{self.output}
    pub fn output/*_ref*/(self: &Self) -> &ControlAxis<Dimension3<T>>{&self.output}
    pub fn output_mut(self: &mut Self) -> &mut ControlAxis<Dimension3<T>>{&mut self.output}
}



/// Pilot specified maximum velocity for each control axis
pub struct MaxVelocity<T>{  //seems like these fields can be public, because we are not validating/restricting any changes
    linear: Dimension3<T>,
    rotational: Dimension3<T>,
}
impl<T> MaxVelocity<T>
    where T: Copy
{
    pub fn new(zero: T) -> Self{
        Self{
            linear: Dimension3::new(zero, zero, zero),
            rotational: Dimension3::new(zero, zero, zero)
        }
    }
    
    //pub fn linear(self: &Self) -> Dimension3<T>{self.linear}
    pub fn linear/*_ref*/(self: &Self) -> &Dimension3<T>{&self.linear}
    pub fn linear_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.linear}
    
    //pub fn rotational(self: &Self) -> Dimension3<T>{self.rotational}
    pub fn rotational/*_ref*/(self: &Self) -> &Dimension3<T>{&self.rotational}
    pub fn rotational_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.rotational}
}



/// the current velocity being experienced by a system. these value should not be assigned directly by the user or developer
pub struct Velocity<T>{
    linear: Dimension3<T>,
    rotational: Dimension3<T>,
}
impl<T> Velocity<T>
    where T: Copy
{
    pub fn new(zero: T) -> Self{
        Self{
            linear: Dimension3::new(zero, zero, zero),
            rotational: Dimension3::new(zero, zero, zero)
        }
    }

    //pub fn linear(self: &Self) -> Dimension3<T>{self.linear}
    pub fn linear/*_ref*/(self: &Self) -> &Dimension3<T>{&self.linear}
    pub fn linear_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.linear}
    
    //pub fn rotational(self: &Self) -> Dimension3<T>{self.rotational}
    pub fn rotational/*_ref*/(self: &Self) -> &Dimension3<T>{&self.rotational}
    pub fn rotational_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.rotational}
}



/// maximum acceleration that a control axis can produce, given the sum of forces for all propulsion devices on that axis
/// this will be calculated, not directly assigned
//#[derive(Clone, Copy)]
pub struct MaxAvailableAcceleration<T>{
    linear: Dimension3<T>,
    rotational: Dimension3<T>,
}
impl<T> MaxAvailableAcceleration<T>
    where T: Copy
{
    pub fn new(zero: T) -> Self{
        Self{
            linear: Dimension3{x: zero, y: zero, z: zero},
            rotational: Dimension3{x: zero, y: zero, z: zero}
        }
    }
    
    //pub fn linear(self: &Self) -> Dimension3<T>{self.linear}
    pub fn linear/*_ref*/(self: &Self) -> &Dimension3<T>{&self.linear}
    pub fn linear_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.linear}
    
    //pub fn rotational(self: &Self) -> Dimension3<T>{self.rotational}
    pub fn rotational/*_ref*/(self: &Self) -> &Dimension3<T>{&self.rotational}
    pub fn rotational_mut(self: &mut Self) -> &mut Dimension3<T>{&mut self.rotational}
}



/// interpereted as a percentage of max available acceleration, allowing the pilot to limit max acceleration as desired.
/// low proportion here, and high max velocity, allows craft to slowly achieve a high velocity
/// high proportion here, and low max velocity, allows craft to quickly achieve a low velocity
/// both are valid flight profiles for different contexts
/// 
/// ex: 
/// ```
/// let mut accel_proportion = DesiredAccelerationProportion::new(1.0, 0.0);
/// accel_proportion.linear_mut().set_x(2.0);
/// println!("{}", accel_proportion.linear().x());
/// ```
pub struct DesiredAccelerationProportion<T>{
    linear: ClampedDimension3<T>,
    rotational: ClampedDimension3<T>,
}
impl<T> DesiredAccelerationProportion<T>
    where
        T: PartialOrd
        + Neg<Output = T>
        + Copy
{
    pub fn new(one: T, zero: T) -> Self{
        Self{
            linear: ClampedDimension3::new(zero, zero, zero, one, zero),
            rotational: ClampedDimension3::new(zero, zero, zero, one, zero),
        }
    }
    
    //pub fn linear(self: &Self) -> ClampedDimension3<T>{self.linear}
    pub fn linear/*_ref*/(self: &Self) -> &ClampedDimension3<T>{&self.linear}
    pub fn linear_mut(self: &mut Self) -> &mut ClampedDimension3<T>{&mut self.linear}
    
    //pub fn rotational(self: &Self) -> ClampedDimension3<T>{self.rotational}
    pub fn rotational/*_ref*/(self: &Self) -> &ClampedDimension3<T>{&self.rotational}
    pub fn rotational_mut(self: &mut Self) -> &mut ClampedDimension3<T>{&mut self.rotational}
}



#[derive(Clone, Copy)]
pub struct ClampedDimension3<T>{
    x: T,
    y: T,
    z: T,
    high_clamp_value: T,
    low_clamp_value: T,
}
impl<T> ClampedDimension3<T>
    where
        T: std::cmp::PartialOrd
        + Neg<Output = T>
        + Copy
{
    pub fn new(x: T, y: T, z: T, high_clamp_value: T, low_clamp_value: T) -> Self{
        Self{
            x: clamp::clamp_assym(x, high_clamp_value, low_clamp_value), 
            y: clamp::clamp_assym(y, high_clamp_value, low_clamp_value),
            z: clamp::clamp_assym(z, high_clamp_value, low_clamp_value),
            high_clamp_value,
            low_clamp_value,
        }
    }

    pub fn x(self: &Self) -> T{self.x}
    pub fn set_x(self: &mut Self, x: T){
        self.x = clamp::clamp_assym(x, self.high_clamp_value, self.low_clamp_value);
    }
    
    pub fn y(self: &Self) -> T{self.y}
    pub fn set_y(self: &mut Self, y: T){
        self.y = clamp::clamp_assym(y, self.high_clamp_value, self.low_clamp_value);
    }
    
    pub fn z(self: &Self) -> T{self.z}
    pub fn set_z(self: &mut Self, z: T){
        self.z = clamp::clamp_assym(z, self.high_clamp_value, self.low_clamp_value);
    }
}



/// PID controller for each control axis
pub struct PID3d<T>{
    linear: Dimension3<PID<T>>,
    rotational: Dimension3<PID<T>>,
}
impl<T> PID3d<T>
    where
        T: std::ops::Add<Output = T> + std::ops::Sub<Output = T> + std::ops::Mul<Output = T>
        + std::ops::Div<Output = T> + std::ops::Neg<Output = T> + std::cmp::PartialOrd + Copy
{
    pub fn new(zero_value: T) -> Self{
        Self{
            linear: Dimension3::new(
                PID::new(zero_value, zero_value, zero_value, zero_value, None),
                PID::new(zero_value, zero_value, zero_value, zero_value, None),
                PID::new(zero_value, zero_value, zero_value, zero_value, None)
            ),
            rotational: Dimension3::new(
                PID::new(zero_value, zero_value, zero_value, zero_value, None),
                PID::new(zero_value, zero_value, zero_value, zero_value, None),
                PID::new(zero_value, zero_value, zero_value, zero_value, None)
            ),
        }
    }

    //pub fn linear(self: &Self) -> Dimension3<PID<T>>{self.linear}
    pub fn linear/*_ref*/(self: &Self) -> &Dimension3<PID<T>>{&self.linear}
    pub fn linear_mut(self: &mut Self) -> &mut Dimension3<PID<T>>{&mut self.linear}
    
    //pub fn rotational(self: &Self) -> Dimension3<PID<T>>{self.rotational}
    pub fn rotational/*_ref*/(self: &Self) -> &Dimension3<PID<T>>{&self.rotational}
    pub fn rotational_mut(self: &mut Self) -> &mut Dimension3<PID<T>>{&mut self.rotational}
}
