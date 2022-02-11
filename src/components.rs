use pid_controller::PID;
use dimension3::{ClampedDimension3, Dimension3, MaxDimension3, MinDimension3};
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





#[derive(Clone, Copy)]
pub struct LinearAssist{
    enabled: bool,
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



/// Holds the state of gforce safety system.
/// the system limits g forces experienced by the pilot, if enabled.
/// represents max allowable g forces, as assigned by the pilot, positive and negative on each control axis.
#[derive(Clone, Copy)]
pub struct GForceSafety<T>{
    enabled: bool,
    linear: MinDimension3<T>, // MinDimension ensures we don't have values below some minimum value
    rotational: MinDimension3<T>,
    neg_linear: MaxDimension3<T>, // MaxDimension ensures we don't have values above some maximum value
    neg_rotational: MaxDimension3<T>,
}
impl<T> GForceSafety<T>
    where T: Copy
    + PartialOrd
{
    pub fn new(zero: T) -> Self{
        Self{
            enabled: true,
            linear: MinDimension3::new(zero, zero, zero, zero),
            rotational: MinDimension3::new(zero, zero, zero, zero),
            neg_linear: MaxDimension3::new(zero, zero, zero, zero),
            neg_rotational: MaxDimension3::new(zero, zero, zero, zero),
        }
    }
    
    pub fn enabled(self: &Self) -> bool{self.enabled}
    pub fn toggle(self: &mut Self){self.enabled = !self.enabled}
    
    //pub fn linear(self: &Self) -> Dimension3<T>{self.linear}
    pub fn linear/*_ref*/(self: &Self) -> &MinDimension3<T>{&self.linear}
    pub fn linear_mut(self: &mut Self) -> &mut MinDimension3<T>{&mut self.linear}
    
    //pub fn rotational(self: &Self) -> Dimension3<T>{self.rotational}
    pub fn rotational/*_ref*/(self: &Self) -> &MinDimension3<T>{&self.rotational}
    pub fn rotational_mut(self: &mut Self) -> &mut MinDimension3<T>{&mut self.rotational}

    pub fn neg_linear(self: &Self) -> &MaxDimension3<T>{&self.neg_linear}
    pub fn neg_linear_mut(self: &mut Self) -> &MaxDimension3<T>{&mut self.neg_linear}

    pub fn neg_rotational(self: &Self) -> &MaxDimension3<T>{&self.neg_rotational}
    pub fn neg_rotational_mut(self: &mut Self) -> &MaxDimension3<T>{&mut self.neg_rotational}
}



/// Holds input and output for the Flight Control System.
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



/// Pilot specified maximum velocity for each control axis.
pub struct MaxVelocity<T>{
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



/// interpereted as a percentage of max available acceleration, allowing the pilot to limit max acceleration as desired.
/// low proportion here, and high max velocity, allows craft to slowly achieve a high velocity.
/// high proportion here, and low max velocity, allows craft to quickly achieve a low velocity.
/// both are valid flight profiles for different contexts.
pub struct OutputProportion<T>{
    linear: ClampedDimension3<T>,
    rotational: ClampedDimension3<T>,
}
impl<T> OutputProportion<T>
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
