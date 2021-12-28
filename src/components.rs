use pid_controller::PID;

pub struct ControlAxis<T>{
    pub linear: T,
    pub rotational: T,
}

/// struct for 3 dimensions of any type
pub struct Dimension3<T>{
    pub x: T,
    pub y: T,
    pub z: T,
}

pub struct LinearAssist{
    pub enabled: bool,
}
impl LinearAssist{
    pub fn toggle(self: &mut Self){
        self.enabled = !self.enabled
    }
}

pub struct RotationalAssist{
    pub enabled: bool,
}
impl RotationalAssist{
    pub fn toggle(self: &mut Self){
        self.enabled = !self.enabled
    }
}

pub struct AssistOutput<T>{
    pub linear: Dimension3<T>,
    pub rotational: Dimension3<T>,
}

/// Holds the state of gforce safety system
/// the system limits g forces experienced by the pilot, if enabled
/// represents max allowable g forces, as assigned by the pilot, for each control axis
pub struct GForceSafety<T>{
    pub enabled: bool,
    pub linear: Dimension3<T>,
    pub rotational: Dimension3<T>,
}

pub struct FlightControlSystem<T>{
    pub input: ControlAxis<Dimension3<T>>,
    pub output: ControlAxis<Dimension3<T>>,
}

/// Pilot specified maximum velocity for each control axis
pub struct MaxVelocity<T>{
    pub linear: Dimension3<T>,
    pub rotational: Dimension3<T>,
}

/// the current velocity being experienced by a system. these value should not be assigned directly by the user or developer
pub struct Velocity<T>{
    pub linear: Dimension3<T>,
    pub rotational: Dimension3<T>,
}

/// maximum acceleration that a control axis can produce, given the sum of forces for all propulsion devices on that axis
/// this will be calculated, not directly assigned
pub struct MaxAvailableAcceleration<T>{
    pub linear: Dimension3<T>,
    pub rotational: Dimension3<T>,
}

/// interpereted as a percentage of max available acceleration, allowing the pilot to limit max acceleration as desired.
/// low proportion here, and high max velocity, allows craft to slowly achieve a high velocity
/// high proportion here, and low max velocity, allows craft to quickly achieve a low velocity
/// both are valid flight profiles for different contexts
pub struct DesiredAccelerationProportion<T>{
    pub linear: Dimension3<T>,
    pub rotational: Dimension3<T>,
}

/// PID controller for each control axis
pub struct PID3d<T>{
    pub linear: Dimension3<PID<T>>,
    pub rotational: Dimension3<PID<T>>,
}