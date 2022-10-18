use num::Float;

#[derive(Debug, PartialEq)]
pub struct FcsError<'a>{
    //source: //line and column number where error is returned
    message: &'a str,
}
impl<'a> FcsError<'a>{
    pub fn new(message: &'a str) -> Self{
        Self{message}
    }
}
impl<'a> std::fmt::Display for FcsError<'a>{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result{
        write!(f, "{}", self.message)
    }
}
impl<'a> std::error::Error for FcsError<'a>{}





pub fn multiply_compare_zero<T: Float + num::Zero>(input: T, /*comparison: T,*/ high_multiplier: T, low_multiplier: T) -> T{
    if input > num::zero()/*comparison*/{
        input * high_multiplier
    }
    else if input < num::zero()/*comparison*/{
        input * low_multiplier
    }
    else{num::zero()/*comparison*/}
}