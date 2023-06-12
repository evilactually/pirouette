// Vehicle
// Defines location and size of wheels

// Path
// f(t): R -> R^4
// Function of displacement and rotation with respect to time

// type Path = fn(t:Float) -> Vector3f;

// fn path_velocity(Path:path, t:Float) -> Vector3f {

// }
#[macro_use] extern crate impl_ops;
use std::ops;
use std::f32;

pub type Float = f32;

#[derive(Debug, Default, Copy, Clone)]
pub struct Vector2f {
    pub x: Float,
    pub y: Float,
}

impl Vector2f {
    pub fn has_nans(&self) -> bool {
        self.x.is_nan() || self.y.is_nan()
    }
    pub fn length_squared(&self) -> Float {
        self.x * self.x + self.y * self.y
    }
    pub fn length(&self) -> Float {
        self.length_squared().sqrt()
    }
}


#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct Vector3f {
    pub x: Float,
    pub y: Float,
    pub z: Float,
}

impl Vector3f {
    pub fn has_nans(&self) -> bool {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }
    pub fn abs(&self) -> Vector3f {
        Vector3f {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs(),
        }
    }
    pub fn length_squared(&self) -> Float {
        self.x * self.x + self.y * self.y + self.z * self.z
    }
    pub fn length(&self) -> Float {
        self.length_squared().sqrt()
    }
    /// Compute a new vector pointing in the same direction but with unit
    /// length.
    pub fn normalize(&self) -> Vector3f {
        *self / self.length()
    }
}

impl_op_ex!(+|a: &Vector3f, b: &Vector3f| -> Vector3f {
    Vector3f {
        x: a.x + b.x,
        y: a.y + b.y,
        z: a.z + b.z,
    }
});

impl_op_ex!(+|a: &Vector2f, b: &Vector2f| -> Vector2f {
    Vector2f {
        x: a.x + b.x,
        y: a.y + b.y,
    }
});

impl_op_ex!(*|a: Float, b: &Vector2f| -> Vector2f {
    Vector2f {
        x: a*b.x,
        y: a*b.y,
    }
});



impl_op_ex!(-|a: &Vector3f, b: &Vector3f| -> Vector3f {
    Vector3f {
        x: a.x - b.x,
        y: a.y - b.y,
        z: a.z - b.z,
    }
});

// impl_op_ex!(-|a: &Point3f, b: &Point3f| -> Vector3f {
//     Vector3f {
//         x: a.x - b.x,
//         y: a.y - b.y,
//         z: a.z - b.z,
//     }
// });


// impl_op_ex!(-|a: &Point2f, b: &Point2f| -> Vector2f {
//     Vector2f {
//         x: a.x - b.x,
//         y: a.y - b.y,
//     }
// });


impl_op_ex!(*|a: &Vector3f, b: Float| -> Vector3f {
    Vector3f {
        x: a.x * b,
        y: a.y * b,
        z: a.z * b,
    }
});

impl_op_ex!(/|a: &Vector3f, b: Float| -> Vector3f {
    assert_ne!(b, 0.0 as Float);
    let inv: Float = 1.0 as Float / b;
    Vector3f {
        x: a.x * inv,
        y: a.y * inv,
        z: a.z * inv,
    }
});


impl_op_ex!(-|a: &Vector2f, b: &Vector2f| -> Vector2f {
    Vector2f {
        x: a.x - b.x,
        y: a.y - b.y,
    }
});


impl_op!(-|a: Vector2f| -> Vector2f { Vector2f { x: -a.x, y: -a.y } });


impl_op_ex!(/|a: &Vector2f, b: Float| -> Vector2f {
    assert_ne!(b, 0.0 as Float);
    let inv: Float = 1.0 as Float / b;
    Vector2f {
        x: a.x * inv,
        y: a.y * inv,
    }
});

/// Given two vectors in 3D, the cross product is a vector that is
/// perpendicular to both of them.
pub fn vec3_cross_vec3(v1: &Vector3f, v2: &Vector3f) -> Vector3f {
    let v1x: f64 = v1.x as f64;
    let v1y: f64 = v1.y as f64;
    let v1z: f64 = v1.z as f64;
    let v2x: f64 = v2.x as f64;
    let v2y: f64 = v2.y as f64;
    let v2z: f64 = v2.z as f64;
    Vector3f {
        x: ((v1y * v2z) - (v1z * v2y)) as Float,
        y: ((v1z * v2x) - (v1x * v2z)) as Float,
        z: ((v1x * v2y) - (v1y * v2x)) as Float,
    }
}

pub trait Trajectory{
    // fn new(F : f) -> Self
    // where
    //     F: Fn(Float) -> Vector3f;        
    fn get_position(&self,t:Float) -> Vector2f;
    fn get_velocity(&self,t:Float) -> Vector2f;
    fn get_acceleration(&self,t:Float) -> Vector2f;
    fn get_angle(&self,t:Float) -> Float;
    fn get_angular_speed(&self,t:Float) -> Float;
    fn get_angular_acceleration(&self,t:Float) -> Float;
    //fn get_pivot(&self,t:Float) -> Vector2f;
    //fn get_path_curvature(&self,t:Float) -> Float; //?
}

//#[derive(Debug)]
struct GenericPath<'a> {
    path: &'a dyn Fn(Float) -> (Vector2f, Float)
}

impl<'a> GenericPath<'a> {
    pub fn new(&self,path:(&'a dyn Fn(Float) -> (Vector2f, Float))) -> Self {
        GenericPath{path}
    }
    pub fn get_position(&self,t:Float) -> Vector2f {
        let (pos,_) = (*self.path)(t);
        return pos;
    }
    pub fn get_angle(&self,t:Float) -> Float {
        let (_,angle) = (*self.path)(t);
        return angle;
    }
    fn get_angular_speed(&self,t:Float) -> Float {
        let dt = 0.01;
        return (self.get_angle(t + dt) - self.get_angle(t))/dt;
    }
    fn get_angular_acceleration(&self,t:Float) -> Float {
        let dt = 0.01;
        return (self.get_angular_speed(t + dt) - self.get_angular_speed(t))/dt;   
    }
    pub fn get_velocity(&self,t:Float) -> Vector2f {
        let dt = 0.01;
        return (self.get_position(t + dt) - self.get_position(t))/dt;
    }
    pub fn get_acceleration(&self,t:Float) -> Vector2f {
        let dt = 0.01;
        return (self.get_velocity(t + dt) - self.get_velocity(t))/dt;
    }
    // DEAD CODE: Don't need this
    // pub fn get_curvature(&self,t:Float) -> Float {
    //     let v = self.get_velocity(t);
    //     let a = self.get_acceleration(t);
    //     let v_cross_a = Vector3f{x:0.0,y:0.0,z:v.x*a.y-v.y*a.x};
    //     v_cross_a.length()/v.length().powf(3.0)
    // }
}

// BASIC PRINCIPLE:
// ARTICULATED CASTER STEERING
// Imagine a chair with caster wheels.
// Observe that Wheels always align with their velocity vector.
// You can do many types of motion with caster wheels.
// You may go straight.
// You may spin the chair around while moving forward.
// You may pivot about arbitrary stationary point on the ground.
// You may pivot about a moving pivot.
// Importantly, if we know velocity of each wheel as a point on a rigit body, we are basically done.
// Orient each wheel according to it's velocity. Spin it up according to magnitude of velocity vector and wheel radius.

struct WheelInfo {
    position: Vector2f,
    radius: Float
}

struct WheelCommand {
    angle: Float,
    angular_velocity: Float
}

fn wheelCommands(wheels:&Vec<WheelInfo>, trajectory:&impl Trajectory, t:Float) -> Vec<WheelCommand> {
    let mut wheelCommands: Vec<WheelCommand> = Vec::new();
    for wheel in wheels {
        let r = wheel.position.length(); // m
        let angle = trajectory.get_angle(t); // rad
        let tangential_speed = r*trajectory.get_angular_speed(t); // m/s 
        let velocity = trajectory.get_velocity(t) + tangential_speed*Vector2f{x:f32::cos(angle), y:f32::sin(angle)}; // m/s
        let wheel_circumference = 2.0*std::f32::consts::PI*wheel.radius; // m
        wheelCommands.push(WheelCommand{angle: velocity.x.atan2(velocity.y), // rad 
                                        angular_velocity: (wheel_circumference/velocity.length())*2.0*std::f32::consts::PI}); // rad/s
    }
    wheelCommands
}  

fn test_circular(t:Float) -> (Vector2f, Float) {
    (Vector2f{x:f32::cos(t), y:f32::sin(t)}, t)
}

fn test_forward_spinning_cw(t:Float) -> (Vector2f, Float) {
    (Vector2f{x:0.0, y:t}, t)
}

fn main() {

    println!("Hello, world!");
}
