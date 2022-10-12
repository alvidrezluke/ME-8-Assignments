use std::f64::consts::PI;
use webots_rs::bindings::{WbDeviceTag};
use nalgebra::DMatrix;

fn main() {
    const TIME_STEP: i32 = 50;

    println!("Rust controller has started");
    webots_rs::wb_robot_init();

    let pan_motor:WbDeviceTag = webots_rs::wb_robot_get_device("pan");
    let tilt_motor:WbDeviceTag = webots_rs::wb_robot_get_device("tilt");

    let _camera: WbDeviceTag = webots_rs::wb_robot_get_device("camera");

    let mut t: f64 = 0.0;

    let mut vec_pan_pos: Vec<f64> = vec![0.0];
    let mut vec_tilt_pos: Vec<f64> = vec![0.0];
    let apple_movement_pan_constants: [f64; 4] = calc_constants(0.0, 7.0 * PI / 4.0, 0.0, 0.0, 0.0, 2.0);
    let apple_movement_tilt_constants: [f64; 4] = calc_constants(0.0, PI / 6.0 + 0.2, 0.0, PI / 2.0, 0.0, 2.0);
    let ball_movement_pan_constants: [f64; 4] = calc_constants(7.0 * PI / 4.0, PI / 4.0, 0.0, PI / 2.0, 2.0, 5.0);
    let ball_movement_tilt_constants: [f64; 4] = calc_constants(PI / 6.0 + 0.2, PI / 6.0 + 0.15, PI / 2.0, 0.0, 2.0, 5.0);
    let origin_movement_pan_constants: [f64; 4] = calc_constants(PI / 4.0, 0.0, PI / 2.0, 0.0, 5.0, 7.0);
    let origin_movement_tilt_constants: [f64; 4] = calc_constants(PI / 6.0 + 0.15, 0.0, 0.0, 0.0, 5.0, 7.0);

    loop {
        if webots_rs::wb_robot_step(TIME_STEP) == -1 {
            break;
        }

        t += TIME_STEP as f64 / 1000.0;

        if t <= 2.0 {
            let new_pan_angle = calc_angle(apple_movement_pan_constants, t).abs();
            let new_pan_vel = calc_velocity(apple_movement_pan_constants, t).abs();
            let new_tilt_angle = -calc_angle(apple_movement_tilt_constants, t).abs();
            let new_tilt_vel = calc_velocity(apple_movement_tilt_constants, t).abs();
            println!("Time: {} | New pan angle: {} | New pan velocity: {} | New tilt angle: {} | New tilt velocity: {}", t, new_pan_angle, new_pan_vel, new_tilt_angle, new_tilt_vel);
            vec_pan_pos.push(new_pan_angle);
            vec_tilt_pos.push(new_tilt_angle);
            webots_rs::wb_motor_set_position(pan_motor, new_pan_angle);
            webots_rs::wb_motor_set_velocity(pan_motor, new_pan_vel);
            webots_rs::wb_motor_set_position(tilt_motor, new_tilt_angle);
            webots_rs::wb_motor_set_velocity(tilt_motor, new_tilt_vel);
        } else if t <= 5.0 {
            let new_pan_angle = calc_angle(ball_movement_pan_constants, t).abs();
            let new_pan_vel = calc_velocity(ball_movement_pan_constants, t).abs();
            let new_tilt_angle = -calc_angle(ball_movement_tilt_constants, t).abs();
            let new_tilt_vel = calc_velocity(ball_movement_tilt_constants, t).abs();
            println!("Time: {} | New pan angle: {} | New pan velocity: {} | New tilt angle: {} | New tilt velocity: {}", t, new_pan_angle, new_pan_vel, new_tilt_angle, new_tilt_vel);
            vec_pan_pos.push(new_pan_angle);
            vec_tilt_pos.push(new_tilt_angle);
            webots_rs::wb_motor_set_position(pan_motor, new_pan_angle);
            webots_rs::wb_motor_set_velocity(pan_motor, new_pan_vel);
            webots_rs::wb_motor_set_position(tilt_motor, new_tilt_angle);
            webots_rs::wb_motor_set_velocity(tilt_motor, new_tilt_vel);
        } else if t <= 7.0 {
            let new_pan_angle = calc_angle(origin_movement_pan_constants, t).abs();
            let new_pan_vel = calc_velocity(origin_movement_pan_constants, t).abs();
            let new_tilt_angle = -calc_angle(origin_movement_tilt_constants, t).abs();
            let new_tilt_vel = calc_velocity(origin_movement_tilt_constants, t).abs();
            println!("Time: {} | New pan angle: {} | New pan velocity: {} | New tilt angle: {} | New tilt velocity: {}", t, new_pan_angle, new_pan_vel, new_tilt_angle, new_tilt_vel);
            vec_pan_pos.push(new_pan_angle);
            vec_tilt_pos.push(new_tilt_angle);
            webots_rs::wb_motor_set_position(pan_motor, new_pan_angle);
            webots_rs::wb_motor_set_velocity(pan_motor, new_pan_vel);
            webots_rs::wb_motor_set_position(tilt_motor, new_tilt_angle);
            webots_rs::wb_motor_set_velocity(tilt_motor, new_tilt_vel);
        } else if t > 7.0 {
            break;
        }
    }

    print!("panAngle = [");
    for angle in vec_pan_pos {
        print!("{}, ", angle);
    }
    println!("]");

    print!("tiltAngle = [");
    for angle in vec_tilt_pos {
        print!("{}, ", angle);
    }
    println!("]");

    webots_rs::wb_robot_cleanup();
}

fn calc_constants(
    theta_initial: f64,
    theta_final: f64,
    velocity_initial: f64,
    velocity_final: f64,
    time_initial: f64,
    time_final: f64
) -> [f64; 4] {
    let matrix = DMatrix::from_row_slice(
        4,
        4,
        &[1.0, time_initial, time_initial.powf(2.0), time_initial.powf(3.0),
        0.0, 1.0, 2.0 * time_initial, 3.0 * time_initial.powf(3.0),
        1.0, time_final, time_final.powf(2.0), time_final.powf(3.0),
        0.0, 1.0, 2.0 * time_final, 3.0 * time_final.powf(2.0)]
    );

    let constant_vec = DMatrix::from_row_slice(4, 1, &[theta_initial, velocity_initial, theta_final, velocity_final]);

    let inv_matrix = matrix.try_inverse().unwrap();

    let solved_matrix = inv_matrix * constant_vec;

    let mut calculated_constants: [f64; 4] = [0.0, 0.0, 0.0, 0.0];

    for (i, val) in solved_matrix.iter().enumerate() {
        calculated_constants[i] = *val;
    }
    println!("Constants: {:?}", calculated_constants);

    return calculated_constants;
}

fn calc_angle(constants: [f64; 4], time: f64) -> f64 {
    return constants[0] + constants[1]*time + constants[2]*(time.powf(2.0)) + constants[3]*(time.powf(3.0));
}

fn calc_velocity(constants: [f64; 4], time: f64) -> f64 {
    return constants[1] + 2.0 * constants[2] * time + 3.0 * constants[3] * (time.powf(2.0));
}
