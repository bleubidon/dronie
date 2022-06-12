#include <Servo.h>  // drone motors control
#include <MPU6050_DMP_YPR_wrapper.h>  // IMU interfacing
#include <PID.h>  // PID correction


// Drone orientation lexical conventions:
    // white legs are "front", red legs are "back"
    // "left" and "right" refer to the drone seen from top view
    // drone goes "forwards" when front legs are lower than back legs. It goes "backwards" otherwise

// PIDs
    PID pid_pitch, pid_roll;

    // PID correction coefficients
    float kp = 1.6F;
    float ki = .001F;
    float kd = 100.F;

    float setpoint = 0.F;
    float pid_debug[3];
    
// Motors
    int front_left_pin = 11;
    int front_right_pin = 9;
    int back_left_pin = 10;
    int back_right_pin = 8;
    Servo front_left_prop, front_right_prop, back_left_prop, back_right_prop;

    unsigned int throttle = 1150;
    unsigned int motors_min_speed = 1100;
    unsigned int motors_max_speed = 1500;
    unsigned int pwm_front_left, pwm_front_right, pwm_back_left, pwm_back_right;

// IMU
    MPU6050_DMP_YPR_wrapper imu;
    float *ypr = nullptr;

    // Calibration offsets
    int x_accel_offset = -1844;
    int y_accel_offset = 1559;
    int z_accel_offset = 1330;
    int x_gyro_offset = 55;
    int y_gyro_offset = -19;
    int z_gyro_offset = 10;

// Misc
    unsigned long serial_baud_rate = 115200;

    // Desired duration between two consecutive loop iterations, in microseconds
    // Should be >= 11000 for the IMU to always provide new angles measures
    unsigned long loop_dt_us = 11000;

    unsigned long start_delay = 0;  // in seconds
    unsigned long spin_duration = 200;  // in seconds
    unsigned long time, time_prev;


// Serial print helpers
// Note: it seems that using this << operator with Serial takes more time at execution than calling Serial.print() directly
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
	obj.print(arg);
	return obj;
}
const char endl = '\n';

void setup()
{
    Serial.begin(serial_baud_rate);
    Serial << "Setup is starting" << endl;
    while (millis() < start_delay *1000);

// Setup IMU
    Serial << "Setting up IMU" << endl;
    if (int imu_init_return_code = imu.setup(x_accel_offset, y_accel_offset, z_accel_offset,
                                             x_gyro_offset, y_gyro_offset, z_gyro_offset))
    {
        Serial << "Error while attempting to initialize the IMU (" << imu_init_return_code << ")" << endl;
        while (1)
            delay(1000);
    }

// Setup motors (all motors start spinning at throttle speed)
    Serial << "Setting up motors" << endl;
    front_left_prop.attach(front_left_pin, MIN_PULSE_WIDTH, motors_max_speed);
    front_right_prop.attach(front_right_pin, MIN_PULSE_WIDTH, motors_max_speed);
    back_left_prop.attach(back_left_pin, MIN_PULSE_WIDTH, motors_max_speed);
    back_right_prop.attach(back_right_pin, MIN_PULSE_WIDTH, motors_max_speed);

    front_left_prop.writeMicroseconds(0);
    front_right_prop.writeMicroseconds(0);
    back_left_prop.writeMicroseconds(0);
    back_right_prop.writeMicroseconds(0);
    delay(300);  // increase this value if motors are not spinning
    front_left_prop.writeMicroseconds(throttle);
    front_right_prop.writeMicroseconds(throttle);
    back_left_prop.writeMicroseconds(throttle);
    back_right_prop.writeMicroseconds(throttle);

// Setup PIDs
    pid_pitch.set_coefficients(kp, ki, kd);
    pid_pitch.set_setpoint(setpoint);
    pid_roll.set_coefficients(kp, ki, kd);
    pid_roll.set_setpoint(setpoint);
    

    Serial << "Setup is complete" << endl << endl;
    time_prev = micros();
}


void loop()
{
    if (millis() < spin_duration *1000)
    {
        // Update yaw, pitch and roll data from the IMU
        if ((ypr = imu.get_ypr()) != nullptr)
            Serial << "ypr: " << ypr[0] << ", " << ypr[1] << ", " << ypr[2];
        else
        {
            Serial << "---------------------" << endl << "NO IMU DATA AVAILABLE" << endl << "---------------------";
            return;
        }

        // Compute PIDs
        while ((time = micros()) - time_prev < loop_dt_us);
        Serial << "  || dt: " << time - time_prev;  // should be very close to loop_dt_us

        float pid_dt = (time - time_prev) / 1000.F;  // pid_dt is in milliseconds
        pid_pitch.compute(ypr[1], pid_dt);
        pid_roll.compute(ypr[2], pid_dt);

        pid_pitch.get_pid_debug(pid_debug);
        Serial << "  || [P] Cur err: " << pid_pitch.get_error();
        Serial << "  | p: " << pid_debug[0] << ", i: " << pid_debug[1] << ", d: " << pid_debug[2];
        pid_roll.get_pid_debug(pid_debug);
        Serial << "  || [R] Cur err: " << pid_roll.get_error();
        Serial << "  | p: " << pid_debug[0] << ", i: " << pid_debug[1] << ", d: " << pid_debug[2];
        Serial << "  || ";

        compute_motors_pwm_widths();
        clamp_motor_commands();
        // serial_print_motor_commands();
        serial_plot_motor_commands();
        send_commands_to_motors();

        time_prev = time;
    }

    else
    {
        Serial << "Stopping all motors" << endl;

        front_left_prop.writeMicroseconds(0);
        front_right_prop.writeMicroseconds(0);
        back_left_prop.writeMicroseconds(0);
        back_right_prop.writeMicroseconds(0);

        while (1)
            delay(1000);
    }
}

void compute_motors_pwm_widths()
{
    pwm_front_left = round(throttle + pid_pitch.get_pid() + pid_roll.get_pid());
    pwm_front_right = round(throttle + pid_pitch.get_pid() - pid_roll.get_pid());
    pwm_back_left = round(throttle - pid_pitch.get_pid() + pid_roll.get_pid());
    pwm_back_right = round(throttle - pid_pitch.get_pid() - pid_roll.get_pid());
}

void clamp_motor_commands()
{
    if (pwm_front_left < motors_min_speed) pwm_front_left = motors_min_speed;
    if (pwm_front_right < motors_min_speed) pwm_front_right = motors_min_speed;
    if (pwm_back_left < motors_min_speed) pwm_back_left = motors_min_speed;
    if (pwm_back_right < motors_min_speed) pwm_back_right = motors_min_speed;

    if (pwm_front_left > motors_max_speed) pwm_front_left = motors_max_speed;
    if (pwm_front_right > motors_max_speed) pwm_front_right = motors_max_speed;
    if (pwm_back_left > motors_max_speed) pwm_back_left = motors_max_speed;
    if (pwm_back_right > motors_max_speed) pwm_back_right = motors_max_speed;
}

void send_commands_to_motors()
{
    front_left_prop.writeMicroseconds(pwm_front_left);
    front_right_prop.writeMicroseconds(pwm_front_right);
    back_left_prop.writeMicroseconds(pwm_back_left);
    back_right_prop.writeMicroseconds(pwm_back_right);
}

void serial_print_motor_commands()
{
    Serial << "Front left motor command: " << pwm_front_left << "; front right motor command: " << pwm_front_right;
    Serial << "; back left motor command: " << pwm_back_left << "; back right motor command: " << pwm_back_right << endl;
}
void serial_plot_motor_commands()
{
    Serial << pwm_front_left << " " << pwm_front_right << " " << pwm_back_left << " " << pwm_back_right;
    Serial << " " << motors_min_speed << " " << motors_max_speed << endl;
}
