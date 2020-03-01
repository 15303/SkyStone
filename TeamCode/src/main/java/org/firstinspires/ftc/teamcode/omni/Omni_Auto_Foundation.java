package org.firstinspires.ftc.teamcode.omni;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous ( name= "Omni Auto Foundation" )

public class Omni_Auto_Foundation extends LinearOpMode {

  boolean is_red = true;
  boolean is_skystone = false;

  private ElapsedTime runtime = new ElapsedTime() ;

  private Servo grabber = null ;
  private Servo dragger = null ;
  private DcMotor slider = null;

  private DistanceSensor sensor_distance_nw = null;
  private DistanceSensor sensor_distance_ne = null;
  private DistanceSensor sensor_distance_se = null;
  private DistanceSensor sensor_distance_sw = null;
  private ColorSensor sensor_color_ne = null;
  private ColorSensor sensor_color_nw = null;

  String task ="initialize";

  BNO055IMU imu;

  String[] motor_names = {
          "driveNW",
          "driveNE",
          "driveSE",
          "driveSW"
  };

  private DcMotor[] motors = {null,null,null,null};
  double[] target_powers = {0,0,0,0};

  double orientation = 0;
  double target_orientation = 0;

  double distance_station_wall = 0 ;
  double distance_north_wall = 0 ;

  double distance_south_wall_to_second_skystone = 0 ;

  private void drive ( double x , double y , double spin ) {
    if ( is_red ) {
      x *= -1 ;
      spin *= -1;
    }
    set_target_powers(
            -x - y + spin,
            -x + y + spin,
             x + y + spin,
            x - y + spin);
    update_sensors();
    update_motors();
  }

  private void drive ( double x , double y , double spin , double ms ) {
    drive ( x , y , spin );
    active_sleep(ms);
  }

  private void lift ( double power ) {
    slider.setPower( power ) ;
  }

  private void set_target_powers ( double NW , double NE , double SE , double SW ) {
    target_powers[0] = NW;
    target_powers[1] = NE;
    target_powers[2] = SE;
    target_powers[3] = SW;
  }

  private void update_motors () {
    for ( int i = 0 ; i < 4 ; i++ ){
      motors[i].setPower(target_powers[i] + (target_orientation-orientation)/60);
    }
  }
  private void grab ( boolean should_grab ) {
    grabber.setPosition ( should_grab ? 0 : 0.6 );
  }
  private void drag ( boolean should_drag ) {
    dragger.setPosition ( should_drag ? 0.85 : 0.5 );
  }
  private boolean check_skystone ( ColorSensor color_sensor ) {
    return color_sensor.red() + color_sensor.green() < 4 * color_sensor.blue();
  }

  private double get_distance ( DistanceSensor distance_sensor ) {
    return distance_sensor.getDistance(DistanceUnit.INCH);
  }

  private void update_sensors () {
    is_skystone = check_skystone ( is_red ? sensor_color_nw : sensor_color_ne);
    distance_station_wall = ( get_distance( sensor_distance_nw ) + get_distance( sensor_distance_ne ) ) / 2 ;
    distance_north_wall = get_distance( is_red ? sensor_distance_sw : sensor_distance_se );
    orientation = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    telemetry.addData(
            "team",
            is_red ? "red" : "blue"
    );
    telemetry.addData(
            "current task",
            task
    );
    telemetry.addData(
            "station wall distance",
            distance_station_wall
    );
    telemetry.addData(
            "north wall distance",
            distance_north_wall
    );
    telemetry.addData(
            "orientation",
            orientation
    );
    telemetry.addData(
            "time",
            runtime.seconds()
    );
    telemetry.update();
  }
  private void active_sleep (double ms) {
    while (runtime.seconds() < ms / 1000 && robot_is_not_going_to_destroy_us_all()) {
      update_sensors();
      update_motors();
    }
  }
  private void set_task ( String newTask ) {
    update_sensors();
    task = newTask;
    for ( int i = 0 ; i < 4 ; i++ ) {
      motors[i].setPower(0);
    }
    sleep(50);
    runtime.reset();
  }
  private void initialize () {

    for ( int i = 0 ; i < 4 ; i++ ){
      motors[i] = hardwareMap.get ( DcMotor.class , motor_names[i] ) ;
    }

    slider = hardwareMap.get ( DcMotor.class , "slider") ;
    grabber = hardwareMap.get ( Servo.class , "grabber" ) ;
    dragger = hardwareMap.get ( Servo.class , "dragger" ) ;

    sensor_distance_nw = hardwareMap.get ( DistanceSensor.class ,"sensorNW" );
    sensor_distance_ne = hardwareMap.get ( DistanceSensor.class ,"sensorNE" );

    sensor_distance_se = hardwareMap.get ( DistanceSensor.class ,"sensorSE" );
    sensor_distance_sw = hardwareMap.get ( DistanceSensor.class ,"sensorSW" );

    sensor_color_nw = hardwareMap.get ( ColorSensor.class,"sensorNW" );
    sensor_color_ne = hardwareMap.get ( ColorSensor.class,"sensorNE" );

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit= BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    is_red = get_distance ( sensor_distance_sw ) < get_distance ( sensor_distance_se ) ;

    telemetry.addData(
            "team",
            is_red ? "red" : "blue"
    );
    telemetry.update();
  }

  private boolean robot_is_not_going_to_destroy_us_all () {
    return ( opModeIsActive() && runtime.seconds() < 5 );
  }

  private void run () {

    drag(false);
    set_task("align latitude with foundation");
    drive(1,0,0,400);

    set_task("back up to foundation");
    drive(0,-1,0,700);

    set_task("back up to foundation");
    drive(0,-1,0,500);

    set_task("grab foundation");
    drag(true);
    sleep(2000);

    set_task("drag foundation");
    drive(0,1,0,1500);

    set_task("spin foundation");
    target_orientation = is_red ? -90 : 90;
    while ( Math.abs(target_orientation - orientation) > 10 ){
      drive(0,0,1);
    }

    set_task("push foundation against north wall");
    drive(0,-1,0,700);

    drag(false);

    set_task("go to outer lane");
    drive(1,0,0,1000);

    set_task("park under bridge");
    drive(0,1,0,1700);

  }

  @Override
  public void runOpMode () {
    initialize();
    waitForStart();
    run();
  }
}