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


@com.qualcomm.robotcore.eventloop.opmode.Autonomous ( name= "Omni Auto Depot" )

public class Omni_Auto_Depot extends LinearOpMode {

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

  double distance_stone = 0 ;
  double distance_station_wall = 0 ;
  double distance_south_wall = 0 ;

  double distance_south_wall_to_second_skystone = 0 ;

  private void drive ( String direction , double power ) {
    if ( direction == "Spin" ) {
      // spin clockwise red , cc blue
      if (is_red) {
        power = -power;
      }
      set_target_powers (power, power, power, power);
    } else if ( direction == "X" ) {
      if(is_red){
        set_target_powers (0, power, 0, -power);
      }else{
        set_target_powers ( -power, 0, power,0);
      }
    } else if ( direction == "Y" ){
      if(is_red){
        set_target_powers ( -power, 0, power,0);
      }else{
        set_target_powers (0, power, 0, -power);
      }
    }
    update_sensors();
    update_motors();
  }

  private void drive ( String direction , double power , double ms ) {
    drive ( direction , power );
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
      motors[i].setPower(target_powers[i] - orientation / 60 );
    }
  }
  private void grab ( boolean should_grab ) {
    grabber.setPosition ( should_grab ? 0 : 0.6 );
  }

  private boolean check_skystone ( ColorSensor color_sensor ) {
    return color_sensor.red() + color_sensor.green() < 4 * color_sensor.blue();
  }

  private double get_distance ( DistanceSensor distance_sensor ) {
    return distance_sensor.getDistance(DistanceUnit.INCH);
  }

  private void update_sensors () {
    is_skystone = check_skystone ( is_red ? sensor_color_nw : sensor_color_ne);
    distance_stone = get_distance ( is_red ? sensor_distance_nw : sensor_distance_ne );
    distance_station_wall = get_distance( is_red ? sensor_distance_se : sensor_distance_sw );
    distance_south_wall = get_distance( is_red ? sensor_distance_sw : sensor_distance_se );
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
            "stone distance",
            distance_stone
    );
    telemetry.addData(
            "station wall distance",
            distance_station_wall
    );
    telemetry.addData(
            "south wall distance",
            distance_south_wall
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

    is_red = get_distance ( sensor_distance_sw ) > get_distance ( sensor_distance_se ) ;

    telemetry.addData(
            "team",
            is_red ? "red" : "blue"
    );
    telemetry.update();
  }
  private boolean robot_is_not_going_to_destroy_us_all () {
    return ( opModeIsActive() && runtime.seconds() < 5 );
  }
  private void find_skystone(){

    grab(false);
    set_task("go near the depot");
    while( distance_station_wall < 25 && robot_is_not_going_to_destroy_us_all() ) {
      drive ( "X" , 1 );
    }

    set_task("go next to depot");
    while ( distance_stone > 1.8 && distance_station_wall < 50 && robot_is_not_going_to_destroy_us_all() ) {
      drive ( "X" , 0.3 );
    }

    set_task("find skystone");
    while ( !is_skystone && distance_south_wall > 5 && robot_is_not_going_to_destroy_us_all() ) {
      drive ( "Y" , -0.5);
    }

    distance_south_wall_to_second_skystone = distance_south_wall - 16 ;

    set_task("align latitude with skystone");
    drive("Y",-1,100);

    set_task("grab skystone");
    grab(true);
    drive("X",1,800);

    set_task("go to inner lane");
    while ( distance_station_wall > 27 && robot_is_not_going_to_destroy_us_all() ) {
      drive ( "X" , -1);
    }

    set_task("go under bridge");
    while ( distance_south_wall < 70 && robot_is_not_going_to_destroy_us_all() ) {
      drive("Y",1);
    }

    set_task("go to foundation");
    drive("Y",1,2000);

    grab(false);
    set_task("go under bridge");
    drive( "Y" , -1 , 1000);
    grab(true);

  }
  private void run () {

    find_skystone();

    set_task("go back to depot");
    while( distance_south_wall > distance_south_wall_to_second_skystone && robot_is_not_going_to_destroy_us_all() ){
      drive("Y",-1 );
    }

    find_skystone();

    set_task("park under bridge");
    drive("Y",-1,1500);
    set_task("end");

  }

  @Override
  public void runOpMode () {
    initialize();
    waitForStart();
    run();
  }
}