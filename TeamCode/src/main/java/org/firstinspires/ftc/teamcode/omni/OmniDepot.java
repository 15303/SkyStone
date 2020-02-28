package org.firstinspires.ftc.teamcode.omni;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode ;
import com.qualcomm.robotcore.hardware.DcMotor ;
import com.qualcomm.robotcore.hardware.Servo ;
import com.qualcomm.robotcore.hardware.DistanceSensor ;
import com.qualcomm.robotcore.hardware.ColorSensor ;
import com.qualcomm.robotcore.util.ElapsedTime ;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit ;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous (

  name= "OmniDepot" ,
  group = "2"

)

public class OmniDepot extends LinearOpMode {
  
  boolean is_red = true;
  boolean is_skystone = false;
  
  private ElapsedTime runtime = new ElapsedTime() ;
  
  private Servo grabber = null ;
  private Servo dragger = null ;
  private DcMotor slider = null;
  
  private DistanceSensor sensor_proximity_east = null;
  private DistanceSensor sensor_proximity_west = null;
  private DistanceSensor sensor_distance_south = null;
  private ColorSensor sensor_color_east = null;
  private ColorSensor sensor_color_west = null;
  
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
  double[] adjusted_powers = {0,0,0,0};
  
  double proximity_stone = 0 ;
  double distance_south = 0 ;
  double distance_first_skystone = 0;
  double distance_second_skystone = 0;
  
  double target_orientation = 0;
  
  double orientation = 0 ;
  
  private void drive ( String direction , double power ) {
  
  if (direction == "X") {
  
  // drive away from drivers
  
  if (is_red) {
  
  power = -power;
  
  }
  
  set_target_powers(-power, -power, power, power);
  
  } else if (direction == "Y") {
  
  // drive toward stones
  
  set_target_powers(-power, power, power, -power);
  
  } else if (direction == "Spin") {
  
  // spin clockwise red , cc blue
  
  if (is_red) {
  
  power = -power;
  
  }
  
  set_target_powers(power, power, power, power);
  
  }
  
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
    double orientationAdjustment = ( target_orientation - orientation ) / 60;
    for ( int i = 0 ; i < 4 ; i++ ){
      adjusted_powers[i] = target_powers[i] + orientationAdjustment;
    }
    
    for ( int i = 0 ; i < 4 ; i++ ){
      motors[i].setPower(adjusted_powers[i]);
    }
  }
  private void grab ( boolean should_grab ) {
    grabber.setPosition ( should_grab ? 0 : 0.5 );
  }
  
  private boolean check_skystone ( ColorSensor color_sensor ) {
    return color_sensor.red() + color_sensor.green() < 4 * color_sensor.blue();
  }
  
  private double get_distance ( DistanceSensor distance_sensor ) {
    return distance_sensor.getDistance(DistanceUnit.INCH);
  }
  
  private void update_sensors () {
    is_skystone = check_skystone ( is_red ? sensor_color_west : sensor_color_east);
    proximity_stone = get_distance ( is_red ? sensor_proximity_west : sensor_proximity_east);
    distance_south = get_distance( sensor_distance_south );
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
      "stone proximity",
      proximity_stone
    );
    telemetry.addData(
      "south distance",
      proximity_stone
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
    task = newTask;
    for ( int i = 0 ; i < 4 ; i++ ) {
      motors[i].setPower(0);
    }
    sleep(100);
    runtime.reset();
  }
  private void initialize () {

    for ( int i = 0 ; i < 4 ; i++ ){
      motors[i] = hardwareMap.get ( DcMotor.class , motor_names[i] ) ;
    }

    slider = hardwareMap.get ( DcMotor.class , "slider") ;
    grabber = hardwareMap.get ( Servo.class , "grabber" ) ;
    dragger = hardwareMap.get ( Servo.class , "dragger" ) ;

    sensor_proximity_west = hardwareMap.get ( DistanceSensor.class ,"sensorW" );
    sensor_proximity_east = hardwareMap.get ( DistanceSensor.class ,"sensorE" );

    sensor_distance_south = hardwareMap.get ( DistanceSensor.class ,"sensorS" );

    sensor_color_west = hardwareMap.get ( ColorSensor.class,"sensorW" );
    sensor_color_east = hardwareMap.get ( ColorSensor.class,"sensorE" );
    
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit= BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled = true;
    parameters.loggingTag = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    is_red = get_distance ( sensor_proximity_west ) > get_distance ( sensor_proximity_east ) ;

    telemetry.addData(
      "team",
      is_red ? "red" : "blue"
    );
    telemetry.update();
  }
  private boolean robot_is_not_going_to_destroy_us_all () {
    return ( opModeIsActive() && runtime.seconds() < 8 );
  }
  private void run () {

    set_task("go to stones far");
    drive ( "X" , 1 , 1000);
    grab(false);

    set_task("go to stones near");
    while ( proximity_stone > 1.8 && robot_is_not_going_to_destroy_us_all() ) {
      drive ( "X" , 0.3 );
      update_sensors();
      update_motors();
    }
  
    set_task("find first skystone");
    while ( !is_skystone && robot_is_not_going_to_destroy_us_all() ) {
      drive ( "Y" , -0.5);
      update_sensors();
      update_motors();
    }
    distance_first_skystone = distance_south;
    distance_second_skystone = distance_first_skystone - 12;

    set_task("align latitude with first skystone");
    drive("Y",-1,200);

    set_task("align longitude with first skystone");
    drive("X",1,600);
    
    set_task("grab first skystone");
    grab(true);
    sleep(2000);

    set_task("go to inner");
    drive("X",-1,900);
    
    set_task("drive to foundation");
    drive("Y",1,2000);
    
    set_task("release first skystone");
    grab(false);

    set_task("align latitude with second skystone");
    drive("Y",-1,2000);
    while ( distance_south > distance_second_skystone && robot_is_not_going_to_destroy_us_all() ) {
      drive("Y",-1);
    }

    set_task("go to second skystone");
    while ( proximity_stone > 1.8 && robot_is_not_going_to_destroy_us_all() ) {
      drive("X",0.3);
    }

    set_task("grab second skystone");
    grab(true);

    set_task("park under the bridge");
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