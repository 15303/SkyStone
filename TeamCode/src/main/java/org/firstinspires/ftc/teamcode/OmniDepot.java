package org.firstinspires.ftc.teamcode ;

import com.qualcomm.robotcore.eventloop.opmode.Disabled ;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode ;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous ;
import com.qualcomm.robotcore.hardware.DcMotor ;
import com.qualcomm.robotcore.hardware.Servo ;
import com.qualcomm.robotcore.hardware.DistanceSensor ;
import com.qualcomm.robotcore.hardware.ColorSensor ;
import com.qualcomm.robotcore.util.ElapsedTime ;
import com.qualcomm.robotcore.util.Range ;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit ;
import java.util.Locale ;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous (

  name  = "OmniDepot"         ,
  group = "2"

)

// @Disabled


public class OmniDepot extends LinearOpMode {


  boolean isRed;
  
  private ElapsedTime runtime = new ElapsedTime() ;

  private Servo   grabber = null ;
  private Servo   dragger = null ;
  private DcMotor slider = null;

  private DistanceSensor sensorRDistance = null;
  private DistanceSensor sensorLDistance = null;
  private DistanceSensor sensorDistance = null;
  private ColorSensor sensorRBrightness = null;
  private ColorSensor sensorLBrightness = null;
  private ColorSensor sensorStoneBrightness = null;
  private ColorSensor sensorAmbientBrightness = null;
  
  String task =  "go to stones far";

  BNO055IMU imu;

  Orientation angles;
  Acceleration gravity;

  String[] motorNames = {
          "driveNW",
          "driveNE",
          "driveSE",
          "driveSW"
  };
  private DcMotor[] motors = {null,null,null,null};
  double[] targetPowers = {0,0,0,0};
  double[] adjustedPowers = {0,0,0,0};
  double[] normalizedPowers = {0,0,0,0};

  double distance       = 0 ;
  double stoneBrightness = 0 ;
  double ambientBrightness = 0;

  double targetOrientation = 0;

  double orientation       = 0 ;

  private void driveX ( double power ) {

    // drive away from drivers

    if ( isRed ) {

      power = -power ;

    }

    targetPowers[0] = -power;
    targetPowers[1] = -power;
    targetPowers[2] = power ;
    targetPowers[3] = power ;

  }


  private void driveY ( double power ) {

    // drive toward stones

    targetPowers[0] = -power;
    targetPowers[1] = power;
    targetPowers[2] = power ;
    targetPowers[3] = -power ;

  }


  private void driveSpn ( double power ) {

    // spin clockwise red , cc blue

    if( isRed ) {

      power = -power ;

    }

    for ( int i = 0 ; i < 4 ; i++ ){
      
      targetPowers[i] = power;
      
    }

  }
  
  private double maxOf ( double[] array ) {
    
    double maxValue = array[0];
    
    for ( int i = 0 ; i < array.length ; i++ ){
      
      if( array[i] > maxValue ){
        
        maxValue = array[i];
        
      }
      
    }
    
    return maxValue;
    
  }


  private void sliderSpn ( double power ) {

    slider.setPower  (  power ) ;

  }

  private void updateMotors () {

    double orientationAdjustment = ( targetOrientation - orientation ) / 30;

    for ( int i = 0 ; i < 4 ; i++ ){
      
      adjustedPowers[i] = targetPowers[i] + orientationAdjustment;
      
    }

    double max = Math.max(maxOf(adjustedPowers),1);

    for ( int i = 0 ; i < 4 ; i++ ){

      normalizedPowers[i] = targetPowers[i]/max;

    }

    for ( int i = 0 ; i < 4 ; i++ ){

      motors[i].setPower(normalizedPowers[i]);
      
    }
    
  }

  private void grab ( boolean shouldGrab ) {

    if ( shouldGrab ) {

      grabber.setPosition ( 0 ) ;

    } else {

      grabber.setPosition ( 1 ) ;

    }

  }

  private void updateSensors () {

    stoneBrightness  = sensorStoneBrightness.red() + sensorStoneBrightness.green() + sensorStoneBrightness.blue();
    ambientBrightness  = sensorAmbientBrightness.red() + sensorAmbientBrightness.green() + sensorAmbientBrightness.blue();
    distance    = sensorDistance.getDistance(DistanceUnit.INCH);
    orientation = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    telemetry.addData(
      "team",
      isRed ? "red" : "blue"
    );
    telemetry.addData(
      "current task",
      task
    );
    telemetry.addData(
      "distance",
      distance
    );
    telemetry.addData(
      "stone brightness",
      stoneBrightness
    );
    telemetry.addData(
      "relative brightness",
      stoneBrightness/ambientBrightness
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

  private void retime () {

    runtime.reset();

  }

  private void activeSleep (double ms) {

    retime();

    while (runtime.seconds() < ms/1000) {

      updateSensors();
      updateMotors();

    }

  }

  private void pause () {

    driveX(0);
    sleep(150);
    retime();

  }

  private void initialize () {

    for ( int i = 0 ; i < 4 ; i++ ){

      motors[i] = hardwareMap.get ( DcMotor.class , motorNames[i] ) ;

    }

    slider     = hardwareMap.get ( DcMotor.class , "slider"  ) ;
    grabber    = hardwareMap.get ( Servo.class   , "grabber" ) ;
    dragger    = hardwareMap.get ( Servo.class   , "dragger" ) ;

    sensorLDistance = hardwareMap.get ( DistanceSensor.class ,"sensorL" );
    sensorRDistance = hardwareMap.get ( DistanceSensor.class ,"sensorR" );
    sensorLBrightness = hardwareMap.get ( ColorSensor.class ,"sensorL" );
    sensorRBrightness = hardwareMap.get ( ColorSensor.class ,"sensorR" );

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled       = true;
    parameters.loggingTag           = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    isRed = sensorRDistance.getDistance(DistanceUnit.INCH) < sensorLDistance.getDistance(DistanceUnit.INCH);

    sensorDistance = isRed ? sensorLDistance : sensorRDistance;
    sensorAmbientBrightness = isRed ? sensorRBrightness : sensorLBrightness;
    sensorStoneBrightness = isRed ? sensorLBrightness : sensorRBrightness;

  }

  private void run () {



    task = "go to stones far";

    driveX ( 1 );

    activeSleep(700);

    grab(false);



    task = "go to stones near";

    while ( distance < 2.5 ) {

      driveX ( 0.3 );

    }



    task = "align with wall";

    driveY (-0.5);

    activeSleep(1000);



    task = "align with 3rd stone";

    driveY ( 0.5 );

    activeSleep(1000);

    task = "find skystone";



    retime();

    while ( stoneBrightness > (3 * ambientBrightness) && runtime.seconds() < 5) {

      driveY ( 0.3 );

    }

    pause();


    task = "go adjacent to stone behind skystone";

    driveY ( -0.5 );

    activeSleep(1000);



    task = "push out stone behind skystone";

    driveX ( 1 );

    activeSleep(1000);



    task = "go towards skystone";

    driveY ( 0.5 );

    activeSleep(2000);

    grab(true);



    task = "grab skystone" ;

    activeSleep(2000);

    task = "go to outer ";

  }

  String formatAngle(AngleUnit angleUnit, double angle) {
    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
  }

  String formatDegrees(double degrees){
    return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
  }


  @Override
  public void runOpMode () {

    initialize();

    waitForStart();

    run();

  }
}