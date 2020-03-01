package org.firstinspires.ftc.teamcode.omni;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


@Autonomous (

  name  = "OmniSenseTesting"         ,
  group = "2"

)

@Disabled

public class OmniSenseTesting extends LinearOpMode {


  boolean isRed = true;
  boolean isSkystone = false;
  
  private ElapsedTime runtime = new ElapsedTime() ;

  private Servo   grabber = null ;
  private Servo   dragger = null ;
  private DcMotor slider = null;

  private DistanceSensor sensorRDistance = null;
  private DistanceSensor sensorLDistance = null;
  private DistanceSensor sensorDistance = null;
  private ColorSensor sensorRColor = null;
  private ColorSensor sensorLColor = null;
  private ColorSensor sensorColor = null;

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
    
    double maxValue = Math.abs(array[0]);
    
    for ( int i = 1 ; i < array.length ; i++ ){

      double newValue = Math.abs(array[i]);
      
      if( newValue > maxValue ){
        
        maxValue = newValue;
        
      }
      
    }
    
    return maxValue;
    
  }


  private void sliderSpn ( double power ) {

    slider.setPower  (  power ) ;

  }

  private void updateMotors () {

    double orientationAdjustment = ( targetOrientation - orientation ) / 60;

    for ( int i = 0 ; i < 4 ; i++ ){
      
      adjustedPowers[i] = targetPowers[i] + orientationAdjustment;
      
    }

    double max = Math.max(maxOf(adjustedPowers),1);

    for ( int i = 0 ; i < 4 ; i++ ){

      normalizedPowers[i] = adjustedPowers[i]/max;

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

    isSkystone  = sensorColor.red() + sensorColor.green() < 2 * sensorColor.blue();
    distance    = sensorDistance.getDistance(DistanceUnit.INCH);
    orientation = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    telemetry.addData("red+green", sensorColor.red() + sensorColor.blue()

    );
    telemetry.addData("blue * 2", 2 * sensorColor.blue());
    telemetry.update();

  }

  private void retime () {

    runtime.reset();

  }

  private void activeSleep (double ms) {

    retime();

    while (opModeIsActive() && runtime.seconds() < ms/1000) {

      updateSensors();
      updateMotors();

    }

  }

  private void pause () {

    driveX(0);
    sleep(1000);
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
    sensorLColor = hardwareMap.get ( ColorSensor.class ,"sensorL" );
    sensorRColor = hardwareMap.get ( ColorSensor.class ,"sensorR" );

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
    sensorColor = isRed ? sensorLColor : sensorRColor;

  }

  private void run () {



    activeSleep(30000);

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