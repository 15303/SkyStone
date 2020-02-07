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

  name  = 'OmniR0'         ,
  group = 'r2'

)

// @Disabled


public class OmniR0 extends LinearOpMode {


  boolean isRed = true ; // IMPORTANT


  private ElapsedTime runtime = new ElapsedTime() ;

  private DcMotor driveNW = null ;
  private DcMotor driveNE = null ;
  private DcMotor driveSE = null ;
  private DcMotor driveSW = null ;
  private DcMotor slider  = null ;
  
  private Servo   grabber = null ;
  private Servo   dragger = null ;

  private ColorSensor     sensorColor    = null ;
  private DistanceSensor  sensorDistance = null ;
  
  String task = 'go to stones';

  BNO055IMU imu;

  Orientation angles;
  Acceleration gravity;

  double distance       = 100 ;
  double brightness     = 800 ;
  double time           =   0 ;

  int orientation       = 0 ;
  int deltaOrientation  = 0 ;
  int targetOrientation = 0 ;

  boolean shouldGrab = true ;
  boolean shouldDrag = false ;


  private void driveX ( double power ) {

    // drive away from drivers

    if ( isRed ) {

      power = -power ;

    }

    driveNW.setPower (  power ) ;
    driveNE.setPower (  power ) ;
    driveSE.setPower ( -power ) ;
    driveSW.setPower ( -power ) ;

  }


  private void driveY ( double power ) {

    // drive toward stones

    driveNW.setPower ( -power ) ;
    driveNE.setPower (  power ) ;
    driveSE.setPower (  power ) ;
    driveSW.setPower ( -power ) ;

  }


  private void driveSpn ( double power ) {

    // spin clockwise red , cc blue

    if( isRed ) {

      power = -power ;

    }

    driveNW.setPower (  power ) ;
    driveNE.setPower (  power ) ;
    driveSE.setPower (  power ) ;
    driveSW.setPower (  power ) ;

  }


  private void sliderSpn ( double power ) {

    slider.setPower  (  power ) ;

  }


  private void updateServos () {

    if ( shouldGrab ) {

      grabber.setPosition ( 1 ) ;

    } else {

      grabber.setPosition ( 0   ) ;

    }

    if ( shouldDrag ) {

      dragger.setPosition ( 1 ) ;

    } else {

      dragger.setPosition ( 0   ) ;

    }

  }
  
  private void updateSensors () {
    
    brightness  = sensorColor.red() + sensorColor.green() + sensorColor.blue();
    distance    = sensorDistance.getDistance(DistanceUnit.INCH);
    orientation = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    time        = runtime.seconds()
    
    telemetry.addData( 'distance',    distance    );
    telemetry.addData( 'brightness',  brightness  );
    telemetry.addData( 'orientation', orientation );
    telemetry.addData( 'current task',task       );
    telemetry.update();
    
  }
  
  private void resetTime () {
    
    runtime.reset();
    
  }
  
  private void correctRotation () {
    
    deltaOrientation = targetOrientation - orientation;

    if ( Math.abs( deltaOrientation ) > 2 )
  
      driveSpn ( Math.signum ( -deltaOrientation ) * 0.1 );

    }
    
  }

  private void run () {
    
    updateSensors();
    updateServos();
    correctOrientation();
    
    if ( task == 'go to stones' ) {
      
      driveX ( 0.5 )
      
      if ( distance < 5 ) {
        
        task = 'find skystone';
        
      }
      
    }
    
    if ( task == 'find skystone' ) {
      
      driveY ( 0.5 )
      
      if ( brightness < 600 ) {
        
        resetTime();
        
        task = 'go adjacent to stone behind skystone';
        
      }
      
    }
    
    if ( task == 'go adjacent to skystone' ) {
      
      driveY ( -0.5 )
      
      if ( time > 1 ) {
        
        resetTime();
        
        task = 'push out stone behind skystone';
        
        shouldGrab = false;
        
      }
      
    }
    
    if ( task == 'push out stone behind skystone' ) {
      
      driveX ( 0.5 )
      
      if ( time > 1 ) {
        
        resetTime();
        
        task = 'go towards skystone';
        
      }
      
    }
    
    if ( task == 'go towards skystone' ) {
      
      driveY ( 0.5 )
      
      if ( time > 2 ) {
        
        resetTime();
        
        task = 'grab skystone';
        
        shouldGrab = true;
        
      }
      
    }
    
    if ( task == 'grab skystone' ) {

      if ( time > 2 ) {
        
        resetTime();
        
        task = 'go to outer ';
        
      }
      
    }


  }

  String formatAngle(AngleUnit angleUnit, double angle) {
    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
  }

  String formatDegrees(double degrees){
    return String.format(Locale.getDefault(), '%.1f', AngleUnit.DEGREES.normalize(degrees));
  }
  
  public void initialize () {
    
    driveNW    = hardwareMap.get ( DcMotor.class , 'driveNW' ) ;
    driveNE    = hardwareMap.get ( DcMotor.class , 'driveNE' ) ;
    driveSE    = hardwareMap.get ( DcMotor.class , 'driveSE' ) ;
    driveSW    = hardwareMap.get ( DcMotor.class , 'driveSW' ) ;

    slider     = hardwareMap.get ( DcMotor.class , 'slider'  ) ;
    grabber    = hardwareMap.get ( Servo.class   , 'grabber' ) ;
    dragger    = hardwareMap.get ( Servo.class   , 'dragger' ) ;

    sensorColor    = isRed ? hardwareMap.get ( ColorSensor.class    , 'sensorE' ) : ( ColorSensor.class    , 'sensorW' ) ;
    sensorDistance = isRed ? hardwareMap.get ( DistanceSensor.class , 'sensorE' ) : ( DistanceSensor.class , 'sensorW' ) ;


    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile  = 'BNO055IMUCalibration.json'; // see the calibration sample opmode
    parameters.loggingEnabled       = true;
    parameters.loggingTag           = 'IMU';
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu = hardwareMap.get(BNO055IMU.class, 'imu');
    imu.initialize(parameters);
    
  }



  @Override
  public void runOpMode () {
    
    initialize();

    waitForStart();
    runtime.reset();

    while ( opModeIsActive () ) {
     
      run();
    
    }

  }
}