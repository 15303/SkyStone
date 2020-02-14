package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

  name  = "OmniFound"         ,
  group = "2"

)

// @Disabled


public class OmniFound extends LinearOpMode {


  boolean isRed;
  
  private ElapsedTime runtime = new ElapsedTime() ;

  private DcMotor driveNW = null ;
  private DcMotor driveNE = null ;
  private DcMotor driveSE = null ;
  private DcMotor driveSW = null ;
  private DcMotor slider  = null ;
  
  private Servo   grabber = null ;
  private Servo   dragger = null ;

  private DistanceSensor sensorRDistance = null;
  private DistanceSensor sensorLDistance = null;
  private DistanceSensor sensorDistance = null;
  private ColorSensor sensorRBrightness = null;
  private ColorSensor sensorLBrightness = null;
  private ColorSensor sensorStoneBrightness = null;
  private ColorSensor sensorAmbientBrightness = null;
  
  String task =  "go to foundation";

  BNO055IMU imu;

  Orientation angles;
  Acceleration gravity;

  double distance       = 0 ;
  double stoneBrightness = 0 ;
  double ambientBrightness = 0;

  double targetNW = 0;
  double targetNE = 0;
  double targetSE = 0;
  double targetSW = 0;
  double targetOrientation = 0;

  double orientation       = 0 ;

  boolean shouldGrab = true ;
  boolean shouldDrag = false ;


  private void driveX ( double power ) {

    // drive away from drivers

    if ( isRed ) {

      power = -power ;

    }

    targetNW = -power;
    targetNE = -power;
    targetSE = power ;
    targetSW = power ;

  }


  private void driveY ( double power ) {

    // drive toward foundation

    targetNW = -power;
    targetNE = power;
    targetSE = power ;
    targetSW = -power ;

  }


  private void driveSpn ( double power ) {

    // spin clockwise red , cc blue

    if( isRed ) {

      power = -power ;

    }

    targetNW = power;
    targetNE = power;
    targetSE = power ;
    targetSW = power ;

  }


  private void sliderSpn ( double power ) {

    slider.setPower  (  power ) ;

  }

  private void updateMotors () {

    double orientationAdjustment = ( targetOrientation - orientation ) / 30;

    targetNW += orientationAdjustment;
    targetNE += orientationAdjustment;
    targetSE += orientationAdjustment;
    targetSW += orientationAdjustment;

    double normalize = Math.max(Math.max(Math.max(Math.abs(targetNW),Math.abs(targetNE)),Math.max(Math.abs(targetSE),Math.abs(targetSW))),1);

    driveNW.setPower(targetNW/normalize);
    driveNE.setPower(targetNE/normalize);
    driveSE.setPower(targetSE/normalize);
    driveSW.setPower(targetSW/normalize);

  }


  private void updateServos () {

    if ( shouldGrab ) {

      grabber.setPosition ( 0 ) ;

    } else {

      grabber.setPosition ( 1 ) ;

    }

    if ( shouldDrag ) {

      dragger.setPosition ( 1 ) ;

    } else {

      dragger.setPosition ( 0   ) ;

    }

  }
  
  private void updateSensors () {

    stoneBrightness  = sensorStoneBrightness.red() + sensorStoneBrightness.green() + sensorStoneBrightness.blue();
    ambientBrightness  = sensorAmbientBrightness.red() + sensorAmbientBrightness.green() + sensorAmbientBrightness.blue();
    distance    = sensorDistance.getDistance(DistanceUnit.INCH);
    orientation = (int) imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    telemetry.addData( "team",isRed ? "red" : "blue"       );
    telemetry.addData( "current task",task       );
    telemetry.addData( "distance",    distance    );
    telemetry.addData( "stone brightness",  stoneBrightness  );
    telemetry.addData( "relative brightness",  stoneBrightness/ambientBrightness  );
    telemetry.addData( "orientation", orientation );
    telemetry.addData( "time",runtime.seconds());
    telemetry.update();

  }

  private void reset () {

    driveX(0);
    runtime.reset();

  }

  private void initialize () {

    driveNW    = hardwareMap.get ( DcMotor.class , "driveNW" ) ;
    driveNE    = hardwareMap.get ( DcMotor.class , "driveNE" ) ;
    driveSE    = hardwareMap.get ( DcMotor.class , "driveSE" ) ;
    driveSW    = hardwareMap.get ( DcMotor.class , "driveSW" ) ;

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

    if ( task == "go to stones far" ) {

      driveX ( 1 );

      if ( runtime.seconds() > 0.7 ) {

        reset();

        shouldGrab = false;

        task = "go to stones near";

      }

    } else if ( task == "go to stones near" ) {

      driveX ( 0.3 );

      if ( distance < 2.5 ) {

        reset();

        task = "align with wall";

      }

    } else if ( task == "align with wall") {

      driveY (-0.5);

      if(runtime.seconds() > 1){

        reset();

        task = "align with 3rd stone";

      }

    }else if ( task == "align with 3rd stone"){

      driveY ( 0.5 );

      if(runtime.seconds() > 1){

        reset();

        task = "find skystone";

      }

    } else if ( task == "find skystone" ) {

      driveY ( 0.3 );

      if ( stoneBrightness < 3 * ambientBrightness ) {

        reset();

        task = "go adjacent to stone behind skystone";

      }

    } else if ( task == "go adjacent to stone behind skystone" ) {

      driveY ( -0.5 );

      if ( runtime.seconds() > 1 ) {

        reset();

        task = "push out stone behind skystone";

      }

    } else if ( task == "push out stone behind skystone" ) {

      driveX ( 1 );

      if ( runtime.seconds() > 1 ) {

        reset();

        task = "go towards skystone";

      }

    } else if ( task == "go towards skystone" ) {

      driveY ( 0.5 );

      if ( runtime.seconds() > 2 ) {

        reset();

        task = "grab skystone";

        shouldGrab = true;

      }

    } else if ( task == "grab skystone" ) {

      if ( runtime.seconds() > 2 ) {

        reset();

        task = "go to outer ";

      }

    }
    
    updateSensors();
    updateServos();
    updateMotors();

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

    reset();

    while ( opModeIsActive () ) {
     
      run();
    
    }

  }
}