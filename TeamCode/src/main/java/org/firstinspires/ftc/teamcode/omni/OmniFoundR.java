package org.firstinspires.ftc.teamcode.omni;

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

        name  = "OmniFoundR"         ,
        group = "2"

)

// @Disabled


public class OmniFoundR extends LinearOpMode {


  boolean isRed = true;
  boolean isSkystone = false;

  private ElapsedTime runtime = new ElapsedTime() ;

  private Servo   dragger = null ;
  private DcMotor slider = null;

  String task =  "initialize";

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

  private void drive ( String direction , double power ) {

    if ( direction == "X" ) {

      // drive away from drivers

      if ( isRed ) {

        power = -power ;

      }

      setTargetPowers(-power,-power,power,power);

    } else if ( direction == "Y" ) {

      // drive toward stones

      setTargetPowers(-power,power,power,-power);

    } else if ( direction == "Spin" ) {

      // spin clockwise red , cc blue

      if( isRed ) {

        power = -power ;

      }

      setTargetPowers(power,power,power,power);

    }

  }

  private void drive ( String direction , double power , double ms ) {

    drive ( direction , power );

    activeSleep(ms);

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

  private void setTargetPowers ( double NW , double NE , double SE , double SW ) {

    targetPowers[0] = NW;
    targetPowers[1] = NE;
    targetPowers[2] = SE;
    targetPowers[3] = SW;

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

  private void drag ( boolean shouldDrag ) {

    if ( shouldDrag ) {

      dragger.setPosition ( 0 ) ;

    } else {

      dragger.setPosition ( 1 ) ;

    }

  }

  private void updateSensors () {

    isSkystone  = sensorColor.red() + sensorColor.green() < 4 * sensorColor.blue();
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
            "yellow",
            sensorColor.red() + sensorColor.green()
    );
    telemetry.addData(
            "blue",
            sensorColor.blue()*4
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

  private void activeSleep (double ms) {

    while (runtime.seconds() < ms / 1000 && robotIsNotGoingToDestroyUsAll()) {

      updateSensors();
      updateMotors();

    }

  }

  private void setTask ( String newTask ) {

    task = newTask;
    for ( int i = 0 ; i < 4 ; i++ ) {
      motors[i].setPower(0);
    }
    sleep(1000);
    runtime.reset();

  }

  private void initialize () {

    for ( int i = 0 ; i < 4 ; i++ ){

      motors[i] = hardwareMap.get ( DcMotor.class , motorNames[i] ) ;

    }

    slider     = hardwareMap.get ( DcMotor.class , "slider"  ) ;
    grabber    = hardwareMap.get ( Servo.class   , "grabber" ) ;
    dragger    = hardwareMap.get ( Servo.class   , "dragger" ) ;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled       = true;
    parameters.loggingTag           = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

  }

  private boolean robotIsNotGoingToDestroyUsAll () {

    return ( opModeIsActive() && runtime.seconds() < 8 );

  }

  private void run () {

    setTask("go to stones far");

    drive ( "X" , 1 , 1000);

    grab(false);

    setTask("go to stones near");

    while ( distance > 2.5 && robotIsNotGoingToDestroyUsAll() ) {

      drive ( "X" , 0.3 );
      updateSensors();
      updateMotors();

    }

    setTask("find skystone");

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