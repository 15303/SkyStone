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


@com.qualcomm.robotcore.eventloop.opmode.Autonomous (

        name  = "OmniFoundR"         ,
        group = "2"

)

public class OmniFoundR extends LinearOpMode {


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

  String task =  "initialize";

  BNO055IMU imu;

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

    double maxValue = 1;

    for ( int i = 0 ; i < array.length ; i++ ){

      double newValue = Math.abs(array[i]);

      if( newValue > maxValue ){

        maxValue = newValue;

      }

    }

    return maxValue;

  }


  private void lift ( double power ) {

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

    double max = maxOf(adjustedPowers);

    for ( int i = 0 ; i < 4 ; i++ ){

      normalizedPowers[i] = adjustedPowers[i]/max;

    }

    for ( int i = 0 ; i < 4 ; i++ ){

      motors[i].setPower(normalizedPowers[i]);

    }

  }

  private void drag ( boolean shouldDrag ) {

    if ( shouldDrag ) {

      dragger.setPosition ( 1 ) ;

    } else {

      dragger.setPosition ( 0 ) ;

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

    sensorDistance = isRed ? sensorLDistance : sensorRDistance;
    sensorColor = isRed ? sensorLColor : sensorRColor;

  }

  private boolean robotIsNotGoingToDestroyUsAll () {

    return ( opModeIsActive() && runtime.seconds() < 8 );

  }

  private void run () {

    drag(false);

    setTask("go to building zone");

    drive( "X" , 1 , 1000);

    setTask("go to foundation");

    drive ( "Y" , -1 , 1200);

    drag(true);

    drive("Y",0,1000);

    setTask("drag foundation to building zone");

    drive("Y",1,2000);

    setTask("turn");

    drive("X", -1, 400);

    targetOrientation = 90;

    drive("Spin",1,2500);

    drag(false);

    setTask("drive under bridge");

    drive("Y",1,2000);

    setTask("end");

  }

  @Override
  public void runOpMode () {

    initialize();

    waitForStart();

    run();

  }
}