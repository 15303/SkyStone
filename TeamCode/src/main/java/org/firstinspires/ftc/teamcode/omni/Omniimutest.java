package org.firstinspires.ftc.teamcode.omni;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous (

  name  = "Omnicmgy.oyimutestP"         ,
  group = "2"

)

 @Disabled



public class Omniimutest extends LinearOpMode {


  boolean isRed = true ; // IMPORTANT


  private ElapsedTime runtime = new ElapsedTime() ;

  private DcMotor         driveNW = null ;
  private DcMotor         driveNE = null ;
  private DcMotor         driveSE = null ;
  private DcMotor         driveSW = null ;

  BNO055IMU imu;

  // State used for updating telemetry
  Orientation angles;
  Acceleration gravity;
  Position position;


  private void driveY ( double power ) {

    // drive toward stones

    driveNW.setPower ( -power ) ;
    driveNE.setPower (  power ) ;
    driveSE.setPower (  power ) ;
    driveSW.setPower ( -power ) ;

  }

  public void runOpMode(){
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.loggingEnabled      = true;
    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    driveNW    = hardwareMap.get ( DcMotor.class ,       "driveNW" ) ;
    driveNE    = hardwareMap.get ( DcMotor.class ,       "driveNE" ) ;
    driveSE    = hardwareMap.get ( DcMotor.class ,       "driveSE" ) ;
    driveSW    = hardwareMap.get ( DcMotor.class ,       "driveSW" ) ;

    waitForStart();

    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    while (opModeIsActive()) {
      telemetry.addData("gravity", imu.getGravity());
      telemetry.update();
    }

    driveY(1);
    sleep(1000);
    driveY(0);

  }
}