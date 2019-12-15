package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp (

  name  = "OmniX v2"         ,
  group = "Linear Opmode"

)

//@Disabled




public class OmniX_v2 extends LinearOpMode {


  private DcMotor driveNW = null;
  private DcMotor driveNE = null;
  private DcMotor driveSE = null;
  private DcMotor driveSW = null;

  private DcMotor slider  = null;
  private Servo   grabber = null;
  private Servo   dragger = null ;

  BNO055IMU imu;

  // State used for updating telemetry
  Orientation angles;
  Acceleration gravity;

  double stickXL = 0 ;
  double stickXR = 0 ;
  double stickYL = 0 ;
  double stickYR = 0 ;
  double trigL   = 0 ;
  double trigR   = 0 ;

  // minimum throttle for motors to have sufficient grip

  double driveRht  = 0 ;
  double driveFwd  = 0 ;
  double driveC    = 0 ;
  double normalize = 1 ;




  double sliderPower = 0   ;
  double grabberPos  = 0.7 ;






















  @Override
  public void runOpMode() {

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
    parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
    parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
    parameters.loggingEnabled      = true;
    parameters.loggingTag          = "IMU";
    parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);



    driveNW = hardwareMap.get ( DcMotor.class , "driveNW" ) ;
    driveNE = hardwareMap.get ( DcMotor.class , "driveNE" ) ;
    driveSE = hardwareMap.get ( DcMotor.class , "driveSE" ) ;
    driveSW = hardwareMap.get ( DcMotor.class , "driveSW" ) ;

    slider  = hardwareMap.get ( DcMotor.class , "slider"  ) ;
    grabber = hardwareMap.get ( Servo.class   , "grabber" ) ;
    dragger = hardwareMap.get ( Servo.class   , "dragger" ) ;


    telemetry.addData ( "Status    " , "OmniX Initialized" ) ;
    telemetry.update  (                                    ) ;

    composeTelemetry();

    waitForStart      (                                    ) ;
    telemetry.addData ( "Status     " , "OmniX Active"     ) ;

    imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

    while ( opModeIsActive () ) {

      //driving define

      stickXL = gamepad1.left_stick_x  + gamepad2.left_stick_x  ;
      stickXR = gamepad1.right_stick_x + gamepad2.right_stick_x ;

      stickYL = gamepad1.left_stick_y  + gamepad2.left_stick_y  ;
      stickYR = gamepad1.right_stick_y + gamepad2.right_stick_y ;

      trigL   = gamepad1.left_trigger  + gamepad2.left_trigger  ;
      trigR   = gamepad1.right_trigger + gamepad2.right_trigger ;



      driveFwd =   ( Math.pow ( stickYL , 3 ) + Math.pow ( stickYR , 3 ) ) / 2 ;

      driveRht = - ( Math.pow ( stickXL , 3 ) + Math.pow ( stickXR , 3 ) ) / 2 ;

      driveC   =  Math.pow ( trigL , 3 ) - Math.pow ( trigR , 3 ) ;


      normalize = Math.max ( Math.abs ( driveRht ) + Math.abs ( driveFwd ) + Math.abs ( driveC ) , 1 ) ;


      //driving do

      driveNW.setPower ( (   driveRht*0.84 + driveFwd + driveC ) / normalize ) ;
      driveNE.setPower ( (   driveRht - driveFwd + driveC ) / normalize ) ;
      driveSE.setPower ( ( - driveRht - driveFwd + driveC ) / normalize ) ;
      driveSW.setPower ( ( - driveRht + driveFwd + driveC ) / normalize ) ;



      // arm define

      sliderPower = ( gamepad1.dpad_left  || gamepad2.dpad_left  ) ?  1
                  : ( gamepad1.dpad_right || gamepad2.dpad_right ) ? -1
                  :                                                   0          ;


      grabberPos  = ( gamepad1.dpad_up    || gamepad2.dpad_up    ) ?  0
                  : ( gamepad1.dpad_down  || gamepad2.dpad_down  ) ?  1
                  :                                                   grabberPos ;



      // arm do

      slider.setPower     ( sliderPower ) ;
      grabber.setPosition ( grabberPos  ) ;



      // telemetry

//
      telemetry.addData ( "gravity", imu.getGravity());
      angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      telemetry.update  (                                    ) ;


    }
  }
  void composeTelemetry() {

    // At the beginning of each telemetry update, grab a bunch of data
    // from the IMU that we will then display in separate lines.
    telemetry.addAction(new Runnable() { @Override public void run()
    {
      // Acquiring the angles is relatively expensive; we don't want
      // to do that in each of the three items that need that info, as that's
      // three times the necessary expense.
      angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      gravity  = imu.getGravity();
    }
    });

    telemetry.addLine()
            .addData("status", new Func<String>() {
              @Override public String value() {
                return imu.getSystemStatus().toShortString();
              }
            })
            .addData("calib", new Func<String>() {
              @Override public String value() {
                return imu.getCalibrationStatus().toString();
              }
            });

    telemetry.addLine()
            .addData("heading", new Func<String>() {
              @Override public String value() {
                return formatAngle(angles.angleUnit, angles.firstAngle);
              }
            })
            .addData("roll", new Func<String>() {
              @Override public String value() {
                return formatAngle(angles.angleUnit, angles.secondAngle);
              }
            })
            .addData("pitch", new Func<String>() {
              @Override public String value() {
                return formatAngle(angles.angleUnit, angles.thirdAngle);
              }
            });

    telemetry.addLine()
            .addData("grvty", new Func<String>() {
              @Override public String value() {
                return gravity.toString();
              }
            })
            .addData("mag", new Func<String>() {
              @Override public String value() {
                return String.format(Locale.getDefault(), "%.3f",
                        Math.sqrt(gravity.xAccel*gravity.xAccel
                                + gravity.yAccel*gravity.yAccel
                                + gravity.zAccel*gravity.zAccel));
              }
            });
  }

  //----------------------------------------------------------------------------------------------
  // Formatting
  //----------------------------------------------------------------------------------------------

  String formatAngle(AngleUnit angleUnit, double angle) {
    return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
  }

  String formatDegrees(double degrees){
    return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
  }

}