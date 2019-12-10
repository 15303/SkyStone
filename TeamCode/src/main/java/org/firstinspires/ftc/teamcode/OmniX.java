package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (

  name  = "OmniX"         ,
  group = "Linear Opmode"

)

//@Disabled




























public class OmniX extends LinearOpMode {



















  private DcMotor driveNW = null;
  private DcMotor driveNE = null;
  private DcMotor driveSE = null;
  private DcMotor driveSW = null;

  private DcMotor slider  = null;
  private Servo   grabber = null;




  double[] stickX  = { 0 , 0 , 0 , 0 } ;
  double[] stickY  = { 0 , 0 , 0 , 0 } ;
  double[] trigL   = { 0 , 0 } ;
  double[] trigR   = { 0 , 0 } ;

  final int MIN_THROTTLE = 16;

  double throttleCurve ( double[] array , degree ) {

    double sum   = 0;

    for ( int i = 0; i < array.length; i++ ) {

      sum += array[i];

    }

    if ( avg > 1 / MIN_THROTTLE ) {

      return 0;

    }

    double avg   = 2 * sum / array.length;

    double sign  = Math.signum ( avg     ) ;

    double power = Math.pow    ( avg , 5 ) ;

    return ( power + ( sign - power ) / MIN_THROTTLE ) ;

  }




  double driveRht  = 0;
  double driveFwd  = 0;
  double driveC    = 0;
  double normalize = 1;




  double sliderPower  = 0;
  double grabberPos   = 0.7;






















  @Override
  public void runOpMode() {



    telemetry.addData ( "Status    " , "OmniX Initialized" ) ;
    telemetry.update  (                                    ) ;



    driveNW = hardwareMap.get ( DcMotor.class , "driveNW" ) ;
    driveNE = hardwareMap.get ( DcMotor.class , "driveNE" ) ;
    driveSE = hardwareMap.get ( DcMotor.class , "driveSE" ) ;
    driveSW = hardwareMap.get ( DcMotor.class , "driveSW" ) ;

    slider  = hardwareMap.get ( DcMotor.class , "slider"  ) ;
    grabber = hardwareMap.get ( Servo.class   , "grabber" ) ;



    waitForStart      (                                    ) ;
    telemetry.addData ( "Status     " , "OmniX Active"     ) ;









    while ( opModeIsActive () ) {



















      //driving define

      stickX  = new double[] { gamepad1.left_stick_x , gamepad2.left_stick_x , gamepad1.right_stick_x , gamepad2.right_stick_x } ;
      stickY  = new double[] { gamepad1.left_stick_y , gamepad2.left_stick_y , gamepad1.right_stick_y , gamepad2.right_stick_y } ;
      trigL   = new double[] { gamepad1.left_trigger , gamepad2.left_trigger                                                   } ;
      trigR   = new double[] { gamepad1.right_trigger, gamepad2.right_trigger                                                  } ;



      driveFwd =   throttleCurve( stickY ) ;

      driveRht = ( gamepad1.left_stick_button  || gamepad2.left_stick_button  ) ? (  1/MIN_THROTTLE                                      )
               : ( gamepad1.right_stick_button || gamepad2.right_stick_button ) ? ( -1/MIN_THROTTLE                                      )
               :                                                                  ( -throttleCurve( stickX )                             ) ;

      driveC   = ( gamepad1.left_bumper        || gamepad2.left_bumper        ) ? (  1/MIN_THROTTLE                                      )
               : ( gamepad1.right_bumper       || gamepad2.right_bumper       ) ? ( -1/MIN_THROTTLE                                      )
               :                                                                  (  throttleCurve ( trigL  ) - throttleCurve ( trigR  ) ) ;


      normalize = Math.max ( Math.abs ( driveRht ) + Math.abs ( driveFwd ) + Math.abs ( driveC ) , 1 ) ;






      //driving do

      driveNW.setPower ( (   driveRht + driveFwd + driveC ) / normalize ) ;
      driveNE.setPower ( (   driveRht - driveFwd + driveC ) / normalize ) ;
      driveSE.setPower ( ( - driveRht - driveFwd + driveC ) / normalize ) ;
      driveSW.setPower ( ( - driveRht + driveFwd + driveC ) / normalize ) ;














      // arm define

      sliderPower = ( gamepad1.dpad_left  || gamepad2.dpad_left  ) ?  1
                  : ( gamepad1.dpad_right || gamepad2.dpad_right ) ? -1
                  :                                                   0          ;


      grabberPos  = ( gamepad1.dpad_up     || gamepad2.dpad_up   ) ?  0
                  : ( gamepad1.dpad_down   || gamepad2.dpad_down ) ?  0.7
                  :                                                   grabberPos ;






      // arm do

      slider.setPower     ( sliderPower ) ;
      grabber.setPosition ( grabberPos  ) ;












      // telemetry

      telemetry.addData ( "Status     " , "OmniX Running"    ) ;
      telemetry.addData ( "DriveRht   " ,  driveRht          ) ;
      telemetry.addData ( "DriveFwd   " ,  driveFwd          ) ;
      telemetry.addData ( "DriveC     " ,  driveC            ) ;
      telemetry.addData ( "SliderPower" ,  sliderPower       ) ;
      telemetry.addData ( "GrabberPos " ,  grabberPos        ) ;
      telemetry.update  (                                    ) ;













    }





  }



}