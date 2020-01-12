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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous (

  name  = "OmniR1"         ,
  group = "r2"

)

// @Disabled






















public class OmniR1 extends LinearOpMode {









  boolean isRed = true ; // IMPORTANT





  private ElapsedTime runtime = new ElapsedTime() ;

  private DcMotor         driveNW = null ;
  private DcMotor         driveNE = null ;
  private DcMotor         driveSE = null ;
  private DcMotor         driveSW = null ;

  private DcMotor         slider  = null ;
  private Servo           grabber = null ;

  private ColorSensor     senseColor   = null ;

  private DistanceSensor  senseDistN   = null ;
  private DistanceSensor  senseDistX   = null ;
  private DistanceSensor  senseDistS   = null ;





  double distN =  30 ;
  double distX =   0 ;
  double distS = 100 ;
  double lumin = 100 ;
  double time  =   0 ;




  // power

  final double CAUTIOUS_POWER         =    0.5 ;
  final double NORMAL_POWER           =    0.7 ;
  final double FULL_POWER             =    1   ;



  // inches

  final int DIST_X_WALL               =    5 ;
  final int DIST_Y_WALL               =    2 ;

  final int DIST_Y_SKYBRIDGE          =   60 ;

  final int DIST_X_INTRACK_OUTER      =   25 ;

  final int DIST_X_DEPOT_CENTER       =   45 ;
  final int DIST_X_DEPOT_OUTER        =  -10 + DIST_X_DEPOT_CENTER ;

  final int DIST_X_FOUNDATION_CENTER  =   45 ;
  final int DIST_X_FOUNDATION_OUTER   =  -15 + DIST_X_FOUNDATION_CENTER ;

  final int DIST_Y_FOUNDATION_CENTER  =   15 ;
  final int DIST_Y_FOUNDATION_OUTER   =   35 + DIST_Y_FOUNDATION_CENTER ;



  // RGB sum

  final int LUMIN_THRESHOLD           =  400 ;



  // milliseconds

  final int TIME_ONE_TURN             =  700 ;
  final int TIME_STONE_MARGIN         =  500 ;
  final int TIME_FOUNDATION_MARGIN    =  500 ;
  final int TIME_TOGGLE_GRABBER       = 1000 ;





  final int OPT_LUMIN_G =  0 ;

  final int OPT_DIST_NL = -1 ;
  final int OPT_DIST_NG =  1 ;
  final int OPT_DIST_XL = -2 ;
  final int OPT_DIST_XG =  2 ;
  final int OPT_DIST_SL = -3 ;
  final int OPT_DIST_SG =  3 ;

  final int OPT_TIME_L  = -4 ;



  boolean shouldGrab = false ;








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


  private void grab () {

    if ( shouldGrab ) {

      grabber.setPosition ( 0.7 ) ;

    } else {

      grabber.setPosition ( 0   ) ;

    }

  }
















  private void update () {

    grab () ;

    time  = getRuntime () ;

    distN = ( distN + senseDistN.getDistance ( DistanceUnit.INCH )                 ) / 2 ;
    distX = ( distX + senseDistX.getDistance ( DistanceUnit.INCH )                 ) / 2 ;
    distS = ( distS + senseDistS.getDistance ( DistanceUnit.INCH )                 ) / 2 ;
    lumin = ( lumin + senseColor.red () + senseColor.green () + senseColor.blue () ) / 2 ;

    telemetry.addData ( "distN" , distN ) ;
    telemetry.addData ( "distX" , distX ) ;
    telemetry.addData ( "distS" , distS ) ;
    telemetry.addData ( "lumin" , lumin ) ;
    telemetry.update  (                 ) ;

  }


  private void runWhile ( int option , int comparator ) {

    switch ( option ) {

      case  0 : while ( opModeIsActive() && lumin > comparator ) { update () ; }
      break ;

      case -1 : while ( opModeIsActive() && distN < comparator ) { update () ; }
      break ;

      case  1 : while ( opModeIsActive() && distN > comparator ) { update () ; }
      break ;

      case -2 : while ( opModeIsActive() && distX < comparator ) { update () ; }
      break ;

      case  2 : while ( opModeIsActive() && distX > comparator ) { update () ; }
      break ;

      case -3 : while ( opModeIsActive() && distS < comparator ) { update () ; }
      break ;

      case  3 : while ( opModeIsActive() && distS > comparator ) { update () ; }
      break ;

      case -4 : while ( opModeIsActive() && time < comparator )  { update () ; }
      break ;

    }


    driveNW.setPower ( 0 ) ;
    driveNE.setPower ( 0 ) ;
    driveSE.setPower ( 0 ) ;
    driveSW.setPower ( 0 ) ;
    slider.setPower  ( 0 ) ;


  }

  private void runFor ( int milliseconds ) {

    runWhile ( OPT_TIME_L , milliseconds + (int)getRuntime() ) ;

  }

















  @Override
  public void runOpMode () {


    driveNW = hardwareMap.get(DcMotor.class, "driveNW");
    driveNE = hardwareMap.get(DcMotor.class, "driveNE");
    driveSE = hardwareMap.get(DcMotor.class, "driveSE");
    driveSW = hardwareMap.get(DcMotor.class, "driveSW");

    slider = hardwareMap.get(DcMotor.class, "slider");
    grabber = hardwareMap.get(Servo.class, "grabber");

    senseColor = hardwareMap.get(ColorSensor.class, "color");
    senseDistN = hardwareMap.get(DistanceSensor.class, "distN");
    senseDistX = hardwareMap.get(DistanceSensor.class, "distX");
    senseDistS = hardwareMap.get(DistanceSensor.class, "distS");


    waitForStart();
    runtime.reset();


    // drive sideways to contact foundation

    driveX(CAUTIOUS_POWER);
    runWhile(OPT_DIST_XL, DIST_X_FOUNDATION_OUTER);


    // grab foundation


    // drive sideways to wall while pulling foundation

    driveX(-CAUTIOUS_POWER);
    runWhile(OPT_DIST_XG, DIST_X_WALL);

    // park under skybridge (outer lane)

    driveY(FULL_POWER);
    runWhile(OPT_DIST_SL, DIST_Y_SKYBRIDGE);

  }
}