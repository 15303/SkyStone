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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous( name="OmniR0" , group="Linear Opmode" )
// @Disabled






















public class OmniR0 extends LinearOpMode {









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

  final int DIST_WALL                 =    5 ;

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

  private void runFor ( double milliseconds ) {

    runwhile ( OPT_TIME_L , milliseconds + getRuntime () ) ;

  }

















  @Override
  public void runOpMode () {



    driveNW    = hardwareMap.get ( DcMotor.class ,       "driveNW" ) ;
    driveNE    = hardwareMap.get ( DcMotor.class ,       "driveNE" ) ;
    driveSE    = hardwareMap.get ( DcMotor.class ,       "driveSE" ) ;
    driveSW    = hardwareMap.get ( DcMotor.class ,       "driveSW" ) ;

    slider     = hardwareMap.get ( DcMotor.class ,       "slider"  ) ;
    grabber    = hardwareMap.get ( Servo.class  ,        "grabber" ) ;

    senseColor = hardwareMap.get ( ColorSensor.class ,    "color"   ) ;
    senseDistN = hardwareMap.get ( DistanceSensor.class , "distN"   ) ;
    senseDistX = hardwareMap.get ( DistanceSensor.class , "distX"   ) ;
    senseDistS = hardwareMap.get ( DistanceSensor.class , "distS"   ) ;



    waitForStart() ;
    runtime.reset() ;



    // drive sideways to within 5in of depot

    driveX    ( CAUTIOUS_POWER                        ) ;
    runwhile  ( OPT_DIST_XL , DIST_X_DEPOT_OUTER      ) ;



    // drive forwards until skystone

    driveY    ( CAUTIOUS_POWER                        ) ;
    runwhile  ( OPT_LUMIN_G , LUMIN_THRESHOLD         ) ;


    // drive back for .2s

    driveY    (-CAUTIOUS_POWER                        ) ;
    runFor    ( TIME_STONE_MARGIN                     ) ;


    // drive sideways to stone depot

    driveX    ( CAUTIOUS_POWER                        ) ;
    runwhile  ( OPT_DIST_XL , DIST_X_DEPOT_CENTER     ) ;


    // drive forward for .2s

    driveY    ( CAUTIOUS_POWER                        ) ;
    runFor    ( TIME_STONE_MARGIN                     ) ;


    // grab

    shouldGrab = true ;


    // wait 1s for grabber to close

    runFor    ( TIME_TOGGLE_GRABBER                   ) ;


    // drive sideways to inner track

    driveX    (-NORMAL_POWER                          ) ;
    runwhile  ( OPT_DIST_XG , DIST_X_INTRACK_OUTER    ) ;


    // drive back until centered on foundation's long side

    driveY    (-FULL_POWER                            ) ;
    runwhile  ( OPT_DIST_SG , DIST_Y_FOUNDATION_CENTER) ;


    // drive sideways until touching foundation

    driveX    ( NORMAL_POWER                          ) ;
    runwhile  ( OPT_DIST_XG , DIST_X_FOUNDATION_OUTER ) ;


    // deploy foundation grabber
    //

    // drive sideways until touching wall

    driveX    (-FULL_POWER                            ) ;
    runwhile  ( OPT_DIST_XG , DIST_WALL               ) ;


    // release foundation grabber
    //


    // drive forward until clear of foundation

    driveY    ( NORMAL_POWER ) ;
    runwhile  ( OPT_DIST_SL , DIST_Y_FOUNDATION_OUTER ) ;


    // drive sideways until centered on foundation's short side

    driveX    ( NORMAL_POWER ) ;
    runwhile  ( OPT_DIST_XL , DIST_X_FOUNDATION_CENTER) ;


    // turn 180deg

    driveSpn  ( FULL_POWER                            ) ;
    runFor    ( TIME_ONE_TURN                         ) ;


    // raise slider

    sliderSpn ( FULL_POWER                            ) ;
    runFor    ( TIME_ONE_LEVEL                        ) ;


    // drive "forward" until touching foundation's short side

    driveY    ( CAUTIOUS_POWER                        ) ;
    runFor    ( TIME_FOUNDATION_MARGIN                ) ;


    // release

    shouldGrab = false ;


    // drive "backward"

    driveY    (-NORMAL_POWER                          ) ;
    runFor    ( TIME_FOUNDATION_MARGIN                ) ;

    shouldGrab = true ;


    // lower slider

    sliderSpn (-FULL_POWER                            ) ;
    runFor    ( TIME_ONE_LEVEL                        ) ;


    // turn 180deg

    driveSpn  (-FULL_POWER                            ) ;
    runFor    ( TIME_ONE_TURN                         ) ;


    // drive sideways to inner track

    driveX    (-NORMAL_POWER                          ) ;
    runwhile  ( OPT_DIST_XG , DIST_X_INTRACK_OUTER    ) ;


    // drive forward to under bridge

    driveY    ( FULL_POWER                            ) ;
    runwhile  ( OPT_DIST_SL , DIST_Y_SKYBRIDGE        ) ;



  }



}