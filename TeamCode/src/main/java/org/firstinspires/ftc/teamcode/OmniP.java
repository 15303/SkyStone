package org.firstinspires.ftc.teamcode ;

import com.qualcomm.robotcore.eventloop.opmode.Disabled ;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode ;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous ;
import com.qualcomm.robotcore.hardware.DcMotor ;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous (

  name  = "OmniP"         ,
  group = "Linear Opmode"

)

// @Disabled


public class OmniP extends LinearOpMode {


  private ElapsedTime runtime = new ElapsedTime() ;

  private DcMotor         driveNW = null ;
  private DcMotor         driveNE = null ;
  private DcMotor         driveSE = null ;
  private DcMotor         driveSW = null ;


  private void driveY ( double power ) {

    driveNW.setPower ( -power ) ;
    driveNE.setPower (  power ) ;
    driveSE.setPower (  power ) ;
    driveSW.setPower ( -power ) ;

  }


  driveY ( 1    );
  sleep  ( 1000 );
  driveY ( 0    );


}