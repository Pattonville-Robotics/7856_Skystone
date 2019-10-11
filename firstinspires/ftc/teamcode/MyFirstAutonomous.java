package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MyFirstAutonomous: Autonomous")

public class MyFirstAutonomous extends LinearOpMode {
    
    DcMotor left_drive_motor;
    DcMotor right_drive_motor;
    DcMotor left_rear_motor;
    DcMotor right_rear_motor;
    
    
    public void runOpMode(){
        left_drive_motor = hardwareMap.dcMotor.get("left_drive_motor");
        right_drive_motor = hardwareMap.dcMotor.get("right_drive_motor");
        left_rear_motor = hardwareMap.dcMotor.get("left_rear_motor");
        right_rear_motor = hardwareMap.dcMotor.get("right_rear_motor");
        
 
        waitForStart();
        
        
        
        right_rear_motor.setPower(1);
        left_rear_motor.setPower(-1);
        sleep(1000);
        right_rear_motor.setPower(1);
        left_drive_motor.setPower(1);
        sleep(2000);
        
        
       
        
        
    }
}