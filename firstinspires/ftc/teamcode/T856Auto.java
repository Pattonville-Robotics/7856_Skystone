package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.pattonvillerobotics.commoncode.robotclasses.drive.MecanumEncoderDrive;
import org.pattonvillerobotics.commoncode.robotclasses.vuforia.VuforiaNavigation;
import org.pattonvillerobotics.commoncode.robotclasses.vuforia.VuforiaParameters;
import org.pattonvillerobotics.commoncode.enums.Direction;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.pattonvillerobotics.robotclasses.CustomizedRobotParameters;

@Autonomous
public class T856Auto extends LinearOpMode {

    // todo: write your code here your code
    private MecanumEncoderDrive drive;
    private VuforiaNavigation vuforia;
    
    @Override
    public void runOpMode() {
        double x = 0.0;
        initialize();
        vuforia.activateTracking();
        waitForStart();
        
        drive.moveInches(Direction.FORWARD,12,0.4);
        Telemetry.Item vuforiaX = telemetry.addData("Robot X", "0").setRetained(true);
        while (opModeIsActive() && vuforia.getRobotX() == 0.0) {
            vuforia.getVisibleTrackableLocation();
            
            x = vuforia.getRobotX();
             vuforiaX.setValue(vuforia.getRobotX());
             
            telemetry.update();
             
        }
        drive.moveInches(Direction.RIGHT, x, 0.4);
        drive.moveInches(Direction.FORWARD,12,0.4);
        //drive.moveInches(Direction.BACKWARD,12,0.4);
        // drive.rotateDegrees(Direction.CLOCKWISE,90,0.8);
        
    }
    public void initialize() {
        drive = new MecanumEncoderDrive(hardwareMap,this,CustomizedRobotParameters.ROBOT_PARAMETERS);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia = new VuforiaNavigation(new VuforiaParameters.Builder()
                .cameraMonitorViewId(/*R.id.cameraMonitorViewId*/cameraMonitorViewId)
                .licenseKey("AclLpHb/////AAAAGa41kVT84EtWtYJZW0bIHf9DHg5EHVYWCqExQMx6bbuBtjFeYdvzZLExJiXnT31qDi3WI3QQnOXH8pLZ4cmb39d1w0Oi7aCwy35ODjMvG5qX+e2+3v0l3r1hPpM8P7KPTkRPIl+CGYEBvoNkVbGGjalCW7N9eFDV/T5CN/RQvZjonX/uBPKkEd8ciqK8vWgfy9aPEipAoyr997DDagnMQJ0ajpwKn/SAfaVPA4osBZ5euFf07/3IUnpLEMdMKfoIH6QYLVgwbPuVtUiJWM6flzWaAw5IIhy0XXWwI0nGXrzVjPwZlN3El4Su73ADK36qqOax/pNxD4oYBrlpfYiaFaX0Q+BNro09weXQEoz/Mfgm")
                .build());
                idle();
    }
}
