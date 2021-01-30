package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.Shooter;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.Constraint;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.Trajectory;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Autonomous")
//@Disabled
public class Autonomous extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime timer = new ElapsedTime();

    Robot robot = new Robot( this );

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private boolean inInitializationState() {
        return (!opModeIsActive() && !isStopRequested());
    }

    @Override
    public void runOpMode() {
        String ringPatten = "";

        // intialize robot submodules
        robot.getDriveTrain().init();
        robot.getGrabber().init();
        robot.getShooter().init();
        robot.getIntake().init();
        robot.getComputerVision().init();

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        // Setting drive train to float unlocks wheels to allow movemment and to confirm odometry readings are working of robot after init
        robot.getDriveTrain().setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
        robot.getComputerVision().activate();;
        robot.getIntake().liftBasket();
        robot.getGrabber().closeGrabber();

        while (inInitializationState()) {
            robot.getDriveTrain().clearBulkCache();

            ringPatten = robot.getComputerVision().detect();

            int leftPosition = robot.getDriveTrain().getLeftOdometryPosition();
            int rightPosition = robot.getDriveTrain().getRightOdometryPosition();

            telemetry.addData("ring pattern",   ringPatten);
            telemetry.addData("", "------------------------------");
            telemetry.addData("left",   leftPosition);
            telemetry.addData("right",  rightPosition);

            telemetry.addData("left Chg",   leftPosition - robot.getDriveTrain().getInitLeftOdometryPosition());
            telemetry.addData("right Chg",  rightPosition - robot.getDriveTrain().getInitRightOdometryPosition());

            telemetry.addData("PosInDegrees",   String.format("%.2f",robot.getDriveTrain().getCurrentPositionInDegreesUsingOdometry(rightPosition, leftPosition) ));

            telemetry.addData("", "------------------------------");
            telemetry.addData( "rate/sec",  String.format("%.2f", ++count / timer.seconds()) );

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        robot.getComputerVision().stop();

        if (!opModeIsActive() || isStopRequested())
            return;

        robot.resetAutonomousTimer();
        robot.getDriveTrain().setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////
        robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_1);
        robot.getGrabber().armUp();
        robot.getShooter().stopPoker();

        Trajectory trajectory;

        if (ringPatten.equals("Quad")) {
            trajectory = runAutoZoneC();
        } else if (ringPatten.equals("Single")) {
            trajectory = runAutoZoneB();
        } else {
            trajectory = runAutoZoneA();
        }

        robot.followTrajectory( trajectory );

        robot.getShooter().burstPoker();

        robot.resumeTrajectory( trajectory );

        dropWobbleGoal();

        robot.resumeTrajectory( trajectory );

        sleep(500);
        grabWobbleGoal();

        robot.resumeTrajectory( trajectory );

        dropWobbleGoal();

        robot.resumeTrajectory( trajectory );

        // Autonomous Complete
        // stop robot and position for Driver Control
        robot.getGrabber().armUp();
        robot.getDriveTrain().stop();
        robot.getShooter().stop();
        robot.getIntake().stop();
        robot.getIntake().dropIntake();
        robot.getIntake().lowerBasket();

    }

    public void dropWobbleGoal() {
         robot.getGrabber().armDown();
         sleep(500);
         robot.getGrabber().openGrabber();
         sleep(750);
    }
    public void grabWobbleGoal() {
        robot.getGrabber().closeGrabber();
        sleep(750);
        robot.getGrabber().armUp();
    }

    public Trajectory runAutoZoneA() {
        Trajectory trajectoryZoneA = robot.trajectoryBuilder()
                .setDefaultTargetPower( 0.40 )
                .moveForward( 60 )
                .turnLeft( 345 )
                .stop()
                .turnRight( 60 )
                .moveForward( 8 )
                .stop()
                .moveBackward( 40 )
                .turnRight( 180 )
                .moveForward( 18, 0.20 )
                .stop()
                .moveBackward( 20 )
                .turnLeft( 60 )
                .moveForward( 33 )
                .turnRight(105)
                .build();

        return trajectoryZoneA;
    }

    public Trajectory runAutoZoneB() {
        Trajectory trajectoryZoneB = robot.trajectoryBuilder()
                .setDefaultTargetPower(0.40)
                .moveForward(60)
                .turnLeft(345)
                .stop()
                .turnRight(0)
                .moveForward(30)
                .stop()
                .moveBackward(30)
                .turnRight(60)
                .moveBackward(34)
                .turnRight(180)
                .moveForward(15, 0.20)
                .stop()
                .moveBackward(21)
                .turnLeft(60)
                .moveForward(32)
                .turnLeft(0)
                .moveForward(14)
                .stop()
                .moveBackward(14)
                .build();
        return trajectoryZoneB;
    }

    public Trajectory runAutoZoneC(){
        Trajectory trajectoryZoneC = robot.trajectoryBuilder()
                .setDefaultTargetPower( 0.40 )
                .moveForward( 60 )
                .turnLeft( 340 )
                .stop()
                .turnRight( 0 )
                .moveForward( 40 )
                .turnRight( 60 )
                .moveForward(8)
                .stop()
                .moveBackward( 40 )
                .turnRight( 180 )
                .moveForward(50)
                .moveForward(10, 0.20)
                .stop()
                .moveBackward( 40 )
                .turnLeft( 45 )
                .moveForward( 40 )
                .stop()
                .moveBackward(17)
                .build();

        return trajectoryZoneC;
    }
}