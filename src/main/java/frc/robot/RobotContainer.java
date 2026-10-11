package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ConditionalCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.RunCommand;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.button.JoystickButton;
import org.wpilib.command2.button.POVButton;
import org.wpilib.command2.button.Trigger;
import frc.robot.commands.ArmHold;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorHold;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmMode;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOReal;
// import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
// import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
// import frc.robot.subsystems.drive.GyroIOSim;
// import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
// import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffectorIOReal;
// import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.util.RobotIdentity;
import frc.robot.util.SmarterDashboard;
import frc.robot.util.simulation.AlgaeHandler;
// import org.ironmaple.simulation.SimulatedArena;
// import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private Drive drive;
    private Elevator elevator;
    public static Arm arm;
    private EndEffector endEffector;
    private Vision vision;
    private Climber climber;
    public static AutoAlign align;

//     private SwerveDriveSimulation driveSimulation = null;

    //     public static final LEDStrip ledstrip = LEDStrip.getInstance();

    // Controller
    public static final NiDsXboxController driverController = new NiDsXboxController(0);
    public static final NiDsXboxController opController = new NiDsXboxController(1);

    // ---------------------------- Driver Controller --------------------------------- //

    private final JoystickButton resetHeading_Start =
            new JoystickButton(driverController, NiDsXboxController.Button.kStart.value);

    private final POVButton reefAlignLeft_PovLeft = new POVButton(driverController, POVDirection.LEFT);
    private final POVButton reefAlignCenter_PovDown = new POVButton(driverController, POVDirection.DOWN);
    private final POVButton reefAlignRight_PovRight = new POVButton(driverController, POVDirection.RIGHT);
    private final POVButton stationAlign_PovUp = new POVButton(driverController, POVDirection.UP);

    private final JoystickButton slowMode_A = new JoystickButton(driverController, NiDsXboxController.Button.kA.value);
    private final JoystickButton zeroClimber_back =
            new JoystickButton(driverController, NiDsXboxController.Button.kBack.value);

    private final Trigger strafe_Triggers = new Trigger(
            () -> Math.abs(driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()) > 0.1);

    // ----------------------------- Op Controller -------------------------------- //

    private final JoystickButton homeElevator_Start =
            new JoystickButton(opController, NiDsXboxController.Button.kStart.value);
    private final JoystickButton station_Back = new JoystickButton(opController, NiDsXboxController.Button.kBack.value);
    private final JoystickButton levelFour_Y = new JoystickButton(opController, NiDsXboxController.Button.kY.value);
    private final JoystickButton levelThree_B = new JoystickButton(opController, NiDsXboxController.Button.kB.value);
    private final JoystickButton levelTwo_X = new JoystickButton(opController, NiDsXboxController.Button.kX.value);
    private final JoystickButton stow_A = new JoystickButton(opController, NiDsXboxController.Button.kA.value);

    private final JoystickButton coralMode_LB =
            new JoystickButton(opController, NiDsXboxController.Button.kLeftBumper.value);
    private final JoystickButton algaeMode_RB =
            new JoystickButton(opController, NiDsXboxController.Button.kRightBumper.value);

    private final POVButton climberStateInc_PovLeft = new POVButton(opController, POVDirection.LEFT);
    private final POVButton climberStateDec_PovRight = new POVButton(opController, POVDirection.RIGHT);
    private final POVButton climberAdjustUp_PovUp = new POVButton(opController, POVDirection.UP);
    private final POVButton climberAdjustDown_PovDown = new POVButton(opController, POVDirection.DOWN);

    private final Trigger elevatorAdjust = new Trigger(() -> Math.abs(opController.getLeftY()) > 0.9);
    private final Trigger armAdjust = new Trigger(() -> Math.abs(opController.getRightX()) > 0.9);
    private final Trigger alignAdjust = new Trigger(() -> Math.abs(opController.getLeftTriggerAxis()) > 0.9);

    public final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        switch (Constants.currentMode) {

                // Real robot, instantiate hardware IO implementations

            case REAL:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.FrontLeft),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.FrontRight),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.BackLeft),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.BackRight),
                        (robotPose) -> {});
                vision = new Vision(drive::accept, new VisionIOLimelight(camera0Name, drive::getRotation));
                // new VisionIOLimelight(camera1Name, drive::getRotation));
                elevator = new Elevator(new ElevatorIOReal());
                arm = new Arm(new ArmIOReal());
                endEffector = new EndEffector(new EndEffectorIOReal());
                climber = new Climber(new ClimberIOReal());
                break;

                // Sim robot, instantiate physics sim IO implementations

            case SIM:
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.FrontLeft),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.FrontRight),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.BackLeft),
                        new ModuleIOTalonFX(Constants.TUNER_CONSTANTS.BackRight),
                        (robotPose) -> {});
                vision = new Vision(drive::accept, new VisionIOLimelight(camera0Name, drive::getRotation));
                // new VisionIOLimelight(camera1Name, drive::getRotation));
                elevator = new Elevator(new ElevatorIOReal());
                arm = new Arm(new ArmIOReal());
                endEffector = new EndEffector(new EndEffectorIOReal());
                climber = new Climber(new ClimberIOReal());
                break;
                // Replaced sim with the case REAL

            default:
                elevator = new Elevator(new ElevatorIO() {});
                arm = new Arm(new ArmIO() {});
                endEffector = new EndEffector(new EndEffectorIO() {});
                climber = new Climber(new ClimberIO() {});
                break;
        }

        align = new AutoAlign(drive::getPose);
        // if (Constants.currentMode == Constants.Mode.SIM) {
        //     align.setRobotSupplier(driveSimulation::getSimulatedDriveTrainPose);
        // }

        setDefaultCommands();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser("Right");
        // autoChooser.addOption("Drive FF Ch", DriveCommands.feedforwardCharacterization(drive));
        // autoChooser.addOption("red barge 2 algae", new PathPlannerAuto("untested Center 2 Algae", true));
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();

        SmarterDashboard.putString("RoboRio Serial", RobotIdentity.getRoboRioSerial());
        SmarterDashboard.putString("Robot Identity", RobotIdentity.getRobotIdentityString());

        // ------------------------------ Event Markers ---------------------------------- //

        new EventTrigger("LevelStation")
                .onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                        .andThen(new InstantCommand(() -> arm.setArmIntake())));
        new EventTrigger("LevelFour")
                .onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                        .andThen(new InstantCommand(() -> arm.setArmL4())));

        DriverStationBackend.silenceJoystickConnectionWarning(true);
    }

    private void configureBindings() {

        // ------------------------------- Driver Bindings ------------------------------- //

        // Reset gyro / odometry
        final Runnable resetOdometry = () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
        resetHeading_Start.onTrue(Commands.runOnce(resetOdometry).ignoringDisable(true));

        reefAlignLeft_PovLeft.whileTrue(align.reefAlignLeft(drive));
        reefAlignRight_PovRight.whileTrue(align.reefAlignRight(drive));
        reefAlignCenter_PovDown.whileTrue(align.reefAlignMid(drive));
        stationAlign_PovUp.whileTrue(align.stationAlign(drive));

        strafe_Triggers.whileTrue(DriveCommands.joystickDrive(
                drive,
                () -> 0,
                () -> (driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis()) * 0.25,
                () -> 0,
                false));

        slowMode_A.onTrue(new InstantCommand(() -> drive.changeSpeedMultiplier()));
        zeroClimber_back.onTrue(new InstantCommand(() -> climber.zeroClimber()));

        // ------------------------------- Operator Bindings ------------------------------- //

        homeElevator_Start
                .whileTrue(new InstantCommand(() -> elevator.setZeroing(true)))
                .onFalse(new InstantCommand(() -> elevator.stopElevator())
                        .withTimeout(0.2)
                        .andThen(new InstantCommand(() -> elevator.zeroElevator())
                                .andThen(new InstantCommand(() -> elevator.setZeroing(false)))));

        station_Back.onTrue(new InstantCommand(() -> elevator.setElevatorStationMode())
                .andThen(new ConditionalCommand(
                        new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                .andThen(new InstantCommand(() -> arm.setArmIntake())));
        stow_A.onTrue(new InstantCommand(() -> arm.setStowed())
                .andThen(new WaitCommand(1))
                .andThen(new InstantCommand(() -> elevator.setElevatorStowedMode())));
        levelTwo_X.onTrue(new InstantCommand(() -> elevator.setElevatorLevelTwoMode())
                .andThen(new ConditionalCommand(
                        new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                .andThen(new InstantCommand(() -> arm.setArmL2())));
        levelThree_B.onTrue(new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                .andThen(new ConditionalCommand(
                        new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                .andThen(new InstantCommand(() -> arm.setArmL3())));
        levelFour_Y.onTrue(new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                .andThen(new ConditionalCommand(
                        new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                .andThen(new InstantCommand(() -> arm.setArmL4())));

        coralMode_LB.onTrue(new InstantCommand(() -> elevator.setCoral())
                .andThen(new InstantCommand(() -> arm.setCoral()))
                .andThen(new InstantCommand(() -> endEffector.setCoralMode()))
                .andThen(new InstantCommand(() -> endEffector.stopEE())));
        algaeMode_RB.onTrue(new InstantCommand(() -> elevator.setAlgae())
                .andThen(new InstantCommand(() -> arm.setAlgae()))
                .andThen(new InstantCommand(() -> endEffector.setAlgaeMode())));

        climberAdjustUp_PovUp
                .whileTrue(new RunCommand(() -> climber.climberUp()))
                .onFalse(new InstantCommand(() -> climber.stop()));
        climberAdjustDown_PovDown
                .whileTrue(new RunCommand(() -> climber.climberDown()))
                .onFalse(new InstantCommand(() -> climber.stop()));
        climberStateInc_PovLeft.onTrue(new InstantCommand(() -> climber.decrementClimbState()));
        climberStateDec_PovRight.onTrue(new InstantCommand(() -> {
                    elevator.setElevatorStowedMode();
                    arm.setAlgae();
                    arm.setStowed();
                })
                .andThen(new WaitCommand(0.2))
                .andThen(new InstantCommand(() -> climber.incrementClimbState())));

        elevatorAdjust.whileTrue(
                new RunCommand(() -> elevator.changeElevatorOffset(.01 * Math.signum(-opController.getLeftY()))));
        armAdjust.whileTrue(new RunCommand(() -> arm.armAdjust(.01 * Math.signum(opController.getRightX()))));
        alignAdjust
                .onTrue(new InstantCommand(() -> arm.setAligning(true))
                        .andThen(new InstantCommand(() -> elevator.setAligning(true))))
                .onFalse(new InstantCommand(() -> arm.setAligning(false))
                        .andThen(new InstantCommand(() -> elevator.setAligning(false))));

        // if (Constants.currentMode == Constants.Mode.SIM) {
        //     csDropLB.onTrue(new InstantCommand(() -> dropCoralFromStation(false)).ignoringDisable(true));
        //     csDropRB.onTrue(new InstantCommand(() -> dropCoralFromStation(true)).ignoringDisable(true));
        // }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStationBackend.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.RED;
        }
        return false;
    }

    // -------------------------------- PathPlanner Commands -------------------------------- /w

    public void registerNamedCommands() {
        NamedCommands.registerCommand(
                "LevelStation",
                new InstantCommand(() -> elevator.setElevatorStationMode())
                        .andThen(new InstantCommand(() -> arm.setArmIntake())));
        NamedCommands.registerCommand("ElevatorStowedMode", new InstantCommand(() -> elevator.setElevatorStowedMode()));
        NamedCommands.registerCommand(
                "LevelTwo",
                new InstantCommand(() -> elevator.setElevatorLevelTwoMode())
                        .andThen(new InstantCommand(() -> arm.setArmL2())));
        NamedCommands.registerCommand(
                "LevelThree",
                new InstantCommand(() -> elevator.setElevatorLevelThreeMode())
                        .andThen(new InstantCommand(() -> arm.setArmL3())));
        NamedCommands.registerCommand(
                "LevelFour",
                new InstantCommand(() -> elevator.setElevatorLevelFourMode())
                        .andThen(new ConditionalCommand(
                                new WaitCommand(0.2), Commands.none(), () -> arm.getArmMode() == ArmMode.Stowed))
                        .andThen(new InstantCommand(() -> arm.setArmL4())));

        NamedCommands.registerCommand("Outtake", new InstantCommand(() -> endEffector.outtakeCoral()));
        NamedCommands.registerCommand(
                "Intake", new RunCommand(() -> endEffector.intakeCoral()).until(() -> endEffector.hasCoral()));
        NamedCommands.registerCommand("Stop EE", new InstantCommand(() -> endEffector.stopCoral()));
        NamedCommands.registerCommand("Hold Coral", new InstantCommand(() -> endEffector.holdCoral()));
        NamedCommands.registerCommand(
                "Home Elevator",
                new InstantCommand(() -> elevator.setZeroing(true))
                        .withTimeout(2.5)
                        .andThen(new InstantCommand(() -> elevator.stopElevator()))
                        .withTimeout(0.2)
                        .andThen(new InstantCommand(() -> elevator.zeroElevator()))
                        .andThen(new InstantCommand(() -> elevator.setZeroing(false))));

        NamedCommands.registerCommand("Stop", new InstantCommand(() -> drive.stopWithX()));

        NamedCommands.registerCommand(
                "Auto Align Left",
                align.reefAlignLeft(drive).until(align::isAlignedDebounced).withTimeout(2));

        NamedCommands.registerCommand(
                "Auto Align Mid", align.reefAlignMid(drive).withTimeout(2));

        NamedCommands.registerCommand(
                "Auto Align Right",
                align.reefAlignRight(drive).until(align::isAlignedDebounced).withTimeout(2));

        NamedCommands.registerCommand(
                "Auto Align Station",
                align.stationAlign(drive).until(align::isAlignedDebounced).withTimeout(2));

        NamedCommands.registerCommand(
                "Set Algae",
                new InstantCommand(() -> elevator.setAlgae())
                        .andThen(new InstantCommand(() -> arm.setAlgae()))
                        .andThen(new InstantCommand(() -> endEffector.setAlgaeMode())));
        NamedCommands.registerCommand(
                "Set Coral",
                new InstantCommand(() -> elevator.setAlgae())
                        .andThen(new InstantCommand(() -> arm.setCoral()))
                        .andThen(new InstantCommand(() -> endEffector.setCoralMode())));

        NamedCommands.registerCommand(
                "Algae Intake", new RunCommand(() -> endEffector.intakeAlgae()).until(() -> endEffector.hasCoral()));
        NamedCommands.registerCommand("Algae Outtake", new InstantCommand(() -> endEffector.outtakeAlgae()));
    }

    // ----------------------------------- Default Commands -------------------------------- //

    public void setDefaultCommands() {
        drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() * drive.getDriveMultiplier(),
                () -> -driverController.getLeftX() * drive.getDriveMultiplier(),
                () -> -driverController.getRightX() * drive.getTurnMultiplier(),
                true));

        elevator.setDefaultCommand(new ElevatorHold(elevator));
        arm.setDefaultCommand(new ArmHold(arm));
        // climber.setDefaultCommand(new ClimbCommand());
        // ledstrip.setDefaultCommand(ledstrip.defaultCommand(() -> endEffector.isCoral()));
    }

//     public void resetSimulation() {
//         if (Constants.currentMode != Constants.Mode.SIM) return;

//         drive.resetOdometry(new Pose2d(12, 2, new Rotation2d()));
//         SimulatedArena.getInstance().resetFieldForAuto();
//         AlgaeHandler.getInstance().reset();
//     }

//     public void displaySimFieldToAdvantageScope() {
//         if (Constants.currentMode != Constants.Mode.SIM) return;

//         SimulatedArena.getInstance().simulationPeriodic();
//         Logger.recordOutput("FieldSimulation/Pose", new Pose3d(driveSimulation.getSimulatedDriveTrainPose()));
//         Logger.recordOutput(
//                 "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
//         Logger.recordOutput(
//                 "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
//         Logger.recordOutput(
//                 "FieldSimulation/Staged Algae", AlgaeHandler.getInstance().periodic());
//     }
}
