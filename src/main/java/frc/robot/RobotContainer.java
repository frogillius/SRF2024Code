package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.Constants.*;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Subsystem local object handles */
    private final SwerveSubsystem          m_swerveSubsystem;

    // Create SmartDashboard chooser for autonomous routines
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Xbox Controllers
    private final CommandXboxController m_xbox;

    // Non-default Commands using Xbox controllers
    private final SwerveParkCmd swerveParkCmd;

    private double lastKnownTestAngle = 0.0;

    //   One way to define Driver Buttons for use in configureButtonBindings: 
    //   private final zeroGyro = new JoystickButton(m_xbox, XboxController.Button.kY.value);
    //   private final JoystickButton robotCentric = new JoystickButton(m_xbox, XboxController.Button.kLeftBumper.value);
    
    //  Constructor for the robot container. Contains subsystems, 
    //  OI devices, and commands.

    public RobotContainer() {
        m_xbox = new CommandXboxController(0);

        m_swerveSubsystem = new SwerveSubsystem();
        m_swerveSubsystem.setDefaultCommand(
                new DefaultDriveCmd(
                    m_swerveSubsystem, 
                    () -> -m_xbox.getLeftY(),            // translate: - fore / + laft
                    () -> -m_xbox.getLeftX(),            // strafe: - left / + right
                    () -> -m_xbox.getRightX()));

        swerveParkCmd = new SwerveParkCmd(m_swerveSubsystem,
                                          () -> m_xbox.getLeftY(),     // translate
                                          () -> m_xbox.getLeftX(),     // strafe
                                          () -> m_xbox.getRightX());   // rotate
        DoNothingCmd m_doNothingAuto = new DoNothingCmd();
        TestAuto m_testAuto = new TestAuto(m_swerveSubsystem);
        
        //m_chooser.addOption("Do nothing: stage out of the way", m_doNothingAuto);
        m_chooser.setDefaultOption("Do Nothing", m_doNothingAuto);
        m_chooser.addOption("Test patterns ", m_testAuto);
        SmartDashboard.putData("Autonomous Selection: ", m_chooser);

        configureButtonBindings();
    }

    private void parkIt() {
        SmartDashboard.putString("Debug trace ", "parkIt called");
        swerveParkCmd.schedule();
    }

    private void rotateWheelsCCW() {
        lastKnownTestAngle += 45;
        RotateModulesToAngleCmd goCCW = new RotateModulesToAngleCmd(m_swerveSubsystem, lastKnownTestAngle);
        goCCW.schedule();
    }
    private void rotateWheelsCW() {
        lastKnownTestAngle -= 45;
        RotateModulesToAngleCmd goCCW = new RotateModulesToAngleCmd(m_swerveSubsystem, lastKnownTestAngle);
        goCCW.schedule();
    }

    //     * Buttons can be created by
    //     * instantiating a {@link GenericHID} or one of its subclasses ({@link
    //     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
    //     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    
    private void configureButtonBindings() {
        // Driver Buttons
        // The following govern the driving UI:
        //      POV() == UP     =>  Field oriented (default on start) 
        //      POV() == DOWN   =>  Robot oriented.
        //      POV() == LEFT   =>  Rotate 90 deg CW from last angle
        //      POV() == RIGHT  =>  Rotate 90 deg CCW from last angle
        //      Back()          =>  Zero the Gyro
        //      Start()         =>  Test: rotate all modules to angle (0, 45, etc.)
        //      LeftTrigger     =>  Set Pivot Point to be LF wheel
        //      RightTrigger    =>  Set Pivot Point to be RF wheel
        //      Left Bumbper    =>  Set Pivot Point to be BL wheel
        //      Right Bumper    =>  Set Pivot Point to be BR wheel
        //      Y()             =>  RESET Pivot Point to be Robot Center
        //      A()             =>  Slow mode
        //      B()             =>  Regular speed
        //      X()             =>  Park (crossed wheel angles)

        m_xbox.povUp().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(true)));
        m_xbox.povDown().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(false)));
        m_xbox.povLeft().onTrue(new InstantCommand(()-> rotateWheelsCCW()));
        m_xbox.povRight().onTrue(new InstantCommand(()-> rotateWheelsCW()));
        /*
        m_xbox.leftTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFLCenOfRotation()));
        m_xbox.leftTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFRCenOfRotation()));                                        
        m_xbox.rightTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));  
        m_xbox.leftBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBLCenOfRotation()));
        m_xbox.leftBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBRCenOfRotation()));
        m_xbox.rightBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        */
        m_xbox.y().onTrue(new TestAuto(m_swerveSubsystem));
        m_xbox.a().onTrue(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.075)));
        m_xbox.b().onTrue(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.4)));    
        m_xbox.x().onTrue(new InstantCommand(()->parkIt()));
        m_xbox.back().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));
     }
    
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
        //return new TestAuto(m_swerveSubsystem);
    }
} 
