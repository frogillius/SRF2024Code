package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
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
    private static CommandXboxController m_xbox;

    // Non-default Commands using Xbox controllers
    private final SwerveParkCmd swerveParkCmd;

    // private double lastKnownTestAngle = 0.0;

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
        TestSquareAuto m_testSquareAuto = new TestSquareAuto(m_swerveSubsystem);
        
        //m_chooser.addOption("Do nothing: stage out of the way", m_doNothingAuto);
        m_chooser.setDefaultOption("Do Nothing", m_doNothingAuto);
        m_chooser.addOption("Test Wheel Directions ", m_testAuto);
        m_chooser.addOption("Auto Square patterns ", m_testSquareAuto);
        SmartDashboard.putData("Autonomous Selection: ", m_chooser);

        configureButtonBindings();
    }
    
public static XboxController getHidXboxCtrl(){
    return m_xbox.getHID();
}

/* Only used for testing
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
*/

    //     * Buttons can be created by
    //     * instantiating a {@link GenericHID} or one of its subclasses ({@link
    //     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
    //     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    
    private void configureButtonBindings() {
        // Driver Buttons
        // The following govern the driving UI:
        //      POV() == UP     =>  avail 
        //      POV() == DOWN   =>  avail
        //      POV() == LEFT   =>  avail
        //      POV() == RIGHT  =>  avail
        //      Back()          =>  Reset all module absolute wheel angles
        //      Start()         =>  Zero the Gyro
        //      LeftTrigger     =>  avail
        //      RightTrigger    =>  avail
        //      Left Bumbper    =>  Alternate mode selections when held
        //      Right Bumper    =>  Slow mode when held
        //      Y()             =>  avail
        //      A()             =>  avail
        //      B()             =>  avail
        //      X()             =>  Park (crossed wheel angles)
        //      L Joystick Y    =>  Translate (move fore/aft)
        //      L Joystick X    =>  Strafe (move side to side)
        //      L Joystk Button =>  Set Field Oriented
        //      R Joystick Y    =>  avail
        //      R Joystick X    =>  Rotate (left = CCW, right = CW)
        //      R Joystk Button =>  Set Robot Oriented
        m_xbox.leftStick().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(true)));
        m_xbox.rightStick().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(false)));
        /*
        prior assignments used to trigger rotation about any given corner. Most useful when
        playing defense. Retained here in case a quick change to defense at competition
        is needed, and these button controls are no longer needed for offense. Or perhaps
        they could be re-mapped to POV buttons?
        m_xbox.leftTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFLCenOfRotation()));
        m_xbox.leftTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFRCenOfRotation()));                                        
        m_xbox.rightTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));  
        m_xbox.leftBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBLCenOfRotation()));
        m_xbox.leftBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBRCenOfRotation()));
        m_xbox.rightBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        */
        // TODO - increase VarMaxOutputFactor to .25 on true and 1.0 on false, after testing
        m_xbox.rightBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.075)));
        m_xbox.rightBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(1.0)));    
        m_xbox.x().onTrue(swerveParkCmd);
        m_xbox.start().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));
        m_xbox.back().onTrue(new InstantCommand(() -> m_swerveSubsystem.resetModulesToAbsolute()));
     }
    
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}