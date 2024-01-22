// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.swerve;

import java.text.DecimalFormat;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/** Add your docs here. */
public class SwerveSBEntries {
    private final ShuffleboardLayout sBE_Layout;
    /*
    // Technically, the following member Entry variables do not need to be saved,
    // as the data written via them is static and should never change after
    // initial setup
    private GenericEntry driveIdEntry;
    private GenericEntry steerIdEntry;
    private GenericEntry encoderIdEntry;
    private GenericEntry absOffsetEntry;
    */
    // However, the following Entry keys are for dynamic variables, and
    // getter methods are provided below for use by whatever modules need
    // to publish (and / or read?) such data to/from the Shuffleboard.
    private GenericEntry steerSetpointDegEntry;
    private GenericEntry motorEncoderDegEntry;
    private GenericEntry absCANCoderDegEntry;
    private GenericEntry steerPIDOutputEntry;
    private GenericEntry wheelCurrPosEntry;
    private GenericEntry wheelCurrSpeedEntry;

    DecimalFormat df = new DecimalFormat("#.#");
    DecimalFormat df2 = new DecimalFormat("#.##");

    SwerveSBEntries(int modNum, SwerveModuleConstants module) {
        sBE_Layout = module.SBE_LAYOUT;
        // See comment above about not needing to retain these Entry keys
        /* driveIdEntry = */    sBE_Layout.add("DrvId", df.format(module.DRIVE_MOTOR_ID)).getEntry();
        /* steerIdEntry = */    sBE_Layout.add("RotId", df.format(module.STEER_MOTOR_ID)).getEntry();
        /* encoderIdEntry = */  sBE_Layout.add("EncId", df.format(module.ENCODER_ID)).getEntry();
        absCANCoderDegEntry =   sBE_Layout.add("CCdeg", "0").getEntry();
        /* absOffsetEntry = */  sBE_Layout.add("Offset", df2.format(module.ABS_ANG_OFFSET2D.getDegrees())).getEntry();
        motorEncoderDegEntry =  sBE_Layout.add("MEdeg", "0").getEntry();
        steerSetpointDegEntry = sBE_Layout.add("SPdeg", "0").getEntry();
         steerPIDOutputEntry =  sBE_Layout.add("PID_O", "0").getEntry();
        wheelCurrSpeedEntry =   sBE_Layout.add("Wspd", "0").getEntry();
        wheelCurrPosEntry =     sBE_Layout.add("Wpos", "0").getEntry();
    }

    public GenericEntry getSteerSetpointDegEntry() {
        return steerSetpointDegEntry;
    }

    public GenericEntry getmotorEncoderDegEntry() {
        return motorEncoderDegEntry;
    }

    public GenericEntry getAbsCANCoderDegEntry() {
        return absCANCoderDegEntry;
    }

    public GenericEntry getSteerPIDOutputEntry() {
        return steerPIDOutputEntry;
    }

    public GenericEntry getWheelCurrPosEntry() {
        return wheelCurrPosEntry;
    }
    
    public GenericEntry getWheelCurrSpeedEntry() {
        return wheelCurrSpeedEntry;
    }
}

