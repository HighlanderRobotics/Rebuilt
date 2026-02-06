// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pitcheck;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.subsystems.intake.LintakeSubsystem;

/** Add your docs here. */
public class Pitcheck {

    LintakeSubsystem intake = new LintakeSubsystem();
 
    public void pitcheck(Subsystem subsystem,Command command,BooleanSupplier endState){
        SmartDashboard.putData("intake", pitCheck(intake,intake.intake(),endState));

    }
    private Command pitCheck(Subsystem subsystem, Command command, BooleanSupplier endstate){
        
            return command;
        
    }
}