package frc.robot.motioncontrol.kalmanfilter.imu;

import java.util.ArrayList;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.motioncontrol.kalmanfilter.CustomKalmanFilter;

import com.github.swrirobotics.bags.reader.BagReader;
import com.github.swrirobotics.bags.reader.BagFile;
import com.github.swrirobotics.bags.reader.MessageHandler;
import com.github.swrirobotics.bags.reader.exceptions.BagReaderException;
import com.github.swrirobotics.bags.reader.exceptions.UninitializedFieldException;
import com.github.swrirobotics.bags.reader.messages.serialization.MessageType;
import com.github.swrirobotics.bags.reader.messages.serialization.StringType;
import com.github.swrirobotics.bags.reader.records.Connection;

public class IMUKalmanFilterCommand extends CommandBase {

    private CustomKalmanFilter filter;
    private BagFile bag;
    private ArrayList<String> messages;

    public IMUKalmanFilterCommand() {

        addRequirements();

    }

    @Override
    public void initialize() {

        messages = new ArrayList<String>();

        // from https://github.com/swri-robotics/bag-reader-java#print-all-of-the-std_msgstring-values-in-a-bag

        try {

            bag = BagReader.readFile("IMUAccelGyroData.bag");
            bag.forMessagesOfType("std_msgs/String", new MessageHandler(){
        
                @Override
                public boolean process(MessageType message, Connection connection) {
    
                    try {
                    
                        messages.add(message.<StringType>getField("data").getValue());

                    } catch (UninitializedFieldException e) {

                        SmartDashboard.putString("Exception Triggered", e.getMessage());
            
                    }

                    return true;
    
                }
                
            });

        } catch (BagReaderException e) {

            SmartDashboard.putString("Exception Triggered", e.getMessage());

        }

    }

    @Override
    public void execute() {

        SmartDashboard.putString("Executed", "Executed");

    }

}