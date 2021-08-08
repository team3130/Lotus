package frc.team3130.robot.SupportingClasses;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collections;

public class SocketProgramming {
    private DataInputStream in;
    private DataOutputStream out;
    private boolean running;
    private Socket socket;

    ArrayList<Byte> temp = new ArrayList<Byte>();
    double message;
    double whole[] = new double[2];

    public void run() {
        System.out.println("socket thread running");

        int len;
        byte [] buffer = new byte[1500];
        try {
            this.in  = new DataInputStream(this.socket.getInputStream());
            this.out = new DataOutputStream(this.socket.getOutputStream());
            running = true;

            while (running){
                len = in.read(buffer);
                if (len < 0)
                    running = false;
                else
                    parsepacket(buffer, len);
            }

            consolidate();

        }catch (IOException ex) {
            System.out.println("socket catch IOException: "+ex);
        }finally{
            try {
                System.out.println("Closing GWsocket");
                fireSocketClosure();
                in.close();
                out.close();
                socket.close();
            }catch (IOException ex) {
                System.out.println("socket finally IOException: "+ex);
            }
        }
    }

    private void fireSocketClosure() {
    }

    private void parsepacket(byte[] buffer, int length) {
        for (int i = 0; i < length; i++) {
            temp.add(buffer[i]);
        }
    }

    public void consolidate() {
        byte[] toBeCopiedInto = new byte[temp.size()];
        for (int i = 0; i < temp.size(); i++) {
            toBeCopiedInto[i] = temp.get(i);
        }
        message = ByteBuffer.wrap(toBeCopiedInto).getDouble();
        temp.clear();
    }

    public void close() throws IOException {
        this.socket.close();
    }
}
