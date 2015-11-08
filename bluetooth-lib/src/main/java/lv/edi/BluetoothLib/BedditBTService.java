package lv.edi.BluetoothLib;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.UUID;
import java.util.Vector;

import lv.edi.SmartWearProcessing.Sensor;

/**
 * Created by Richards on 07.11.2015..
 */
public class BedditBTService {

    public static final int BT_CONNECTING = 1;
    public static final int BT_CONNECTED = 2;
    public static final int BT_DISCONNECTED = 3;

    private BluetoothEventListener btEventListener;
    private boolean isConnected=false;
    private boolean isConnecting=false;
    private Vector<Sensor> sensorbuffer; // main data buffer where accelerometer data is stored
    private final UUID M_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"); //UUID for SPP profile;
    private BluetoothSocket mSocket; // bluetooth socket from this object data streams can be created
    private BluetoothDevice mDevice;// bluetooth connection target device
    private int bytesInPacket=13;
    private int batteryPacketIndex=63;
    private BatteryLevel batteryLevel;

    private BedditResultListener resultListener;

    private String continueStr = "CONT\n";


    ConnectThread connectThread; // instance of thread that creates bluetooth connection
    ReceiveThread receiveThread; // instance of thread that continously fetches data from bluetooth adapter

    public BedditBTService(){

    }

    public void setBeditResultListener(BedditResultListener listener){
        resultListener=listener;
    }

    public void connectDevice(BluetoothDevice device){
        mDevice = device;
        connectThread = new ConnectThread(mDevice);
        connectThread.start();
    }

    public void disconnectDevice(){

        receiveThread.cancel();
        connectThread.cancel();
        isConnected=false;
        connectThread = null;
        receiveThread=null;
        if(btEventListener!=null){
            btEventListener.onBluetoothDeviceDisconnected();
        }
    }

    public boolean isConnected(){
        return isConnected;
    }

    public class ConnectThread extends Thread{
        private BluetoothDevice mDevice; // connection target device
        ConnectThread(BluetoothDevice mDevice){ // connect thread constructor
            this.mDevice = mDevice; // set target device
        }
        public void run(){
            try{
                isConnecting = true;
                mSocket = mDevice.createInsecureRfcommSocketToServiceRecord(M_UUID); // create rfcomm protocol socket
                Log.d("BLUETOOTH_SERVICE", "created RFCOMM socket");
                if(btEventListener!=null){
                    btEventListener.onBluetoothDeviceConnecting();
                }
                mSocket.connect(); // connect to the bluetooth device
                Log.d("BLUETOOTH_SERVICE","connection estabilished");

                isConnected=true; // set connected indicator to true
                if(btEventListener!=null){
                    btEventListener.onBluetoothDeviceConnected();
                }
                isConnecting=false;

                receiveThread = new ReceiveThread(mSocket); // create instanct to new recieveThread
                receiveThread.start(); // start receive thread
            } catch(IOException ex){ // if exception accures
                Log.d("connection thread","could not connect");
                isConnecting=false;
                isConnected=false;
                if(btEventListener!=null){
                    btEventListener.onBluetoothDeviceDisconnected();
                }
            }
        }
        public void cancel() { // calback that allows to cancel bluetooth connection
            try{
                //receiveThread.cancel(); // stop recevie thread
                mSocket.close();	// close bluetooth socket
            }
            catch(IOException ex){
                // catch exceltio, exceptoin may accure if mSocket is already closed
            }
            isConnected=false;
        }
    }

    // thread that reads data from bluetooth connection
    public class ReceiveThread extends Thread{
        private BluetoothSocket socket; // bluetooth socket
        private InputStream mInputStream; // data input stream
        private OutputStream mOutputStream; // data outputstream
        private int b;

        ReceiveThread(BluetoothSocket mSocket){
            socket = mSocket;
        }
        int[] payloadBuffer = new int[128];
        // thread run
        public void run(){
            try{
                mInputStream = socket.getInputStream(); // creating input stream from socket
                mOutputStream = socket.getOutputStream(); // outputStream
                Log.d("ReceiveThread", "StreamCreated");
            } catch (IOException ex){
                Log.e("Receive Thread","Could not create stream");
                isConnected=false;
            }
            byte[] message = "START 3\n".getBytes();
            try {
                mOutputStream.write(message);
            } catch (IOException e) {
                e.printStackTrace();
            }
            while(true){ // this cycle repeatedly reads data from input stream, and forms acc data array with normalized accelerometer data
                try{	// the cycle breaks if exception accures
                    //Log.d("READbYTE","PrepearingToReadByte");

                    // read header and CRC
                    mOutputStream.write("CONT\n".getBytes());
                    long packetNumber = 0;
                    for(int i=0; i<4; i++){
                        packetNumber+=mInputStream.read()*Math.pow(2,8*i);
                    }
                    int packetLen = 0;
                    for(int i=0; i<2; i++){
                        packetLen+= mInputStream.read()*Math.pow(2,8*i);
                    }
                    for(int i=0; i<packetLen; i++){
                        payloadBuffer[i]=mInputStream.read();
                    }

                    // read crc
                    for(int i=0; i<4; i++){
                        mInputStream.read();
                    }

                    Log.d("BLUETOOTH_SERVICE",  "Packet index "+packetNumber);
                    Log.d("BLUETOOTH_SERVICE", "Packet length "+packetLen);
                    int[] data = new int[2];
                    data[0]=payloadBuffer[0]+payloadBuffer[1]*256;
                    data[1]=payloadBuffer[2]+payloadBuffer[3]*256;
                    Log.d("BLUETOOTH_SERVICE", "PACKET DATA: "+data[0]+" "+data[1]);

                    if(resultListener!=null){
                        resultListener.onBedditData(data);
                    }

//                    }


                } catch(IOException ex){
                    // exception can acure if bluetooth connection is lost
                    isConnected=false;
                    if(btEventListener!=null){
                        btEventListener.onBluetoothDeviceDisconnected();
                    }

                    Log.d("RECEIVE_DATA","Catched exception" +ex.toString());
                    break; // break the main cycle
                }
            }
        }
        public void cancel(){ // cancel method closes input stream
            try{			  // this results to that exception is thrown, and main cycle is braked
                mOutputStream.write("STOP\n".getBytes());
                mInputStream.close();
                mOutputStream.close();
                if(btEventListener!=null){
                    btEventListener.onBluetoothDeviceDisconnected();
                }
            } catch(IOException ex){

            }
        }
    }
}
