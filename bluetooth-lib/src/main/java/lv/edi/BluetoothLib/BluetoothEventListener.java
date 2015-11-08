package lv.edi.BluetoothLib;

/**
 * Created by Richards on 18/06/2015.
 */
public interface BluetoothEventListener {

    public void onBluetoothDeviceConnecting();
    public void onBluetoothDeviceConnected();
    public void onBluetoothDeviceDisconnected();

}
