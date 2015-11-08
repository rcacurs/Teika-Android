package lv.teika.www.teikaandroid;

import android.app.Application;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.res.Resources;
import android.os.Bundle;
import android.preference.Preference;
import android.preference.PreferenceManager;
import android.util.Log;

import java.util.Vector;

import lv.edi.BluetoothLib.BatteryLevel;
import lv.edi.BluetoothLib.BatteryLevelEventListener;
import lv.edi.BluetoothLib.BedditBTService;
import lv.edi.BluetoothLib.BluetoothEventListener;
import lv.edi.BluetoothLib.DeviceListActivity;
import lv.edi.SmartWearProcessing.Sensor;

/**
 * Created by Richards on 07.11.2015..
 */
public class TeikaApplication extends Application implements SharedPreferences.OnSharedPreferenceChangeListener {

    BluetoothAdapter btAdapter;
    BluetoothDevice btDevice;
    SharedPreferences sharedPrefs;

    BedditBTService bedditService;
    int movementTriggerThreshold = 1000;
    long lastActivityTime = 0;
    long passivnesTimeThreshold = 5000; // [ms]


    public void onCreate(){
        super.onCreate();
        sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this);
        sharedPrefs.registerOnSharedPreferenceChangeListener(this);
        bedditService = new BedditBTService();
        String thresholdValue = sharedPrefs.getString("inactivity_threshold", "5000");
        passivnesTimeThreshold = Long.parseLong(thresholdValue);

    }

    @Override
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
        if (key.equals("pref_bluetooth_target")) {
            String btDeviceAddress = sharedPreferences.getString("pref_bluetooth_target", "none");
            if (btDeviceAddress.equals("none")) {
                btDevice = null;
            } else {
                btDevice = btAdapter.getRemoteDevice(btDeviceAddress);
            }
        }
        if(key.equals("inactivity_threshold")){
            String value = sharedPreferences.getString("inactivity_threshold", "5000");
            passivnesTimeThreshold = Long.parseLong(value);
        }
    }
}
