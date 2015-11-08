package lv.teika.www.teikaandroid;

import android.bluetooth.BluetoothAdapter;
import android.content.Context;
import android.content.Intent;
import android.content.res.Resources;
import android.graphics.Color;
import android.os.Vibrator;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.HashMap;
import java.util.Map;

import lv.edi.BluetoothLib.BedditResultListener;

public class Teika extends AppCompatActivity implements BedditResultListener {
    final int REQUEST_ENABLE_BT = 1;
    private TeikaApplication app;
    private Resources res;
    private int preveious1 = 0;
    private int previous2 = 0;
    private int delta1 = 0;
    private int delta2 = 0;
    ImageView inactivityView;
    TextView inactivityTimeView;
    Vibrator v;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.content_main);
        res = getResources();

        app = (TeikaApplication)getApplication();
        app.btAdapter = BluetoothAdapter.getDefaultAdapter();
        if(app.btAdapter == null){
            Toast.makeText(this, res.getString(R.string.toast_bt_not_supported), Toast.LENGTH_SHORT).show();
            finish();
        }
        // check if bluetooth is turned on
        if(!app.btAdapter.isEnabled()){
            // intnet to open activity, to turn on bluetooth if bluetooth no turned on
            Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableIntent, REQUEST_ENABLE_BT); //start activity for result
        }

        String btAddress = app.sharedPrefs.getString("pref_bluetooth_target", "none");
        if(btAddress.equals("none")){
            app.btDevice = null;
        } else{
            app.btDevice = app.btAdapter.getRemoteDevice(btAddress);
        }

        app.bedditService.setBeditResultListener(this);
        app.lastActivityTime = System.currentTimeMillis();
        inactivityView = (ImageView)findViewById(R.id.icon1);
        inactivityTimeView = (TextView)findViewById(R.id.secondLine1);

        v = (Vibrator) this.getSystemService(Context.VIBRATOR_SERVICE);
        //pushPebbleNotification();

    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_teika, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            Intent intent = new Intent(this, SettingsActivity.class);
            startActivity(intent);
            return true;
        }
        if(id == R.id.action_connectbt){
            if(app.btDevice!=null) {
                //app.bedditService.connectDevice(app.btDevice);
                if(!(app.bedditService.isConnected())) {
                    app.bedditService.connectDevice(app.btDevice);
                } else{
                    app.bedditService.disconnectDevice();
                }
            } else{
                Toast.makeText(this, "Bt device not set", Toast.LENGTH_SHORT).show();
            }
        }

        return super.onOptionsItemSelected(item);
    }

    public void onBedditData(int[] data){
        Log.d("BEDDIT_ACITIVY", "DATA"+data[0] + " " + data[1]);
        delta1=preveious1-data[0];
        delta2=previous2-data[1];
        Log.d("BEDDIT_ACITIVY", "DELTA: "+delta1+" "+delta2);
        if(Math.abs(delta1)>app.movementTriggerThreshold){
            Log.d("MOVEMENT_OVER_THRESHOLD", "TRUE");
            app.lastActivityTime = System.currentTimeMillis();

            this.runOnUiThread(new Runnable() {
                                   public void run() {
                                       //Toast.makeText(getApplicationContext(), "Please check patient", Toast.LENGTH_SHORT).show();
                                       inactivityView.setBackgroundColor(Color.GREEN);
                                   }
                               }
            );
        }
        preveious1=data[0];
        previous2=data[1];

        // check current time
        long currentTime = System.currentTimeMillis();
        final long deltaTime = currentTime - app.lastActivityTime;

        inactivityView.post(new Runnable() {
            public void run() {
                inactivityTimeView.setText(String.format("%.1f",((float)deltaTime/1000)));
            }
        });


        if(deltaTime>app.passivnesTimeThreshold){
            Log.d("NOT_ACTIVE", "TRUE");
            app.lastActivityTime=currentTime;
            v.vibrate(500);
            this.runOnUiThread(new Runnable() {
                                   public void run() {
                                       Toast.makeText(getApplicationContext(), "Please check patient", Toast.LENGTH_SHORT).show();
                                       inactivityView.setBackgroundColor(Color.RED);
                                       pushPebbleNotification();
                                   }
                               }
            );
        }

    }

    @Override
    public void onDestroy()
    {
        super.onDestroy();
        app.bedditService.disconnectDevice();
    }

    // Function for notification pushing on Pebble
    public void pushPebbleNotification() {
        // Push a notification
        final Intent i = new Intent("com.getpebble.action.SEND_NOTIFICATION");

        final Map data = new HashMap();
        data.put("title", "Attention!");
        data.put("body", "Patient number X needs to be repositioned.");
        final JSONObject jsonData = new JSONObject(data);
        final String notificationData = new JSONArray().put(jsonData).toString();

        i.putExtra("messageType", "PEBBLE_ALERT");
        i.putExtra("sender", "PebbleKit Android");
        i.putExtra("notificationData", notificationData);
        sendBroadcast(i);
    }
}
