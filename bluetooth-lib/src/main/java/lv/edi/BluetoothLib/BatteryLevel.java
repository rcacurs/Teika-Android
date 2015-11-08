package lv.edi.BluetoothLib;

import android.util.Log;

/**
 * Created by Richards on 26/06/2015.
 */
public class BatteryLevel {

    private int previousAdcValue=0;
    private int adcValue=0;
    private float referenceVoltageP = 2.5f;
    private float referenceVoltageN = 0.0f;
    private int resolution=10;
    private float inputScaling=0.5f;
    private BatteryLevelEventListener listener;

    private float percentHysteresis = 15;
    private float currentPercentage = 0;
    private float previousPercentage = 100;
    private float outputPercentage = 100;

    // curve fir coeefficients
    float a1 = 95.9f;
    float b1 = 4.16f;
    float c1 = 0.37f;
    float a2 = 24.09f;
    float b2 = 3.833f;
    float c2 = -0.1082f;

    /**
     * return integer representing raw ADC value
     */
    public int getRawADCValue(){
        return adcValue;
    }

    public float getADCVoltage(){
        return (float)(adcValue*(referenceVoltageP-referenceVoltageN)/(Math.pow(2, resolution)-1)+referenceVoltageN);
    }

    public void registerListener(BatteryLevelEventListener listener){
        this.listener = listener;
    }
    public float getBatteryVoltage(){
        return getADCVoltage()/inputScaling;
    }

    public void updateAdcValue(int adcValue){
        this.adcValue = adcValue;
        this.previousAdcValue = adcValue;
        Log.d("BATTERY_LEVEL", " adc value: " + adcValue);
        Log.d("BATTERY_LEVEL", " battery voltage" + getBatteryVoltage());
        Log.d("BATTERY_LEVEL", " battery percentage " + batteryVoltageLevelToPercent(getBatteryVoltage()));
        currentPercentage = batteryVoltageLevelToPercent(getBatteryVoltage());

        if(Math.abs(currentPercentage-previousPercentage)>percentHysteresis){
           previousPercentage=currentPercentage;
            Log.d("BATTERY_LEVEL_UPDATE", "BATTERY LEVEL UPDATE "+previousPercentage);
            if(listener!=null){
                listener.onBatteryLevelChange(this);
            }
        }
    }

    public float batteryVoltageLevelToPercent(float x){
        return (float)(a1*Math.exp(-Math.pow((x-b1)/c1,2))+a2*Math.exp(-Math.pow((x-b2)/c2,2)));
    }

    public float getBatteryPercentage(){
        return previousPercentage;
    }

}
