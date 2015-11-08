package lv.edi.SmartWearProcessing;

/**
 * Created by Richards on 27/06/2015. class implementing running filtering for accelerometer data
 *
 */
public class Filter {

    private float[] num = {0.0306f, -0.1005f, 0.1747f, -0.2022f, 0.1747f, -0.1005f, 0.0306f};
    private float[] den ={-4.1971f, 7.5534f, -7.4038f, 4.1551f, -1.2626f, 0.1625f};

    private float[] acc_bufx = new float[num.length];
    private float[] acc_bufy = new float[num.length];

    private float[] accy_temp = new float[den.length];


    public float filter(float val){
        System.arraycopy(acc_bufx, 0, acc_bufx, 1, acc_bufx.length-1);
        acc_bufx[0]=val;
        System.arraycopy(acc_bufy, 0, acc_bufy, 1, acc_bufy.length-1);
        System.arraycopy(acc_bufy, 1, accy_temp, 0, accy_temp.length);

        float filteredVal = SensorDataProcessing.dotProduct(num, acc_bufx)-SensorDataProcessing.dotProduct(den, accy_temp);
        acc_bufy[0]=filteredVal;
        return filteredVal;
    }
}
