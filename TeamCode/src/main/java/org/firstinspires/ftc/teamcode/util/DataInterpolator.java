package org.firstinspires.ftc.teamcode.util;

import gay.zharel.fateweaver.telemetry.Fate;
import kotlin.jvm.functions.Function3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiFunction;

public class DataInterpolator<T> {

    public List<TimestampedData<T>> data = new ArrayList<>();

    private final Function3<Double, T, T, T> lerp;
    private double KEEP_SECONDS = -1.0;

    public DataInterpolator(BiFunction<T, T, T> addition, BiFunction<T, Double, T> scalarMult) {
        this((Double t, T e1, T e2) ->
            addition.apply(scalarMult.apply(e1, 1-t), scalarMult.apply(e2, t)));
    }

    public DataInterpolator(Function3<Double, T, T, T> lerp) {
        this.lerp = lerp;
    }

    public void addData(TimestampedData<T> data) {
        this.data.add(data);
        this.data.sort(TimestampedData::compareTo);
    }

    public void clearOld() {
        if (KEEP_SECONDS < 0.0) {
            return;
        }

        // clear out oldest
        int i = data.size() - 1;
        double currSeconds = TimeSource.getTimestamp();
        while (data.get(i).seconds < currSeconds - KEEP_SECONDS) {
            data.remove(i);
            i--;
        }
    }

    public void clear() {
        data.clear();
    }

    public T getData(double seconds) {
        clearOld();

        int i = 0;
        T out = data.get(0).data; // latest entry
        Fate.log("DataLerp/Size", data.size());
        Fate.log("DataLerp/First", data.get(0).seconds);
        Fate.log("DataLog/Request", seconds);
        Fate.log("DataLog/Current", TimeSource.getTimestamp());
        while (i <= data.size() - 1 && seconds - data.get(i).seconds < 0) {

            i++;
        } // go through until we reach the first entry that is older than the requested timestamp

        if (i == 0) { // if we don't have an entry at the time requested (too far in future), return latest
            Fate.log("DataLerp/Index", 0);
            return out;
        } else if (i == data.size()) { // we went through too many iterations! this request is too old, return oldest
            Fate.log("DataLerp/Index", i);
            return data.get(data.size() - 1).data;
        }

        // check for dbz
        double delta = (seconds - data.get(i).seconds);
        if (delta == 0.0) {
            Fate.log("DataLerp/Note", "halfway");
            return lerp.invoke(0.5, data.get(i).data, data.get(i-1).data); // same timestamp, average them
        }

        Fate.log("DataLerp/Index", i);

        // otherwise, apply a lerp between the two closest entries
        return lerp.invoke(delta / (data.get(i-1).getDeltaSeconds(data.get(i))),
                data.get(i).data, data.get(i-1).data);
    }

    public double getKeepSeconds() {
        return KEEP_SECONDS;
    }

    public void setKeepSeconds(double keepSeconds) {
        this.KEEP_SECONDS = keepSeconds;
    }
}
