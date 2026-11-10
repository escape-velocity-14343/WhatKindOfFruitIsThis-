package org.firstinspires.ftc.teamcode.util;

import kotlin.jvm.functions.Function3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.BiFunction;

public class DataInterpolator<T> {

    public List<TimestampedData<T>> data = new ArrayList<>();

    private final Function3<Double, T, T, T> lerp;

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

    public T getData(double seconds) {
        int i = 0;
        T out = data.get(0).data; // latest entry
        while (i <= data.size() - 1 && seconds - data.get(i).seconds < 0) {
            i++;
        } // go through until we reach the first entry that is older than the requested timestamp

        if (i == 0) { // if we don't have an entry at the time requested (too far in future), return latest
            return out;
        } else if (i == data.size()) { // we went through too many iterations! this request is too old, return oldest
            return data.get(data.size() - 1).data;
        }

        // check for dbz
        double delta = (seconds - data.get(i).seconds);
        if (delta == 0.0) {
            return lerp.invoke(0.5, data.get(i).data, data.get(i-1).data); // same timestamp, average them
        }

        // otherwise, apply a lerp between the two closest entries
        return lerp.invoke(delta / (data.get(i-1).getDeltaSeconds(data.get(i))),
                data.get(i).data, data.get(i-1).data);
    }

}
