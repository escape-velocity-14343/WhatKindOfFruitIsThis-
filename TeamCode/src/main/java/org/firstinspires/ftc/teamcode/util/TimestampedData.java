package org.firstinspires.ftc.teamcode.util;

import java.sql.Time;

public class TimestampedData<T> extends Comparable<TimestampedData<T>> {

    public T data;
    public long nanoTime;
    public double seconds;

    public TimestampedData(T data) {
        this(data, System.nanoTime());
    }

    public TimestampedData(T data, long time) {
        this.data = data;
        this.nanoTime = time;
        this.seconds = nanoTime / 1e9d;
    }


    public double getDeltaSeconds() {
        return System.nanoTime() / 1e9d - this.seconds;
    }

    public double getDeltaSeconds(TimestampedData<T> other) {
        return other.seconds - this.seconds;
    }

    @Override
    public int compareTo(TimestampedData<T> other) {
        return Double.compare(other.seconds, this.seconds);
    }
}
