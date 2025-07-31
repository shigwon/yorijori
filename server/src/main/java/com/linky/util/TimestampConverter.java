package com.linky.util;

import com.google.protobuf.Timestamp;
import lombok.experimental.UtilityClass;

import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.ZonedDateTime;

@UtilityClass
public class TimestampConverter {

    public static Timestamp toTimestamp(LocalDateTime localDateTime) {
        if(localDateTime == null) {
            return Timestamp.getDefaultInstance();
        }

        ZonedDateTime zonedDateTime = localDateTime.atZone(ZoneId.systemDefault());

        Instant instant = zonedDateTime.toInstant();

        return Timestamp.newBuilder()
                .setSeconds(instant.getEpochSecond())
                .setNanos(instant.getNano())
                .build();
    }
}
