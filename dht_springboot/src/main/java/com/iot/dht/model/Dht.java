package com.iot.dht.model;

import jakarta.persistence.Entity;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;

@Entity
public class Dht {
    @Id @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Integer id;
    private float temperature;
    private float humidity;

    public Dht() {
    }

    public Dht(Integer id, float temperature, float humidity) {
        this.id = id;
        this.temperature = temperature;
        this.humidity = humidity;
    }

    public Integer getId() {
        return id;
    }

    public void setId(Integer id) {
        this.id = id;
    }

    public float getTemperature() {
        return temperature;
    }

    public void setTemperature(float temperature) {
        this.temperature = temperature;
    }

    public float getHumidity() {
        return humidity;
    }

    public void setHumidity(float humidity) {
        this.humidity = humidity;
    }
}
