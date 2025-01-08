package com.iot.dht.controller;

import com.iot.dht.dao.DhtDao;
import com.iot.dht.model.Dht;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@CrossOrigin("*")
@RestController
@RequestMapping("/api")
public class AppCRT {

    @Autowired
    private DhtDao dao;

    private static Float latestTemp;
    private static Float latestHum;



    @GetMapping("/dht/liste")
    public List<Dht> getAllDht() {
        return dao.findAll();
    }

    @PostMapping("/dht/stream")
    public ResponseEntity<Void> updateDhtData(@RequestBody Map<String, Float> data) {
        latestTemp = data.get("Temperature");
        latestHum = data.get("Humidity");
        return ResponseEntity.ok().build();
    }

    @GetMapping("/dht/live")
    public ResponseEntity<Map<String, Float>> getLatestDhtData() {
        Map<String, Float> response = new HashMap<>();
        response.put("Temperature", latestTemp);
        response.put("Humidity", latestHum);
        return new ResponseEntity<>(response, HttpStatus.OK);
    }

    // Methode 2 de sauvegarde
    @PostMapping("/dht/add")
    public ResponseEntity<Dht> addDht(@RequestBody Dht dht) {
        if (dht.getTemperature() < 0 || dht.getHumidity() < 0) {
            return new ResponseEntity<>(HttpStatus.BAD_REQUEST);
        }
        Dht savedDht = dao.save(dht);
        return new ResponseEntity<>(savedDht, HttpStatus.CREATED);
    }

}
