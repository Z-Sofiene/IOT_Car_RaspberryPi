package com.iot.dht.dao;

import com.iot.dht.model.Dht;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

@Repository
public interface DhtDao extends JpaRepository<Dht, Integer> {
}
