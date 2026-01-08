# projet-os-embarque

- je me rappelle plus comment on setup le projet cote arduino

## demarrer et tester le broker mqtt

```
cd mqtt_broker
docker compose up -d
```
écouter les messages sur le topic `topix`
```
docker exec -it mosquitto mosquitto_sub -t "topix"
```
envoyer le message `msg` sur le topic `topix`
```
docker exec -it mosquitto mosquitto_pub -t "topix" -m "msg"
```