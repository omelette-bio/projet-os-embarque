# projet-os-embarque

## demarrer le projet sur arduino

executer la commande à la racine du projet pour decompresser le core arduino et freertos
```bash
./install_dep.sh
```

compiler le code source arduino avr-gcc et upload SANS brancher le capteur sur le port UART (sinon ça marche pas)
```bash
make
make upload
```

## démarrer la passerelle
### installer les packages nécessaires
- mosquitto
- sqlite3
- i2c

### lancer le maitre i2c
demarrer un environnement virtuel python (optionnel)
```bash
python3 -m venv .venv
source .venv/bin/activate
```

installer les requirements
```bash
pip install -r requirements.txt
```

enfin pour démarrer le maitre i2c
```bash
python3 main.py <ip broker mqtt>
```
par exemple, si je dois me connecter au broker mqtt localhost

```bash
python3 main.py localhost
```

## démarrer le fog
### démarrer la base de données et le broker MQTT
se placer dans le répertoire `Fog/` et exécuter la commande
```bash
docker compose up -d
```
qui devrait démarrer la base de données InfluxDB et le broker MQTT

### démarrer le consumer mqtt
se placer dans le répertoire `Fog/bridge_mqtt_bdd`, puis installer les dépendances python, donc pour cela, démarrer un environnement virtuel python (optionnel)
```bash
python3 -m venv .venv
source .venv/bin/activate
```
et ensuite
```bash
pip install -r requirements.txt
```

et enfin pour démarrer le programme
```bash
python3 main.py
``` 
