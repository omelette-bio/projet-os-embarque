# lib standard python
import time
import logging
import argparse
from datetime import datetime
import json

# lib supplementaires
import smbus
import sqlite3
import paho.mqtt.client as mqtt


UID_LENGTH  = 16 # Nombre d'octets à lire
I2C_BUS     = 1 # Le bus I2C 


logger = logging.getLogger("Maitre I2C")
logging.basicConfig(
  level=logging.INFO,
  format="[%(name)s] [%(levelname)s] %(message)s"
)




parser = argparse.ArgumentParser(
prog='Passerelle IoT',
description="""
            """
)

parser.add_argument('room_id', 
                  type = int, 
                  help = 'Liste des id de salle à contrôler',
                  nargs='+'
)

parser.add_argument('ip_mqtt', 
                  type = str, 
                  help = 'L\'adresse IP du broker MQTT',
                  default = "localhost"
)

# =====================================
# ==    FONCTIONS I2C                ==
# =====================================

def send_command(i2c_add: int, command: str):
  try:
    bus.write_byte(i2c_add, ord(command))
    logger.info(f"Envoi de la commande de l'actionneur")
  except Exception as e:
    logger.error(f"Erreur lors de l'envoi de la commande de l'actionneur : {e}")

def send_actuator_command_ok(i2c_add: int):
  send_command(i2c_add, 'O')

def send_actuator_command_not_ok(i2c_add: int):
  send_command(i2c_add, 'N')

def get_card_uid(i2c_add: int):
  try:
    data = bus.read_i2c_block_data(i2c_add, 0x00, UID_LENGTH)
    uid_str = "".join([chr(x) for x in data if x != 0])
    return uid_str
  except:
    return

# =====================================
# ==    FONCTIONS I2C                ==
# =====================================

# =====================================
# ==    FONCTIONS DE FETCH BDD       ==
# =====================================

def get_room_permissions(i2c_add: int):
  global connection
  c = connection.cursor()
  query = "SELECT min_level, min_level_spe FROM room WHERE room_id = ?"
  c.execute(query, (i2c_add,))
  resultat = c.fetchone() 
  if resultat:
    return (resultat[0], resultat[1])
  else:
    return None

def get_user_permission(uid):
  global connection
  c = connection.cursor()
  query = "SELECT level, level_spe FROM employees WHERE uid = ?"
  c.execute(query, (uid,))
  resultat = c.fetchone()
  return (resultat[0], resultat[1]) if resultat else None

# =====================================
# ==    FONCTIONS DE FETCH BDD       ==
# =====================================

def is_permitted(perm_room, perm_user):
  (level_room, spe_room) = perm_room
  (level_user, spe_user) = perm_user
  return level_room <= level_user or spe_room <= spe_user

def send_mqtt(room_id: int, uid: str, decision: str):
  try:
      data = dict()
      data["timestamp"] = datetime.now().timestamp()
      data["uid"] = uid
      data["decision"] = decision

      data_final = json.dumps(data)
      topic = f"{room_id}/transactions_carte"
      client.publish(topic, data_final)
      logger.info(f"Transaction envoyée sur le topic {room_id}/transactions_carte")
  except Exception as e:
      logger.error(f"{e}")


def main():
  global rooms

  permission_list = dict()

  for room in rooms:
    logger.info(f"Récuperation des permissions de la salle {room}")

    room_permissions = get_room_permissions(room)

    if room_permissions : 
      logger.info(f"Permissions de la salle {room} obtenues : {room_permissions}")
      permission_list[room] = room_permissions
    else:
      logger.error(f"La salle {room} n'existe pas.")
      exit(1)

  logger.info(f"En attente de carte...")

  while True:
    for room in rooms:
      uid = get_card_uid(room)
      if uid:
        logger.info(f"Salle no.{room} Récuperation des permissions de l'employé no.{uid}")
        employee_permissions = get_user_permission(uid)
        state = "NON_EXISTANT"

        if employee_permissions: 
          logger.info(f'Employé no. {uid}, vérification des permissions d\'accès')
          res = is_permitted(permission_list[room], employee_permissions)

          if res:
            state = "ALLOWED"
            logger.info(f"Niveau de sécurité vérifié, accès autorisé")
            send_actuator_command_ok(room)

          else:
            state = "NON_ALLOWED"
            logger.warning(f"Niveau de sécurité trop bas, accès refusé")
            send_actuator_command_not_ok(room)

        else:
          logger.error(f"Employé no. {uid}, n'existe pas dans la base de données")
          send_actuator_command_not_ok(room)

        send_mqtt(room, uid, state)
        logger.info(f"En attente de carte...")

    time.sleep(0.5)

if __name__ == "__main__":

  args = parser.parse_args()
  rooms = args.room_id
  MQTT_BROKER_HOST = args.ip_mqtt

  not_connected = True
  client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
  while not_connected:
    try:
      client.connect(MQTT_BROKER_HOST, 1883, 60)
      logger.info(f"Connection au broker MQTT réussie")
      not_connected = False
    except Exception as e:
      logger.error(f"Erreur de connexion au broker MQTT : {e}")

  not_connected = True
  while not_connected:
    try:
      connection = sqlite3.connect('rooms-control.db')
      logger.info(f'Connection a la bdd réussie')
      not_connected = False
    except Exception as e:
      logger.error(f"Problème de connection a la bdd : {e}")

  not_connected = True
  while not_connected:
    try:
      bus = smbus.SMBus(I2C_BUS)
      logger.info(f'Connection au bus I2C réussie')
      not_connected = False
    except Exception as e:
      logger.error(f"Impossible d'ouvrir le bus I2C : {e}")


  main()
