import influxdb_client, os, time 
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

import paho.mqtt.client as mqtt

import json

MQTT_BROKER = "localhost"
MQTT_PORT = 1883

token = "mon-super-token-admin"
org = "ma_maison"
url = "http://localhost:8086"
bucket = "capteurs"

client = InfluxDBClient(url=url, token=token, org=org)
write_api = client.write_api(write_options=SYNCHRONOUS)

# point = (
#   Point("Transactions")
#   .field("timestamp", 1768487457.830432)
#   .field("uid", "1C00C336FA13")
#   .field("decision", "ALLOWED")
# )

# write_api.write(bucket=bucket, org=org, record=point)


def main():
  def on_message(client, userdata, message):
    room = message.topic.split('/')[0][-2:]
    data = json.loads(message.payload.decode('utf-8'))

    point = (
      Point("Transactions")
      .field("timestamp", float(data["timestamp"]))
      .field("room_id", room)
      .field("uid", data["uid"])
      .field("decision", data["decision"])
    )

    write_api.write(bucket=bucket, org=org, record=point)
  
  client = mqtt.Client()
  client.on_message = on_message
  client.connect(MQTT_BROKER, MQTT_PORT, 60)

  client.subscribe("+/transactions_carte")

  client.loop_forever()

if __name__ == "__main__":
  main()