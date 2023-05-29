import csv
import serial
import threading
from fastapi import FastAPI
# import time
from typing import List
from uuid import uuid4
from fastapi import FastAPI
# from fastapi import HTTPException
from fastapi.middleware.cors import CORSMiddleware
import csv
# from fastapi.responses import StreamingResponse
from datetime import datetime
from datetime import datetime, timedelta
from typing import List

# ser = serial.Serial('COM29', 9600)
filename = "gps_measurements.csv"

app = FastAPI()

# add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:8081", "http://localhost:8082"], # replace with your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/magnetometer")
async def get_accel():
 filename = "gps_measurements.csv"
 with open(filename, "r") as file:
        # Leer el contenido del archivo CSV
        csv_content = file.read()
        # Convertir el contenido a una lista de listas usando csv.reader
        csv_reader = csv.reader(csv_content.splitlines())
        data = [row[8:11] for row in csv_reader]     
        # Agregar la hora a cada fila de data
        start_time = datetime.now() # Hora actual
        time_delta = timedelta(seconds=5) # Incremento de 5 segundos
        for i, row in enumerate(data):
            row.append((start_time - i*time_delta).strftime("%H:%M:%S"))   
        return data

@app.get("/accelerometer")
async def get_accel():
 filename = "gps_measurements.csv"
 with open(filename, "r") as file:
        # Leer el contenido del archivo CSV
        csv_content = file.read()
        # Convertir el contenido a una lista de listas usando csv.reader
        csv_reader = csv.reader(csv_content.splitlines())
        data = [row[2:5] for row in csv_reader]
        # Agregar la hora a cada fila de data
        start_time = datetime.now() # Hora actual
        time_delta = timedelta(seconds=5) # Incremento de 5 segundos
        for i, row in enumerate(data):
            row.append((start_time - i*time_delta).strftime("%H:%M:%S"))
        # Retornar la lista de listas como JSON        
        return data
    
@app.get("/giroscope")
async def get_giros():
 filename = "gps_measurements.csv"
 with open(filename, "r") as file:
        # Leer el contenido del archivo CSV
        csv_content = file.read()
        # Convertir el contenido a una lista de listas usando csv.reader
        csv_reader = csv.reader(csv_content.splitlines())
        data = [row[5:8] for row in csv_reader]
        # Agregar la hora a cada fila de data
        start_time = datetime.now() # Hora actual
        time_delta = timedelta(seconds=5) # Incremento de 5 segundos
        for i, row in enumerate(data):
            row.append((start_time - i*time_delta).strftime("%H:%M:%S"))
        # Retornar la lista de listas como JSON
        return data

@app.get("/GPS")
async def get_GPS():
    filename = "gps_measurements.csv"
    with open(filename, "r") as file:
        # Leer el contenido del archivo CSV
        csv_content = file.read()
        # Convertir el contenido a una lista de listas usando csv.reader
        csv_reader = csv.reader(csv_content.splitlines())
        data = [row[:2] for row in csv_reader]
        # Retornar la lista de listas como JSON
        return data

@app.get("/")
def read_csv():
    # Lee el archivo CSV y devuelve los datos
    with open(filename, mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        return [row for row in csv_reader]
    
@app.put("/load_data")
def write_csv(data: list):
    # Escribe la lista como una línea en el archivo CSV
    with open(filename, mode='a', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(data)
    return {"message": "Datos escritos en el archivo CSV"}
    
if __name__ == "__main__":
    # Inicia la lectura del puerto serial en un hilo separado
    # start_reading_serial()
    # Inicia la API de FastAPI
    import uvicorn
    uvicorn.run(app, host="localhost", port=8002)

# def read_serial_and_update_csv():
#     with open(filename, mode='a', newline='') as csv_file:
#         fieldnames = [ 'lat', 'lon','accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'mag_x','mag_y','mag_z']
#         writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
#         while True:
#             try:
#                 # Leer los datos del puerto serial y decodificarlos a Unicode
#                 for i in [1,2]:
#                     line = ser.readline().decode('utf-8').rstrip()
#                     # Parsea los datos en una lista                
#                     data = str(line).split(',')
#                 print(data)

#                 if (len(data)==11):
#                     # Calculo de long
#                     long = data[1]
                    
#                     if (long == ''):
#                         lat_decimal = 0
#                         long_decimal = 0
#                     else: 
#                         # Separar los dos primeros decimales de lat
#                         long_str = str(long)
#                         grados = long_str[:2]
#                         flag = False
#                         if (grados[0]=='0'):
#                             grados = long_str[1:3]
#                             flag = True
#                         if len(long_str) >= 3:
#                             if (flag):
#                                 minutos = float(long_str[3:])/60.0    
#                             else:
#                                 minutos = float(long_str[2:])/60.0
#                         else:
#                             minutos = 0.0
#                         if (grados!=' ' and grados!=''):
#                             long_decimal = - (float(grados) + minutos)
#                         else :
#                             long_decimal = 0

#                         # Calculo de lat
#                         lat = data[0]
#                         # Separar los dos primeros decimales de lat
#                         lat_str = str(lat)
#                         grados = lat_str[:2]
#                         if len(lat_str) >= 3:
#                             minutos = float(lat_str[2:])/60.0
#                         else:
#                             minutos = 0.0
#                         if (grados!=' ' and grados!=''):
#                             lat_decimal = float(grados) + minutos
#                         else :
#                             lat_decimal = 0


#                     # Escribe los datos en el archivo CSV
#                     writer.writerow({
#                         'lat': lat_decimal,
#                         'lon': long_decimal,
#                         'accel_x': data[2],
#                         'accel_y': data[3],
#                         'accel_z': data[4],
#                         'gyro_x': data[5],
#                         'gyro_y': data[6],
#                         'gyro_z': data[7],
#                         'mag_x': data[8],
#                         'mag_y': data[9],
#                         'mag_z': data[10]
#                     })

#                 else:
#                     # lat_decimal = 6.26800384658793
#                     # long_decimal = -75.5688201473909
#                     lat_decimal = 0
#                     long_decimal = 0
#                     # Escribe los datos en el archivo CSV
#                     writer.writerow({
#                         'lat': lat_decimal,
#                         'lon': long_decimal,
#                         'accel_x': data[1],
#                         'accel_y': data[2],
#                         'accel_z': data[3],
#                         'gyro_x': data[4],
#                         'gyro_y': data[5],
#                         'gyro_z': data[6],
#                         'mag_x': data[7],
#                         'mag_y': data[8],
#                         'mag_z': data[9]
#                     })
#             except KeyboardInterrupt:
#                 print('Programa detenido')
#                 break
#         ser.close() # Cerrar la conexión serial

# def start_reading_serial():
#     read_serial_thread = threading.Thread(target=read_serial_and_update_csv)
#     read_serial_thread.start()